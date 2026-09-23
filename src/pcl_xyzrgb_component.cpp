#include <algorithm>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <limits>
#include <cstring>
#include <stdexcept>
#include <vector>

#include <rclcpp/expand_topic_or_service_name.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/synchronizer.hpp>
#include <message_filters/sync_policies/exact_time.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <point_cloud_transport/point_cloud_transport.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <image_geometry/pinhole_camera_model.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>

#include "hma_pcl_reconst2/depth_denoise.hpp"
#include "hma_pcl_reconst2/depth_traits.hpp"

namespace hma_pcl_reconst2
{

namespace enc = sensor_msgs::image_encodings;
using PointCloud = sensor_msgs::msg::PointCloud2;

using namespace std::chrono_literals;

namespace
{

struct ImageTopic
{
  std::string base_topic;
  std::string transport;
};

std::string pointCloudTransportPluginName(const std::string & transport)
{
  if (transport.find('/') != std::string::npos) {
    return transport;
  }
  return "point_cloud_transport/" + transport;
}

std::string pointCloudTransportTopic(const std::string & base_topic, const std::string & transport)
{
  return base_topic + "/" + transport;
}

bool stripTransportSuffix(
  const std::string & topic, const std::string & transport, std::string & base_topic)
{
  const std::string suffix = "/" + transport;
  if (topic.size() <= suffix.size()) {
    return false;
  }
  if (topic.compare(topic.size() - suffix.size(), suffix.size(), suffix) != 0) {
    return false;
  }

  base_topic = topic.substr(0, topic.size() - suffix.size());
  return !base_topic.empty();
}

ImageTopic resolveImageTopic(
  const std::string & requested_topic, const std::string & default_transport)
{
  std::string base_topic;
  for (const auto & transport :
    {std::string("compressedDepth"), std::string("compressed")})
  {
    if (stripTransportSuffix(requested_topic, transport, base_topic)) {
      return {base_topic, transport};
    }
  }

  return {requested_topic, default_transport};
}

}  // namespace

class PointCloudXyzrgb : public rclcpp::Node
{
public:
  explicit PointCloudXyzrgb(const rclcpp::NodeOptions & options)
  : Node("pointcloud_xyzrgb", options), subscribed_(false)
  {
    this->declare_parameter("queue_size", 5);
    this->declare_parameter("exact_sync", false);
    this->declare_parameter("topic_rgb", "/head_rgbd_sensor/rgb/image_rect_color");
    this->declare_parameter("topic_depth", "/head_rgbd_sensor/depth_registered/image_rect_raw");
    this->declare_parameter("topic_camera_info", "/head_rgbd_sensor/rgb/camera_info");
    this->declare_parameter("output_topic", "/hma_pcl_reconst/depth_registered/points");
    this->declare_parameter("compressed_transport", "zstd");
    //this->declare_parameter("topic_rgb", "/head_rgbd_sensor/rgb/image_raw");
    //this->declare_parameter("topic_depth", "/head_rgbd_sensor/depth_registered/image_raw");
    this->declare_parameter("use_compressed", false);
    this->declare_parameter("use_pointcloud_compressed", false);
    // History depth of the image subscriptions. 1 always processes the newest frame:
    // a deeper queue only adds latency (a full queue of stale frames) whenever
    // processing takes about as long as the frame period.
    this->declare_parameter("input_queue_depth", 1);
    DepthDenoiser::declareParameters(this);

    denoiser_.configure(this);
    RCLCPP_INFO(this->get_logger(), "Depth denoise: %s", denoiser_.describe().c_str());

    // Re-configure the denoiser whenever a denoise.* parameter is changed at runtime
    // (ros2 param set / rqt_reconfigure), so it can be tuned against a live camera.
    param_cb_ = this->add_post_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        const bool touched = std::any_of(
          params.begin(), params.end(), [](const rclcpp::Parameter & p) {
            return p.get_name().rfind("denoise.", 0) == 0;
          });
        if (!touched) {
          return;
        }
        std::lock_guard<std::mutex> lock(denoise_mutex_);
        denoiser_.configure(this);
        RCLCPP_INFO(
          this->get_logger(), "Depth denoise reconfigured: %s", denoiser_.describe().c_str());
      });

    use_image_compressed_ = this->get_parameter("use_compressed").as_bool();
    use_pointcloud_compressed_ = this->get_parameter("use_pointcloud_compressed").as_bool();
    compressed_transport_ = this->get_parameter("compressed_transport").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();

    const std::string default_rgb_transport = use_image_compressed_ ? "compressed" : "raw";
    const std::string default_depth_transport = use_image_compressed_ ? "compressedDepth" : "raw";

    const auto rgb_topic = resolveImageTopic(
      this->get_parameter("topic_rgb").as_string(),
      default_rgb_transport);
    const auto depth_topic = resolveImageTopic(
      this->get_parameter("topic_depth").as_string(),
      default_depth_transport);

    topic_rgb_ = rgb_topic.base_topic;
    topic_depth_ = depth_topic.base_topic;
    topic_camera_info_ = this->get_parameter("topic_camera_info").as_string();
    rgb_transport_ = rgb_topic.transport;
    depth_transport_ = depth_topic.transport;

    RCLCPP_INFO(
      this->get_logger(),
      "Configured image inputs: rgb=%s (%s), depth=%s (%s), camera_info=%s, compressed_input=%s",
      topic_rgb_.c_str(), rgb_transport_.c_str(), topic_depth_.c_str(), depth_transport_.c_str(),
      topic_camera_info_.c_str(), use_image_compressed_ ? "true" : "false");

    int queue_size = this->get_parameter("queue_size").as_int();
    bool exact_sync = this->get_parameter("exact_sync").as_bool();

    //rclcpp::QoS qos(10);
    auto qos = rclcpp::SensorDataQoS();

    if (use_pointcloud_compressed_) {
      configurePointCloudTransportPublisher(qos);
    } else {
      pub_point_cloud_ = this->create_publisher<PointCloud>(output_topic_, qos);
      RCLCPP_INFO(
        this->get_logger(),
        "Publishing raw PointCloud2 on %s", output_topic_.c_str());
    }

    if (exact_sync) {
      exact_sync_ = std::make_shared<ExactSync>(
        ExactSyncPolicy(queue_size), sub_depth_, sub_rgb_);
      exact_sync_->registerCallback(
        std::bind(&PointCloudXyzrgb::imageCb, this, std::placeholders::_1, std::placeholders::_2));
    } else {
      sync_ = std::make_shared<Sync>(
        SyncPolicy(queue_size), sub_depth_, sub_rgb_);
      sync_->registerCallback(
        std::bind(&PointCloudXyzrgb::imageCb, this, std::placeholders::_1, std::placeholders::_2));
    }

    // Image processing runs in its own callback group so that the parameter services,
    // which stay in the default group, keep answering while imageCb is busy (under
    // component_container_mt). Otherwise a 30 Hz 1280x720 stream starves them and
    // rqt_reconfigure times out.
    image_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    image_sub_options_.callback_group = image_cb_group_;
    image_qos_ = rmw_qos_profile_sensor_data;
    image_qos_.depth = static_cast<size_t>(
      std::max<int64_t>(1, this->get_parameter("input_queue_depth").as_int()));

    //camera info
    sub_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      topic_camera_info_, 1,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        model_.fromCameraInfo(msg);
      }, image_sub_options_);

    timer_ = this->create_wall_timer(1000ms, [this]() {
    auto subs = this->getPointCloudSubscriptionCount();
    
    if (subs > 0) {
      if (!subscribed_) {
        this->startSubscribing();
        RCLCPP_INFO(this->get_logger(), "hma_pcl_reconst2.-> New subscriber detected, subscribing to topics.");
      }
    } else {
      if (subscribed_) {
        this->stopSubscribing();
        RCLCPP_INFO(this->get_logger(), "hma_pcl_reconst2.-> No subscribers, unsubscribing from topics.");
        }
      }
    }, image_cb_group_);

    RCLCPP_INFO(this->get_logger(), "hma_pcl_reconst2.-> component initialized");
  }

private:

  std::string topic_rgb_, topic_depth_, topic_camera_info_;
  std::string rgb_transport_, depth_transport_;
  std::string output_topic_, compressed_transport_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr image_cb_group_;
  rclcpp::SubscriptionOptions image_sub_options_;
  rmw_qos_profile_t image_qos_;
  bool subscribed_;
  bool use_image_compressed_;
  bool use_pointcloud_compressed_;
  DepthDenoiser denoiser_;
  std::mutex denoise_mutex_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr param_cb_;

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
  using ExactSyncPolicy = message_filters::sync_policies::ExactTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;
  using ExactSync = message_filters::Synchronizer<ExactSyncPolicy>;

  std::shared_ptr<image_transport::ImageTransport> rgb_it_, depth_it_;
  image_transport::SubscriberFilter sub_rgb_, sub_depth_;
  std::shared_ptr<Sync> sync_;
  std::shared_ptr<ExactSync> exact_sync_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
  rclcpp::Publisher<PointCloud>::SharedPtr pub_point_cloud_;
  point_cloud_transport::Publisher pub_point_cloud_transport_;

  image_geometry::PinholeCameraModel model_;

  std::vector<float> x_lut_;
  std::vector<float> y_lut_;
  uint32_t lut_width_ = 0;
  uint32_t lut_height_ = 0;
  double lut_fx_ = 0.0;
  double lut_fy_ = 0.0;
  double lut_cx_ = 0.0;
  double lut_cy_ = 0.0;

  std::string getPointCloudTransportPluginParameterName(const std::string & topic) const
  {
    auto expanded_topic = rclcpp::expand_topic_or_service_name(
      topic, this->get_name(), this->get_namespace());
    const auto namespace_length = this->get_effective_namespace().length();
    auto parameter_base_name = expanded_topic.substr(namespace_length);
    std::replace(parameter_base_name.begin(), parameter_base_name.end(), '/', '.');
    if (!parameter_base_name.empty() && parameter_base_name.front() == '.') {
      parameter_base_name = parameter_base_name.substr(1);
    }
    return parameter_base_name + ".enable_pub_plugins";
  }

  std::string getPointCloudTransportParameterName(
    const std::string & topic, const std::string & transport, const std::string & parameter) const
  {
    auto expanded_topic = rclcpp::expand_topic_or_service_name(
      pointCloudTransportTopic(topic, transport), this->get_name(), this->get_namespace());
    const auto namespace_length = this->get_effective_namespace().length();
    auto parameter_base_name = expanded_topic.substr(namespace_length);
    std::replace(parameter_base_name.begin(), parameter_base_name.end(), '/', '.');
    if (!parameter_base_name.empty() && parameter_base_name.front() == '.') {
      parameter_base_name = parameter_base_name.substr(1);
    }
    return parameter_base_name + "." + parameter;
  }

  void configurePointCloudTransportPublisher(const rclcpp::QoS & qos)
  {
    const auto plugin_name = pointCloudTransportPluginName(compressed_transport_);
    const auto enable_plugins_parameter = getPointCloudTransportPluginParameterName(output_topic_);
    const auto zstd_encode_level_parameter =
      getPointCloudTransportParameterName(output_topic_, compressed_transport_, "encode_level");
    constexpr int zstd_encode_level = 1;

    if (!this->has_parameter(enable_plugins_parameter)) {
      this->declare_parameter<std::vector<std::string>>(
        enable_plugins_parameter, std::vector<std::string>{plugin_name});
    }

    auto node_ptr = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});
    try {
    pub_point_cloud_transport_ = point_cloud_transport::create_publisher(
      node_ptr, output_topic_, qos.get_rmw_qos_profile());

    if (compressed_transport_ == "zstd" && this->has_parameter(zstd_encode_level_parameter)) {
      this->set_parameter(rclcpp::Parameter(zstd_encode_level_parameter, zstd_encode_level));
      RCLCPP_INFO(
        this->get_logger(),
        "Configured zstd point cloud encode_level=%d", zstd_encode_level);
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Publishing compressed PointCloud2 on %s/%s using %s",
        output_topic_.c_str(), compressed_transport_.c_str(), plugin_name.c_str());
    } catch (const std::exception & e) {
      use_pointcloud_compressed_ = false;
      pub_point_cloud_ = this->create_publisher<PointCloud>(output_topic_, qos);
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to create point_cloud_transport publisher with %s: %s",
        plugin_name.c_str(), e.what());
      RCLCPP_WARN(
        this->get_logger(),
        "Falling back to raw PointCloud2 on %s. Install the %s package to use compressed point clouds.",
        output_topic_.c_str(), compressed_transport_.c_str());
    }
  }

  uint32_t getPointCloudSubscriptionCount() const
  {
    if (use_pointcloud_compressed_) {
      return pub_point_cloud_transport_.getNumSubscribers();
    }
    return pub_point_cloud_ ? pub_point_cloud_->get_subscription_count() : 0;
  }

  void publishPointCloud(const PointCloud::SharedPtr & cloud_msg) const
  {
    if (use_pointcloud_compressed_) {
      pub_point_cloud_transport_.publish(*cloud_msg);
      return;
    }
    pub_point_cloud_->publish(*cloud_msg);
  }

  void updateLookupTables(uint32_t width, uint32_t height)
  {
    const double fx = model_.fx();
    const double fy = model_.fy();
    const double cx = model_.cx();
    const double cy = model_.cy();

    if (width == lut_width_ && height == lut_height_ &&
        fx == lut_fx_ && fy == lut_fy_ && cx == lut_cx_ && cy == lut_cy_) {
      return;
    }

    x_lut_.resize(width);
    y_lut_.resize(height);

    for (uint32_t u = 0; u < width; ++u) {
      x_lut_[u] = (static_cast<float>(u) - static_cast<float>(cx)) / static_cast<float>(fx);
    }
    for (uint32_t v = 0; v < height; ++v) {
      y_lut_[v] = (static_cast<float>(v) - static_cast<float>(cy)) / static_cast<float>(fy);
    }

    lut_width_ = width;
    lut_height_ = height;
    lut_fx_ = fx;
    lut_fy_ = fy;
    lut_cx_ = cx;
    lut_cy_ = cy;
  }

  static uint32_t fieldOffset(const PointCloud & cloud, const std::string & name)
  {
    for (const auto & field : cloud.fields) {
      if (field.name == name) {
        return field.offset;
      }
    }
    throw std::runtime_error("PointCloud2 field not found: " + name);
  }

  static inline void writeFloat(uint8_t * point, uint32_t offset, float value)
  {
    std::memcpy(point + offset, &value, sizeof(value));
  }

  static inline void writeUint32(uint8_t * point, uint32_t offset, uint32_t value)
  {
    std::memcpy(point + offset, &value, sizeof(value));
  }

  void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
               const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg)
  {
    if (!model_.initialized()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Camera model not initialized yet.");
      return;
    }

    std::lock_guard<std::mutex> lock(denoise_mutex_);

    // Active-stereo sensors (RealSense, Gemini) need the depth image cleaned up
    // before it is reprojected; ToF sensors pass straight through.
    // The denoised depth is consumed straight from the filter in metres: no 16UC1
    // re-encode, and no 1 mm quantization of the result.
    bool denoised_ok = false;
    if (denoiser_.enabled()) {
      denoiser_.setIntrinsics(model_.fx(), model_.fy(), model_.cx(), model_.cy());
      denoised_ok = denoiser_.process(depth_msg);
    }

    PointCloud::SharedPtr cloud_msg;
    if (denoised_ok) {
      const cv::Mat & depth = denoiser_.depthMeters();
      cloud_msg = makeCloud(depth_msg->header, depth.cols, depth.rows);
      convert<float>(depth.ptr<float>(0), depth.step1(), rgb_msg, cloud_msg);
    } else {
      cloud_msg = buildCloud(depth_msg, rgb_msg);
    }
    if (!cloud_msg) {
      return;
    }

    publishPointCloud(cloud_msg);
  }

  // Organized, unfilled XYZRGB cloud: 16 bytes per point (x, y, z, rgb as FLOAT32, no
  // padding), half the size of the padded 32-byte layout, which matters most when the
  // cloud is sent to another process.
  static PointCloud::SharedPtr makeCloud(
    const std_msgs::msg::Header & header, uint32_t w, uint32_t h)
  {
    PointCloud::SharedPtr cloud_msg(new PointCloud);
    cloud_msg->header = header;
    cloud_msg->is_dense = false;
    cloud_msg->is_bigendian = false;

    sensor_msgs::PointCloud2Modifier mod(*cloud_msg);
    mod.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
    mod.resize(w * h);

    cloud_msg->width = w;
    cloud_msg->height = h;
    cloud_msg->row_step = cloud_msg->point_step * cloud_msg->width;
    return cloud_msg;
  }

  // Reprojects `depth_msg` into an organized XYZRGB cloud. Returns nullptr for an
  // unsupported depth encoding.
  PointCloud::SharedPtr buildCloud(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg)
  {
    auto cloud_msg = makeCloud(depth_msg->header, depth_msg->width, depth_msg->height);

    // convert depends on depth type
    if (depth_msg->encoding == enc::TYPE_16UC1) {
      convert<uint16_t>(
        reinterpret_cast<const uint16_t *>(depth_msg->data.data()),
        depth_msg->step / sizeof(uint16_t), rgb_msg, cloud_msg);
    } else if (depth_msg->encoding == enc::TYPE_32FC1) {
      convert<float>(
        reinterpret_cast<const float *>(depth_msg->data.data()),
        depth_msg->step / sizeof(float), rgb_msg, cloud_msg);
    } else {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Unsupported depth encoding: %s", depth_msg->encoding.c_str());
      return nullptr;
    }
    return cloud_msg;
  }

  void startSubscribing()
  {
    if (!subscribed_) {
      RCLCPP_INFO(this->get_logger(), "Start subscribing RGB(%s) and Depth(%s)",
                  rgb_transport_.c_str(), depth_transport_.c_str());

      try {
        sub_rgb_.subscribe(
          this, topic_rgb_, rgb_transport_, image_qos_, image_sub_options_);
        sub_depth_.subscribe(
          this, topic_depth_, depth_transport_, image_qos_, image_sub_options_);
      } catch (const std::exception & e) {
        sub_rgb_.unsubscribe();
        sub_depth_.unsubscribe();
        RCLCPP_ERROR(
          this->get_logger(),
          "Failed to subscribe image_transport topics. rgb=%s (%s), depth=%s (%s): %s",
          topic_rgb_.c_str(), rgb_transport_.c_str(),
          topic_depth_.c_str(), depth_transport_.c_str(), e.what());
        return;
      }
  
      if (exact_sync_) {
        exact_sync_->connectInput(sub_depth_, sub_rgb_);
      } else if (sync_) {
        sync_->connectInput(sub_depth_, sub_rgb_);
      }
      subscribed_ = true;
    }
  }
  
  void stopSubscribing()
  {
    if (subscribed_) {
      RCLCPP_INFO(this->get_logger(), "Stop subscribing RGB and Depth");
      sub_rgb_.unsubscribe();
      sub_depth_.unsubscribe();
      subscribed_ = false;
      std::lock_guard<std::mutex> lock(denoise_mutex_);
      denoiser_.reset();
    }
  }

  // Rows are independent, so they are split across OpenCV's thread pool.
  template<typename T>
  void convert(const T * depth_data, size_t depth_row_step,
               const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg,
               const PointCloud::SharedPtr & cloud_msg)
  {
    const uint32_t width = cloud_msg->width;
    const uint32_t height = cloud_msg->height;
    updateLookupTables(width, height);

    const float bad_point = std::numeric_limits<float>::quiet_NaN();
    const uint8_t * rgb_data = &rgb_msg->data[0];
    const size_t rgb_row_step = rgb_msg->step;

    const uint32_t x_offset = fieldOffset(*cloud_msg, "x");
    const uint32_t y_offset = fieldOffset(*cloud_msg, "y");
    const uint32_t z_offset = fieldOffset(*cloud_msg, "z");
    const uint32_t rgb_offset = fieldOffset(*cloud_msg, "rgb");
    const uint32_t point_step = cloud_msg->point_step;
    uint8_t * cloud_data = cloud_msg->data.data();
    const size_t cloud_row_step = cloud_msg->row_step;

    cv::parallel_for_(
      cv::Range(0, static_cast<int>(height)), [&](const cv::Range & range) {
        for (int v = range.start; v < range.end; ++v) {
          const T * depth_row = depth_data + static_cast<size_t>(v) * depth_row_step;
          const uint8_t * rgb_row = rgb_data + static_cast<size_t>(v) * rgb_row_step;
          uint8_t * cloud_row = cloud_data + static_cast<size_t>(v) * cloud_row_step;
          const float y_scale = y_lut_[v];

          for (uint32_t u = 0; u < width; ++u) {
            uint8_t * point = cloud_row + static_cast<size_t>(u) * point_step;
            const T depth = depth_row[u];

            if (!hma_pcl_reconst2::DepthTraits<T>::valid(depth)) {
              writeFloat(point, x_offset, bad_point);
              writeFloat(point, y_offset, bad_point);
              writeFloat(point, z_offset, bad_point);
            } else {
              const float z = hma_pcl_reconst2::DepthTraits<T>::toMeters(depth);
              writeFloat(point, x_offset, x_lut_[u] * z);
              writeFloat(point, y_offset, y_scale * z);
              writeFloat(point, z_offset, z);
            }

            const uint8_t * pixel = rgb_row + static_cast<size_t>(u) * 3;
            const uint32_t rgb =
              (static_cast<uint32_t>(pixel[0]) << 16) |
              (static_cast<uint32_t>(pixel[1]) << 8) |
              static_cast<uint32_t>(pixel[2]);
            writeUint32(point, rgb_offset, rgb);
          }
        }
      });
  }
};

} // namespace hma_pcl_reconst2
RCLCPP_COMPONENTS_REGISTER_NODE(hma_pcl_reconst2::PointCloudXyzrgb)
