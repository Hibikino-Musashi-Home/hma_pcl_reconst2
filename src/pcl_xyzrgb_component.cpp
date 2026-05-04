#include <cstdint>
#include <memory>
#include <string>
#include <limits>
#include <cstring>
#include <stdexcept>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>

#include "hma_pcl_reconst2/depth_traits.hpp"

namespace hma_pcl_reconst2
{

namespace enc = sensor_msgs::image_encodings;
using PointCloud = sensor_msgs::msg::PointCloud2;

using namespace std::chrono_literals;

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
    //this->declare_parameter("topic_rgb", "/head_rgbd_sensor/rgb/image_raw");
    //this->declare_parameter("topic_depth", "/head_rgbd_sensor/depth_registered/image_raw");
    this->declare_parameter("use_compressed", false);

    bool use_compressed = this->get_parameter("use_compressed").as_bool();

    topic_rgb_ = this->get_parameter("topic_rgb").as_string();
    topic_depth_ = this->get_parameter("topic_depth").as_string();

    rgb_transport_ = use_compressed ? "compressed" : "raw";
    depth_transport_ = use_compressed ? "compressedDepth" : "raw";

    RCLCPP_INFO(this->get_logger(), "Configured transports: rgb=%s, depth=%s",
      rgb_transport_.c_str(), depth_transport_.c_str());

    int queue_size = this->get_parameter("queue_size").as_int();
    bool exact_sync = this->get_parameter("exact_sync").as_bool();

    //rclcpp::QoS qos(10);
    auto qos = rclcpp::SensorDataQoS();

    pub_point_cloud_ = this->create_publisher<PointCloud>("/hma_pcl_reconst/depth_registered/points", qos);

    std::string rgb_transport = use_compressed ? "compressed" : "raw";
    std::string depth_transport = use_compressed ? "compressedDepth" : "raw";

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

    //camera info
    sub_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/head_rgbd_sensor/rgb/camera_info", 1,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        model_.fromCameraInfo(msg);
      });

    timer_ = this->create_wall_timer(1000ms, [this]() {
    auto subs = pub_point_cloud_->get_subscription_count();
    
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
    });

    RCLCPP_INFO(this->get_logger(), "hma_pcl_reconst2.-> component initialized");
  }

private:

  std::string topic_rgb_, topic_depth_;
  std::string rgb_transport_, depth_transport_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool subscribed_;
  bool use_camera_info_;

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

  image_geometry::PinholeCameraModel model_;

  std::vector<float> x_lut_;
  std::vector<float> y_lut_;
  uint32_t lut_width_ = 0;
  uint32_t lut_height_ = 0;
  double lut_fx_ = 0.0;
  double lut_fy_ = 0.0;
  double lut_cx_ = 0.0;
  double lut_cy_ = 0.0;

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

    PointCloud::SharedPtr cloud_msg(new PointCloud);
    cloud_msg->header = depth_msg->header;
    cloud_msg->is_dense = false;
    cloud_msg->is_bigendian = false;

    const uint32_t w = depth_msg->width;
    const uint32_t h = depth_msg->height;

    sensor_msgs::PointCloud2Modifier mod(*cloud_msg);
    mod.setPointCloud2FieldsByString(2, "xyz", "rgb");
    mod.resize(w * h);

    cloud_msg->width = w;
    cloud_msg->height = h;

    cloud_msg->row_step = cloud_msg->point_step * cloud_msg->width;

    // RCLCPP_INFO(this->get_logger(),
    //   "Cloud initialized: %dx%d point_step=%d row_step=%d",
    //   cloud_msg->width, cloud_msg->height,
    //   cloud_msg->point_step, cloud_msg->row_step);

    // convert depends on depth type
    if (depth_msg->encoding == enc::TYPE_16UC1) {
      convert<uint16_t>(depth_msg, rgb_msg, cloud_msg);
    } else if (depth_msg->encoding == enc::TYPE_32FC1) {
      convert<float>(depth_msg, rgb_msg, cloud_msg);
    } else {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Unsupported depth encoding: %s", depth_msg->encoding.c_str());
      return;
    }

    pub_point_cloud_->publish(*cloud_msg);
  }

  void startSubscribing()
  {
    if (!subscribed_) {
      RCLCPP_INFO(this->get_logger(), "Start subscribing RGB(%s) and Depth(%s)",
                  rgb_transport_.c_str(), depth_transport_.c_str());
  
      sub_rgb_.subscribe(this, topic_rgb_, rgb_transport_, rmw_qos_profile_sensor_data);
      sub_depth_.subscribe(this, topic_depth_, depth_transport_, rmw_qos_profile_sensor_data);
  
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
    }
  }

  template<typename T>
  void convert(const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
               const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg,
               const PointCloud::SharedPtr & cloud_msg)
  {
    const uint32_t width = cloud_msg->width;
    const uint32_t height = cloud_msg->height;
    updateLookupTables(width, height);

    const float bad_point = std::numeric_limits<float>::quiet_NaN();
    const T * depth_data = reinterpret_cast<const T *>(&depth_msg->data[0]);
    const int depth_row_step = depth_msg->step / sizeof(T);
    const uint8_t * rgb_data = &rgb_msg->data[0];
    const int rgb_row_step = rgb_msg->step;

    const uint32_t x_offset = fieldOffset(*cloud_msg, "x");
    const uint32_t y_offset = fieldOffset(*cloud_msg, "y");
    const uint32_t z_offset = fieldOffset(*cloud_msg, "z");
    const uint32_t rgb_offset = fieldOffset(*cloud_msg, "rgb");
    const uint32_t point_step = cloud_msg->point_step;

    for (int v = 0; v < static_cast<int>(height); ++v) {
      const T * depth_row = depth_data + static_cast<size_t>(v) * depth_row_step;
      const uint8_t * rgb_row = rgb_data + static_cast<size_t>(v) * rgb_row_step;
      uint8_t * cloud_row = cloud_msg->data.data() + static_cast<size_t>(v) * cloud_msg->row_step;
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
  }
};

} // namespace hma_pcl_reconst2
RCLCPP_COMPONENTS_REGISTER_NODE(hma_pcl_reconst2::PointCloudXyzrgb)
