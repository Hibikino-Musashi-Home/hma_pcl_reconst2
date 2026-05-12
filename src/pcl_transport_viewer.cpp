#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <point_cloud_transport/point_cloud_transport.hpp>
#include <point_cloud_transport/transport_hints.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace
{

using PointCloud = sensor_msgs::msg::PointCloud2;

struct InputTopic
{
  std::string base_topic;
  std::string transport;
};

bool stripSuffix(const std::string & value, const std::string & suffix, std::string & stripped)
{
  if (value.size() <= suffix.size()) {
    return false;
  }
  if (value.compare(value.size() - suffix.size(), suffix.size(), suffix) != 0) {
    return false;
  }
  stripped = value.substr(0, value.size() - suffix.size());
  return !stripped.empty();
}

InputTopic resolveInputTopic(const std::string & requested_topic, const std::string & transport)
{
  std::string base_topic;
  if (stripSuffix(requested_topic, "/" + transport, base_topic)) {
    return {base_topic, transport};
  }

  for (const auto & suffix : {std::string("zstd"), std::string("zlib"), std::string("draco"),
      std::string("raw")})
  {
    if (stripSuffix(requested_topic, "/" + suffix, base_topic)) {
      return {base_topic, suffix};
    }
  }

  return {requested_topic, transport};
}

struct Stats
{
  rclcpp::Time last_log_time;
  uint64_t messages = 0;
  uint64_t bytes = 0;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("pcl_transport_viewer");

  node->declare_parameter<std::string>(
    "input_topic", "/hma_pcl_reconst/depth_registered/points/zstd");
  node->declare_parameter<std::string>(
    "output_topic", "/hma_pcl_reconst/depth_registered/points/zstd_decompressed");
  node->declare_parameter<std::string>("transport", "zstd");
  node->declare_parameter<bool>("republish", true);
  node->declare_parameter<double>("log_interval_sec", 1.0);

  const auto requested_topic = node->get_parameter("input_topic").as_string();
  const auto requested_transport = node->get_parameter("transport").as_string();
  const auto output_topic = node->get_parameter("output_topic").as_string();
  const auto republish = node->get_parameter("republish").as_bool();
  const auto log_interval_sec = node->get_parameter("log_interval_sec").as_double();
  const auto input = resolveInputTopic(requested_topic, requested_transport);

  auto qos = rclcpp::SensorDataQoS();
  auto pub = node->create_publisher<PointCloud>(output_topic, qos);
  auto stats = std::make_shared<Stats>();
  stats->last_log_time = node->now();

  point_cloud_transport::PointCloudTransport pct(node);
  point_cloud_transport::TransportHints hints(input.transport);
  point_cloud_transport::Subscriber sub;
  const std::shared_ptr<void> tracked_object;

  try {
    sub = pct.subscribe(
      input.base_topic, qos.get_rmw_qos_profile(),
      [node, pub, stats, republish, output_topic, log_interval_sec](
        const PointCloud::ConstSharedPtr & msg)
      {
        if (republish && pub->get_subscription_count() > 0) {
          pub->publish(*msg);
        }

        ++stats->messages;
        stats->bytes += msg->data.size();

        const auto now = node->now();
        const auto elapsed = (now - stats->last_log_time).seconds();
        if (elapsed < log_interval_sec) {
          return;
        }

        const auto hz = static_cast<double>(stats->messages) / elapsed;
        const auto mb_per_msg =
          stats->messages == 0 ? 0.0 :
          static_cast<double>(stats->bytes) / static_cast<double>(stats->messages) / 1000000.0;
        const auto points = static_cast<uint64_t>(msg->width) * static_cast<uint64_t>(msg->height);

        RCLCPP_INFO(
          node->get_logger(),
          "decoded %ux%u (%llu points), %.2f Hz, %.2f MB/msg raw, frame_id=%s, republish=%s",
          msg->width, msg->height, static_cast<unsigned long long>(points), hz, mb_per_msg,
          msg->header.frame_id.c_str(),
          republish ? output_topic.c_str() : "off");

        stats->messages = 0;
        stats->bytes = 0;
        stats->last_log_time = now;
      },
      tracked_object, &hints);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      node->get_logger(),
      "Failed to subscribe %s with point_cloud_transport=%s: %s",
      input.base_topic.c_str(), input.transport.c_str(), e.what());
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Subscribing %s via point_cloud_transport=%s. Decompressed output: %s",
    input.base_topic.c_str(), input.transport.c_str(), republish ? output_topic.c_str() : "off");

  rclcpp::spin(node);
  sub.shutdown();
  rclcpp::shutdown();
  return 0;
}
