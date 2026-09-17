#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "hma_pcl_reconst2/pcl_transport_decompressor.hpp"

namespace
{

using PointCloud = sensor_msgs::msg::PointCloud2;

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

  auto qos = rclcpp::SensorDataQoS();
  auto pub = node->create_publisher<PointCloud>(output_topic, qos);
  auto stats = std::make_shared<Stats>();
  stats->last_log_time = node->now();

  std::unique_ptr<hma_pcl_reconst2::PointCloudTransportDecompressor> decompressor;
  try {
    decompressor = std::make_unique<hma_pcl_reconst2::PointCloudTransportDecompressor>(
      node, requested_topic, requested_transport, qos,
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
      });
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      node->get_logger(),
      "Failed to subscribe %s with point_cloud_transport=%s: %s",
      requested_topic.c_str(), requested_transport.c_str(), e.what());
    rclcpp::shutdown();
    return 1;
  }

  const auto & input = decompressor->input();
  RCLCPP_INFO(
    node->get_logger(),
    "Subscribing %s via point_cloud_transport=%s. Decompressed output: %s",
    input.base_topic.c_str(), input.transport.c_str(), republish ? output_topic.c_str() : "off");

  rclcpp::spin(node);
  decompressor->shutdown();
  rclcpp::shutdown();
  return 0;
}
