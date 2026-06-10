#ifndef HMA_PCL_RECONST2__PCL_TRANSPORT_DECOMPRESSOR_HPP_
#define HMA_PCL_RECONST2__PCL_TRANSPORT_DECOMPRESSOR_HPP_

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include <point_cloud_transport/point_cloud_transport.hpp>
#include <point_cloud_transport/transport_hints.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace hma_pcl_reconst2
{

class PointCloudTransportDecompressor
{
public:
  using PointCloud = sensor_msgs::msg::PointCloud2;
  using Callback = std::function<void(const PointCloud::ConstSharedPtr &)>;

  struct InputTopic
  {
    std::string base_topic;
    std::string transport;
  };

  PointCloudTransportDecompressor(
    const rclcpp::Node::SharedPtr & node,
    const std::string & input_topic,
    const std::string & transport,
    const rclcpp::QoS & qos,
    Callback callback)
  : node_(node),
    input_(resolveInputTopic(input_topic, transport)),
    hints_(input_.transport),
    callback_(std::move(callback))
  {
    point_cloud_transport::PointCloudTransport pct(node_);
    const std::shared_ptr<void> tracked_object;
    subscriber_ = pct.subscribe(
      input_.base_topic, qos.get_rmw_qos_profile(), callback_, tracked_object, &hints_);
  }

  ~PointCloudTransportDecompressor()
  {
    shutdown();
  }

  PointCloudTransportDecompressor(const PointCloudTransportDecompressor &) = delete;
  PointCloudTransportDecompressor & operator=(const PointCloudTransportDecompressor &) = delete;

  PointCloudTransportDecompressor(PointCloudTransportDecompressor &&) = default;
  PointCloudTransportDecompressor & operator=(PointCloudTransportDecompressor &&) = default;

  const InputTopic & input() const
  {
    return input_;
  }

  void shutdown()
  {
    subscriber_.shutdown();
  }

  static InputTopic resolveInputTopic(
    const std::string & requested_topic, const std::string & transport)
  {
    std::string base_topic;
    if (stripSuffix(requested_topic, "/" + transport, base_topic)) {
      return {base_topic, transport};
    }

    for (const auto & suffix : {
      std::string("zstd"), std::string("zlib"), std::string("draco"), std::string("raw")})
    {
      if (stripSuffix(requested_topic, "/" + suffix, base_topic)) {
        return {base_topic, suffix};
      }
    }

    return {requested_topic, transport};
  }

private:
  static bool stripSuffix(
    const std::string & value, const std::string & suffix, std::string & stripped)
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

  rclcpp::Node::SharedPtr node_;
  InputTopic input_;
  point_cloud_transport::TransportHints hints_;
  Callback callback_;
  point_cloud_transport::Subscriber subscriber_;
};

}  // namespace hma_pcl_reconst2

#endif  // HMA_PCL_RECONST2__PCL_TRANSPORT_DECOMPRESSOR_HPP_
