from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    queue_size = LaunchConfiguration("queue_size")
    exact_sync = LaunchConfiguration("exact_sync")
    topic_rgb = LaunchConfiguration("topic_rgb")
    topic_depth = LaunchConfiguration("topic_depth")
    topic_camera_info = LaunchConfiguration("topic_camera_info")
    rgb_transport = LaunchConfiguration("rgb_transport")
    depth_transport = LaunchConfiguration("depth_transport")
    output_topic = LaunchConfiguration("output_topic")
    compressed_transport = LaunchConfiguration("compressed_transport")
    use_compressed = LaunchConfiguration("use_compressed")

    container = ComposableNodeContainer(
        name="cloud_reconst_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="hma_pcl_reconst2",
                plugin="hma_pcl_reconst2::PointCloudXyzrgb",
                name="pcl_reconst",
                parameters=[
                    {
                        "queue_size": ParameterValue(queue_size, value_type=int),
                        "exact_sync": ParameterValue(exact_sync, value_type=bool),
                        "topic_rgb": topic_rgb,
                        "topic_depth": topic_depth,
                        "topic_camera_info": topic_camera_info,
                        "rgb_transport": rgb_transport,
                        "depth_transport": depth_transport,
                        "output_topic": output_topic,
                        "compressed_transport": compressed_transport,
                        "use_compressed": ParameterValue(
                            use_compressed, value_type=bool
                        ),
                    }
                ],
            )
        ],
        output="screen",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("queue_size", default_value="5"),
            DeclareLaunchArgument("exact_sync", default_value="false"),
            DeclareLaunchArgument(
                "topic_rgb",
                default_value="/head_rgbd_sensor/rgb/image_rect_color",
                description="RGB image base topic. A /compressed suffix is also accepted.",
            ),
            DeclareLaunchArgument(
                "topic_depth",
                default_value="/head_rgbd_sensor/depth_registered/image_rect_raw",
                description="Depth image base topic. A /compressedDepth suffix is also accepted.",
            ),
            DeclareLaunchArgument(
                "topic_camera_info",
                default_value="/head_rgbd_sensor/rgb/camera_info",
            ),
            DeclareLaunchArgument("rgb_transport", default_value="raw"),
            DeclareLaunchArgument("depth_transport", default_value="raw"),
            DeclareLaunchArgument(
                "output_topic",
                default_value="/hma_pcl_reconst/depth_registered/points",
            ),
            DeclareLaunchArgument("compressed_transport", default_value="zstd"),
            DeclareLaunchArgument("use_compressed", default_value="true"),
            container,
        ]
    )
