from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    input_topic = LaunchConfiguration("input_topic")
    output_topic = LaunchConfiguration("output_topic")
    transport = LaunchConfiguration("transport")
    republish = LaunchConfiguration("republish")
    log_interval_sec = LaunchConfiguration("log_interval_sec")

    viewer = Node(
        package="hma_pcl_reconst2",
        executable="pcl_transport_viewer",
        name="pcl_transport_viewer",
        output="screen",
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "input_topic": input_topic,
            "output_topic": output_topic,
            "transport": transport,
            "republish": ParameterValue(republish, value_type=bool),
            "log_interval_sec": ParameterValue(log_interval_sec, value_type=float),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value=EnvironmentVariable("USE_SIM_TIME", default_value="false"),
            description="Use the simulation clock. Defaults to the USE_SIM_TIME env var.",
        ),
        DeclareLaunchArgument(
            "input_topic",
            default_value="/hma_pcl_reconst/depth_registered/points/zstd",
            description="Compressed point cloud transport topic, or the base topic.",
        ),
        DeclareLaunchArgument(
            "output_topic",
            default_value="/hma_pcl_reconst/depth_registered/points/zstd_decompressed",
            description="Raw PointCloud2 topic to republish decompressed clouds for RViz.",
        ),
        DeclareLaunchArgument(
            "transport",
            default_value="zstd",
            description="point_cloud_transport subscriber transport.",
        ),
        DeclareLaunchArgument(
            "republish",
            default_value="true",
            description="Republish decoded clouds as raw PointCloud2.",
        ),
        DeclareLaunchArgument(
            "log_interval_sec",
            default_value="1.0",
            description="Terminal display interval in seconds.",
        ),
        viewer,
    ])
