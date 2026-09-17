from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration

# Default input topics per mode. Explicit launch arguments always take precedence.
SIM_TOPICS = {
    'topic_rgb': '/head_rgbd_sensor/rgb/image_rect_raw',
    'topic_depth': '/head_rgbd_sensor/depth_registered/image_rect_raw',
    'topic_camera_info': '/head_rgbd_sensor/rgb/camera_info',
}
REAL_TOPICS = {
    'topic_rgb': '/head_rgbd_sensor/rgb/image_rect_color',
    'topic_depth': '/head_rgbd_sensor/depth_registered/image_rect_raw',
    'topic_camera_info': '/head_rgbd_sensor/rgb/camera_info',
}


def launch_setup(context):
    use_sim_time = IfCondition(LaunchConfiguration('use_sim_time')).evaluate(context)
    default_topics = SIM_TOPICS if use_sim_time else REAL_TOPICS

    def topic_or_default(name):
        value = LaunchConfiguration(name).perform(context)
        return value if value != '' else default_topics[name]

    queue_size = LaunchConfiguration('queue_size')
    exact_sync = LaunchConfiguration('exact_sync')
    output_topic = LaunchConfiguration('output_topic')
    compressed_transport = LaunchConfiguration('compressed_transport')
    use_compressed = LaunchConfiguration('use_compressed')
    use_pointcloud_compressed = LaunchConfiguration(
        'use_pointcloud_compressed')

    container = ComposableNodeContainer(
        name='cloud_reconst_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='hma_pcl_reconst2',
                plugin='hma_pcl_reconst2::PointCloudXyzrgb',
                name='pcl_reconst',
                parameters=[
                    {
                        'use_sim_time': use_sim_time,
                        'queue_size': ParameterValue(queue_size, value_type=int),
                        'exact_sync': ParameterValue(exact_sync, value_type=bool),
                        'topic_rgb': topic_or_default('topic_rgb'),
                        'topic_depth': topic_or_default('topic_depth'),
                        'topic_camera_info': topic_or_default('topic_camera_info'),
                        'output_topic': output_topic,
                        'compressed_transport': compressed_transport,
                        'use_compressed': ParameterValue(use_compressed, value_type=bool),
                        'use_pointcloud_compressed': ParameterValue(
                            use_pointcloud_compressed, value_type=bool
                        ),
                    }
                ],
            )
        ],
        output='screen',
    )
    return [container]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=EnvironmentVariable('USE_SIM_TIME', default_value='false'),
            description='Use the simulation clock and sim input topic defaults. '
                        'Defaults to the USE_SIM_TIME env var.',
        ),
        DeclareLaunchArgument('queue_size', default_value='5'),
        DeclareLaunchArgument('exact_sync', default_value='false'),
        DeclareLaunchArgument(
            'topic_rgb',
            default_value='',
            description='RGB image base topic. Empty selects the default for use_sim_time.',
        ),
        DeclareLaunchArgument(
            'topic_depth',
            default_value='',
            description='Depth image base topic. Empty selects the default for use_sim_time.',
        ),
        DeclareLaunchArgument(
            'topic_camera_info',
            default_value='',
            description='Camera info topic. Empty selects the default for use_sim_time.',
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/hma_pcl_reconst/depth_registered/points',
        ),
        DeclareLaunchArgument('compressed_transport', default_value='zstd'),
        DeclareLaunchArgument(
            'use_compressed',
            default_value='false',
            description='Subscribe compressed images.',
        ),
        DeclareLaunchArgument('use_pointcloud_compressed',
                              default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
