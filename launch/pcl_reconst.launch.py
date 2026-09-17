import os

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim = os.getenv('USE_SIM_TIME', 'false')

    if use_sim:
        default_topic_rgb = '/head_rgbd_sensor/rgb/image_rect_raw'
        default_topic_depth = '/head_rgbd_sensor/depth_registered/image_rect_raw'
        default_topic_camera_info = '/head_rgbd_sensor/rgb/camera_info'
        default_use_compressed = 'true'
    else:
        default_topic_rgb = '/head_rgbd_sensor/rgb/image_rect_color'
        default_topic_depth = '/head_rgbd_sensor/depth_registered/image_rect_raw'
        default_topic_camera_info = '/head_rgbd_sensor/rgb/camera_info'
        default_use_compressed = 'false'

    use_sim_time = LaunchConfiguration('use_sim_time')
    queue_size = LaunchConfiguration('queue_size')
    exact_sync = LaunchConfiguration('exact_sync')
    topic_rgb = LaunchConfiguration('topic_rgb')
    topic_depth = LaunchConfiguration('topic_depth')
    topic_camera_info = LaunchConfiguration('topic_camera_info')
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
                        'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                        'queue_size': ParameterValue(queue_size, value_type=int),
                        'exact_sync': ParameterValue(exact_sync, value_type=bool),
                        'topic_rgb': topic_rgb,
                        'topic_depth': topic_depth,
                        'topic_camera_info': topic_camera_info,
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
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true' if use_sim else 'false',
            description='Use the simulation clock. Defaults to the USE_SIM_TIME env var.',
        ),
        DeclareLaunchArgument('queue_size', default_value='5'),
        DeclareLaunchArgument('exact_sync', default_value='false'),
        DeclareLaunchArgument(
            'topic_rgb',
            default_value=default_topic_rgb,
            description='RGB image base topic.',
        ),
        DeclareLaunchArgument(
            'topic_depth',
            default_value=default_topic_depth,
            description='Depth image base topic.',
        ),
        DeclareLaunchArgument(
            'topic_camera_info',
            default_value=default_topic_camera_info,
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/hma_pcl_reconst/depth_registered/points',
        ),
        DeclareLaunchArgument('compressed_transport', default_value='zstd'),
        DeclareLaunchArgument(
            'use_compressed', default_value=default_use_compressed),
        DeclareLaunchArgument('use_pointcloud_compressed',
                              default_value='false'),
        container,
    ])
