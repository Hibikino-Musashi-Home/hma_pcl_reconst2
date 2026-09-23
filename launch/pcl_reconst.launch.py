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

    # Tuning arguments left at '' are simply not passed, so the node's own default
    # applies. The ones with a value are tuned for active-stereo sensors (a live
    # Gemini 336, floor at 0.8 m, wall at 1.9 m) and only matter with denoise:=true.
    denoise_params = {
        'denoise.enable': ParameterValue(
            LaunchConfiguration('denoise'), value_type=bool),
        'denoise.snap.enable': ParameterValue(
            LaunchConfiguration('denoise_snap'), value_type=bool),
    }
    tuning = {
        'denoise.clip.min_depth': float,
        'denoise.clip.max_depth': float,
        'denoise.speckle.max_size': int,
        'denoise.speckle.diff_quad': float,
        'denoise.bilateral.radius': int,
        'denoise.bilateral.sigma_quad': float,
        'denoise.temporal.alpha': float,
        'denoise.snap.diff_quad': float,
        'denoise.num_threads': int,
    }
    for name, value_type in tuning.items():
        arg = name.replace('.', '_')
        raw = LaunchConfiguration(arg).perform(context)
        if raw != '':
            denoise_params[name] = value_type(raw)

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
                        **denoise_params,
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
        DeclareLaunchArgument(
            'denoise',
            default_value='false',
            description='Denoise the depth image before reprojecting it. Needed for '
                        'active-stereo sensors (RealSense, Orbbec Gemini) whose planes '
                        'are noisy; leave off for ToF sensors such as the Xtion.',
        ),
        DeclareLaunchArgument(
            'denoise_clip_min_depth', default_value='',
            description='[m] Depth below this is discarded. Empty keeps the node default.',
        ),
        DeclareLaunchArgument(
            'denoise_clip_max_depth', default_value='',
            description='[m] Depth above this is discarded. Set it comfortably beyond the '
                        'furthest surface you care about: a surface sitting on the limit '
                        'loses half its pixels to the clip.',
        ),
        DeclareLaunchArgument(
            'denoise_speckle_max_size', default_value='',
            description='[px] Connected components smaller than this are dropped.',
        ),
        DeclareLaunchArgument(
            'denoise_speckle_diff_quad', default_value='',
            description='[1/m] z^2 term of the speckle connectivity tolerance. Raise it if '
                        'distant surfaces are being deleted.',
        ),
        DeclareLaunchArgument(
            'denoise_bilateral_radius', default_value='2',
            description='Bilateral window radius (2 => 5x5). 0 disables the stage.',
        ),
        DeclareLaunchArgument(
            'denoise_bilateral_sigma_quad', default_value='0.004',
            description='[1/m] z^2 term of the bilateral range sigma, i.e. the sigma in '
                        'inverse-depth space. The main knob for flattening planes.',
        ),
        DeclareLaunchArgument(
            'denoise_temporal_alpha', default_value='',
            description='Weight of the new frame in the temporal EMA. 0 disables it.',
        ),
        DeclareLaunchArgument(
            'denoise_snap_diff_quad', default_value='0.008',
            description='[1/m] z^2 term of the plane snap tolerance (~9 mm at 0.8 m). Larger '
                        'flattens planes more but also flattens thicker objects lying on them.',
        ),
        DeclareLaunchArgument(
            'denoise_num_threads', default_value='0',
            description='OpenCV thread count (process-global). 0 leaves it alone.',
        ),
        DeclareLaunchArgument(
            'denoise_snap', default_value='true',
            description='Snap pixels close to a dominant plane onto it (removes the '
                        'low-frequency waviness of stereo depth; flattens anything thinner '
                        'than the snap tolerance lying on a plane).',
        ),
        OpaqueFunction(function=launch_setup),
    ])
