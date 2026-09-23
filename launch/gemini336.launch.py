"""Orbbec Gemini 336 (330 series) -> denoised XYZRGB point cloud.

Starts the OrbbecSDK_ROS2 driver with depth registered to color and pcl_reconst with
the denoiser (plane snap on) tuned for this camera, and optionally RViz.

    ros2 launch hma_pcl_reconst2 gemini336.launch.py rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# Launch arguments forwarded as-is to gemini_330_series.launch.py, with our defaults.
CAMERA_ARGS = {
    'camera_name': ('camera', 'Namespace of the camera topics.'),
    'color_width': ('1280', 'Color width. 848 for 848x480.'),
    'color_height': ('720', 'Color height.'),
    'color_fps': ('30', 'Color fps.'),
    'depth_width': ('1280', 'Depth width. 848 for 848x480.'),
    'depth_height': ('720', 'Depth height.'),
    'depth_fps': ('30', 'Depth fps.'),
    # On-camera post filters are off by default so only our denoiser is under test.
    'enable_noise_removal_filter': ('false', 'On-camera noise removal filter.'),
    'enable_spatial_filter': ('false', 'On-camera spatial filter.'),
    'enable_temporal_filter': ('false', 'On-camera temporal filter.'),
    'enable_hole_filling_filter': ('false', 'On-camera hole filling filter.'),
}

# Launch arguments forwarded as-is to pcl_reconst.launch.py, with our defaults. The
# denoise tuning (bilateral, plane snap, ...) comes from pcl_reconst.launch.py's own
# defaults; its denoise_* arguments can still be given on this command line.
RECONST_ARGS = {
    'denoise': ('true', 'Enable the depth denoiser (the Gemini is active stereo).'),
    'output_topic': ('/hma_pcl_reconst/depth_registered/points', 'Output cloud topic.'),
}


def generate_launch_description():
    orbbec_launch = os.path.join(
        get_package_share_directory('orbbec_camera'), 'launch', 'gemini_330_series.launch.py')
    reconst_launch = os.path.join(
        get_package_share_directory('hma_pcl_reconst2'), 'launch', 'pcl_reconst.launch.py')
    rviz_config = os.path.join(
        get_package_share_directory('hma_pcl_reconst2'), 'rviz', 'gemini336.rviz')

    camera_name = LaunchConfiguration('camera_name')

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(orbbec_launch),
        launch_arguments={
            **{name: LaunchConfiguration(name) for name in CAMERA_ARGS},
            # hma_pcl_reconst2 needs depth aligned to the color image.
            'depth_registration': 'true',
            'enable_point_cloud': 'false',
            'enable_colored_point_cloud': 'false',
        }.items(),
    )

    reconst = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(reconst_launch),
        launch_arguments={
            **{name: LaunchConfiguration(name) for name in RECONST_ARGS},
            'use_sim_time': 'false',
            'use_compressed': 'false',
            'topic_rgb': ['/', camera_name, '/color/image_raw'],
            'topic_depth': ['/', camera_name, '/depth/image_raw'],
            'topic_camera_info': ['/', camera_name, '/color/camera_info'],
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='gemini336_rviz',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    args = [
        DeclareLaunchArgument(name, default_value=default, description=desc)
        for name, (default, desc) in {**CAMERA_ARGS, **RECONST_ARGS}.items()
    ]
    args.append(DeclareLaunchArgument('rviz', default_value='false', description='Start RViz.'))

    return LaunchDescription(args + [camera, reconst, rviz])
