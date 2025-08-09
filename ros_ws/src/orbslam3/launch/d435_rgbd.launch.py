import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():

    # Get the package share directory
    package_share_directory = get_package_share_directory('orbslam3')

    rviz_config_path = '/home/shye0930/Desktop/fyp/config/rgbd/rviz/d435_orbslam3_rgbd.rviz'

    # Declare launch arguments
    vocab_file_arg = DeclareLaunchArgument(
        'vocab_file',
        default_value='/home/shye0930/Desktop/fyp/ORB_SLAM_3_COMMUNITY/Vocabulary/ORBvoc.txt',
        description='Path to the vocabulary file')

    settings_file_arg = DeclareLaunchArgument(
        'settings_file',
        default_value='/home/shye0930/Desktop/fyp/config/rgbd/settings/D435-D.yaml',
        description='Path to the settings file')

    # Get launch argument values
    vocab_file = LaunchConfiguration('vocab_file')
    settings_file = LaunchConfiguration('settings_file')

    # Define the ORB_SLAM3 node
    orb_slam3_node = Node(
        package='orbslam3',
        executable='rgbd',
        name='orb_slam3',
        output='screen',
        arguments=[
            vocab_file,
            settings_file
        ],
        parameters=[
            {'rgb_topic': '/camera/camera/color/image_raw'},
            {'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw'}
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    # Define the static transform publisher node (added here)
    world_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_orb_to_world',
        output='screen',
        arguments=[
            '0', '0', '1.0',                 # x, y, z translation
            '-0.7071068', '0', '0', '0.7071068',      # quaternion (x, y, z, w) for 90-degree rotation
            '/world',                          # parent frame_id
            '/map'                       # child frame_id
        ],
        parameters=[
            {'frequency': 100.0}
        ]
    )

    return LaunchDescription([
        world_transform_node,
        vocab_file_arg,
        settings_file_arg,
        orb_slam3_node,
        rviz_node 
    ])


# For Intel realsense d435, run for rgbd
# ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=true enable_infra1:=false enable_infra2:=false align_depth.enable:=true enable_sync:=true rgb_camera.color_profile:=640x480x30 depth_module.depth_profile:=640x480x30

# Turn on emitter
# ros2 param set /camera/camera depth_module.emitter_enabled 1
