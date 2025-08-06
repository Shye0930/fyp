import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():

    # Get the package share directory
    package_share_directory = get_package_share_directory('orbslam3')

    rviz_config_path = '/home/shye0930/Desktop/fyp/config/stereo/rviz/esp32_orb_slam3_stereo.rviz'

    # Declare launch arguments
    vocab_file_arg = DeclareLaunchArgument(
        'vocab_file',
        default_value='/home/shye0930/Desktop/fyp/ORB_SLAM_3_COMMUNITY/Vocabulary/ORBvoc.bin',
        description='Path to the vocabulary file')

    settings_file_arg = DeclareLaunchArgument(
        'settings_file',
        default_value='/home/shye0930/Desktop/fyp/config/stereo/settings/Esp32s.yaml',
        description='Path to the settings file')

    # Get launch argument values
    vocab_file = LaunchConfiguration('vocab_file')
    settings_file = LaunchConfiguration('settings_file')

    # Define the ORB_SLAM3 node
    orb_slam3_node = Node(
        package='orbslam3',
        executable='stereo',
        name='orb_slam3',
        output='screen',
        arguments=[
            vocab_file,
            settings_file
        ],
        parameters=[
            {'left_topic': '/stereo/left/rectified_images'},
            {'right_topic': '/stereo/right/rectified_images'}
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        vocab_file_arg,
        settings_file_arg,
        orb_slam3_node,
        rviz_node,
    ])

