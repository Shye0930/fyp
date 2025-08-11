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
        default_value='/home/shye0930/Desktop/fyp/config/stereo/settings/KITTI00-02.yaml',
        description='Path to the settings file')

    sequence_file_arg = DeclareLaunchArgument(
        'sequence_file',
        default_value='/home/shye0930/Desktop/fyp/datasets/kitti/sequences/00/',
        description='Path to the sequence_file file'
    )

    # Get launch argument values
    vocab_file = LaunchConfiguration('vocab_file')
    settings_file = LaunchConfiguration('settings_file')
    sequence_file = LaunchConfiguration('sequence_file')

    # Define the ORB_SLAM3 node
    orb_slam3_node = Node(
        package='orbslam3',
        executable='kitti-stereo',
        name='orb_slam3',
        output='screen',
        arguments=[
            vocab_file,
            settings_file,
            sequence_file
        ],
        parameters=[
        ]
    )


    return LaunchDescription([
        vocab_file_arg,
        settings_file_arg,
        sequence_file_arg,
        orb_slam3_node,
    ])
