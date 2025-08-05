from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='envision_pointcloud_to_grid',
            executable='goal_publisher',
            name='goal_publisher',
            output='screen',
            parameters=[
                {'goal_x': 5.0},
                {'goal_y': 3.0},
                {'goal_yaw': 90.0},
                {'goal_radius': 0.5},
                {'pose_topic': '/orb_slam3/camera_pose'},
                {'goal_frame': 'map'},
            ]
        ),
    ])
