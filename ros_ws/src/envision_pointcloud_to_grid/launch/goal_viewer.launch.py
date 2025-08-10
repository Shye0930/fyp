from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='envision_pointcloud_to_grid',
            executable='goal_viewer',
            name='goal_viewer',
            output='screen',
            parameters=[
                {'grid_resolution': 0.1},# meters per cell
                {'grid_width': 100}, # cells
                {'grid_height': 100},# cells
                {'goal_x': 4.0},
                {'goal_y': 1.0},
                {'goal_yaw': 90.0},
                {'goal_radius': 1},
                {'pose_topic': '/orb_slam3/camera_pose'},
                {'goal_frame': 'world'},
                {'map_path': '~/Desktop/fyp/maps/rosbag/rgbd/occu_map'},
                {'visualization_mode': 'matplotlib'},  # Options: 'matplotlib (working)', 'rviz2', 'none'
            ]
        ),
    ])