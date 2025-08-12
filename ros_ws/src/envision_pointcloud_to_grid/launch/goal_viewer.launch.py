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
                {'goal_x':  -0.7},
                {'goal_y': 3.3},
                {'goal_yaw': 90.0},
                {'goal_radius': 1},
                {'pose_topic': '/orb_slam3/camera_pose'},
                {'goal_frame': 'world'},
                {'map_path': '/home/shye0930/Desktop/fyp/maps/hesl/occu_map'},
                {'visualization_mode': 'matplotlib'},  # Options: 'matplotlib (working)', 'rviz2', 'none'
            ]
        ),
    ])