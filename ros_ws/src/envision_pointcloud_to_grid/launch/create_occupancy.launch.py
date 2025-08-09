from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='envision_pointcloud_to_grid',
            executable='pointcloud_to_occupancy_node',
            name='pointcloud_to_occupancy_node',
            output='screen',
            parameters=[
                {'pointcloud_topic': '/orb_slam3/all_points'},
                {'map_topic': '/grid_map'},
                {'map_frame': 'world'},
                {'grid_resolution': 0.1},# meters per cell
                {'grid_width': 300}, # cells
                {'grid_height': 300},# cells
                {'height_min': 0.2},
                {'height_max': 100.0},
                {'min_points_per_cell': 5},  # Minimum points to mark cell as occupied
                {'map_save_path': '~/Desktop/fyp/maps/rosbag/stereo/occu_map'},
            ]
        ),
    ])
