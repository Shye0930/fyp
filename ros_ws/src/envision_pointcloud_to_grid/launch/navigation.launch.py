from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    navigation_node = Node(
            package='envision_pointcloud_to_grid',
            executable='navigation_node',
            name='navigation_node',
            output='screen',
            parameters=[
                {'start_x': 1.5},
                {'start_y': 0.1},
                {'is_camera_pose_available': False},
                {'goal_x': 4.0},
                {'goal_y': 1.0},
                {'goal_yaw': 90.0},
                {'goal_radius': 1},
                {'pose_topic': '/orb_slam3/camera_pose'},
                {'goal_frame': 'world'},
                {'grid_resolution': 0.1},  # meters per cell
                {'grid_width': 100},       # cells
                {'grid_height': 100},      # cells
                {'map_path': '~/Desktop/fyp/maps/rosbag/rgbd/occu_map'},
                {'path_save_path': '~/Desktop/fyp/maps/rosbag/rgbd/path.json'},
            ]
    )

    return LaunchDescription([
        navigation_node
    ])