from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    navigation_node = Node(
            package='envision_pointcloud_to_grid',
            executable='navigation_node',
            name='navigation_node',
            output='screen',
            parameters=[
                {'start_x': 0.5},
                {'start_y': 0.3},
                {'is_camera_pose_available': True},
                {'goal_x':  -5.0},
                {'goal_y': 15.3},
                {'goal_yaw': 90.0},
                {'goal_radius': 1},
                {'pose_topic': '/orb_slam3/camera_pose'},
                {'goal_frame': 'world'},
                {'grid_resolution': 0.1},  # meters per cell
                {'grid_width': 500},       # cells
                {'grid_height': 500},      # cells
                {'map_path': '/home/shye0930/Desktop/fyp/maps/hesl/occu_map'},
                {'path_save_path': '/home/shye0930/Desktop/fyp/maps/hesl/path.json'},
            ]
    )

    return LaunchDescription([
        navigation_node
    ])