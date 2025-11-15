from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    navigation_node = Node(
            package='envision_pointcloud_to_grid',
            executable='navigation_node',
            name='navigation_node',
            output='screen',
            parameters=[
                {'start_x': 0.5}, #0.5 (hesl rgbd) #1.0 (hesl stereo) 
                {'start_y': 0.3}, #0.3  (hesl rgbd)#0.3 (hesl stereo) 
                {'is_camera_pose_available': True},
                {'current_direction': 1},
                {'goal_x':  -15.5},#-5.5 (hesl stereo) # -2.5 (hesl rgbd) #-0.3 (kitchen)
                {'goal_y': 0.3}, #10.3 (hesl stereo) #15.3 (hesl rgbd) #4.0 (kitchen)
                {'goal_yaw': 90.0},
                {'goal_radius': 1.5},
                {'pose_topic': '/orb_slam3/camera_pose'},
                {'goal_frame': 'world'},
                {'grid_resolution': 0.1},  # meters per cell
                {'grid_width': 500},       # cells
                {'grid_height': 500},      # cells
                {'map_path': '/home/shye0930/Desktop/fyp/maps/hpl_stereo/occu_map'},
                {'path_save_path': '/home/shye0930/Desktop/fyp/maps/hpl_stereo/path.json'},
            ]
    )

    return LaunchDescription([
        navigation_node
    ])
