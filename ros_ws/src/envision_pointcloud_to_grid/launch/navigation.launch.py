from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # world_transform_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='world',
    #     output='screen',
    #     arguments=[
    #         '0', '0', '0',                 # x, y, z translation
    #         '1', '0', '0.382683', '0.923879',  # quaternion (x, y, z, w) for 45-degree rotation
    #         '/world',                          # parent frame_id
    #         '/base_link'                       # child frame_id
    #     ],
    #     parameters=[
    #         {'frequency': 100.0}
    #     ]
    # )

    navigation_node = Node(
            package='envision_pointcloud_to_grid',
            executable='navigation_node',
            name='navigation_node',
            output='screen',
            parameters=[
                {'goal_x': 3.0},
                {'goal_y': 0.0},
                {'goal_yaw': 90.0},
                {'goal_radius': 0.5},
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
        #world_transform_node,
        navigation_node
    ])