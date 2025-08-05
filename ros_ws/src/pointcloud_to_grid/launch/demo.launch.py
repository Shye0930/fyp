from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():
    pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/orb_slam3/all_points',
        description='The point cloud topic to subscribe to.'
    )

    return LaunchDescription([
        pointcloud_topic_arg,
        Node(
            package='pointcloud_to_grid',
            executable='pointcloud_to_grid_node',
            output='screen',
            parameters=[
                {'cloud_in_topic': LaunchConfiguration("pointcloud_topic")},
                {'position_x': -5.0},
                {'position_z': 0.0}, # <--- NEW: position_y changed to position_z
                {'verbose1': False},
                {'verbose2': False},
                {'cell_size': 0.1},
                {'length_x': 10.0},
                {'length_z': 10.0}, # <--- NEW: length_y changed to length_z
                
                # These topics will now contain the XZ map
                {'maph_topic_name': 'xz_map_grid'},
                {'maph_gridmap_topic_name': 'xz_map_gridmap'},
            ]
        )
    ])

# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.substitutions import LaunchConfiguration
# from launch.actions import DeclareLaunchArgument
# import os

# def generate_launch_description():
#     # Define a launch argument for the point cloud topic, with a sensible default
#     pointcloud_topic_arg = DeclareLaunchArgument(
#         'pointcloud_topic',
#         default_value='/orb_slam3/all_points',
#         description='The point cloud topic to subscribe to.'
#     )

#     return LaunchDescription([
#         pointcloud_topic_arg,
#         Node(
#             package='pointcloud_to_grid',
#             executable='pointcloud_to_grid_node',
#             output='screen',
#             parameters=[
#                 {'cloud_in_topic': LaunchConfiguration("pointcloud_topic")},
#                 {'position_x': -5.0},
#                 {'position_z': 0.0}, # <--- UPDATED: position_y changed to position_z
#                 {'verbose1': False},
#                 {'verbose2': False},
#                 {'cell_size': 0.5},
#                 {'length_x': 40.0},
#                 {'length_z': 60.0}, # <--- UPDATED: length_y changed to length_z
                
#                 # We no longer generate an intensity grid, so these are commented out
#                 # {'mapi_topic_name': 'intensity_grid'},
#                 # {'mapi_gridmap_topic_name': 'intensity_gridmap'},
                
#                 # These parameters are now used for the XZ map
#                 {'maph_topic_name': 'gridmap'},
#                 {'maph_gridmap_topic_name': 'height_gridmap'},
#             ]
#         )
#     ])

# from launch import LaunchDescription
# from launch_ros.actions import Node

# from launch import LaunchDescription
# from launch.substitutions import LaunchConfiguration
# from launch.actions import DeclareLaunchArgument
# import os

# def generate_launch_description():

#     return LaunchDescription([
#         DeclareLaunchArgument("topic", description="a pointcloud topic to process", default_value="nonground"),
#         Node(
#             package='pointcloud_to_grid',
#             executable='pointcloud_to_grid_node',
#             output='screen',
#             parameters=[
#                 {'cloud_in_topic': LaunchConfiguration("topic")},
#                 {'position_x': -5.0},
#                 {'position_y': 0.0},
#                 {'verbose1': False},
#                 {'verbose2': False},
#                 {'cell_size': 0.5},
#                 {'length_x': 40.0},
#                 {'length_y': 60.0},
#                 #{'frame_out': 'os1_sensor'},
#                 # OccupancyGrid topics
#                 {'mapi_topic_name': 'intensity_grid'},
#                 {'maph_topic_name': 'height_grid'},
#                 # GridMap topics
#                 {'mapi_gridmap_topic_name': 'intensity_gridmap'},
#                 {'maph_gridmap_topic_name': 'height_gridmap'},
#             ]
#         )

#     ])

