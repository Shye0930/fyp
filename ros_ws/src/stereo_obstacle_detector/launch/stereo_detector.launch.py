import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription


def generate_launch_description():

    # Node for stereo obstacle detector
    return LaunchDescription([ 
        Node(
        package='stereo_obstacle_detector',
        executable='stereo_detector_node',
        name='stereo_obstacle_detector_node',
        output='screen',
        parameters=[
            # Default SGBM parameters (optional, can be omitted if defaults in code are fine)
            {'min_disparity': 0},
            {'num_disparities': 64},# was 64 (must be factor of 16)
            {'block_size': 5}, 
            {'disp12_max_diff': 1}, # was 1 
            {'uniqueness_ratio': 10}, # was 10,13
            {'speckle_window_size': 100},
            {'speckle_range': 1},


            # Backup
            # {'min_disparity': 0},
            # {'num_disparities': 32},# was 64 (must be factor of 16)
            # {'block_size': 4}, 
            # {'disp12_max_diff': 4}, # was 1 
            # {'uniqueness_ratio': 16}, # was 10,13
            # {'speckle_window_size': 16384},
            # {'speckle_range': 1024},

            
            # Obstacle detection threshold
            {'obstacle_distance_threshold_m': 1.5},
            
            # --- Your new parameters for focal length and baseline ---
            {'focal_length': 485.9461},
            {'baseline': 0.078453409},
            ]
        )
    ])