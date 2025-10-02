from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'compressed_rgb_topic',
            default_value='/camera/color/image_raw/compressed',
            description='Input topic for compressed JPEG RGB image.'
        ),
        
        DeclareLaunchArgument(
            'compressed_depth_topic',
            default_value='/camera/depth/image_rect_raw/compressedDepth',
            description='Input topic for compressed PNG depth image.'
        ),
        
        # Output Decompressed Topics
        DeclareLaunchArgument(
            'decompressed_rgb_topic',
            default_value='/decompressed_rgb',
            description='Output topic for raw BGR8 RGB image.'
        ),
        
        DeclareLaunchArgument(
            'decompressed_depth_topic',
            default_value='/decompressed_depth',
            description='Output topic for raw 16UC1 depth image.'
        ),

        Node(
            package='stereo_image_compressor',
            executable='rgbd_decompressor',
            name='stereo_image_compressor',
            output='screen',
            parameters=[{
                'compressed_rgb_topic': LaunchConfiguration('compressed_rgb_topic'),
                'decompressed_rgb_topic': LaunchConfiguration('decompressed_rgb_topic'),
                'compressed_depth_topic': LaunchConfiguration('compressed_depth_topic'),
                'decompressed_depth_topic': LaunchConfiguration('decompressed_depth_topic'),
            }]
        )
    ])