from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'rgb_topic',
            default_value='/camera/camera/color/image_raw',
            description='Input topic for raw RGB image.'
        ),
        
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera/camera/aligned_depth_to_color/image_raw',
            description='Input topic for raw depth image (16UC1).'
        ),
        
        # Output Compressed Topics
        DeclareLaunchArgument(
            'rgb_compressed_topic',
            default_value='/camera/color/image_raw/compressed',
            description='Output topic for compressed JPEG RGB image.'
        ),
        
        DeclareLaunchArgument(
            'depth_compressed_topic',
            default_value='/camera/depth/image_rect_raw/compressedDepth',
            description='Output topic for compressed PNG depth image.'
        ),

        # Compression Settings
        DeclareLaunchArgument(
            'jpeg_quality',
            default_value='100',
            description='JPEG compression quality (0-100).'
        ),
        
        DeclareLaunchArgument(
            'png_level',
            default_value='1',
            description='PNG compression level (0-9, 1 is fast/low compression, 9 is slow/high compression).'
        ),

        Node(
            package='stereo_image_compressor',
            executable='rgbd_compressor',
            name='stereo_image_compressor',
            output='screen',
            parameters=[{
                'rgb_topic': LaunchConfiguration('rgb_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'rgb_compressed_topic': LaunchConfiguration('rgb_compressed_topic'),
                'depth_compressed_topic': LaunchConfiguration('depth_compressed_topic'),
                'jpeg_quality': LaunchConfiguration('jpeg_quality'),
                'png_level': LaunchConfiguration('png_level'),
            }]
        )
    ])