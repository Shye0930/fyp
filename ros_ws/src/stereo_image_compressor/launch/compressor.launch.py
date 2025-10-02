from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('left_topic', default_value='/camera/camera/infra1/image_rect_raw', description='Left camera topic'),
        DeclareLaunchArgument('right_topic', default_value='/camera/camera/infra2/image_rect_raw', description='Right camera topic'),
        DeclareLaunchArgument('compressed_topic', default_value='/camera/stereo/compressed', description='Compressed output topic'),
        DeclareLaunchArgument('image_width', default_value='640', description='Individual image width'),
        DeclareLaunchArgument('image_height', default_value='480', description='Individual image height'),
        DeclareLaunchArgument('jpeg_quality', default_value='100', description='JPEG compression quality'),

        Node(
            package='stereo_image_compressor',
            executable='rgbd_compressor',
            name='stereo_image_compressor',
            output='screen',
            parameters=[{
                'left_topic': LaunchConfiguration('left_topic'),
                'right_topic': LaunchConfiguration('right_topic'),
                'compressed_topic': LaunchConfiguration('compressed_topic'),
                'image_width': LaunchConfiguration('image_width'),
                'image_height': LaunchConfiguration('image_height'),
                'jpeg_quality': LaunchConfiguration('jpeg_quality'),
            }]
        )
    ])