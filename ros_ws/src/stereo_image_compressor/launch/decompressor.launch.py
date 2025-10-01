from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('input_topic', default_value='/camera/stereo/compressed', description='Compressed input topic'),
        DeclareLaunchArgument('left_output_topic', default_value='/camera/stereo/left_decompressed', description='Left decompressed topic'),
        DeclareLaunchArgument('right_output_topic', default_value='/camera/stereo/right_decompressed', description='Right decompressed topic'),
        DeclareLaunchArgument('image_width', default_value='640', description='Individual image width'),
        DeclareLaunchArgument('image_height', default_value='480', description='Individual image height'),

        Node(
            package='stereo_image_compressor',
            executable='decompressor',
            name='stereo_image_compressor',
            output='screen',
            parameters=[{
                'input_topic': LaunchConfiguration('input_topic'),
                'left_output_topic': LaunchConfiguration('left_output_topic'),
                'right_output_topic': LaunchConfiguration('right_output_topic'),
                'image_width': LaunchConfiguration('image_width'),
                'image_height': LaunchConfiguration('image_height'),
            }]
        )
    ])