from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix # Import get_package_prefix
import os

def generate_launch_description():
    # [INFO] Get the path to the config directory
    package_share_directory = get_package_share_directory('image_masker')

    model_dir = os.path.join(package_share_directory, 'model')
    yolo_model_path = os.path.join(package_share_directory, 'model', 'yolo11n-seg.pt')    

    package_prefix = get_package_prefix('image_masker')

    return LaunchDescription([
          Node(
            package='image_masker',
            executable='image_masker_node', # This should match the entry_point in setup.py
            name='image_masker',
            output='screen',
            parameters=[
                # Pass parameters to the node
                {'yolo_model_path': yolo_model_path},
                {'person_class_id': 0}, # COCO dataset ID for 'person'
                {'publish_masked_images': True}
            ]
        )
    ])