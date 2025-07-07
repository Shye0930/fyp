from setuptools import find_packages, setup
import os # Add this import
from glob import glob # Add this import

package_name = 'stereo_obstacle_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shye',
    maintainer_email='',
    description='Stereo obstacle detector package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stereo_detector_node = stereo_obstacle_detector.stereo_obstacle_detector:main',
        ],
    },
)