from setuptools import setup
from glob import glob
import os


package_name = 'envision_pointcloud_to_grid'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shye0930',
    maintainer_email='gg@gg.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pointcloud_to_occupancy_node = envision_pointcloud_to_grid.pointcloud_to_occupancy_node:main',
            'goal_publisher = envision_pointcloud_to_grid.goal_publisher:main',
            'navigation_node= envision_pointcloud_to_grid.navigation_node:main',
        ],
    },
)