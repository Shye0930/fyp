from setuptools import setup
from glob import glob
import os

package_name = 'image_masker'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*launch.py'))),
        ('share/' + package_name + '/model', glob(os.path.join('model', '*.pt'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shye',
    maintainer_email='test@test.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_masker_node = image_masker.image_masker:main', # Add this line
        ],
    },
)
