from setuptools import setup
import os # Add this import
from glob import glob # Add this import

package_name = 'stereo_image_compressor'

setup(
    name='stereo_image_compressor',
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
    ],
    install_requires=['setuptools','numpy>=1.24.0'],
    zip_safe=True,
    maintainer='Shye0930',
    maintainer_email='test@test.com',
    description='Stereo image compressor and decompressor',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'compressor = stereo_image_compressor.compressor_node:main',
            'decompressor = stereo_image_compressor.decompressor_node:main',
        ],
    },
)