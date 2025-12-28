from setuptools import setup
import os
from glob import glob

package_name = 'wbk_yolo'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Install config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Install message files
        (os.path.join('share', package_name, 'msg'), glob('wbk_yolo/msg/*.msg')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'sensor_msgs',
        'std_msgs',
        'geometry_msgs',
        'cv_bridge',
        'image_transport',
        'numpy',
        'opencv-python',
        'ultralytics',
        'torch',
    ],
    zip_safe=True,
    maintainer='Wolverbot Kickers Team',
    maintainer_email='team@wolverbotkickers.com',
    description='Wolverbot Kickers YOLO-based vision stack for RoboCup Humanoid League',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_node = wbk_yolo.yolo_node:main',
            'field_mask_node = wbk_yolo.field_mask_node:main',
            'distance_node = wbk_yolo.distance_node:main',
        ],
    },
)
