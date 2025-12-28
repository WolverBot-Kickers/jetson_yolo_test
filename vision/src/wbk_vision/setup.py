from setuptools import setup

package_name = 'wbk_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/usb_camera.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wolverbot Kickers Team',
    maintainer_email='team@wolverbotkickers.com',
    description='USB webcam publisher for ROS2 Humble (image + camera_info).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'usb_camera_node = wbk_vision.usb_camera_node:main',
        ],
    },
)



