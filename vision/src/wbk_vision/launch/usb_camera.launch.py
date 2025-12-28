#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    device_id_arg = DeclareLaunchArgument(
        "device_id",
        default_value="0",
        description="Video device index (e.g. 0 for /dev/video0)",
    )
    width_arg = DeclareLaunchArgument(
        "width",
        default_value="640",
        description="Image width",
    )
    height_arg = DeclareLaunchArgument(
        "height",
        default_value="480",
        description="Image height",
    )
    fps_arg = DeclareLaunchArgument(
        "fps",
        default_value="30",
        description="Frames per second",
    )
    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="camera",
        description="Frame ID for published images",
    )

    camera_node = Node(
        package="wbk_vision",
        executable="usb_camera_node",
        name="usb_camera_node",
        output="screen",
        parameters=[{
            "device_id": LaunchConfiguration("device_id"),
            "width": LaunchConfiguration("width"),
            "height": LaunchConfiguration("height"),
            "fps": LaunchConfiguration("fps"),
            "frame_id": LaunchConfiguration("frame_id"),
        }],
    )

    return LaunchDescription([
        device_id_arg,
        width_arg,
        height_arg,
        fps_arg,
        frame_id_arg,
        camera_node,
    ])

