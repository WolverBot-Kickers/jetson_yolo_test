#!/usr/bin/env python3
"""
Launch file for Wolverbot Kickers Vision Stack.

Launches the YOLO detection node and field mask node with configurable parameters.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for the vision stack."""
    
    # Package directory
    pkg_share = FindPackageShare(package='wbk_yolo').find('wbk_yolo')
    
    # Declare launch arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='yolo11n.engine',
        description='Path to YOLO model file (.engine, .pt, or .onnx)'
    )
    
    imgsz_arg = DeclareLaunchArgument(
        'imgsz',
        default_value='416',
        description='Input image size for YOLO inference'
    )
    
    conf_arg = DeclareLaunchArgument(
        'conf',
        default_value='0.35',
        description='Confidence threshold for detections'
    )
    
    nms_iou_arg = DeclareLaunchArgument(
        'nms_iou',
        default_value='0.5',
        description='NMS IoU threshold'
    )
    
    max_det_arg = DeclareLaunchArgument(
        'max_det',
        default_value='50',
        description='Maximum number of detections per image'
    )
    
    img_topic_arg = DeclareLaunchArgument(
        'img_topic',
        default_value='/camera/image_raw',
        description='Input image topic'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='camera_optical_frame',
        description='Frame ID for published messages'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('wbk_yolo'),
            'config',
            'params.yaml'
        ]),
        description='Path to parameter configuration file'
    )
    
    camera_info_arg = DeclareLaunchArgument(
        'camera_info_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('wbk_yolo'),
            'config',
            'camera_info.yaml'
        ]),
        description='Path to camera info file'
    )
    
    launch_camera_arg = DeclareLaunchArgument(
        'launch_camera',
        default_value='false',
        description='Launch USB camera node for Logitech C930'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='true',
        description='Enable debug image publishing'
    )
    
    # YOLO detection node
    yolo_node = Node(
        package='wbk_yolo',
        executable='yolo_node',
        name='yolo_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'model_path': LaunchConfiguration('model_path'),
                'imgsz': LaunchConfiguration('imgsz'),
                'conf': LaunchConfiguration('conf'),
                'nms_iou': LaunchConfiguration('nms_iou'),
                'max_det': LaunchConfiguration('max_det'),
                'img_topic': LaunchConfiguration('img_topic'),
                'frame_id': LaunchConfiguration('frame_id'),
                'classes': [0, 1, 2, 3, 4, 5],  # All 6 classes
            }
        ],
        remappings=[
            ('/vision/debug_image', '/vision/debug_image'),
        ]
    )
    
    # Field mask node
    field_mask_node = Node(
        package='wbk_yolo',
        executable='field_mask_node',
        name='field_mask_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'img_topic': LaunchConfiguration('img_topic'),
                'frame_id': LaunchConfiguration('frame_id'),
                # HSV thresholds for green field
                'h_min': 35,
                'h_max': 85,
                's_min': 50,
                's_max': 255,
                'v_min': 50,
                'v_max': 255,
                # Morphological operations
                'morph_kernel_size': 5,
                'erosion_iterations': 1,
                'dilation_iterations': 2,
            }
        ]
    )
    
    # USB camera node (optional)
    usb_camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_camera',
        output='screen',
        parameters=[
            {
                'video_device': '/dev/video0',
                'image_width': 640,
                'image_height': 480,
                'framerate': 30.0,
                'pixel_format': 'yuyv',
                'camera_frame_id': LaunchConfiguration('frame_id'),
                'camera_info_url': LaunchConfiguration('camera_info_file'),
            }
        ],
        condition=IfCondition(LaunchConfiguration('launch_camera'))
    )
    
    # Static transform from camera_optical_frame to head
    static_tf_camera_to_head = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_head_tf',
        arguments=['0.0', '0.0', '0.1', '0.0', '0.0', '0.0', 'head', 'camera_optical_frame']
    )
    
    # Static transform from head to base_link
    static_tf_head_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='head_to_base_tf',
        arguments=['0.0', '0.0', '0.2', '0.0', '0.0', '0.0', 'base_link', 'head']
    )
    
    # Log launch information
    log_info = LogInfo(
        msg=[
            'Launching Wolverbot Kickers Vision Stack\n',
            'Model: ', LaunchConfiguration('model_path'), '\n',
            'Image size: ', LaunchConfiguration('imgsz'), '\n',
            'Confidence: ', LaunchConfiguration('conf'), '\n',
            'Image topic: ', LaunchConfiguration('img_topic'), '\n',
            'Frame ID: ', LaunchConfiguration('frame_id'), '\n'
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        model_path_arg,
        imgsz_arg,
        conf_arg,
        nms_iou_arg,
        max_det_arg,
        img_topic_arg,
        frame_id_arg,
        config_file_arg,
        camera_info_arg,
        launch_camera_arg,
        debug_arg,
        
        # Log info
        log_info,
        
        # Static transforms
        static_tf_camera_to_head,
        static_tf_head_to_base,
        
        # Vision nodes
        yolo_node,
        field_mask_node,
        
        # Optional camera node
        usb_camera_node,
    ])
