#!/usr/bin/env python3
"""
Distance Estimation Node for Wolverbot Kickers Vision Stack.

Subscribes to ball detections and publishes distance estimates.
Can be integrated into yolo_node or run as separate node.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
from pathlib import Path

# ROS2 message types
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header, Float32
from sensor_msgs.msg import CameraInfo

# Import detection messages
try:
    from vision_msgs.msg import Detection2DArray, Detection2D
    VISION_MSGS_AVAILABLE = True
except ImportError:
    from wbk_yolo.msg import Detection2DArray, Detection2D
    VISION_MSGS_AVAILABLE = False

# Import distance estimator
from wbk_yolo.distance_estimator import DistanceEstimator, load_camera_params_from_yaml


class DistanceEstimationNode(Node):
    """
    Node that estimates distance to detected objects.
    
    Subscribes to detection topics and publishes:
    - Distance estimates (/vision/ball_distance)
    - 3D positions (/vision/ball_position)
    """
    
    def __init__(self):
        super().__init__('distance_estimation_node')
        
        # Declare parameters
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('ball_detections_topic', '/vision/ball_dets')
        self.declare_parameter('camera_info_yaml', 
                              'package://wbk_yolo/config/camera_info.yaml')
        self.declare_parameter('frame_id', 'camera_optical_frame')
        
        # Get parameters
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.ball_detections_topic = self.get_parameter('ball_detections_topic').value
        self.camera_info_yaml = self.get_parameter('camera_info_yaml').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Initialize distance estimator with default values
        # Will be updated from camera_info when available
        self.distance_estimator = DistanceEstimator(
            focal_length=525.0,  # Default, will be updated
            image_width=640,
            image_height=480
        )
        
        # Try to load camera params from YAML file
        try:
            # Resolve package path
            if self.camera_info_yaml.startswith('package://'):
                package_path = self.camera_info_yaml.replace(
                    'package://wbk_yolo/config/camera_info.yaml',
                    ''
                )
                # For now, use relative path
                yaml_path = Path(__file__).parent.parent / 'config' / 'camera_info.yaml'
            else:
                yaml_path = self.camera_info_yaml
            
            if yaml_path.exists():
                fx, width, height = load_camera_params_from_yaml(str(yaml_path))
                self.distance_estimator.update_camera_params(fx, width, height)
                self.get_logger().info(f"Loaded camera params: fx={fx}, {width}x{height}")
            else:
                self.get_logger().warn(f"Camera info YAML not found: {yaml_path}")
                self.get_logger().warn("Using default camera parameters")
        except Exception as e:
            self.get_logger().warn(f"Failed to load camera params: {e}")
        
        # Setup QoS profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscriber for camera info (to get real-time calibration)
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile
        )
        
        # Subscriber for ball detections
        self.ball_detections_sub = self.create_subscription(
            Detection2DArray,
            self.ball_detections_topic,
            self.ball_detections_callback,
            qos_profile
        )
        
        # Publishers
        self.ball_distance_pub = self.create_publisher(
            Float32,
            '/vision/ball_distance',
            10
        )
        
        self.ball_position_pub = self.create_publisher(
            PointStamped,
            '/vision/ball_position',
            10
        )
        
        self.get_logger().info("Distance estimation node initialized")
    
    def camera_info_callback(self, msg: CameraInfo):
        """Update camera parameters when camera_info is received."""
        if len(msg.k) >= 4:
            # Camera matrix K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            fx = msg.k[0]  # Focal length in x direction
            fy = msg.k[4]  # Focal length in y direction (usually same as fx)
            
            # Use average if fx and fy differ slightly
            focal_length = (fx + fy) / 2.0 if abs(fx - fy) < 10 else fx
            
            image_width = msg.width
            image_height = msg.height
            
            self.distance_estimator.update_camera_params(
                focal_length, image_width, image_height
            )
            
            self.get_logger().debug(
                f"Updated camera params: fx={focal_length:.1f}, "
                f"{image_width}x{image_height}"
            )
    
    def ball_detections_callback(self, msg: Detection2DArray):
        """Process ball detections and estimate distances."""
        if len(msg.detections) == 0:
            # No detections - publish empty/zero
            distance_msg = Float32()
            distance_msg.data = 0.0
            self.ball_distance_pub.publish(distance_msg)
            return
        
        # Use the highest confidence detection (or closest if multiple)
        best_detection = max(msg.detections, key=lambda d: d.score)
        
        # Extract bounding box
        bbox = best_detection.bbox_xyxy
        x1, y1, x2, y2 = bbox[0], bbox[1], bbox[2], bbox[3]
        
        # Estimate distance
        distance = self.distance_estimator.estimate_ball_distance((x1, y1, x2, y2))
        
        if distance is not None and distance > 0:
            # Publish distance
            distance_msg = Float32()
            distance_msg.data = float(distance)
            self.ball_distance_pub.publish(distance_msg)
            
            # Calculate 3D position
            x, y, z = self.distance_estimator.calculate_3d_position((x1, y1, x2, y2), distance)
            
            # Publish 3D position
            position_msg = PointStamped()
            position_msg.header = msg.header
            position_msg.header.frame_id = self.frame_id
            position_msg.point = Point(x=float(x), y=float(y), z=float(z))
            self.ball_position_pub.publish(position_msg)
            
            self.get_logger().debug(
                f"Ball: distance={distance:.2f}m, "
                f"position=({x:.2f}, {y:.2f}, {z:.2f})"
            )
        else:
            # Invalid distance - publish zero
            distance_msg = Float32()
            distance_msg.data = 0.0
            self.ball_distance_pub.publish(distance_msg)


def main(args=None):
    """Main entry point for distance estimation node."""
    rclpy.init(args=args)
    
    try:
        node = DistanceEstimationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in distance estimation node: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

