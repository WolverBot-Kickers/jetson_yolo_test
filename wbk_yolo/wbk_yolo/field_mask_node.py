#!/usr/bin/env python3
"""
Field Mask Node for Wolverbot Kickers Vision Stack.

This node creates HSV-based green field masks to reduce false positives
and provide field segmentation for localization.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import cv2
import numpy as np
from typing import Tuple

# ROS2 message types
from sensor_msgs.msg import Image, CompressedImage
import cv_bridge


class FieldMaskNode(Node):
    """
    ROS2 node for HSV-based green field masking.
    
    Publishes compressed field masks to reduce false positive detections
    and provide field segmentation for the localization team.
    """
    
    def __init__(self):
        super().__init__('field_mask_node')
        
        # Declare parameters
        self.declare_parameter('img_topic', '/camera/image_raw')
        self.declare_parameter('frame_id', 'camera_optical_frame')
        
        # HSV threshold parameters for green field
        self.declare_parameter('h_min', 35)    # Green hue minimum
        self.declare_parameter('h_max', 85)    # Green hue maximum
        self.declare_parameter('s_min', 50)    # Saturation minimum
        self.declare_parameter('s_max', 255)   # Saturation maximum
        self.declare_parameter('v_min', 50)    # Value minimum
        self.declare_parameter('v_max', 255)   # Value maximum
        
        # Morphological operations
        self.declare_parameter('morph_kernel_size', 5)
        self.declare_parameter('erosion_iterations', 1)
        self.declare_parameter('dilation_iterations', 2)
        
        # Get parameters
        self.img_topic = self.get_parameter('img_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # HSV thresholds
        self.h_min = self.get_parameter('h_min').value
        self.h_max = self.get_parameter('h_max').value
        self.s_min = self.get_parameter('s_min').value
        self.s_max = self.get_parameter('s_max').value
        self.v_min = self.get_parameter('v_min').value
        self.v_max = self.get_parameter('v_max').value
        
        # Morphological parameters
        self.morph_kernel_size = self.get_parameter('morph_kernel_size').value
        self.erosion_iterations = self.get_parameter('erosion_iterations').value
        self.dilation_iterations = self.get_parameter('dilation_iterations').value
        
        # Initialize CV bridge
        self.bridge = cv_bridge.CvBridge()
        
        # Setup QoS profile for image topics
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscriber for camera images
        self.image_sub = self.create_subscription(
            Image,
            self.img_topic,
            self.image_callback,
            qos_profile
        )
        
        # Publisher for compressed field mask
        self.mask_pub = self.create_publisher(
            CompressedImage, 
            '/vision/field_mask/compressed', 
            10
        )
        
        # Create morphological kernel
        self.kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, 
            (self.morph_kernel_size, self.morph_kernel_size)
        )
        
        self.get_logger().info("Field mask node initialized")
        self.get_logger().info(f"HSV thresholds: H({self.h_min}-{self.h_max}), "
                             f"S({self.s_min}-{self.s_max}), V({self.v_min}-{self.v_max})")
        self.get_logger().info(f"Morphological kernel size: {self.morph_kernel_size}")
    
    def image_callback(self, msg: Image):
        """Process incoming camera image and create field mask."""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Create field mask
            field_mask = self.create_field_mask(cv_image)
            
            # Publish compressed mask
            self.publish_field_mask(field_mask, msg.header)
            
        except Exception as e:
            self.get_logger().error(f"Error processing image for field mask: {e}")
    
    def create_field_mask(self, cv_image: np.ndarray) -> np.ndarray:
        """
        Create HSV-based green field mask.
        
        Args:
            cv_image: Input BGR image
            
        Returns:
            Binary mask where 255 = field, 0 = non-field
        """
        # Convert BGR to HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Define HSV range for green field
        lower_green = np.array([self.h_min, self.s_min, self.v_min])
        upper_green = np.array([self.h_max, self.s_max, self.v_max])
        
        # Create mask
        mask = cv2.inRange(hsv_image, lower_green, upper_green)
        
        # Apply morphological operations to clean up the mask
        mask = self.apply_morphological_ops(mask)
        
        return mask
    
    def apply_morphological_ops(self, mask: np.ndarray) -> np.ndarray:
        """
        Apply morphological operations to clean up the field mask.
        
        Args:
            mask: Binary mask
            
        Returns:
            Cleaned binary mask
        """
        # Erosion to remove noise
        if self.erosion_iterations > 0:
            mask = cv2.erode(mask, self.kernel, iterations=self.erosion_iterations)
        
        # Dilation to fill holes
        if self.dilation_iterations > 0:
            mask = cv2.dilate(mask, self.kernel, iterations=self.dilation_iterations)
        
        return mask
    
    def publish_field_mask(self, mask: np.ndarray, header):
        """Publish field mask as compressed image."""
        try:
            # Encode mask as JPEG for compression
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
            result, encimg = cv2.imencode('.jpg', mask, encode_param)
            
            if result:
                # Create compressed image message
                compressed_msg = CompressedImage()
                compressed_msg.header = header
                compressed_msg.header.frame_id = self.frame_id
                compressed_msg.format = 'jpeg'
                compressed_msg.data = encimg.tobytes()
                
                # Publish
                self.mask_pub.publish(compressed_msg)
            else:
                self.get_logger().warn("Failed to encode field mask")
                
        except Exception as e:
            self.get_logger().error(f"Error publishing field mask: {e}")
    
    def update_hsv_thresholds(self, h_min: int, h_max: int, s_min: int, s_max: int, 
                            v_min: int, v_max: int):
        """Update HSV thresholds at runtime (for tuning)."""
        self.h_min = h_min
        self.h_max = h_max
        self.s_min = s_min
        self.s_max = s_max
        self.v_min = v_min
        self.v_max = v_max
        
        self.get_logger().info(f"Updated HSV thresholds: H({h_min}-{h_max}), "
                             f"S({s_min}-{s_max}), V({v_min}-{v_max})")


def main(args=None):
    """Main entry point for the field mask node."""
    rclpy.init(args=args)
    
    try:
        node = FieldMaskNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in field mask node: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
