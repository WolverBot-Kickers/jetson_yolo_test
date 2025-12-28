#!/usr/bin/env python3
"""
YOLO Detection Node for Wolverbot Kickers Vision Stack.

This node subscribes to camera images, runs YOLO11 inference, and publishes
detections to separate topics for each class (ball, goalpost, robot, landmarks).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import cv2
import numpy as np
from ultralytics import YOLO
import time
from typing import List, Dict, Optional

# ROS2 message types
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import cv_bridge

# Try to import vision_msgs, fallback to custom messages
try:
    from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
    VISION_MSGS_AVAILABLE = True
    print("Using vision_msgs for detection messages")
except ImportError:
    from wbk_yolo.msg import Detection2DArray, Detection2D
    VISION_MSGS_AVAILABLE = False
    print("Using custom messages for detection messages")


class YOLODetectionNode(Node):
    """
    ROS2 node for YOLO-based object detection.
    
    Publishes detections to separate topics for each class to support
    the ILM paper localization requirements.
    """
    
    def __init__(self):
        super().__init__('yolo_node')
        
        # Declare parameters
        self.declare_parameter('model_path', 'yolo11n.pt')
        self.declare_parameter('imgsz', 416)
        self.declare_parameter('conf', 0.35)
        self.declare_parameter('nms_iou', 0.5)
        self.declare_parameter('max_det', 50)
        self.declare_parameter('img_topic', '/camera/image_raw')
        self.declare_parameter('classes', [0, 1, 2, 3, 4, 5])  # All 6 classes
        self.declare_parameter('frame_id', 'camera_optical_frame')
        
        # Get parameters
        self.model_path = self.get_parameter('model_path').value
        self.imgsz = self.get_parameter('imgsz').value
        self.conf = self.get_parameter('conf').value
        self.nms_iou = self.get_parameter('nms_iou').value
        self.max_det = self.get_parameter('max_det').value
        self.img_topic = self.get_parameter('img_topic').value
        self.classes = self.get_parameter('classes').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Class names for ILM paper support
        self.class_names = {
            0: 'ball',
            1: 'goalpost', 
            2: 'robot',
            3: 'L_intersection',
            4: 'T_intersection',
            5: 'X_intersection'
        }
        
        # Initialize YOLO model
        self.get_logger().info(f"Loading YOLO model from: {self.model_path}")
        try:
            self.model = YOLO(self.model_path)
            self.get_logger().info("YOLO model loaded successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            raise
        
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
        
        # Publishers for different detection types
        self.ball_pub = self.create_publisher(Detection2DArray, '/vision/ball_dets', 10)
        self.goal_pub = self.create_publisher(Detection2DArray, '/vision/goal_dets', 10)
        self.robot_pub = self.create_publisher(Detection2DArray, '/vision/robot_dets', 10)
        self.landmarks_pub = self.create_publisher(Detection2DArray, '/vision/landmarks', 10)
        
        # Debug image publisher
        self.debug_pub = self.create_publisher(Image, '/vision/debug_image', 10)
        
        # Performance tracking
        self.frame_count = 0
        self.start_time = time.time()
        self.last_fps_time = time.time()
        
        self.get_logger().info("YOLO detection node initialized")
        self.get_logger().info(f"Model: {self.model_path}")
        self.get_logger().info(f"Image size: {self.imgsz}")
        self.get_logger().info(f"Confidence: {self.conf}")
        self.get_logger().info(f"Classes: {self.classes}")
    
    def image_callback(self, msg: Image):
        """Process incoming camera image and run YOLO inference."""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Run YOLO inference
            results = self.model(
                cv_image,
                imgsz=self.imgsz,
                conf=self.conf,
                iou=self.nms_iou,
                max_det=self.max_det,
                classes=self.classes,
                verbose=False
            )
            
            # Process results and publish detections
            self.process_and_publish_detections(results[0], msg.header, cv_image)
            
            # Log FPS every 30 frames
            self.frame_count += 1
            if self.frame_count % 30 == 0:
                current_time = time.time()
                fps = 30.0 / (current_time - self.last_fps_time)
                self.get_logger().info(f"FPS: {fps:.1f}")
                self.last_fps_time = current_time
                
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
    
    def process_and_publish_detections(self, result, header: Header, cv_image: np.ndarray):
        """
        Process YOLO results and publish to appropriate topics.
        
        Args:
            result: YOLO detection result
            header: ROS message header from input image
            cv_image: Original OpenCV image for debug visualization
        """
        if result.boxes is None or len(result.boxes) == 0:
            # No detections - publish empty arrays
            self.publish_empty_detections(header)
            return
        
        # Group detections by class
        detections_by_class = {0: [], 1: [], 2: [], 3: [], 4: [], 5: []}
        
        for i, box in enumerate(result.boxes):
            # Extract detection info
            class_id = int(box.cls[0])
            confidence = float(box.conf[0])
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            
            # Create detection message
            detection = self.create_detection_msg(
                header, class_id, confidence, x1, y1, x2, y2
            )
            
            # Add to appropriate class group
            if class_id in detections_by_class:
                detections_by_class[class_id].append(detection)
        
        # Publish detections for each class
        self.publish_detection_arrays(header, detections_by_class)
        
        # Publish debug image with drawn boxes
        self.publish_debug_image(header, cv_image, result.boxes)
    
    def create_detection_msg(self, header: Header, class_id: int, confidence: float, 
                           x1: float, y1: float, x2: float, y2: float) -> Detection2D:
        """Create a single detection message."""
        detection = Detection2D()
        detection.header = header
        detection.header.frame_id = self.frame_id
        
        # Bounding box coordinates
        detection.bbox_xyxy = [float(x1), float(y1), float(x2), float(y2)]
        
        # Class information
        detection.class_name = self.class_names.get(class_id, f'unknown_{class_id}')
        detection.class_id = class_id
        detection.score = confidence
        
        # Pixel center
        detection.px_center = [float((x1 + x2) / 2), float((y1 + y2) / 2)]
        
        return detection
    
    def publish_detection_arrays(self, header: Header, detections_by_class: Dict[int, List]):
        """Publish detection arrays to appropriate topics."""
        
        # Ball detections (class 0)
        ball_msg = Detection2DArray()
        ball_msg.header = header
        ball_msg.header.frame_id = self.frame_id
        ball_msg.detections = detections_by_class[0]
        self.ball_pub.publish(ball_msg)
        
        # Goal post detections (class 1)
        goal_msg = Detection2DArray()
        goal_msg.header = header
        goal_msg.header.frame_id = self.frame_id
        goal_msg.detections = detections_by_class[1]
        self.goal_pub.publish(goal_msg)
        
        # Robot detections (class 2)
        robot_msg = Detection2DArray()
        robot_msg.header = header
        robot_msg.header.frame_id = self.frame_id
        robot_msg.detections = detections_by_class[2]
        self.robot_pub.publish(robot_msg)
        
        # Landmark detections (classes 3, 4, 5: L/T/X intersections)
        landmarks_msg = Detection2DArray()
        landmarks_msg.header = header
        landmarks_msg.header.frame_id = self.frame_id
        landmarks_msg.detections = (
            detections_by_class[3] +  # L_intersection
            detections_by_class[4] +  # T_intersection
            detections_by_class[5]    # X_intersection
        )
        self.landmarks_pub.publish(landmarks_msg)
    
    def publish_empty_detections(self, header: Header):
        """Publish empty detection arrays when no objects are detected."""
        empty_msg = Detection2DArray()
        empty_msg.header = header
        empty_msg.header.frame_id = self.frame_id
        empty_msg.detections = []
        
        self.ball_pub.publish(empty_msg)
        self.goal_pub.publish(empty_msg)
        self.robot_pub.publish(empty_msg)
        self.landmarks_pub.publish(empty_msg)
    
    def publish_debug_image(self, header: Header, cv_image: np.ndarray, boxes):
        """Publish debug image with drawn bounding boxes."""
        debug_image = cv_image.copy()
        
        if boxes is not None and len(boxes) > 0:
            for box in boxes:
                # Extract box info
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                class_id = int(box.cls[0])
                confidence = float(box.conf[0])
                
                # Draw bounding box
                color = self.get_class_color(class_id)
                cv2.rectangle(debug_image, (x1, y1), (x2, y2), color, 2)
                
                # Draw label
                label = f"{self.class_names.get(class_id, f'class_{class_id}')}: {confidence:.2f}"
                label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
                cv2.rectangle(debug_image, (x1, y1 - label_size[1] - 10), 
                            (x1 + label_size[0], y1), color, -1)
                cv2.putText(debug_image, label, (x1, y1 - 5), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        # Convert back to ROS image and publish
        try:
            debug_ros_image = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_ros_image.header = header
            debug_ros_image.header.frame_id = self.frame_id
            self.debug_pub.publish(debug_ros_image)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish debug image: {e}")
    
    def get_class_color(self, class_id: int) -> tuple:
        """Get BGR color for class ID."""
        colors = {
            0: (0, 0, 255),      # Red for ball
            1: (0, 255, 0),      # Green for goalpost
            2: (255, 0, 0),      # Blue for robot
            3: (0, 255, 255),    # Yellow for L_intersection
            4: (255, 0, 255),    # Magenta for T_intersection
            5: (255, 255, 0),    # Cyan for X_intersection
        }
        return colors.get(class_id, (128, 128, 128))  # Gray for unknown


def main(args=None):
    """Main entry point for the YOLO detection node."""
    rclpy.init(args=args)
    
    try:
        node = YOLODetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in YOLO detection node: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
