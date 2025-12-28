#!/usr/bin/env python3
"""
Distance Estimation Module for Wolverbot Kickers Vision Stack.

Estimates distance to detected objects using:
1. Known object size (e.g., RoboCup ball is ~22cm diameter)
2. Object size in pixels (from bounding box)
3. Camera calibration (focal length)

This is the standard approach for RoboCup robots with monocular cameras.
"""

import numpy as np
from typing import Tuple, Optional
import math
from numpy import ndarray


class DistanceEstimator:
    """
    Estimates distance to objects using monocular vision.
    
    Uses the pinhole camera model:
    distance = (real_size * focal_length) / pixel_size
    
    Where:
    - real_size: Known physical size of object (meters)
    - focal_length: Camera focal length in pixels (from calibration)
    - pixel_size: Size of object in image (pixels)
    """
    
    # Known object sizes (RoboCup standards in meters)
    BALL_DIAMETER = 0.22  # RoboCup ball: 21.5-22.5cm diameter
    GOALPOST_WIDTH = 0.10  # Approximate goalpost width (meters)
    ROBOT_HEIGHT = 0.50   # Approximate humanoid robot height (meters)
    
    def __init__(self, focal_length: float = 525.0, 
                 image_width: int = 640, 
                 image_height: int = 480,
                 homography_matrix: Optional[ndarray] = None):
        """
        Initialize distance estimator.
        
        Args:
            focal_length: Camera focal length in pixels (fx from camera calibration)
            image_width: Image width in pixels
            image_height: Image height in pixels
        """
        self.focal_length = focal_length
        self.image_width = image_width
        self.image_height = image_height
        self.homography_matrix = homography_matrix
        
    def estimate_ball_distance(self, bbox_xyxy: Tuple[float, float, float, float],
                               use_diameter: bool = True) -> Optional[float]:
        """
        Estimate distance to ball using bounding box.
        
        Args:
            bbox_xyxy: Bounding box [x1, y1, x2, y2] in pixels
            use_diameter: If True, use average of width/height. If False, use larger dimension.
            
        Returns:
            Distance in meters, or None if calculation fails
        """
        x1, y1, x2, y2 = bbox_xyxy
        
        # Calculate ball size in pixels
        width_px = abs(x2 - x1)
        height_px = abs(y2 - y1)
        
        if width_px < 1 or height_px < 1:
            return None  # Invalid bounding box
        
        # Use average size or larger dimension
        if use_diameter:
            # Average of width and height (better for circular objects)
            pixel_size = (width_px + height_px) / 2.0
        else:
            # Use larger dimension (more conservative)
            pixel_size = max(width_px, height_px)
        
        # Calculate distance using pinhole camera model
        # distance = (real_size * focal_length) / pixel_size
        distance = (self.BALL_DIAMETER * self.focal_length) / pixel_size
        
        return distance
    
    def estimate_goalpost_distance(self, bbox_xyxy: Tuple[float, float, float, float]) -> Optional[float]:
        """
        Estimate distance to goalpost using bounding box width.
        
        Args:
            bbox_xyxy: Bounding box [x1, y1, x2, y2] in pixels
            
        Returns:
            Distance in meters, or None if calculation fails
        """
        x1, y1, x2, y2 = bbox_xyxy
        
        width_px = abs(x2 - x1)
        if width_px < 1:
            return None
        
        # Use width (goalposts are vertical, width is more reliable)
        distance = (self.GOALPOST_WIDTH * self.focal_length) / width_px
        
        return distance
    
    def estimate_robot_distance(self, bbox_xyxy: Tuple[float, float, float, float]) -> Optional[float]:
        """
        Estimate distance to robot using bounding box height.
        
        Args:
            bbox_xyxy: Bounding box [x1, y1, x2, y2] in pixels
            
        Returns:
            Distance in meters, or None if calculation fails
        """
        x1, y1, x2, y2 = bbox_xyxy
        
        height_px = abs(y2 - y1)
        if height_px < 1:
            return None
        
        # Use height (robots are typically taller than wide)
        distance = (self.ROBOT_HEIGHT * self.focal_length) / height_px
        
        return distance
    
    def estimate_robot_distance_ground_plane(
    self,
    bbox_xyxy: Tuple[float, float, float, float]
    ) -> Optional[float]:
    
        if not hasattr(self, "homography_matrix") or self.homography_matrix is None:
            return None

        x1, y1, x2, y2 = bbox_xyxy

        if x2 <= x1 or y2 <= y1:
            return None

        H = np.asarray(self.homography_matrix, dtype=float)
        if H.shape != (3, 3):
            return None

        u = 0.5 * (x1 + x2)
        v = y2

        p_img = np.array([u, v, 1.0], dtype=float)
        p_field = H @ p_img

        W = p_field[2]
        if abs(W) < 1e-6:
            return None

        X = p_field[0] / W
        Y = p_field[1] / W

        distance = math.hypot(X, Y)
        return distance


    def pixel_to_angle(self, pixel_x: float, pixel_y: float) -> Tuple[float, float]:
        """
        Convert pixel coordinates to horizontal/vertical angles.
        
        Args:
            pixel_x: X coordinate in pixels (0 = left edge)
            pixel_y: Y coordinate in pixels (0 = top edge)
            
        Returns:
            (horizontal_angle, vertical_angle) in radians
        """
        # Calculate principal point (center of image)
        cx = self.image_width / 2.0
        cy = self.image_height / 2.0
        
        # Offset from center
        dx = pixel_x - cx
        dy = pixel_y - cy
        
        # Convert to angles using focal length
        horizontal_angle = math.atan2(dx, self.focal_length)
        vertical_angle = math.atan2(dy, self.focal_length)
        
        return (horizontal_angle, vertical_angle)
    
    def calculate_3d_position(self, bbox_xyxy: Tuple[float, float, float, float],
                             distance: float) -> Tuple[float, float, float]:
        """
        Calculate 3D position (x, y, z) in camera frame.
        
        Args:
            bbox_xyxy: Bounding box [x1, y1, x2, y2] in pixels
            distance: Estimated distance in meters
            
        Returns:
            (x, y, z) position in camera frame (meters)
            x: right (positive), y: down (positive), z: forward (positive)
        """
        x1, y1, x2, y2 = bbox_xyxy
        
        # Center of bounding box
        center_x = (x1 + x2) / 2.0
        center_y = (y1 + y2) / 2.0
        
        # Calculate angles
        horizontal_angle, vertical_angle = self.pixel_to_angle(center_x, center_y)
        
        # Convert to 3D position in camera frame
        # Camera frame: x=right, y=down, z=forward
        x = distance * math.sin(horizontal_angle)
        y = distance * math.sin(vertical_angle)
        z = distance * math.cos(horizontal_angle) * math.cos(vertical_angle)
        
        return (x, y, z)
    
    def update_camera_params(self, focal_length: float, 
                           image_width: Optional[int] = None, 
                           image_height: Optional[int] = None):
        """
        Update camera parameters (useful if camera changes or recalibrated).
        
        Args:
            focal_length: New focal length in pixels
            image_width: New image width (optional)
            image_height: New image height (optional)
        """
        self.focal_length = focal_length
        if image_width is not None:
            self.image_width = image_width
        if image_height is not None:
            self.image_height = image_height


def load_camera_params_from_yaml(yaml_path: str) -> Tuple[float, int, int]:
    """
    Load camera parameters from camera_info.yaml.
    
    Args:
        yaml_path: Path to camera_info.yaml file
        
    Returns:
        (focal_length, image_width, image_height)
    """
    try:
        import yaml
        
        with open(yaml_path, 'r') as f:
            cam_info = yaml.safe_load(f)
        
        # Extract focal length (fx, assume fx ≈ fy)
        if 'camera_matrix' in cam_info:
            fx = cam_info['camera_matrix']['data'][0]  # fx is first element
        else:
            fx = 525.0  # Default fallback
        
        # Extract image dimensions
        image_width = cam_info.get('image_width', 640)
        image_height = cam_info.get('image_height', 480)
        
        return (fx, image_width, image_height)
    
    except Exception as e:
        print(f"Warning: Failed to load camera params from {yaml_path}: {e}")
        print("Using default values")
        return (525.0, 640, 480)


if __name__ == '__main__':
    # Test distance estimation
    print("Testing Distance Estimator")
    print("=" * 40)
    
    # Initialize with default camera params
    estimator = DistanceEstimator(focal_length=525.0, image_width=640, image_height=480)
    
    # Test case: Ball at different sizes (simulating different distances)
    test_cases = [
        # (bbox, expected_rough_distance)
        ([200, 150, 300, 250], 2.0),   # Large ball (close)
        ([250, 200, 300, 240], 5.0),   # Medium ball (mid)
        ([310, 235, 330, 245], 10.0),   # Small ball (far)
    ]
    
    print("\nBall Distance Tests:")
    for i, (bbox, expected_dist) in enumerate(test_cases):
        bbox_tuple: Tuple[float, float, float, float] = (float(bbox[0]), float(bbox[1]), float(bbox[2]), float(bbox[3]))
        distance = estimator.estimate_ball_distance(bbox_tuple)
        error = abs(distance - expected_dist) if distance else None
        print(f"  Test {i+1}: bbox={bbox}")
        print(f"    Distance: {distance:.2f}m" if distance else "    Distance: None")
        if error:
            print(f"    Error: {error:.2f}m")
        print()

