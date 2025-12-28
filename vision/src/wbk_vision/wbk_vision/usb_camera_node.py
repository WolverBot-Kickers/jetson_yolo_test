#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo


class USBCameraNode(Node):
    def __init__(self):
        super().__init__("usb_camera_node")

        self.declare_parameter("device_id", 0)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30)
        self.declare_parameter("frame_id", "camera")

        self.device_id = int(self.get_parameter("device_id").value)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.fps = int(self.get_parameter("fps").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        self.bridge = CvBridge()
        self.cap = None

        self.image_pub = self.create_publisher(Image, "/camera/image_raw", 10)
        self.info_pub = self.create_publisher(CameraInfo, "/camera/camera_info", 10)

        self._open_camera()

        period = 1.0 / self.fps if self.fps > 0 else 0.033
        self.timer = self.create_timer(period, self._capture_and_publish)

        self.get_logger().info(
            f"USB camera node started on /dev/video{self.device_id} "
            f"({self.width}x{self.height} @ {self.fps}fps)"
        )

    def _open_camera(self):
        self.cap = cv2.VideoCapture(self.device_id, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open /dev/video{self.device_id}")
            return

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

    def _capture_and_publish(self):
        if self.cap is None or not self.cap.isOpened():
            self._open_camera()
            if self.cap is None or not self.cap.isOpened():
                self.get_logger().error("Camera not available")
                return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to read frame from camera")
            return

        stamp = self.get_clock().now().to_msg()

        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = self.frame_id
        self.image_pub.publish(image_msg)

        info_msg = self._make_camera_info(stamp)
        self.info_pub.publish(info_msg)

    def _make_camera_info(self, stamp):
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id
        info.width = self.width
        info.height = self.height

        # Placeholder focal length (typical for 640x480 webcam ~525px)
        # Replace with real calibration values after running camera_calibration
        fx = 525.0 * (self.width / 640.0)
        fy = 525.0 * (self.height / 480.0)
        cx = self.width / 2.0
        cy = self.height / 2.0

        info.k = [fx, 0.0, cx,
                  0.0, fy, cy,
                  0.0, 0.0, 1.0]

        info.p = [fx, 0.0, cx, 0.0,
                  0.0, fy, cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]

        info.r = [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]

        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.distortion_model = "plumb_bob"
        return info


def main(args=None):
    rclpy.init(args=args)
    try:
        node = USBCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()



