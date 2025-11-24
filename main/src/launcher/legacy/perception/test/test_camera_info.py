#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

class CameraInfoNode(Node):
    def __init__(self):
        super().__init__('camera_info_node')

        # Parameters
        self.declare_parameter('camera_device', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)

        self.camera_device = self.get_parameter('camera_device').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value

        # Publisher
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        # Open camera to get actual info
        self.cap = cv2.VideoCapture(self.camera_device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        # Get actual camera properties
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)

        self.get_logger().info(f"Camera opened: {self.camera_device}")
        self.get_logger().info(f"Resolution: {actual_width}x{actual_height}")
        self.get_logger().info(f"FPS: {actual_fps}")

        # Estimate focal length (assuming 60 degree FOV)
        fov_deg = 90.0
        fx = actual_width / (2 * np.tan(np.deg2rad(fov_deg / 2)))
        fy = fx  # Assume square pixels
        cx = actual_width / 2.0
        cy = actual_height / 2.0

        self.get_logger().info(f"Estimated intrinsics: fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}")

        # Store camera info
        self.camera_info = CameraInfo()
        self.camera_info.header.frame_id = "camera"
        self.camera_info.width = actual_width
        self.camera_info.height = actual_height

        # Intrinsic matrix K (3x3 row-major)
        self.camera_info.k = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0
        ]

        # Distortion coefficients (assume none)
        self.camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Rectification matrix (identity)
        self.camera_info.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]

        # Projection matrix P (3x4)
        self.camera_info.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        self.camera_info.distortion_model = "plumb_bob"

        # Timer to publish camera info
        self.timer = self.create_timer(1.0, self.publish_camera_info)

        self.get_logger().info("CameraInfo node started!")

    def publish_camera_info(self):
        self.camera_info.header.stamp = self.get_clock().now().to_msg()
        self.camera_info_pub.publish(self.camera_info)
        self.get_logger().info(f"Published CameraInfo: {self.camera_info.width}x{self.camera_info.height}")

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
