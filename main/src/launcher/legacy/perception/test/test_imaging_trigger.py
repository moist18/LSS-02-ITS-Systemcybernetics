#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_services_default
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import sys

class ImagingTriggerTest(Node):
    def __init__(self):
        super().__init__('imaging_trigger_test')

        # Publishers
        self.surface_pub = self.create_publisher(CompressedImage, '/robot/image/surface', 10)
        self.underwater_pub = self.create_publisher(CompressedImage, '/robot/image/underwater', 10)
        self.navigation_vision_pub = self.create_publisher(CompressedImage, '/robot/vision/navigation', qos_profile_services_default)

        # Camera setup
        self.cam = cv2.VideoCapture(0)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cam.set(cv2.CAP_PROP_FPS, 20)

        if not self.cam.isOpened():
            self.get_logger().error('Failed to open camera /dev/video0')
            sys.exit(1)

        self.bridge = CvBridge()

        self.get_logger().info('Imaging Trigger Test Node Started')
        self.get_logger().info('Camera opened: /dev/video0 (640x480@20fps)')
        self.get_logger().info('Press keys to capture:')
        self.get_logger().info('  s - Capture Surface Image')
        self.get_logger().info('  u - Capture Underwater Image')
        self.get_logger().info('  q - Quit')

    def capture_and_publish(self, topic_name, publisher):
        """Capture frame and publish compressed image"""
        ret, frame = self.cam.read()
        if not ret:
            self.get_logger().error('Failed to capture frame')
            return

        # Compress image to JPEG
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        _, buffer = cv2.imencode('.jpg', frame, encode_param)

        # Create CompressedImage message
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = buffer.tobytes()

        # Publish 3 times for reliability
        for _ in range(3):
            publisher.publish(msg)

        self.get_logger().info(f'Published to {topic_name}')

    def publish_navigation_vision(self, frame):
        """Publish navigation vision frame (always publishing)"""
        # Compress image to JPEG
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        _, buffer = cv2.imencode('.jpg', frame, encode_param)

        # Create CompressedImage message
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = buffer.tobytes()

        self.navigation_vision_pub.publish(msg)

    def run(self):
        """Main loop with OpenCV display"""
        try:
            while rclpy.ok():
                ret, frame = self.cam.read()
                if not ret:
                    self.get_logger().error('Failed to read frame')
                    break

                # Always publish navigation vision
                self.publish_navigation_vision(frame)

                # Display frame
                cv2.imshow('Imaging Test - Press s/u to capture, q to quit', frame)

                key = cv2.waitKey(1) & 0xFF

                if key == ord('s'):
                    self.capture_and_publish('/robot/image/surface', self.surface_pub)
                elif key == ord('u'):
                    self.capture_and_publish('/robot/image/underwater', self.underwater_pub)
                elif key == ord('q'):
                    self.get_logger().info('Quitting...')
                    break

                rclpy.spin_once(self, timeout_sec=0.01)

        except KeyboardInterrupt:
            pass
        finally:
            self.cam.release()
            cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = ImagingTriggerTest()

    try:
        node.run()
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
