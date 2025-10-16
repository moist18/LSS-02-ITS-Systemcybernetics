#!/usr/bin/env python3

import cv2
import os
from pathlib import Path

import rclpy
from rclpy.node import Node

class RecorderNode(Node):
    def __init__ (self):
        super().__init__('recorder_node')

        # Declare parameters
        self.declare_parameter('video_dir', '/home/ubuntu/ASV-2025/recordings')
        self.declare_parameter('fps', 20)
        self.declare_parameter('resolution', [640, 480])
        self.declare_parameter('codec', 'MJPG')

        # Get parameters
        video_dir_param = self.get_parameter('video_dir').get_parameter_value().string_value
        self.video_dir = Path(video_dir_param).expanduser()
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        resolution = self.get_parameter('resolution').get_parameter_value().integer_array_value
        codec_str = self.get_parameter('codec').get_parameter_value().string_value

        # Variable Initialization
        self.width = resolution[0]
        self.height = resolution[1]
        self.size = (self.width, self.height)
        self.fourcc = cv2.VideoWriter_fourcc(*codec_str)
        self.index = 1
        self.prev_time = self.get_clock().now().nanoseconds / 1e9
        self.frame_count = 0
        self.error_count = 0

        self.get_logger().info(f"📝 Recorder config: {self.width}x{self.height} @ {self.fps}fps, codec={codec_str}")

        # Create video directory if not exists (BEFORE video_writer)
        os.makedirs(self.video_dir, exist_ok=True)
        self.get_logger().info(f"📂 Video directory: {self.video_dir}")

        # Camera Configuration (BEFORE video writer to check camera first)
        camera_device = self.get_parameter('camera_device').get_parameter_value().integer_value
        self.cap = cv2.VideoCapture(camera_device)
        self.get_logger().info(f"📷 Recorder using camera device: /dev/video{camera_device}")

        if not self.cap.isOpened():
            self.get_logger().error(f"❌ Failed to open camera /dev/video{camera_device}")
            raise RuntimeError(f"Camera /dev/video{camera_device} not available")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        # Verify camera resolution
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f"📷 Camera resolution: {actual_width}x{actual_height}")

        # Initialize video writer
        self.out = self.video_writer()

        if not self.out.isOpened():
            self.get_logger().error("❌ Failed to create video writer")
            raise RuntimeError("VideoWriter initialization failed")

        self.get_logger().info("✅ Recorder initialized successfully")

        # Rate Timer Initialization
        self.loop_timer = self.create_timer(1.0/self.fps, self.timer_callback)

    def video_writer(self):
        while os.path.exists(os.path.join(self.video_dir, f'output{self.index}.avi')):
            self.index += 1

        video_path = os.path.join(self.video_dir, f'output{self.index}.avi')
        self.get_logger().info(f"🎥 Recording to: {video_path}")

        # Create video writer
        writer = cv2.VideoWriter(str(video_path), self.fourcc, self.fps, self.size)

        if not writer.isOpened():
            codec_name = ''.join([chr((self.fourcc >> 8 * i) & 0xFF) for i in range(4)])
            self.get_logger().error(f"❌ Failed to open video writer for {video_path}")
            self.get_logger().error(f"   Codec: {codec_name}, FPS: {self.fps}, Size: {self.size}")

        return writer

    def calculate_fps(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        frame_fps = 1 / (current_time - self.prev_time)
        self.prev_time = current_time
        self.get_logger().info(f"FPS: {int(frame_fps)}", throttle_duration_sec=1)
    
    def timer_callback(self):
        success, frame = self.cap.read()
        if success:
            if self.out.isOpened():
                self.out.write(frame)
                self.frame_count += 1
                self.calculate_fps()

                # Log progress every 100 frames
                if self.frame_count % 100 == 0:
                    self.get_logger().info(f"📹 Recorded {self.frame_count} frames")
            else:
                self.error_count += 1
                self.get_logger().error("❌ Video writer not opened, cannot write frame", throttle_duration_sec=5)
        else:
            self.error_count += 1
            self.get_logger().error("❌ Failed to capture frame from camera", throttle_duration_sec=5)

    def destroy_node(self):
        self.get_logger().info("🛑 Shutting down recorder...")
        self.get_logger().info(f"📊 Final stats: {self.frame_count} frames recorded, {self.error_count} errors")

        if self.cap.isOpened():
            self.cap.release()

        if self.out.isOpened():
            self.out.release()
            self.get_logger().info(f"✅ Video saved: output{self.index}.avi")

        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RecorderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()