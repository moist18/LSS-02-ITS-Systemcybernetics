#!/usr/bin/env python3

import cv2
import os
from pathlib import Path
import sys

import rclpy
from rclpy.node import Node

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))

class RecorderNode(Node):
    def __init__ (self):
        super().__init__('recorder_node')
        
        # Variable Initialization
        self.fps = 20
        self.index = 1
        self.prev_time = self.get_clock().now().nanoseconds / 1e9
        self.size = (640, 480)
        self.width = 640
        self.height = 480
        self.video_dir = f'{ROOT}/video'
        self.fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self.out = self.video_writer()
        
        # Create video directory if not exists
        os.makedirs(self.video_dir, exist_ok=True)
        
        # Camera Configuration
        self.cap = cv2.VideoCapture(0)
        self.cap.set(3, self.width)
        self.cap.set(4, self.height)
        
        # Rate Timer Initialization
        self.loop_timer = self.create_timer(1.0/self.fps, self.timer_callback)

    def video_writer(self):
        while os.path.exists(os.path.join(self.video_dir, f'output{self.index}.avi')):
            self.index += 1
        
        video_path = os.path.join(self.video_dir, f'output{self.index}.avi')
        self.get_logger().info(f"Recording to: {video_path}")
        return cv2.VideoWriter(video_path, self.fourcc, self.fps, self.size)

    def calculate_fps(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        frame_fps = 1 / (current_time - self.prev_time)
        self.prev_time = current_time
        self.get_logger().info(f"FPS: {int(frame_fps)}", throttle_duration_sec=1)
    
    def timer_callback(self):
        success, frame = self.cap.read()
        if success:
            self.out.write(frame)
            self.calculate_fps()
        else:
            self.get_logger().error("Failed to capture frame from camera")

    def destroy_node(self):
        self.get_logger().info("Shutting down recorder...")
        self.cap.release()
        self.out.release()
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