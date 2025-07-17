#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from jtop import jtop, JtopException

class GPUNode(Node):
    def __init__(self):
        super().__init__('gpu_node')
        self.declare_parameter('update_rate', 1.0)  # Hz
        
        # Publisher
        self.gpu_pub = self.create_publisher(Float32, '/robot/gpu', 10)
        
        # Timer
        update_rate = self.get_parameter('update_rate').value
        self.timer = self.create_timer(1.0/update_rate, self.monitor_gpu)
        
        self.get_logger().info("GPU Node initialized")

    def monitor_gpu(self):
        with jtop() as jetson:
            if jetson.ok():
                gpu_usage = jetson.stats['GPU']
                msg = Float32()
                msg.data = float(gpu_usage)
                self.gpu_pub.publish(msg)
                
                self.get_logger().debug(
                    f"GPU Usage: {gpu_usage}%", 
                    throttle_duration_sec=1.0
                )

def main(args=None):
    rclpy.init(args=args)
    node = GPUNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()