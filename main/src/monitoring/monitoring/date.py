#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_services_default
from std_msgs.msg import Float32MultiArray
from datetime import datetime

class DateNode(Node):
    def __init__(self):
        super().__init__('date_node')
        
        # Publisher
        self.date_pub = self.create_publisher(Float32MultiArray, '/robot/date', qos_profile_services_default)
        
        # Timer for publishing at 1Hz
        self.loop_timer = self.create_timer(1.0, self.data_publisher)

    def data_publisher(self):
        now = datetime.now()
        day = now.strftime("%A")
        date = now.strftime("%d %B %Y")
        time = now.strftime("%H:%M:%S")
        
        date_data = Float32MultiArray()
        date_data.data = [day, date, time]
        self.date_pub.publish(date_data)
        self.get_logger().info(f"Datetime: {day}, {date} [{time}]")

def main(args=None):
    rclpy.init(args=args)
    node = DateNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()