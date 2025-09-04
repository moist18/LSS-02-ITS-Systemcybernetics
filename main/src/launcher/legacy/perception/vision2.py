#!/usr/bin/env python3

import numpy as np
import math
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_services_default, qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from mavros_msgs.msg import OverrideRCIn

from pathlib import Path
import sys
import os

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))

from ultralytics import YOLO
from utils.novisual import BBoxVisualization  # Competition
# from utils.visual import BBoxVisualization  # Development

class PID:
    def __init__(self, kp, ki, kd, min_output, max_output, clock):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_output = min_output
        self.max_output = max_output
        self.clock = clock
        self.reset()

    def reset(self):
        current_time = self.clock.now().nanoseconds / 1e9
        self.prev_time = current_time
        self.prev_error = 0
        self.integral = 0

    def compute(self, error):
        current_time = self.clock.now().nanoseconds / 1e9
        dt = max(current_time - self.prev_time, 1e-6)
        
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = max(self.min_output, min(self.max_output, output))
        
        self.prev_error = error
        self.prev_time = current_time
        
        return output

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        # Vision Parameter Declaration
        self.declare_parameter('model_path', f'{ROOT}/model/lesgo_fp16.engine')
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        
        # RC Parameter Declaration
        self.declare_parameter('vision.rc.throttle', [2000, 1700])
        self.declare_parameter('vision.rc.min_value', [1000, 1200])
        self.declare_parameter('vision.rc.mid_value', 1500)
        self.declare_parameter('vision.rc.max_value', [2000, 1800])
        
        # PID Controller Parameter Declaration
        self.declare_parameter('vision.pid.yaw.kp', 1.0)
        self.declare_parameter('vision.pid.yaw.ki', 0.0)
        self.declare_parameter('vision.pid.yaw.kd', 0.1)
        self.declare_parameter('vision.pid.yaw.min_output', -1.0)
        self.declare_parameter('vision.pid.yaw.max_output', 1.0)
        
        # Subscriber Initialization
        self.mode_sub = self.create_subscription(Bool, '/robot/mode', self.mode_callback, qos_profile_services_default)
        self.gps_control_sub = self.create_subscription(Bool, '/robot/control/gps', self.gps_control_callback, qos_profile_services_default)
        self.compass_control_sub = self.create_subscription(Bool, '/robot/control/compass', self.compass_control_callback, qos_profile_services_default)
        self.vision_control_sub = self.create_subscription(Bool, '/robot/control/vision', self.vision_control_callback, qos_profile_services_default)
        self.lidar_sub = self.create_subscription(Bool, '/robot/lidar', self.lidar_callback, qos_profile_sensor_data)
        self.mission_sub = self.create_subscription(String, '/robot/mission', self.mission_callback, qos_profile_services_default)
        self.underwater_mission_sub = self.create_subscription(Bool, '/robot/mission/underwater', self.underwater_callback, qos_profile_services_default)
        self.docking_sub = self.create_subscription(String, '/detect/docking', self.docking_callback, qos_profile_services_default)
        self.left_buoy_sub = self.create_subscription(String, '/detect/left_buoy', self.left_buoy_callback, qos_profile_services_default)
        self.right_buoy_sub = self.create_subscription(String, '/detect/right_buoy', self.right_buoy_callback, qos_profile_services_default)
        self.reset_mission_sub = self.create_subscription(Bool, '/mission/reset', self.reset_mission_callback, qos_profile_services_default)
        
        # Publisher Initialization
        self.navigation_vision_pub = self.create_publisher(Image, '/robot/vision/navigation', qos_profile_services_default)
        self.surface_image_pub = self.create_publisher(Image, '/robot/image/surface', qos_profile_services_default)
        self.surface_mission_pub = self.create_publisher(Bool, '/robot/mission/surface', qos_profile_services_default)
        self.obj_detection_pub = self.create_publisher(Bool, '/detect/object/bool', qos_profile_services_default)
        self.rc_override_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', qos_profile_sensor_data)

        # Load TensorRT Model
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.model = YOLO(model=model_path, task='detect')
        self.cls_dict = self.model.names
        
        # Constant Initialization
        self.WIDTH = self.get_parameter('image_width').get_parameter_value().integer_value
        self.HEIGHT = self.get_parameter('image_height').get_parameter_value().integer_value
        
        # Variable Initialization
        self.bridge = CvBridge()
        self.rc_override = OverrideRCIn()
        self.vis = BBoxVisualization(self.cls_dict)
        
        self.auto_mode = True
        self.gps_control = False
        self.compass_control = False
        self.vision_control = False
        self.lidar = False
        
        self.is_surface = False
        self.is_underwater = False
        self.is_finish = False
        
        self.mission = ''
        self.docking = 'green_dock'
        self.left_buoy = 'green_buoy'
        self.right_buoy = 'red_buoy'
        self.prev_time = 0
        self.fps = 0
        
        # Camera Configuration
        camera_index = self.get_parameter('camera_index').get_parameter_value().integer_value
        self.cam = cv2.VideoCapture(camera_index)
        self.cam.set(3, self.WIDTH)
        self.cam.set(4, self.HEIGHT)
        
        # Rate Timer Initialization
        self.loop_timer = self.create_timer(1.0/20, self.runner)  # 20 Hz
        self.param_timer = self.create_timer(1.0, self.param_callback)
        
        self.load_parameter()
        self.get_logger().info("Vision node started")
    
    def load_parameter(self):
        # RC Parameter Initialization
        self.rc_channels_default = [65535] * 18
        self.rc_throttle = self.get_parameter('vision.rc.throttle').get_parameter_value().integer_array_value
        self.min_rc_value = self.get_parameter('vision.rc.min_value').get_parameter_value().integer_array_value
        self.mid_rc_value = self.get_parameter('vision.rc.mid_value').get_parameter_value().integer_value
        self.max_rc_value = self.get_parameter('vision.rc.max_value').get_parameter_value().integer_array_value
        
        # PID Controller Initialization
        yaw_kp = self.get_parameter('vision.pid.yaw.kp').get_parameter_value().double_value
        yaw_ki = self.get_parameter('vision.pid.yaw.ki').get_parameter_value().double_value
        yaw_kd = self.get_parameter('vision.pid.yaw.kd').get_parameter_value().double_value
        yaw_min = self.get_parameter('vision.pid.yaw.min_output').get_parameter_value().double_value
        yaw_max = self.get_parameter('vision.pid.yaw.max_output').get_parameter_value().double_value
        
        self.pid_yaw = PID(
            kp=yaw_kp, 
            ki=yaw_ki, 
            kd=yaw_kd, 
            min_output=yaw_min, 
            max_output=yaw_max,
            clock=self.get_clock()
        )
    
    def param_callback(self):
        self.load_parameter()
    
    def mode_callback(self, msg):
        self.auto_mode = msg.data
    
    def gps_control_callback(self, msg):
        self.gps_control = msg.data
    
    def compass_control_callback(self, msg):
        self.compass_control = msg.data
    
    def vision_control_callback(self, msg):
        self.vision_control = msg.data
    
    def lidar_callback(self, msg):
        self.lidar = msg.data
    
    def mission_callback(self, msg):
        self.mission = msg.data
    
    def underwater_callback(self, msg):
        self.is_underwater = msg.data
    
    def docking_callback(self, msg):
        self.docking = msg.data
    
    def left_buoy_callback(self, msg):
        self.left_buoy = msg.data
    
    def right_buoy_callback(self, msg):
        self.right_buoy = msg.data
    
    def reset_mission_callback(self, msg):
        if msg.data:
            self.mission = ""
            self.is_surface = False
            self.is_finish = False
            self.get_logger().info("S-VIZ: Successfully Reset Mission!")
    
    def processor(self, frame):
        if self.model is None:
            # Return defaults if no model
            return self.mid_rc_value, self.rc_throttle[0] if len(self.rc_throttle) > 0 else 1500
            
        results = self.model.predict(frame, imgsz=416, conf=0.65, iou=0.5, stream=True, half=True)
        raw_yaw = self.mid_rc_value
        objects_detected = False
        
        for r in results:
            if len(r.boxes) > 0:
                objects_detected = True
            
            frame = r.plot()                                     # Competition
            if self.vis:
                frame, raw_yaw, is_finish = self.vis.processor(  # Competition / Development
                        frame,
                        self.fps,
                        self.mission,
                        self.docking,
                        self.left_buoy,
                        self.right_buoy,
                        r.boxes.xyxy.tolist(),
                        r.boxes.conf.tolist(),
                        r.boxes.cls.tolist(),
                        frame.shape[1],
                        frame.shape[0],
                        self.min_rc_value[0 if self.mission == "Floating Ball" else 1],
                        self.mid_rc_value,
                        self.max_rc_value[0 if self.mission == "Floating Ball" else 1]
                    )
        
        img_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        self.navigation_vision_pub.publish(img_msg)
        
        if objects_detected:
            yaw_error = (raw_yaw - self.mid_rc_value) / (self.max_rc_value[0 if self.mission == "Floating Ball" else 1] - self.mid_rc_value)
            rc_yaw = self.calculate_rc_value(self.pid_yaw.compute(yaw_error))
            rc_throttle = self.rc_throttle[0 if self.mission == "Floating Ball" else 1]
            self.obj_detection_pub.publish(Bool(data=True))
            self.get_logger().info(f"RC YAW: {rc_yaw}, Raw Yaw: {raw_yaw}, Error: {yaw_error}")
        else:
            rc_yaw = self.mid_rc_value
            rc_throttle = self.rc_throttle[0 if self.mission == "Floating Ball" else 1]
            self.pid_yaw.reset()
            self.obj_detection_pub.publish(Bool(data=False))

        if self.auto_mode:
            if not self.gps_control and not self.compass_control and self.vision_control:
                if self.mission == "Surface Imaging":
                    if self.lidar and not self.is_surface:
                        self.is_surface = True
                        rc_yaw = self.mid_rc_value
                        rc_throttle = self.mid_rc_value - 200
                        self.pid_yaw.reset()
                        self.surface_image_pub.publish(img_msg)
                    elif self.is_surface:
                        rc_yaw = self.mid_rc_value
                        rc_throttle = self.mid_rc_value - 200
                        self.pid_yaw.reset()
                elif self.mission == "Underwater Imaging":
                    if self.lidar and not self.is_underwater:
                        rc_yaw = self.mid_rc_value
                        rc_throttle = self.mid_rc_value - 200
                        self.pid_yaw.reset()
                    elif self.is_underwater:
                        rc_yaw = self.mid_rc_value
                        rc_throttle = self.mid_rc_value - 200
                        self.pid_yaw.reset()
                elif self.mission == "Docking":
                    if self.lidar and not self.is_finish:
                        self.is_finish = True
        
        return rc_yaw, rc_throttle
    
    def calculate_fps(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.fps = 1 / (current_time - self.prev_time)
        self.prev_time = current_time
        self.get_logger().info(f"FPS: {int(self.fps)}")
    
    def calculate_rc_value(self, pid_output):
        rc_value = self.mid_rc_value + pid_output * (self.max_rc_value[0 if self.mission == "Floating Ball" else 1] - self.mid_rc_value)
        return int(max(self.min_rc_value[0 if self.mission == "Floating Ball" else 1], min(self.max_rc_value[0 if self.mission == "Floating Ball" else 1], rc_value)))
    
    def override_control(self, yaw, pitch, throttle, lateral):
        self.rc_override.channels = self.rc_channels_default.copy()
        self.rc_override.channels[0] = int(yaw)
        self.rc_override.channels[1] = int(pitch)
        self.rc_override.channels[2] = int(throttle)
        self.rc_override.channels[3] = int(lateral)
        self.rc_override_pub.publish(self.rc_override)
        self.get_logger().info(f"YAW: {int(yaw)}")
    
    def runner(self):
        success, frame = self.cam.read()
        if not success:
            self.get_logger().error("Failed to capture frame from camera")
            return
        
        yaw, throttle = self.processor(frame)
        self.surface_mission_pub.publish(Bool(data=self.is_surface))
        
        if self.auto_mode:
            if not self.gps_control and not self.compass_control and self.vision_control:
                if not self.is_finish:
                    self.override_control(yaw, 1500, throttle, 1500)
                else:
                    self.override_control(1500, 1500, 1500, 1500)
                    self.get_logger().info("VISION_FINISH: FINISH WOYYY!!!")
            
                self.calculate_fps()

    def destroy_node(self):
        if hasattr(self, 'cam'):
            self.cam.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()