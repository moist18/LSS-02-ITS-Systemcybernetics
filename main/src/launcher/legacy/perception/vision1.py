#!/usr/bin/env python3

import numpy as np
import math
import cv2

import rospy
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
from utils.novisual import BBoxVisualization   # Competition
# from utils.visual import BBoxVisualization   # Development

class PID:
    def __init__(self, kp, ki, kd, min_output, max_output):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_output = min_output
        self.max_output = max_output
        self.reset()

    def reset(self):
        self.prev_time = rospy.get_time()
        self.prev_error = 0
        self.integral = 0

    def compute(self, error):
        current_time = rospy.get_time()
        dt = max(current_time - self.prev_time, 1e-6)
        
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = max(self.min_output, min(self.max_output, output))
        
        self.prev_error = error
        self.prev_time = current_time
        
        return output

class Detection:
    def __init__(self):
        rospy.init_node('detection_gps', anonymous=True)
        self.rate = rospy.Rate(20)
        
        # Subscribers
        self.mode_sub = rospy.Subscriber('/robot/mode', Bool, self.mode_callback)
        self.gps_control_sub = rospy.Subscriber('/robot/control/gps', Bool, self.gps_control_callback)
        self.compass_control_sub = rospy.Subscriber('/robot/control/compass', Bool, self.compass_control_callback)
        self.vision_control_sub = rospy.Subscriber('/robot/control/vision', Bool, self.vision_control_callback)
        self.lidar_sub = rospy.Subscriber('/robot/lidar', Bool, self.lidar_callback)
        self.mission_sub = rospy.Subscriber('/robot/mission', String, self.mission_callback)
        self.underwater_mission_sub = rospy.Subscriber('/robot/mission/underwater', Bool, self.underwater_callback)
        self.docking_sub = rospy.Subscriber('/detect/docking', String, self.docking_callback)
        self.left_buoy_sub = rospy.Subscriber('/detect/left_buoy', String, self.left_buoy_callback)
        self.right_buoy_sub = rospy.Subscriber('/detect/right_buoy', String, self.right_buoy_callback)
        self.reset_mission_sub = rospy.Subscriber('/mission/reset', Bool, self.reset_mission_callback)
        
        # Publishers
        self.navigation_vision_pub = rospy.Publisher('/robot/vision/navigation', Image, queue_size=1)
        self.surface_image_pub = rospy.Publisher('/robot/image/surface', Image, queue_size=1)
        self.surface_mission_pub = rospy.Publisher('/robot/mission/surface', Bool, queue_size=1)
        self.obj_detection_pub = rospy.Publisher('/detect/object/bool', Bool, queue_size=1)
        self.rc_override_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=1)
        
        # Load TensorRT Model
        self.model = YOLO(model=f'{ROOT}/model/lesgo_fp16.engine', task='detect')
        self.cls_dict = self.model.names
        
        # Constants
        self.WIDTH = 640
        self.HEIGHT = 480
        
        # Variables
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
        self.dock_start_time = None
        
        self.mission = ''
        self.docking = 'green_dock'
        self.left_buoy = 'green_buoy'
        self.right_buoy = 'red_buoy'
        self.prev_time = 0
        self.fps = 0
        
        # Parameters
        self.load_rosparam()
        self.param_timer = rospy.Timer(rospy.Duration(1.0), self.param_callback)
    
    def load_rosparam(self):
        # RC Parameter
        self.rc_channels_default = [65535] * 18
        self.rc_throttle = rospy.get_param('/vision/rc/throttle', [2000, 1700])
        self.min_rc_value = rospy.get_param('/vision/rc/min_value', [1000, 1200])
        self.mid_rc_value = rospy.get_param('/vision/rc/mid_value', 1500)
        self.max_rc_value = rospy.get_param('/vision/rc/max_value', [2000, 1800])
        
        # PID Controller
        self.yaw_params = rospy.get_param("/vision/pid/yaw")
        self.pid_yaw = PID(
            kp=self.yaw_params['kp'], 
            ki=self.yaw_params['ki'], 
            kd=self.yaw_params['kd'], 
            min_output=self.yaw_params['min_output'], 
            max_output=self.yaw_params['max_output']
        )
        
        # Docking Timer
        self.dock_duration = rospy.get_param("/vision/timer/docking")
    
    def param_callback(self, event):
        self.load_rosparam()
    
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
            self.dock_start_time = None
            rospy.loginfo("S-VIZ: Successfully Reset Mission!")
    
    def processor(self, frame):
        results = self.model.predict(frame, imgsz=416, conf=0.65, iou=0.5, stream=True, half=True)
        raw_yaw = self.mid_rc_value
        objects_detected = False
        
        for r in results:
            if len(r.boxes) > 0:
                objects_detected = True
            
            frame = r.plot()                                 # Competition
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
            self.obj_detection_pub.publish(True)
            rospy.loginfo(f"RC YAW: {rc_yaw}, Raw Yaw: {raw_yaw}, Error: {yaw_error}")
        else:
            rc_yaw = self.mid_rc_value
            rc_throttle = self.rc_throttle[0 if self.mission == "Floating Ball" else 1]
            self.pid_yaw.reset()
            self.obj_detection_pub.publish(False)

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
                    if not self.is_finish:  # tambahkan pengecekan "is_finish" jika memungkinkan
                        self.is_finish = True
        
        return rc_yaw, rc_throttle
    
    def calculate_fps(self):
        current_time = rospy.get_time()
        self.fps = 1 / (current_time - self.prev_time)
        self.prev_time = current_time
        rospy.loginfo(f"FPS: {int(self.fps)}")
    
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
        rospy.loginfo(f"YAW: {int(yaw)}")
    
    def run(self):
        cam = cv2.VideoCapture(0)
        cam.set(3, self.WIDTH)
        cam.set(4, self.HEIGHT)
        
        try:
            while not rospy.is_shutdown():
                success, frame = cam.read()
                if not success:
                    break
                
                yaw, throttle = self.processor(frame)
                self.surface_mission_pub.publish(self.is_surface)
                
                if self.auto_mode:
                    if not self.gps_control and not self.compass_control and self.vision_control:
                        if not self.is_finish:
                            self.override_control(yaw, 1500, throttle, 1500)
                        else:
                            current_time = rospy.get_time()
                            
                            if self.dock_start_time is None:
                                self.dock_start_time = current_time
                            
                            phase_duration = self.dock_duration
                            elapsed_time = current_time - self.dock_start_time
                            
                            if elapsed_time >= phase_duration:
                                self.override_control(1500, 1500, 1500, 1500)
                                rospy.loginfo_throttle(1, "VISION_FINISH: FINISH WOYYY!!!")
                            else:
                                self.override_control(yaw, 1500, throttle, 1500)
                                rospy.loginfo_throttle(1, "VISION_FINISH: Road to Finish!")
                    
                        self.calculate_fps()
                
                self.rate.sleep()
            
        except Exception as e:
            rospy.logerr(f"An error occurred: {e}")
        
        finally:
            cam.release()
            cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        Detection().run()
    except rospy.ROSInterruptException:
        pass