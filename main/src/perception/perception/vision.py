#!/usr/bin/env python3

import numpy as np
import cv2
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_services_default
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Bool, Int32
from cv_bridge import CvBridge

from pathlib import Path
import sys
import os

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

from ultralytics import YOLO
# from perception.utils.novisual import BBoxVisualization  # Competition
from perception.utils.visual import BBoxVisualization  # Development

def optimize_rknn_model():
    # Set environment variables for NPU optimization
    os.environ['RKNN_NPU_CORE'] = '0,1,2'
    os.environ['RKNN_OPTIMIZE'] = '1'
    os.environ['RKNN_TARGET_PLATFORM'] = 'rk3588'
    
    # Memory optimization configuration
    os.environ['RKNN_MEM_OPTIMIZE'] = '1'
    os.environ['RKNN_BATCH_SIZE'] = '1'
    
    # Thread configuration for Orange Pi 5 Pro (8 CPU cores, 3 NPU cores)
    os.environ['OMP_NUM_THREADS'] = '8'
    os.environ['RKNN_NUM_THREADS'] = '3'

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # Vision Parameter Declaration
        self.declare_parameter('model_path', f'{ROOT}/models/LESGO11s_x640_rknn_model')
        self.declare_parameter('config.camera_device', 0)
        self.declare_parameter('config.width', 640)
        self.declare_parameter('config.height', 480)
        self.declare_parameter('config.fps', 30)
        self.declare_parameter('config.debug', False)
        self.declare_parameter('config.use_lidar', True)
        
        # Detection Parameter Declaration
        self.declare_parameter('vision.detection.image_size', 640)
        self.declare_parameter('vision.detection.confidence', 0.5)
        self.declare_parameter('vision.detection.iou_threshold', 0.5)
        self.declare_parameter('vision.detection.agnostic_nms', False)
        self.declare_parameter('vision.detection.half', False)
        self.declare_parameter('vision.detection.verbose', False)
        self.declare_parameter('vision.detection.min_buoy_area', 480)
        self.declare_parameter('vision.detection.min_box_area', 480)

        # Distance Measurement Parameter Declaration (Ground-Plane Projection)
        self.declare_parameter('vision.distance.use_distance_based', False)
        self.declare_parameter('vision.distance.camera_height', 0.40)
        self.declare_parameter('vision.distance.camera_tilt_deg', 45.0)
        self.declare_parameter('vision.distance.focal_length_y', 320.0)
        self.declare_parameter('vision.distance.principal_point_y', 240.0)
        self.declare_parameter('vision.distance.trigger_distance', 1.0)

        # RC Parameter Declaration
        self.declare_parameter('vision.rc.min_value', [1000, 1300])
        self.declare_parameter('vision.rc.mid_value', 1500)
        self.declare_parameter('vision.rc.max_value', [2000, 1700])
        
        # Camera Parameter Initialization
        camera_device = self.get_parameter('config.camera_device').get_parameter_value().integer_value
        self.WIDTH = self.get_parameter('config.width').get_parameter_value().integer_value
        self.HEIGHT = self.get_parameter('config.height').get_parameter_value().integer_value
        self.FPS = self.get_parameter('config.fps').get_parameter_value().integer_value
        self.DEBUG = self.get_parameter('config.debug').get_parameter_value().bool_value
        self.USE_LIDAR = self.get_parameter('config.use_lidar').get_parameter_value().bool_value
        
        # Subscriber Initialization
        self.mode_sub = self.create_subscription(Bool, '/robot/mode', self.mode_callback, qos_profile_services_default)
        self.gps_control_sub = self.create_subscription(Bool, '/robot/control/gps', self.gps_control_callback, qos_profile_services_default)
        self.compass_control_sub = self.create_subscription(Bool, '/robot/control/compass', self.compass_control_callback, qos_profile_services_default)
        self.vision_control_sub = self.create_subscription(Bool, '/robot/control/vision', self.vision_control_callback, qos_profile_services_default)
        self.lidar_sub = self.create_subscription(Bool, '/robot/lidar', self.lidar_callback, qos_profile_services_default)
        self.mission_sub = self.create_subscription(String, '/robot/mission', self.mission_callback, qos_profile_services_default)
        self.docking_sub = self.create_subscription(String, '/detect/docking', self.docking_callback, qos_profile_services_default)
        self.left_buoy_sub = self.create_subscription(String, '/detect/left_buoy', self.left_buoy_callback, qos_profile_services_default)
        self.right_buoy_sub = self.create_subscription(String, '/detect/right_buoy', self.right_buoy_callback, qos_profile_services_default)
        self.reset_mission_sub = self.create_subscription(Bool, '/mission/reset', self.reset_mission_callback, qos_profile_services_default)

        # Publisher Initialization
        self.navigation_vision_pub = self.create_publisher(CompressedImage, '/robot/vision/navigation', qos_profile_services_default)
        self.surface_image_pub = self.create_publisher(CompressedImage, '/robot/image/surface', qos_profile_services_default)
        self.surface_mission_pub = self.create_publisher(Bool, '/robot/mission/surface', qos_profile_services_default)
        self.obj_detection_pub = self.create_publisher(Bool, '/detect/object/bool', qos_profile_services_default)
        self.raw_yaw_pub = self.create_publisher(Int32, '/robot/vision/raw_yaw', qos_profile_services_default)
        self.snapshot_pub = self.create_publisher(Bool, '/robot/snapshot', qos_profile_services_default)

        # RKNN Optimization Initialization
        optimize_rknn_model()

        # Load multiple YOLO model instances for NPU cores
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        try:
            self.models = [YOLO(model=model_path, task='detect') for _ in range(3)]
            self.cls_dict = self.models[0].names
            self.get_logger().info("Successfully loaded 3 RKNN-optimized YOLO models")
        except Exception as e:
            self.get_logger().warn(f"Failed to load multiple models, falling back to single model: {e}")
            self.models = [YOLO(model=model_path, task='detect')]
            self.cls_dict = self.models[0].names

        # Threading and Buffering System Initialization
        self.frame_buffer = deque(maxlen=5)
        self.latest_result = None
        self.result_lock = threading.Lock()

        self.running = True
        self.stats = {'processed': 0, 'dropped': 0}

        # Variable Initialization
        self.bridge = CvBridge()
        self.vis = BBoxVisualization(self.cls_dict)

        # Control State Variables
        self.auto_mode = True
        self.gps_control = False
        self.compass_control = False
        self.vision_control = False
        self.is_surface = False
        self.lidar = False
        
        # Mission Variables
        self.mission = 'Floating Ball'
        self.docking = 'green_dock'
        self.left_buoy = 'green_buoy'
        self.right_buoy = 'red_buoy'
        self.fps = 0
        self.frame_count = 0
        self.fps_update_time = time.time()

        # Camera Configuration
        self.cam = cv2.VideoCapture(camera_device)
        self.get_logger().info(f"Vision using camera /dev/video{camera_device}")

        if not self.cam.isOpened():
            self.get_logger().error(f"Failed to open camera /dev/video{camera_device}")
            raise RuntimeError(f"Camera /dev/video{camera_device} not available")

        self.cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.WIDTH)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.HEIGHT)
        self.cam.set(cv2.CAP_PROP_FPS, self.FPS)
        self.cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))

        # Rate Timer Initialization
        self.loop_timer = self.create_timer(1.0/self.FPS, self.runner)
        self.param_timer = self.create_timer(1.0, self.param_callback)
        
        # Parameter Setup
        self.load_parameter()

        # Threading Setup
        self.setup_rknn_threading()
        self.get_logger().info("S-VIZ → node started")

    def warm_up_models(self):
        self.get_logger().info("Warming up RKNN models...")

        dummy_frame = np.zeros((self.HEIGHT, self.WIDTH, 3), dtype=np.uint8)
        for i, model in enumerate(self.models):
            try:
                self.get_logger().info(f"Warming up model {i+1}/{len(self.models)}")
                _ = model(dummy_frame, verbose=False)
            except Exception as e:
                self.get_logger().warn(f"Failed to warm up model {i+1}: {e}")

        self.get_logger().info("Model warm-up completed!")

    def process_worker(self, worker_id):
        model = self.models[worker_id]

        while self.running:
            if self.frame_buffer:
                try:
                    frame = self.frame_buffer.popleft()
                    result = model(
                        frame,
                        imgsz=self.detection_imgsz,
                        conf=self.detection_conf,
                        iou=self.detection_iou,
                        agnostic_nms=self.agnostic_nms,
                        half=self.detection_half,
                        verbose=self.detection_verbose
                    )

                    with self.result_lock:
                        self.latest_result = result[0]
                        self.stats['processed'] += 1

                except IndexError:
                    pass

                except Exception as e:
                    self.get_logger().error(f"Worker {worker_id} processing error: {e}")

            else:
                time.sleep(0.001)

    def setup_rknn_threading(self):
        try:
            # Warm up models to reduce cold start latency
            self.warm_up_models()

            # Start processing worker threads
            self.workers = []
            for i in range(len(self.models)):
                worker = threading.Thread(target=self.process_worker, args=(i,), daemon=True)
                worker.start()
                self.workers.append(worker)
                self.get_logger().info(f"Started RKNN processing worker {i+1}/{len(self.models)}")

        except Exception as e:
            self.get_logger().error(f"Failed to initialize RKNN threading: {e}")
            # Fallback to single-threaded mode
            self.running = False

    def load_parameter(self):
        # YOLO Detection Parameter Initialization
        self.detection_imgsz = self.get_parameter('vision.detection.image_size').get_parameter_value().integer_value
        self.detection_conf = self.get_parameter('vision.detection.confidence').get_parameter_value().double_value
        self.detection_iou = self.get_parameter('vision.detection.iou_threshold').get_parameter_value().double_value
        self.agnostic_nms = self.get_parameter('vision.detection.agnostic_nms').get_parameter_value().bool_value
        self.detection_half = self.get_parameter('vision.detection.half').get_parameter_value().bool_value
        self.detection_verbose = self.get_parameter('vision.detection.verbose').get_parameter_value().bool_value
        self.min_buoy_area = self.get_parameter('vision.detection.min_buoy_area').get_parameter_value().integer_value
        self.min_box_area = self.get_parameter('vision.detection.min_box_area').get_parameter_value().integer_value

        # Distance Measurement Parameter Initialization (Ground-Plane Projection)
        self.use_distance_based = self.get_parameter('vision.distance.use_distance_based').get_parameter_value().bool_value
        self.camera_height = self.get_parameter('vision.distance.camera_height').get_parameter_value().double_value
        self.camera_tilt_deg = self.get_parameter('vision.distance.camera_tilt_deg').get_parameter_value().double_value
        self.focal_length_y = self.get_parameter('vision.distance.focal_length_y').get_parameter_value().double_value
        self.principal_point_y = self.get_parameter('vision.distance.principal_point_y').get_parameter_value().double_value
        self.trigger_distance = self.get_parameter('vision.distance.trigger_distance').get_parameter_value().double_value

        # Update vis utils distance parameters
        if self.vis:
            self.vis.use_distance_based = self.use_distance_based
            self.vis.camera_height = self.camera_height
            self.vis.camera_tilt_deg = self.camera_tilt_deg
            self.vis.focal_length_y = self.focal_length_y
            self.vis.principal_point_y = self.principal_point_y
            self.vis.trigger_distance = self.trigger_distance
            self.vis.v_horizon = self.vis.principal_point_y - self.vis.focal_length_y * np.tan(np.deg2rad(self.vis.camera_tilt_deg))

        # RC Parameter Initialization
        self.min_rc_value = self.get_parameter('vision.rc.min_value').get_parameter_value().integer_array_value
        self.mid_rc_value = self.get_parameter('vision.rc.mid_value').get_parameter_value().integer_value
        self.max_rc_value = self.get_parameter('vision.rc.max_value').get_parameter_value().integer_array_value

    def param_callback(self):
        self.load_parameter()
        self.get_logger().info('S-VIZ → Parameters Reloaded!', throttle_duration_sec=5.0)
    
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
            self.get_logger().info("S-VIZ → Successfully Reset Mission!")

    def publish_compressed_image(self, frame, publisher, quality=90, repeat=1):
        compressed_msg = CompressedImage()
        compressed_msg.header.stamp = self.get_clock().now().to_msg()
        compressed_msg.format = "jpeg"
        compressed_msg.data = np.array(cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, quality])[1]).tobytes()

        if repeat > 1:
            # Multiple publish with threading to ensure delivery
            def publish_repeat():
                for _ in range(repeat):
                    publisher.publish(compressed_msg)
                    time.sleep(0.1)
            threading.Thread(target=publish_repeat, daemon=True).start()
        else:
            # Good for vision streaming
            publisher.publish(compressed_msg)

    def handle_surface_imaging(self, frame, snapshot):
        if self.auto_mode:
            if not self.gps_control and not self.compass_control and self.vision_control:
                if self.mission == "Surface Imaging":
                    proximity_trigger = self.lidar if self.USE_LIDAR else snapshot

                    if proximity_trigger and not self.is_surface:
                        self.is_surface = True
                        self.publish_compressed_image(frame, self.surface_image_pub, quality=90, repeat=3)
                        self.get_logger().info("S-VIZ → Surface Imaging!")

    def processor(self, input_frame):
        # 1. Add the raw frame to the buffer for the worker thread
        if len(self.frame_buffer) >= self.frame_buffer.maxlen:
            self.frame_buffer.popleft()
            self.stats['dropped'] += 1

        self.frame_buffer.append(input_frame)

        # 2. Prepare the frame to display
        display_frame = input_frame.copy()
        
        # 3. Initialize default values
        raw_yaw = self.mid_rc_value
        objects_detected = False

        with self.result_lock:
            result = self.latest_result

        # 4. If there is a result, process it on top of the frame
        if result and hasattr(result, 'boxes') and len(result.boxes) > 0:
            # plot_frame = result.plot()
            plot_frame = display_frame
            objects_detected = True

            if self.vis:
                display_frame, raw_yaw, snapshot, _ = self.vis.processor(
                    display_frame,
                    self.fps,
                    self.mission,
                    self.docking,
                    self.left_buoy,
                    self.right_buoy,
                    result.boxes.xyxy.tolist(),
                    result.boxes.conf.tolist(),
                    result.boxes.cls.tolist(),
                    display_frame.shape[1],
                    display_frame.shape[0],
                    self.min_rc_value[0 if self.mission == "Floating Ball" else 1],
                    self.mid_rc_value,
                    self.max_rc_value[0 if self.mission == "Floating Ball" else 1],
                    self.min_buoy_area,
                    self.min_box_area
                )
        else:
            plot_frame = display_frame
            objects_detected = False

            if self.vis:
                display_frame, raw_yaw, snapshot, _ = self.vis.processor(
                    display_frame,
                    self.fps,
                    self.mission,
                    self.docking,
                    self.left_buoy,
                    self.right_buoy,
                    [],
                    [],
                    [],
                    display_frame.shape[1],
                    display_frame.shape[0],
                    self.min_rc_value[0 if self.mission == "Floating Ball" else 1],
                    self.mid_rc_value,
                    self.max_rc_value[0 if self.mission == "Floating Ball" else 1],
                    self.min_buoy_area,
                    self.min_box_area
                )
        
        # 6. Publish ROS data from the composed frame
        self.obj_detection_pub.publish(Bool(data=objects_detected))
        self.raw_yaw_pub.publish(Int32(data=int(raw_yaw)))
        self.snapshot_pub.publish(Bool(data=snapshot))

        # 7. Publish compressed surface images
        self.handle_surface_imaging(plot_frame, snapshot)
        
        # 8. Publish compressed navigation vision streaming
        self.publish_compressed_image(display_frame, self.navigation_vision_pub, quality=80)

        return display_frame

    def calculate_fps(self):
        self.frame_count += 1
        current_time = time.time()
        if current_time - self.fps_update_time >= 1.0:
            self.fps = self.frame_count / (current_time - self.fps_update_time)
            self.frame_count = 0
            self.fps_update_time = current_time
            self.get_logger().info(f"S-VIZ → FPS: {self.fps:.2f}")

    def runner(self):
        success, frame = self.cam.read()
        if not success:
            self.get_logger().error("Failed to capture frame from camera")
            return
        
        display_frame = self.processor(frame)

        # DEBUG: show frame
        if self.DEBUG:
            cv2.imshow('Vision Cam', display_frame)
            cv2.waitKey(1)
        
        # Calculate FPS
        self.calculate_fps()
        
        # Publish surface mission status
        self.surface_mission_pub.publish(Bool(data=self.is_surface))

    def destroy_node(self):
        self.get_logger().info("Shutting down RKNN-optimized vision node...")
        
        # Stop all worker threads
        self.running = False
        
        # Wait for worker threads to finish (with timeout)
        if hasattr(self, 'workers'):
            for i, worker in enumerate(self.workers):
                try:
                    worker.join(timeout=2.0)
                    if worker.is_alive():
                        self.get_logger().warn(f"Worker {i+1} did not stop gracefully")
                except Exception as e:
                    self.get_logger().error(f"Error stopping worker {i+1}: {e}")

        # Release camera resources
        if hasattr(self, 'cam'):
            self.cam.release()
            
        # Clear buffers
        if hasattr(self, 'frame_buffer'):
            self.frame_buffer.clear()
        if hasattr(self, 'result_buffer'):
            self.result_buffer.clear()
            
        # Log final stats
        if hasattr(self, 'stats'):
            self.get_logger().info(f"Final stats - Processed: {self.stats['processed']}, Dropped: {self.stats['dropped']}")
        
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