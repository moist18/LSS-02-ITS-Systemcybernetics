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
from std_msgs.msg import String, Bool, Int32, Float32
from cv_bridge import CvBridge

from pathlib import Path
import sys
import os
# Same environment fixes vision_depth.py applies: pin OpenCV's Qt backend to xcb so
# the config.debug window opens over X/SSH forwarding instead of failing on backend
# selection, and drop the MSMF video backend priority (Windows-only, noise on Linux).
os.environ['OPENCV_VIDEOIO_PRIORITY_MSMF'] = '0'
os.environ['QT_QPA_PLATFORM'] = 'xcb'

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

# pyrefly: ignore [missing-import]
from ultralytics import YOLO
# from perception.utils.nonvisualMidpoint import BBoxVisualization    # Competition
from perception.utils.visualMidpoint import BBoxVisualization    # Development

def configure_jetson_cuda():
    """Apply CUDA/TensorRT environment tweaks for Jetson Orin."""
    # Use all available CPU threads (Jetson Orin Nano Super has 6 cores)
    os.environ['OMP_NUM_THREADS'] = '6'
    # Force CUDA device 0
    os.environ['CUDA_VISIBLE_DEVICES'] = '0'
    # Reduce TensorRT rebuild overhead
    os.environ['YOLO_VERBOSE'] = '0'

class VisionNode(Node):
    """Plain USB/webcam twin of vision_depth.py — same YOLO + midpoint steering
    pipeline, no RealSense D455. No depth stream means no per-box distance and no
    obstacle-trigger warning; /robot/vision/depth_distance is still published so
    downstream subscribers (course_2_gps_vision.cpp) keep working, always -1.0
    (the same sentinel vision_depth.py uses for "no valid measurement").
    """

    def __init__(self):
        super().__init__('vision_node')

        # Vision Parameter Declaration
        self.declare_parameter('model_path', f'{ROOT}/models/teknov8s-new.engine')
        self.declare_parameter('config.camera_device', 0)
        self.declare_parameter('config.width', 640)
        self.declare_parameter('config.height', 480)
        self.declare_parameter('config.fps', 30)
        self.declare_parameter('config.debug', True)
        self.declare_parameter('config.use_lidar', True)

        # Detection Parameter Declaration
        self.declare_parameter('vision.detection.image_size', 640)
        self.declare_parameter('vision.detection.confidence', 0.5)
        self.declare_parameter('vision.detection.iou_threshold', 0.5)
        self.declare_parameter('vision.detection.agnostic_nms', False)
        self.declare_parameter('vision.detection.half', False)
        self.declare_parameter('vision.detection.verbose', False)
        self.declare_parameter('vision.detection.min_buoy_area', 480)
        self.declare_parameter('vision.detection.class_names',
                               ['black_buoy', 'green_buoy', 'orange_buoy',
                                'red_buoy', 'yellow_buoy'])

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
        self.left_buoy_sub = self.create_subscription(String, '/detect/left_buoy', self.left_buoy_callback, qos_profile_services_default)
        self.right_buoy_sub = self.create_subscription(String, '/detect/right_buoy', self.right_buoy_callback, qos_profile_services_default)
        self.reset_mission_sub = self.create_subscription(Bool, '/mission/reset', self.reset_mission_callback, qos_profile_services_default)

        # Publisher Initialization
        self.navigation_vision_pub = self.create_publisher(CompressedImage, '/robot/vision/navigation', qos_profile_services_default)
        self.obj_detection_pub = self.create_publisher(Bool, '/detect/object/bool', qos_profile_services_default)
        self.raw_yaw_pub = self.create_publisher(Int32, '/robot/vision/raw_yaw', qos_profile_services_default)
        self.depth_distance_pub = self.create_publisher(Float32, '/robot/vision/depth_distance', qos_profile_services_default)
        # Detection feedback for the planner: max confidence of the colour it asked
        # for on each side. main_course_planner reads the left one to decide whether
        # its kamikaze target is still visible.
        self.left_score_pub = self.create_publisher(Float32, '/detect/left_buoy/score', qos_profile_services_default)
        self.right_score_pub = self.create_publisher(Float32, '/detect/right_buoy/score', qos_profile_services_default)

        # Jetson CUDA Optimization
        configure_jetson_cuda()

        # Load single YOLO model
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.models = [YOLO(model=model_path, task='detect')]

        configured_names = list(
            self.get_parameter('vision.detection.class_names').get_parameter_value().string_array_value
        )
        raw_names = self.models[0].names or {}
        has_proper_names = any(
            isinstance(v, str) and not v.startswith('class')
            for v in raw_names.values()
        )
        self.cls_dict = raw_names if has_proper_names else dict(enumerate(configured_names))
        self.get_logger().info(f"Class names: {self.cls_dict}")

        if model_path.endswith('.engine'):
            self.get_logger().info(f"YOLO TensorRT engine loaded: {model_path}")
        elif model_path.endswith('.pt'):
            try:
                self.models[0].to('cuda')
                self.get_logger().info(f"YOLO PyTorch model loaded on CUDA: {model_path}")
            except Exception as e:
                self.get_logger().warn(f"CUDA not available, using CPU: {e}")
        else:
            self.get_logger().info(f"YOLO model loaded: {model_path}")

        # Threading and Buffering System Initialization
        self.frame_buffer = deque(maxlen=5)
        self.latest_result_scale = (1.0, 1.0)
        self.latest_result = None
        self.result_lock = threading.Lock()

        self.running = True
        self.stats = {'processed': 0, 'dropped': 0}

        # Variable Initialization
        self.bridge = CvBridge()
        self.vis = BBoxVisualization(self.cls_dict, logger=self.get_logger())

        # Control State Variables
        self.auto_mode = True
        self.gps_control = False
        self.compass_control = False
        self.vision_control = False
        self.lidar = False

        # Mission Variables
        self.mission = 'Floating Ball'
        self.left_buoy = 'green_buoy'
        self.right_buoy = 'red_buoy'
        self.fps = 0
        self.frame_count = 0
        self.fps_update_time = time.time()

        # Camera Configuration — plain USB/webcam only, no RealSense/depth
        self.cam = None
        self._init_usb_camera(camera_device)

        # Rate Timer Initialization
        self.loop_timer = self.create_timer(1.0/self.FPS, self.runner)
        self.param_timer = self.create_timer(5.0, self.param_callback)

        # Parameter Setup
        self.load_parameter()

        # Threading Setup
        self.setup_cuda_threading()
        self.get_logger().info("S-VIZ → node started (Jetson CUDA + USB camera, no depth)")

    # Camera source

    def _init_usb_camera(self, camera_device):
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

    def warm_up_models(self):
        self.get_logger().info("Warming up YOLO model on Jetson CUDA...")

        imgsz = getattr(self, 'detection_imgsz', 640)
        dummy_frame = np.zeros((imgsz, imgsz, 3), dtype=np.uint8)
        try:
            _ = self.models[0](dummy_frame, imgsz=imgsz, verbose=False)
        except Exception as e:
            self.get_logger().warn(f"Failed to warm up model: {e}")

        self.get_logger().info("Model warm-up completed!")

    def process_worker(self, worker_id):
        model = self.models[worker_id]

        while self.running:
            if self.frame_buffer:
                try:
                    frame = self.frame_buffer.popleft()

                    orig_h, orig_w = frame.shape[:2]
                    imgsz = self.detection_imgsz

                    # Resize to imgsz for inference
                    frame_resized = cv2.resize(frame, (imgsz, imgsz))

                    result = model(
                        frame_resized,
                        imgsz=imgsz,
                        conf=self.detection_conf,
                        iou=self.detection_iou,
                        agnostic_nms=self.agnostic_nms,
                        half=self.detection_half,
                        verbose=self.detection_verbose
                    )

                    scale_x = orig_w / imgsz
                    scale_y = orig_h / imgsz

                    with self.result_lock:
                        self.latest_result = result[0]
                        self.latest_result_scale = (scale_x, scale_y)
                        self.stats['processed'] += 1

                except IndexError:
                    pass

                except Exception as e:
                    self.get_logger().error(f"Worker {worker_id} processing error: {e}")

            else:
                time.sleep(0.001)

    def setup_cuda_threading(self):
        try:
            self.warm_up_models()

            self.workers = []
            worker = threading.Thread(target=self.process_worker, args=(0,), daemon=True)
            worker.start()
            self.workers.append(worker)
            self.get_logger().info("Started CUDA inference worker thread")

        except Exception as e:
            self.get_logger().error(f"Failed to initialize threading: {e}")
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

    def left_buoy_callback(self, msg):
        self.left_buoy = msg.data

    def right_buoy_callback(self, msg):
        self.right_buoy = msg.data

    def reset_mission_callback(self, msg):
        if msg.data:
            self.mission = ""
            self.get_logger().info("S-VIZ → Successfully Reset Mission!")

    def publish_compressed_image(self, frame, publisher, quality=90, repeat=1):
        """Always async JPEG encoding + publishing to avoid blocking main thread."""
        frame_copy = frame.copy()
        def _publish():
            compressed_msg = CompressedImage()
            compressed_msg.header.stamp = self.get_clock().now().to_msg()
            compressed_msg.format = "jpeg"
            compressed_msg.data = np.array(
                cv2.imencode('.jpg', frame_copy, [cv2.IMWRITE_JPEG_QUALITY, quality])[1]
            ).tobytes()

            if repeat > 1:
                for _ in range(repeat):
                    publisher.publish(compressed_msg)
                    time.sleep(0.1)
            else:
                publisher.publish(compressed_msg)
        threading.Thread(target=_publish, daemon=True).start()

    def processor(self, input_frame):
        # 1. Add raw frame to buffer
        if len(self.frame_buffer) >= self.frame_buffer.maxlen:
            self.frame_buffer.popleft()
            self.stats['dropped'] += 1
        self.frame_buffer.append(input_frame)

        # 2. Copy frame to visualize
        display_frame = input_frame.copy()

        # 3. Default values
        raw_yaw = self.mid_rc_value
        objects_detected = False
        left_score = 0.0
        right_score = 0.0

        with self.result_lock:
            result = self.latest_result
            scale_x, scale_y = getattr(self, 'latest_result_scale', (1.0, 1.0))

        # 4. Process YOLO results
        if result and hasattr(result, 'boxes') and len(result.boxes) > 0:
            objects_detected = True

            # Scale boxes back to original camera resolution
            scaled_boxes = []
            for box in result.boxes.xyxy.tolist():
                x1, y1, x2, y2 = box[:4]
                scaled_boxes.append([
                    x1 * scale_x, y1 * scale_y,
                    x2 * scale_x, y2 * scale_y
                ])

            # No depth stream on a plain webcam — distances/max_range stay at their
            # BBoxVisualization defaults (None), so every box is range-filtered out
            # only by min_buoy_area, same as vision_depth.py with the D455 unplugged.
            if self.vis:
                display_frame, raw_yaw, left_score, right_score = self.vis.processor(
                    display_frame,
                    self.fps,
                    self.mission,
                    self.left_buoy,
                    self.right_buoy,
                    scaled_boxes,
                    result.boxes.conf.tolist(),
                    result.boxes.cls.tolist(),
                    display_frame.shape[1],
                    display_frame.shape[0],
                    self.min_rc_value[0 if self.mission in ("Floating Ball", "Kamikaze") else 1],
                    self.mid_rc_value,
                    self.max_rc_value[0 if self.mission in ("Floating Ball", "Kamikaze") else 1],
                    self.min_buoy_area,
                )
        else:
            objects_detected = False

            if self.vis:
                display_frame, raw_yaw, left_score, right_score = self.vis.processor(
                    display_frame,
                    self.fps,
                    self.mission,
                    self.left_buoy,
                    self.right_buoy,
                    [],
                    [],
                    [],
                    display_frame.shape[1],
                    display_frame.shape[0],
                    self.min_rc_value[0 if self.mission in ("Floating Ball", "Kamikaze") else 1],
                    self.mid_rc_value,
                    self.max_rc_value[0 if self.mission in ("Floating Ball", "Kamikaze") else 1],
                    self.min_buoy_area
                )

        # 6. Publish ROS messages
        self.obj_detection_pub.publish(Bool(data=objects_detected))
        self.raw_yaw_pub.publish(Int32(data=int(raw_yaw)))
        # No depth sensor on this variant — same "no valid measurement" sentinel
        # vision_depth.py uses, so course_2_gps_vision.cpp's closest-distance log
        # still means the same thing (-1.0 = unknown) instead of a stale/fake reading.
        self.depth_distance_pub.publish(Float32(data=-1.0))
        self.left_score_pub.publish(Float32(data=float(left_score)))
        self.right_score_pub.publish(Float32(data=float(right_score)))

        # 7. Publish image topic asynchronously
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

        # Show cv2 window if debug enabled
        if self.DEBUG:
            cv2.imshow('Vision Cam', display_frame)
            cv2.waitKey(1)

        # Calculate FPS
        self.calculate_fps()

    def destroy_node(self):
        self.get_logger().info("Shutting down Jetson CUDA vision node...")

        # Stop worker threads
        self.running = False

        if hasattr(self, 'workers'):
            for i, worker in enumerate(self.workers):
                try:
                    worker.join(timeout=2.0)
                except Exception as e:
                    self.get_logger().error(f"Error stopping worker {i+1}: {e}")

        # Release camera resources
        if self.cam is not None:
            self.cam.release()

        if hasattr(self, 'frame_buffer'):
            self.frame_buffer.clear()

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
