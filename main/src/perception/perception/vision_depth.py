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
# Same environment fixes visionDepth.py applies: pin OpenCV's Qt backend to xcb so
# the config.debug window opens over X/SSH forwarding instead of failing on backend
# selection, and drop the MSMF video backend priority (Windows-only, noise on Linux).
os.environ['OPENCV_VIDEOIO_PRIORITY_MSMF'] = '0'
os.environ['QT_QPA_PLATFORM'] = 'xcb'

try:
    # pyrefly: ignore [missing-import]
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError:
    REALSENSE_AVAILABLE = False

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
    def __init__(self):
        super().__init__('vision_node')

        # Vision Parameter Declaration
        self.declare_parameter('model_path', f'{ROOT}/models/teknov8s-new.engine')
        self.declare_parameter('config.camera_device', 0)
        self.declare_parameter('config.width', 640)
        self.declare_parameter('config.height', 480)
        self.declare_parameter('config.fps', 30)
        self.declare_parameter('config.debug', False)
        self.declare_parameter('config.use_lidar', True)

        # RealSense D455 Parameter Declaration
        # enable=false falls back to the plain USB camera path (/dev/videoN), so this
        # file still runs on a laptop without a D455 plugged in.
        self.declare_parameter('realsense.enable', True)
        self.declare_parameter('realsense.depth_min', 0.1)          # meters — reject readings below this
        self.declare_parameter('realsense.depth_max', 20.0)         # meters — beyond D455 reliable outdoor range
        self.declare_parameter('realsense.trigger_distance', 2.0)   # meters — obstacle warning threshold
        self.declare_parameter('realsense.depth_patch_half', 4)     # half-size of the median sampling window (px)

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
        self.RS_WIDTH = self.WIDTH
        self.RS_HEIGHT = self.HEIGHT

        # RealSense Parameter Initialization
        self.rs_enable = self.get_parameter('realsense.enable').get_parameter_value().bool_value
        self.rs_depth_min = self.get_parameter('realsense.depth_min').get_parameter_value().double_value
        self.rs_depth_max = self.get_parameter('realsense.depth_max').get_parameter_value().double_value
        self.rs_trigger_dist = self.get_parameter('realsense.trigger_distance').get_parameter_value().double_value
        self.depth_patch_half = self.get_parameter('realsense.depth_patch_half').get_parameter_value().integer_value
        self.rs_depth_scale = 0.001   # meters per depth unit; replaced by the sensor's own scale

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

        # Latest aligned color+depth from the RealSense capture thread.
        self.latest_color_image = None
        self.latest_depth_frame_rs = None   # raw rs.frame for get_distance()
        self.capture_lock = threading.Lock()
        self.closest_distance = -1.0     # meters, -1 = no valid measurement
        self._last_processed_frame_id = None

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

        # Camera Configuration — RealSense D455 first, plain USB camera as fallback
        self.cam = None
        self.use_realsense = False
        if self.rs_enable:
            if not REALSENSE_AVAILABLE:
                self.get_logger().error(
                    "pyrealsense2 not installed — falling back to USB camera (NO depth)")
            else:
                try:
                    self._init_realsense()
                    self.use_realsense = True
                except Exception as e:
                    self.get_logger().error(
                        f"RealSense D455 init failed ({e}) — falling back to USB camera (NO depth)")

        if not self.use_realsense:
            self._init_usb_camera(camera_device)

        # Rate Timer Initialization
        self.loop_timer = self.create_timer(1.0/self.FPS, self.runner)
        self.param_timer = self.create_timer(5.0, self.param_callback)

        # Parameter Setup
        self.load_parameter()

        # Threading Setup
        self.setup_cuda_threading()
        source = "RealSense D455 (depth)" if self.use_realsense else "USB camera (no depth)"
        self.get_logger().info(f"S-VIZ → node started (Jetson CUDA + {source})")

    # Camera sources

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

    def _init_realsense(self):
        self.rs_pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, self.WIDTH, self.HEIGHT, rs.format.bgr8, self.FPS)
        config.enable_stream(rs.stream.depth, self.WIDTH, self.HEIGHT, rs.format.z16, self.FPS)

        try:
            profile = self.rs_pipeline.start(config)
        except Exception:
            # Requested resolution/fps not supported by this unit — let the camera pick.
            self.get_logger().warn(
                f"Could not start D455 @ {self.WIDTH}x{self.HEIGHT} {self.FPS}fps, "
                "retrying with auto-resolution...")
            config2 = rs.config()
            config2.enable_stream(rs.stream.color)
            config2.enable_stream(rs.stream.depth)
            profile = self.rs_pipeline.start(config2)

        # Actual color stream size — bboxes and depth lookups both live in this space
        color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
        self.RS_WIDTH = color_profile.width()
        self.RS_HEIGHT = color_profile.height()

        # Depth units (meters per raw uint16 count)
        depth_sensor = profile.get_device().first_depth_sensor()
        self.rs_depth_scale = depth_sensor.get_depth_scale()

        # Align depth INTO the color frame, so a bbox pixel from YOLO indexes the
        # matching depth pixel directly — without this the two streams are offset by
        # the physical baseline between the sensors.
        self.rs_align = rs.align(rs.stream.color)

        self.get_logger().info(
            f"D455 ready @ {self.RS_WIDTH}x{self.RS_HEIGHT}, "
            f"depth scale {self.rs_depth_scale:.6f} m/unit")

        # Auto-exposure needs a few frames to settle; early frames are dark and depth-less.
        self.get_logger().info("D455 warming up (flushing initial frames)...")
        for _ in range(30):
            self.rs_pipeline.wait_for_frames(timeout_ms=5000)

    def _realsense_capture_thread(self):
        """Pull aligned color+depth off the D455, decoupled from the ROS timer."""
        while self.running:
            try:
                frames = self.rs_pipeline.wait_for_frames(timeout_ms=5000)
            except Exception as e:
                if self.running:
                    self.get_logger().error(
                        f"RealSense capture timeout: {e}", throttle_duration_sec=2.0)
                continue

            aligned = self.rs_align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            # .copy() is required, not cosmetic: np.asanyarray only wraps the memory
            # owned by color_frame, and that buffer returns to the pipeline's frame
            # pool as soon as this iteration ends — the processor would then be
            # reading a buffer the camera is refilling (torn frames, no error).
            # depth_frame is safe to hand over as-is because storing it keeps its
            # own reference alive.
            color_image = np.asanyarray(color_frame.get_data()).copy()

            with self.capture_lock:
                # Store the pair together so the processor always measures depth on
                # the same capture it detected on.
                self.latest_color_image = color_image
                self.latest_depth_frame_rs = depth_frame

    def get_distance_at_center(self, depth_frame_rs, cx_orig, cy_orig):
        try:
            cx = max(0, min(self.RS_WIDTH - 1, int(cx_orig)))
            cy = max(0, min(self.RS_HEIGHT - 1, int(cy_orig)))
            dist_m = depth_frame_rs.get_distance(cx, cy)

            if dist_m <= 0.0 or not (self.rs_depth_min < dist_m < self.rs_depth_max):
                return -1.0
            return float(dist_m)

        except Exception as e:
            self.get_logger().warn(
                f"get_distance failed at ({cx_orig},{cy_orig}): {e}",
                throttle_duration_sec=5.0)
            return -1.0

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

            if self.use_realsense:
                self.capture_thread = threading.Thread(
                    target=self._realsense_capture_thread, daemon=True)
                self.capture_thread.start()
                self.get_logger().info("Started RealSense capture thread")

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

        # RealSense Depth Parameter Initialization (live-retunable every 5s)
        self.rs_depth_min = self.get_parameter('realsense.depth_min').get_parameter_value().double_value
        self.rs_depth_max = self.get_parameter('realsense.depth_max').get_parameter_value().double_value
        self.rs_trigger_dist = self.get_parameter('realsense.trigger_distance').get_parameter_value().double_value
        self.depth_patch_half = self.get_parameter('realsense.depth_patch_half').get_parameter_value().integer_value

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

    def measure_distances(self, depth_frame_rs, scaled_boxes):
        """Measure each detection's distance off the aligned depth map.

        Returns the list of per-box distances (meters, or None where depth is
        unavailable, index-aligned with scaled_boxes) and updates
        self.closest_distance. Drawing is left to BBoxVisualization, which labels
        each box it keeps.
        """
        distances = []
        valid = []

        for x1, y1, x2, y2 in scaled_boxes:
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0

            d = -1.0
            if depth_frame_rs is not None:
                d = self.get_distance_at_center(depth_frame_rs, cx, cy)

            if d > 0:
                distances.append(d)
                valid.append(d)
            else:
                distances.append(None)

        self.closest_distance = min(valid) if valid else -1.0

        if 0 < self.closest_distance <= self.rs_trigger_dist:
            self.get_logger().warn(
                f"S-VIZ → Obstacle at {self.closest_distance:.2f}m "
                f"(trigger={self.rs_trigger_dist}m)", throttle_duration_sec=1.0)

        return distances

    def processor(self, input_frame, depth_frame_rs=None):
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

            # 5. Measure real distance to every detection off the D455 depth map,
            #    before steering, so the overlay can label each buoy with its range.
            distances = self.measure_distances(depth_frame_rs, scaled_boxes)

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
                    distances,
                    self.rs_depth_max
                )
        else:
            objects_detected = False
            self.closest_distance = -1.0

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
        self.depth_distance_pub.publish(Float32(data=float(self.closest_distance)))
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
        depth_frame_rs = None

        if self.use_realsense:
            with self.capture_lock:
                frame = self.latest_color_image
                depth_frame_rs = self.latest_depth_frame_rs

            if frame is None:
                return  # capture thread hasn't produced a frame yet

            # Skip-same-frame guard: don't re-process the same numpy array
            frame_id = id(frame)
            if frame_id == self._last_processed_frame_id:
                return
            self._last_processed_frame_id = frame_id
        else:
            success, frame = self.cam.read()
            if not success:
                self.get_logger().error("Failed to capture frame from camera")
                return

        display_frame = self.processor(frame, depth_frame_rs)

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
        if self.use_realsense and hasattr(self, 'rs_pipeline'):
            try:
                self.rs_pipeline.stop()
                self.get_logger().info("RealSense D455 pipeline stopped")
            except Exception as e:
                self.get_logger().error(f"Error stopping RealSense: {e}")

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
