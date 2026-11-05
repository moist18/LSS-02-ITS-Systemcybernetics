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
from std_msgs.msg import Float32
from cv_bridge import CvBridge

from pathlib import Path
import sys
import os
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
from perception.utils.nonvisualPGF import BBoxVisualization    # Competition
# from perception.utils.visualPGF import BBoxVisualization    # Development

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
        # self.declare_parameter('model_path', f'{ROOT}/models/best.engine')  
        self.declare_parameter('model_path', f'{ROOT}/models/teknov8s-new.engine')      
        # self.declare_parameter('model_path', f'{ROOT}/models/yolov8n.pt')
        self.declare_parameter('config.camera_device', 0)
        self.declare_parameter('config.width', 640)
        self.declare_parameter('config.height', 480)
        self.declare_parameter('config.fps', 30)
        self.declare_parameter('config.debug', False)
        self.declare_parameter('config.use_lidar', True)

        # RealSense D455 Parameter Declaration
        self.declare_parameter('realsense.depth_min', 0.1)   # meters
        self.declare_parameter('realsense.depth_max', 20.0)  # meters
        self.declare_parameter('realsense.trigger_distance', 2.0)  # meters - obstacle alert
        
        # Detection Parameter Declaration
        self.declare_parameter('vision.detection.image_size', 416)  
        self.declare_parameter('vision.detection.confidence', 0.5)
        self.declare_parameter('vision.detection.iou_threshold', 0.5)
        self.declare_parameter('vision.detection.agnostic_nms', False)
        self.declare_parameter('vision.detection.half', False)
        self.declare_parameter('vision.detection.verbose', False)
        self.declare_parameter('vision.detection.min_buoy_area', 480)
        # Class names in class-id order. Raw TensorRT engines carry no name metadata,
        # so this MUST match the engine that model_path points at (Tekno.engine = 5 classes).
        self.declare_parameter('vision.detection.class_names',
                               ['black_buoy', 'green_buoy', 'orange_buoy',
                                'red_buoy', 'yellow_buoy'])
        self.declare_parameter('vision.detection.use_bytetrack', True)

        # Active course selects Floating Ball steering logic in BBoxVisualization:
        # 1 = orange-buoy gate centerline, 2 = PGF gap-finder, 3 = Kamikaze single-color ram
        self.declare_parameter('vision.course', 2)

        # PGF gap-scoring tuning (see BBoxVisualization._score_gap in
        # visualPGF.py / nonvisualPGF.py). Penalises gaps not flanked by a
        # real buoy on both sides (e.g. open water outside the arena), and
        # mildly biases toward gaps centred straight ahead.
        self.declare_parameter('vision.pgf.unbounded_gap_penalty', 400.0)
        self.declare_parameter('vision.pgf.heading_bias_weight', 2.0)

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
        self.left_score_pub = self.create_publisher(Float32, '/detect/left_buoy/score', qos_profile_services_default)
        self.right_score_pub = self.create_publisher(Float32, '/detect/right_buoy/score', qos_profile_services_default)

        # Jetson CUDA Optimization
        configure_jetson_cuda()

        # Load single YOLO model — Jetson CUDA handles parallelism internally
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.models = [YOLO(model=model_path, task='detect')]

        # Raw TensorRT engines carry no class-name metadata, so ultralytics hands back
        # generic {0:'class0', 1:'class1', ...}. Fall back to the configured names, whose
        # list order is the class-id order — visualPGF matches buoys BY NAME, so a list
        # that doesn't match the loaded engine silently mislabels every detection.
        configured_names = list(
            self.get_parameter('vision.detection.class_names').get_parameter_value().string_array_value
        )
        raw_names = self.models[0].names or {}
        # Check if model already has proper (non-generic) class names
        has_proper_names = any(
            isinstance(v, str) and not v.startswith('class')
            for v in raw_names.values()
        )
        self.cls_dict = raw_names if has_proper_names else dict(enumerate(configured_names))
        self.get_logger().info(f"Class names: {self.cls_dict}")

        # TensorRT engine (.engine) runs on GPU automatically, no .to('cuda') needed
        if model_path.endswith('.engine'):
            self.get_logger().info(f"YOLO TensorRT engine loaded: {model_path}")
        elif model_path.endswith('.pt'):
            # Only move PyTorch models to CUDA
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

        # Latest aligned color+depth dari RealSense capture thread
        self.latest_color_image = None
        self.latest_depth_frame_rs = None  # raw rs.frame untuk get_distance()
        self.capture_lock = threading.Lock()
        self.closest_distance = -1.0  # meters, -1 = invalid
        self.running = True
        self.stats = {'processed': 0, 'dropped': 0}

        #  ByteTrack velocity estimator state
        self.track_history       = {}
        self.track_history_lock  = threading.Lock()
        self.TRACK_HISTORY_LEN   = 8   
        self.VELOCITY_MIN_FRAMES = 2   

        # Variable Initialization
        self.bridge = CvBridge()
        self.vis = BBoxVisualization(self.cls_dict)

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

        # Same-frame skip: track the id() of the last processed color array
        self._last_processed_frame_id = None

        # DEBUG timing accumulators (reset every second)
        self._dbg_t = {}
        self._dbg_count = 0
        self._dbg_reset = time.time()

        # RealSense D455 Depth Parameters
        self.rs_depth_min = self.get_parameter('realsense.depth_min').get_parameter_value().double_value
        self.rs_depth_max = self.get_parameter('realsense.depth_max').get_parameter_value().double_value
        self.rs_trigger_dist = self.get_parameter('realsense.trigger_distance').get_parameter_value().double_value

        # RealSense Pipeline Initialization
        self._init_realsense()

        # Rate Timer Initialization
        self.loop_timer = self.create_timer(1.0/self.FPS, self.runner)
        self.param_timer = self.create_timer(5.0, self.param_callback)
        
        # Parameter Setup
        self.load_parameter()

        # Threading Setup
        self.setup_cuda_threading()
        self.get_logger().info("S-VIZ → node started (Jetson Orin + RealSense D455 + ByteTrack MOT)")

        if self.DEBUG:
            cv2.namedWindow('D455 Vision', cv2.WINDOW_NORMAL)
            dummy = np.zeros((640, 480, 3), dtype=np.uint8)
            cv2.imshow('D455 Vision', dummy)
            cv2.waitKey(1)


    # RealSense D455 helpers
    def _init_realsense(self):
        if not REALSENSE_AVAILABLE:
            self.get_logger().error("pyrealsense2 not installed – cannot use D455!")
            raise RuntimeError("pyrealsense2 not available")

        self.rs_pipeline = rs.pipeline()
        config = rs.config()

        # Let RealSense auto-select the best supported resolution/fps
        config.enable_stream(rs.stream.color, self.WIDTH, self.HEIGHT,
                             rs.format.bgr8, self.FPS)
        config.enable_stream(rs.stream.depth, self.WIDTH, self.HEIGHT,
                             rs.format.z16, self.FPS)

        try:
            profile = self.rs_pipeline.start(config)
        except Exception:
            # Fallback: let camera choose resolution automatically
            self.get_logger().warn(
                f"Could not start D455 @ {self.WIDTH}x{self.HEIGHT} {self.FPS}fps, "
                "retrying with auto-resolution...")
            config2 = rs.config()
            config2.enable_stream(rs.stream.color)
            config2.enable_stream(rs.stream.depth)
            profile = self.rs_pipeline.start(config2)

        # Read actual stream resolution
        color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
        self.RS_WIDTH  = color_profile.width()
        self.RS_HEIGHT = color_profile.height()
        self.get_logger().info(
            f"D455 color stream: {self.RS_WIDTH}x{self.RS_HEIGHT}")

        # Depth sensor scale
        depth_sensor = profile.get_device().first_depth_sensor()
        self.rs_depth_scale = depth_sensor.get_depth_scale()
        self.get_logger().info(f"D455 depth scale: {self.rs_depth_scale:.6f} m/unit")

        # Store depth intrinsics for accurate deprojection
        depth_profile = profile.get_stream(rs.stream.depth).as_video_stream_profile()
        self.rs_depth_intrinsics = depth_profile.get_intrinsics()
        self.get_logger().info(
            f"D455 depth intrinsics: fx={self.rs_depth_intrinsics.fx:.2f}, "
            f"fy={self.rs_depth_intrinsics.fy:.2f}, "
            f"ppx={self.rs_depth_intrinsics.ppx:.2f}, "
            f"ppy={self.rs_depth_intrinsics.ppy:.2f}")

        # Feed real intrinsics to PGF so buoy angle/distance use the aligned
        # depth stream instead of the fallback monocular bbox-width estimate.
        if self.vis:
            self.vis.set_depth_intrinsics({
                'fx': self.rs_depth_intrinsics.fx,
                'fy': self.rs_depth_intrinsics.fy,
                'ppx': self.rs_depth_intrinsics.ppx,
                'ppy': self.rs_depth_intrinsics.ppy,
            })

        # Align depth to color frame
        self.rs_align = rs.align(rs.stream.color)
        self.get_logger().info("D455 warming up (flushing initial frames)...")
        for _ in range(30):
            self.rs_pipeline.wait_for_frames(timeout_ms=5000)

        self.get_logger().info(
            f"RealSense D455 ready @ {self.RS_WIDTH}x{self.RS_HEIGHT} {self.FPS}fps")

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

        # Use detection_imgsz (matches TRT engine build size, e.g. 416x416)
        imgsz = getattr(self, 'detection_imgsz', 640)
        dummy_frame = np.zeros((imgsz, imgsz, 3), dtype=np.uint8)
        self.get_logger().info(f"Warm-up dummy frame size: {imgsz}x{imgsz}")

        for i, model in enumerate(self.models):
            try:
                self.get_logger().info(f"Warming up model {i+1}/{len(self.models)}")
                _ = model(dummy_frame, imgsz=imgsz, verbose=False)
            except Exception as e:
                self.get_logger().warn(f"Failed to warm up model {i+1}: {e}")

        self.get_logger().info("CUDA model warm-up completed!")

    # ByteTrack: velocity estimator 

    def _update_velocities(self, track_ids, boxes_xyxy):
        """
        Update centroid history per tracked object and compute average
        velocity vectors (pixels/frame) for all active tracks.

        Returns:
            dict {track_id: (vx, vy)} — pixel displacement per frame.
        """
        velocities = {}
        with self.track_history_lock:
            active_ids = set()
            for tid, bb in zip(track_ids, boxes_xyxy):
                if tid is None:
                    continue
                tid = int(tid)
                active_ids.add(tid)
                x1, y1, x2, y2 = map(float, bb)
                cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
                if tid not in self.track_history:
                    self.track_history[tid] = deque(maxlen=self.TRACK_HISTORY_LEN)
                self.track_history[tid].append((cx, cy))

            for tid in active_ids:
                hist = self.track_history[tid]
                if len(hist) >= self.VELOCITY_MIN_FRAMES:
                    pts = list(hist)
                    vx = np.mean([pts[i+1][0] - pts[i][0] for i in range(len(pts)-1)])
                    vy = np.mean([pts[i+1][1] - pts[i][1] for i in range(len(pts)-1)])
                    velocities[tid] = (float(vx), float(vy))

            # Prune stale IDs no longer detected
            for k in [k for k in self.track_history if k not in active_ids]:
                del self.track_history[k]
        return velocities

    def process_worker(self, worker_id):
        model = self.models[worker_id]

        while self.running:
            if self.frame_buffer:
                try:
                    frame = self.frame_buffer.popleft()   # BGR uint8, resolusi asli RS

                    orig_h, orig_w = frame.shape[:2]
                    imgsz = self.detection_imgsz  # 416

                    # Resize ke 416x416 untuk inferensi (kunci 30 FPS)
                    frame_resized = cv2.resize(frame, (imgsz, imgsz))

                    # ByteTrack: model.track() with persist=True. Disable it via config to fall
                    # back to plain detection — tracking needs a few frames to confirm a new
                    # track, so newly appeared buoys show up later than with predict alone.
                    if self.use_bytetrack:
                        try:
                            result = model.track(
                                frame_resized,
                                imgsz=imgsz,
                                conf=self.detection_conf,
                                iou=self.detection_iou,
                                agnostic_nms=self.agnostic_nms,
                                half=self.detection_half,
                                verbose=self.detection_verbose,
                                tracker="bytetrack.yaml",
                                persist=True
                            )
                        except Exception as e:
                            # Don't swallow this silently — tracking dying unnoticed used to look
                            # like a detection problem.
                            self.get_logger().warn(
                                f"ByteTrack failed, falling back to plain detection: {e}",
                                throttle_duration_sec=5.0)
                            result = None
                    else:
                        result = None

                    if result is None:
                        result = model(
                            frame_resized,
                            imgsz=imgsz,
                            conf=self.detection_conf,
                            iou=self.detection_iou,
                            agnostic_nms=self.agnostic_nms,
                            half=self.detection_half,
                            verbose=self.detection_verbose
                        )

                    # Simpan skala agar processor bisa memetakan bbox → resolusi asli
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

    def _realsense_capture_thread(self):
        while self.running:
            try:
                frames = self.rs_pipeline.wait_for_frames(timeout_ms=5000)
            except Exception as e:
                if self.running:
                    self.get_logger().error(
                        f"RealSense capture timeout: {e}",
                        throttle_duration_sec=2.0)
                continue

            aligned = self.rs_align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            # Langsung ambil data tanpa filter (sesuai visionTes_30.py)
            color_image = np.asanyarray(color_frame.get_data())

            with self.capture_lock:
                self.latest_color_image = color_image
                self.latest_depth_frame_rs = depth_frame

    def setup_cuda_threading(self):
        """Warm up CUDA model, start inference worker + RealSense capture threads."""
        try:
            # Warm up CUDA model to reduce cold-start latency
            self.warm_up_models()

            # Single YOLO inference worker thread
            self.workers = []
            worker = threading.Thread(target=self.process_worker, args=(0,), daemon=True)
            worker.start()
            self.workers.append(worker)
            self.get_logger().info("Started CUDA inference worker thread")

            # Dedicated RealSense capture thread — decouples camera from ROS timer
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
        self.use_bytetrack = self.get_parameter('vision.detection.use_bytetrack').get_parameter_value().bool_value
        self.course = self.get_parameter('vision.course').get_parameter_value().integer_value

        # PGF gap-scoring tuning — pushed into BBoxVisualization so it can be
        # retuned live (param_callback reloads every 5s) without restarting.
        self.vis.pgf_unbounded_gap_penalty = self.get_parameter('vision.pgf.unbounded_gap_penalty').get_parameter_value().double_value
        self.vis.pgf_heading_bias_weight = self.get_parameter('vision.pgf.heading_bias_weight').get_parameter_value().double_value

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
        """Always async: JPEG encode + publish in a background thread to avoid
        blocking the ROS timer callback (encode is ~15-20ms on Jetson CPU)."""
        frame_copy = frame.copy()  # safe snapshot for background thread
        def _publish():
            compressed_msg = CompressedImage()
            compressed_msg.header.stamp = self.get_clock().now().to_msg()
            compressed_msg.format = "jpeg"
            compressed_msg.data = np.array(
                cv2.imencode('.jpg', frame_copy,
                             [cv2.IMWRITE_JPEG_QUALITY, quality])[1]).tobytes()
            if repeat > 1:
                for _ in range(repeat):
                    publisher.publish(compressed_msg)
                    time.sleep(0.1)
            else:
                publisher.publish(compressed_msg)
        threading.Thread(target=_publish, daemon=True).start()

    def _dbg_mark(self, key):
        """Record a timing checkpoint for debug profiling."""
        self._dbg_t[key] = time.perf_counter()

    def _dbg_log(self):
        """Every second, log ms cost of each pipeline stage."""
        self._dbg_count += 1
        now = time.perf_counter()
        if now - self._dbg_reset >= 1.0:
            t = self._dbg_t
            def ms(a, b): return f"{(t.get(b,0)-t.get(a,0))*1000:.1f}ms" if a in t and b in t else "?"
            self.get_logger().info(
                f"[TIMING] copy={ms('t0','t1')} vis={ms('t1','t2')} "
                f"depth={ms('t2','t3')} pub={ms('t3','t4')} "
                f"imshow={ms('t4','t5')} total={ms('t0','t5')} "
                f"calls={self._dbg_count}")
            self._dbg_count = 0
            self._dbg_reset = now

    def processor(self, input_frame):
        self._dbg_mark('t0')

        # 1. Tambah frame asli ke buffer untuk worker thread
        if len(self.frame_buffer) >= self.frame_buffer.maxlen:
            self.frame_buffer.popleft()
            self.stats['dropped'] += 1
        self.frame_buffer.append(input_frame)

        # 2. Siapkan frame display (resolusi asli)
        display_frame = input_frame.copy()
        self._dbg_mark('t1')

        # 3. Inisialisasi default values
        raw_yaw = self.mid_rc_value
        objects_detected = False

        with self.result_lock:
            result = self.latest_result
            # Ambil skala pemetaan bbox (416→asli) yang disimpan oleh process_worker
            scale_x, scale_y = getattr(self, 'latest_result_scale', (1.0, 1.0))

        # 4. Extract ByteTrack IDs and compute velocity vectors
        track_ids  = []
        velocities = {}
        scaled_boxes = []

        if result and hasattr(result, 'boxes') and len(result.boxes) > 0:
            plot_frame = display_frame
            objects_detected = True

            # Petakan koordinat bbox dari ruang YOLO (416x416) ke resolusi asli
            orig_h, orig_w = display_frame.shape[:2]
            for box in result.boxes.xyxy.tolist():
                x1, y1, x2, y2 = box[:4]
                scaled_boxes.append([
                    x1 * scale_x, y1 * scale_y,
                    x2 * scale_x, y2 * scale_y
                ])

            # Extract ByteTrack tracking IDs
            if result.boxes.id is not None:
                track_ids = [int(tid) for tid in result.boxes.id.tolist()]
                # Velocity only means anything with persistent IDs — index-based
                # placeholders would difference two unrelated objects frame to frame.
                velocities = self._update_velocities(track_ids, scaled_boxes)
            else:
                # Tracking off (or unavailable): sequential placeholder IDs, no velocity.
                track_ids = list(range(len(scaled_boxes)))
                velocities = {}
                with self.track_history_lock:
                    self.track_history.clear()

            # 5. Ukur jarak setiap objek dengan RealSense depth_frame.get_distance()
            with self.capture_lock:
                depth_frame_rs = self.latest_depth_frame_rs

            # Raw depth map (aligned to color) for PGF's per-pixel distance sampling.
            depth_image_np = np.asanyarray(depth_frame_rs.get_data()) if depth_frame_rs is not None else None

            distances_for_vis = []
            valid_distances = []
            for box_orig in scaled_boxes:
                x1, y1, x2, y2 = box_orig
                cx_orig = (x1 + x2) / 2
                cy_orig = (y1 + y2) / 2

                d = -1.0
                if depth_frame_rs is not None:
                    d = self.get_distance_at_center(depth_frame_rs, cx_orig, cy_orig)

                if d > 0:
                    distances_for_vis.append(d)
                    valid_distances.append(d)
                else:
                    distances_for_vis.append(None)

            self.closest_distance = min(valid_distances) if valid_distances else -1.0

            if 0 < self.closest_distance <= self.rs_trigger_dist:
                self.get_logger().warn(
                    f"S-VIZ → Obstacle at {self.closest_distance:.2f}m "
                    f"(trigger={self.rs_trigger_dist}m)",
                    throttle_duration_sec=1.0)

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
                    orig_w,
                    orig_h,
                    self.min_rc_value[0 if self.mission in ("Floating Ball", "Kamikaze") else 1],
                    self.mid_rc_value,
                    self.max_rc_value[0 if self.mission in ("Floating Ball", "Kamikaze") else 1],
                    self.min_buoy_area,
                    track_ids,
                    velocities,
                    distances_for_vis,
                    course=self.course,
                    depth_image=depth_image_np,
                    depth_scale=self.rs_depth_scale
                )
            self._dbg_mark('t2')
            self._dbg_mark('t3')

        else:
            plot_frame = display_frame
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
                    self.min_buoy_area,
                    [],     # No track IDs
                    {},     # No velocities
                    [],     # No distances
                    course=self.course
                )
            self._dbg_mark('t2')
            self._dbg_mark('t3')  # no depth loop when no detections

        # 6. Publish ROS data
        self.obj_detection_pub.publish(Bool(data=objects_detected))
        self.raw_yaw_pub.publish(Int32(data=int(raw_yaw)))
        self.depth_distance_pub.publish(Float32(data=float(self.closest_distance)))
        self.left_score_pub.publish(Float32(data=float(left_score)))
        self.right_score_pub.publish(Float32(data=float(right_score)))

        # 7. Publish compressed navigation vision streaming (async — non-blocking)
        self.publish_compressed_image(plot_frame, self.navigation_vision_pub, quality=80)
        self._dbg_mark('t4')

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
        """ROS timer callback: grab the latest pre-captured frame (non-blocking)."""
        with self.capture_lock:
            color_image = self.latest_color_image

        if color_image is None:
            return  # capture thread hasn't produced a frame yet

        # Skip-same-frame guard: don't re-process the same numpy array
        frame_id = id(color_image)
        if frame_id == self._last_processed_frame_id:
            return
        self._last_processed_frame_id = frame_id

        display_frame = self.processor(color_image)

        # DEBUG: tampilkan display frame langsung (tanpa hstack depth agar ringan)
        if self.DEBUG:
            cv2.imshow('D455 Vision', display_frame)
            cv2.waitKey(1)
        self._dbg_mark('t5')
        self._dbg_log()

        # Calculate FPS
        self.calculate_fps()

    def destroy_node(self):
        self.get_logger().info("Shutting down vision node (Jetson Orin + D455)...")

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

        # Stop RealSense pipeline
        if hasattr(self, 'rs_pipeline'):
            try:
                self.rs_pipeline.stop()
                self.get_logger().info("RealSense D455 pipeline stopped")
            except Exception as e:
                self.get_logger().error(f"Error stopping RealSense: {e}")

        # Clear buffers
        if hasattr(self, 'frame_buffer'):
            self.frame_buffer.clear()

        # Log final stats
        if hasattr(self, 'stats'):
            self.get_logger().info(
                f"Final stats - Processed: {self.stats['processed']}, "
                f"Dropped: {self.stats['dropped']}")

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
        try:
            rclpy.shutdown()
        except Exception:
            pass  

if __name__ == "__main__":
    main()