#!/usr/bin/env python3

import numpy as np
import cv2
import threading
import time
import os
from collections import deque
from pathlib import Path
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_services_default
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Int32
from cv_bridge import CvBridge

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

from ultralytics import YOLO
from perception.utils.visual import BBoxVisualization

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
        self.declare_parameter('config.debug', True)

        # Recorder Parameter Declaration
        self.declare_parameter('video_dir', '/home/ubuntu/ASV-2025/recordings')
        self.declare_parameter('config.codec', 'MJPG')

        # Detection Parameter Declaration
        self.declare_parameter('vision.detection.image_size', 640)
        self.declare_parameter('vision.detection.confidence', 0.5)
        self.declare_parameter('vision.detection.iou_threshold', 0.5)
        self.declare_parameter('vision.detection.half', False)
        self.declare_parameter('vision.detection.verbose', False)

        # RC Parameter Declaration
        self.declare_parameter('vision.rc.min_value', [1000, 1300])
        self.declare_parameter('vision.rc.mid_value', 1500)
        self.declare_parameter('vision.rc.max_value', [2000, 1700])

        # Parameter Setup
        self.load_parameter()

        # Camera Parameter Initialization
        camera_device = self.get_parameter('config.camera_device').get_parameter_value().integer_value
        self.WIDTH = self.get_parameter('config.width').get_parameter_value().integer_value
        self.HEIGHT = self.get_parameter('config.height').get_parameter_value().integer_value
        self.FPS = self.get_parameter('config.fps').get_parameter_value().integer_value
        self.DEBUG = self.get_parameter('config.debug').get_parameter_value().bool_value

        # Recorder Initialization
        video_dir_param = self.get_parameter('video_dir').get_parameter_value().string_value
        self.video_dir = Path(video_dir_param).expanduser()
        codec_str = self.get_parameter('config.codec').get_parameter_value().string_value
        self.fourcc = cv2.VideoWriter_fourcc(*codec_str)
        self.video_index = 1
        self.recording_frame_count = 0

        # Create video directory
        os.makedirs(self.video_dir, exist_ok=True)
        self.get_logger().info(f"📂 Video directory: {self.video_dir}")

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
        self.reset_mission_sub = self.create_subscription(Bool, '/robot/reset', self.reset_mission_callback, qos_profile_services_default)

        # Publisher Initialization
        self.yaw_pub = self.create_publisher(Int32, '/robot/vision/yaw', qos_profile_services_default)
        self.objects_detected_pub = self.create_publisher(Bool, '/robot/vision/objects_detected', qos_profile_services_default)
        self.surface_image_pub = self.create_publisher(Image, '/robot/image/surface', qos_profile_services_default)

        # Vision Model Initialization
        optimize_rknn_model()

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
        self.lidar = False
        self.is_surface = False

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

        # Video Writer Initialization
        self.video_writer = self.init_video_writer()

        if not self.video_writer.isOpened():
            self.get_logger().error("Failed to create video writer")
            raise RuntimeError("VideoWriter initialization failed")

        # Model Warm-up
        self.get_logger().info("Warming up models...")
        dummy_frame = np.zeros((self.HEIGHT, self.WIDTH, 3), dtype=np.uint8)
        for i, model in enumerate(self.models):
            model(dummy_frame, imgsz=self.detection_imgsz, conf=self.detection_conf, verbose=False)
            self.get_logger().info(f"Model {i+1} warmed up")

        self.get_logger().info("Model warm-up completed!")

        # Start worker threads
        self.setup_rknn_threading()

        # Timer for camera capture and processing
        self.loop_timer = self.create_timer(1.0/self.FPS, self.timer_callback)

        self.get_logger().info("Vision Recorder Node initialized successfully!")

    def init_video_writer(self):
        while os.path.exists(os.path.join(self.video_dir, f'output{self.video_index}.avi')):
            self.video_index += 1

        video_path = os.path.join(self.video_dir, f'output{self.video_index}.avi')
        self.get_logger().info(f"🎥 Recording to: {video_path}")

        writer = cv2.VideoWriter(str(video_path), self.fourcc, self.FPS, (self.WIDTH, self.HEIGHT))

        if not writer.isOpened():
            codec_name = ''.join([chr((self.fourcc >> 8 * i) & 0xFF) for i in range(4)])
            self.get_logger().error(f"Failed to open video writer for {video_path}")
            self.get_logger().error(f"   Codec: {codec_name}, FPS: {self.FPS}, Size: {self.WIDTH}x{self.HEIGHT}")

        return writer

    def load_parameter(self):
        self.detection_imgsz = self.get_parameter('vision.detection.image_size').get_parameter_value().integer_value
        self.detection_conf = self.get_parameter('vision.detection.confidence').get_parameter_value().double_value
        self.detection_iou = self.get_parameter('vision.detection.iou_threshold').get_parameter_value().double_value
        self.detection_half = self.get_parameter('vision.detection.half').get_parameter_value().bool_value
        self.detection_verbose = self.get_parameter('vision.detection.verbose').get_parameter_value().bool_value

        self.min_rc_value = self.get_parameter('vision.rc.min_value').get_parameter_value().integer_array_value
        self.mid_rc_value = self.get_parameter('vision.rc.mid_value').get_parameter_value().integer_value
        self.max_rc_value = self.get_parameter('vision.rc.max_value').get_parameter_value().integer_array_value

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
            for i in range(len(self.models)):
                worker_thread = threading.Thread(target=self.process_worker, args=(i,), daemon=True)
                worker_thread.start()
                self.get_logger().info(f"Started worker thread {i}")
        except Exception as e:
            self.get_logger().error(f"Failed to start worker threads: {e}")

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
            self.get_logger().info("VIZ-REC → Successfully Reset Mission!")

    def _handle_surface_imaging(self, img_msg):
        if self.auto_mode:
            if not self.gps_control and not self.compass_control and self.vision_control:
                if self.mission == "Surface Imaging":
                    if self.lidar and not self.is_surface:
                        self.is_surface = True
                        self.surface_image_pub.publish(img_msg)
                        self.get_logger().info("VIZ-REC → Surface Imaging!")

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
            objects_detected = True

            if self.vis:
                display_frame, raw_yaw, _ = self.vis.processor(
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
                    self.max_rc_value[0 if self.mission == "Floating Ball" else 1]
                )
        else:
            objects_detected = False

            if self.vis:
                display_frame, raw_yaw, _ = self.vis.processor(
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
                    self.max_rc_value[0 if self.mission == "Floating Ball" else 1]
                )

        # 5. FPS calculation and overlay
        self.frame_count += 1
        current_time = time.time()
        elapsed = current_time - self.fps_update_time

        if elapsed >= 1.0:
            self.fps = self.frame_count / elapsed
            self.frame_count = 0
            self.fps_update_time = current_time

        cv2.putText(display_frame, f'FPS: {int(self.fps)}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # 6. Publish yaw and objects_detected
        yaw_msg = Int32()
        yaw_msg.data = raw_yaw
        self.yaw_pub.publish(yaw_msg)

        objects_detected_msg = Bool()
        objects_detected_msg.data = objects_detected
        self.objects_detected_pub.publish(objects_detected_msg)

        # 7. Handle surface imaging
        img_msg = self.bridge.cv2_to_imgmsg(input_frame, encoding='bgr8')
        self._handle_surface_imaging(img_msg)

        # 8. Display frame (if debug enabled)
        if self.DEBUG:
            cv2.imshow('Vision Cam', display_frame)
            cv2.waitKey(1)

        # 9. Record frame (annotated with detection)
        if self.video_writer.isOpened():
            self.video_writer.write(display_frame)
            self.recording_frame_count += 1

            if self.recording_frame_count % 100 == 0:
                self.get_logger().info(f"📹 Recorded {self.recording_frame_count} frames")

    def timer_callback(self):
        success, frame = self.cam.read()

        if success:
            self.processor(frame)
        else:
            self.get_logger().error("Failed to capture frame from camera", throttle_duration_sec=5)

    def destroy_node(self):
        self.get_logger().info("Shutting down Vision Recorder Node...")
        self.get_logger().info(f"📊 Final stats: {self.recording_frame_count} frames recorded")

        self.running = False

        if self.cam.isOpened():
            self.cam.release()

        if self.video_writer.isOpened():
            self.video_writer.release()
            self.get_logger().info(f"✅ Video saved: output{self.video_index}.avi")

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
