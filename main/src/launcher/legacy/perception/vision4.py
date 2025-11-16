#!/usr/bin/env python3

import numpy as np
import cv2
import threading
import time
import queue  # Mengganti deque dengan queue
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_services_default
from sensor_msgs.msg import Image
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
from perception.utils.novisual import BBoxVisualization  # Competition
# from perception.utils.visual import BBoxVisualization  # Development

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
        self.declare_parameter('config.fps', 20)
        
        # Detection Parameter Declaration
        self.declare_parameter('vision.detection.image_size', 640)
        self.declare_parameter('vision.detection.confidence', 0.5)
        self.declare_parameter('vision.detection.iou_threshold', 0.5)
        self.declare_parameter('vision.detection.half', False)
        self.declare_parameter('vision.detection.verbose', False)

        # RC Parameter Declaration
        self.declare_parameter('vision.rc.min_value', [1000, 1200])
        self.declare_parameter('vision.rc.mid_value', 1500)
        self.declare_parameter('vision.rc.max_value', [2000, 1800])
        
        # Parameter Setup
        self.load_parameter()
        
        # Camera Parameter Initialization
        camera_device = self.get_parameter('config.camera_device').get_parameter_value().integer_value
        self.WIDTH = self.get_parameter('config.width').get_parameter_value().integer_value
        self.HEIGHT = self.get_parameter('config.height').get_parameter_value().integer_value
        self.FPS = self.get_parameter('config.fps').get_parameter_value().integer_value
        
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
        self.navigation_vision_pub = self.create_publisher(Image, '/robot/vision/navigation', qos_profile_services_default)
        self.surface_image_pub = self.create_publisher(Image, '/robot/image/surface', qos_profile_services_default)
        self.surface_mission_pub = self.create_publisher(Bool, '/robot/mission/surface', qos_profile_services_default)
        self.obj_detection_pub = self.create_publisher(Bool, '/detect/object/bool', qos_profile_services_default)
        self.raw_yaw_pub = self.create_publisher(Int32, '/robot/vision/raw_yaw', qos_profile_services_default)

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

        # Threading and Buffering System Initialization - Mengganti deque dengan queue
        self.frame_queue = queue.Queue(maxsize=10)  # Buffer untuk frame yang akan diproses
        self.result_queue = queue.Queue(maxsize=5)  # Buffer untuk hasil proses
        self.running = True
        self.stats = {'processed': 0, 'dropped': 0}
        
        # Tambahkan variabel untuk menyimpan frame terakhir yang akan ditampilkan
        self.latest_frame_for_display = None
        self.display_lock = threading.Lock()  # Kunci untuk menghindari race condition
        self.last_detection_result = None  # Menyimpan hasil deteksi terakhir
        self.last_raw_yaw = 1500  # Menyimpan nilai raw_yaw terakhir

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
        self.prev_time = time.time()
        self.fps = 0
        self.frame_count = 0  # Untuk perhitungan FPS yang lebih akurat
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
            try:
                # Ambil frame dari antrian (blocking, dengan timeout)
                frame = self.frame_queue.get(timeout=0.1)
                
                # Proses dengan YOLO
                result = model(
                    frame, 
                    imgsz=self.detection_imgsz, 
                    conf=self.detection_conf, 
                    iou=self.detection_iou, 
                    half=self.detection_half, 
                    verbose=self.detection_verbose
                )
                
                # Simpan hasil ke antrian
                self.result_queue.put((frame, result[0]))
                
                # Update frame terakhir untuk ditampilkan (dengan kunci)
                with self.display_lock:
                    # Gambar hasil plot dari YOLO lebih stabil untuk ditampilkan
                    self.latest_frame_for_display = result[0].plot()
                    self.last_detection_result = result[0]
                    
                    # Apply visualization and get raw yaw data
                    if self.vis and hasattr(result[0], 'boxes') and len(result[0].boxes) > 0:
                        _, raw_yaw, _ = self.vis.processor(
                            self.latest_frame_for_display,
                            self.fps,
                            self.mission,
                            self.docking,
                            self.left_buoy,
                            self.right_buoy,
                            result[0].boxes.xyxy.tolist(),
                            result[0].boxes.conf.tolist(),
                            result[0].boxes.cls.tolist(),
                            self.latest_frame_for_display.shape[1],
                            self.latest_frame_for_display.shape[0],
                            self.min_rc_value[0 if self.mission == "Floating Ball" else 1],
                            self.mid_rc_value,
                            self.max_rc_value[0 if self.mission == "Floating Ball" else 1]
                        )
                        self.last_raw_yaw = raw_yaw

                self.frame_queue.task_done()
                self.stats['processed'] += 1
                
            except queue.Empty:
                # Tidak ada frame dalam antrian, lanjutkan
                continue
            except Exception as e:
                self.get_logger().error(f"Worker {worker_id} processing error: {e}")

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
        self.detection_half = self.get_parameter('vision.detection.half').get_parameter_value().bool_value
        self.detection_verbose = self.get_parameter('vision.detection.verbose').get_parameter_value().bool_value

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

    def _handle_surface_imaging(self, img_msg):
        if self.auto_mode:
            if not self.gps_control and not self.compass_control and self.vision_control:
                if self.mission == "Surface Imaging":
                    if self.lidar and not self.is_surface:
                        self.is_surface = True
                        self.surface_image_pub.publish(img_msg)
                        self.get_logger().info("S-VIZ → Surface Imaging!")

    def processor(self, input_frame):
        # Coba masukkan frame ke antrian pekerja
        try:
            # Jika antrian penuh, buang satu frame lama untuk mengurangi lag
            if self.frame_queue.full():
                try:
                    self.frame_queue.get_nowait()
                    self.stats['dropped'] += 1
                except queue.Empty:
                    pass
            self.frame_queue.put_nowait(input_frame)
        except queue.Full:
            self.stats['dropped'] += 1

        # --- LOGIKA TAMPILAN BARU ---
        frame_to_show = None
        objects_detected = False
        
        # Prioritaskan menampilkan hasil olahan jika ada
        with self.display_lock:
            if self.latest_frame_for_display is not None:
                frame_to_show = self.latest_frame_for_display.copy()
                
                # Periksa apakah ada objek terdeteksi
                if self.last_detection_result and hasattr(self.last_detection_result, 'boxes') and len(self.last_detection_result.boxes) > 0:
                    objects_detected = True
        
        # Jika tidak ada frame olahan, gunakan frame mentah
        if frame_to_show is None:
            frame_to_show = input_frame.copy()

        # Tampilkan frame yang sudah dipilih
        cv2.imshow('Vision Cam', frame_to_show)
        cv2.waitKey(1)

        # Konversi dan publikasikan
        img_msg = self.bridge.cv2_to_imgmsg(frame_to_show, 'bgr8')
        self._handle_surface_imaging(img_msg)
        
        # Publikasikan status deteksi dan raw_yaw
        self.obj_detection_pub.publish(Bool(data=objects_detected))
        self.raw_yaw_pub.publish(Int32(data=int(self.last_raw_yaw)))

    def calculate_fps(self):
        self.frame_count += 1
        current_time = time.time()
        
        # Update FPS setiap 1 detik untuk nilai yang lebih stabil
        if current_time - self.fps_update_time >= 1.0:
            self.fps = self.frame_count / (current_time - self.fps_update_time)
            self.frame_count = 0
            self.fps_update_time = current_time
            self.get_logger().info(f"S-VIZ → FPS: {self.fps:.2f}")

    def runner(self):
        # Kontrol FPS agar lebih konsisten
        loop_start_time = time.time()
        
        success, frame = self.cam.read()
        if not success:
            self.get_logger().error("Failed to capture frame from camera")
            return
        
        self.processor(frame)
        self.calculate_fps()
        
        # Publish surface mission status
        self.surface_mission_pub.publish(Bool(data=self.is_surface))
        
        # Hitung waktu yang dibutuhkan dan tidur sebentar jika terlalu cepat
        loop_duration = time.time() - loop_start_time
        target_duration = 1.0 / self.FPS
        sleep_time = target_duration - loop_duration
        if sleep_time > 0:
            time.sleep(sleep_time)

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
        if hasattr(self, 'frame_queue'):
            while not self.frame_queue.empty():
                try:
                    self.frame_queue.get_nowait()
                except queue.Empty:
                    break
                    
        if hasattr(self, 'result_queue'):
            while not self.result_queue.empty():
                try:
                    self.result_queue.get_nowait()
                except queue.Empty:
                    break
            
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