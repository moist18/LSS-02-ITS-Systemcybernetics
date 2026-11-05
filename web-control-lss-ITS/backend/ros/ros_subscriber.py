"""
ROS 2 Subscriber and Telemetry State Store for LSS-02 USV Cockpit.
Maintains live vehicle state, sensor telemetry, and topic health diagnostics.
"""
import copy
import logging
import math
import random
import threading
import time
from typing import Any, Dict, Optional

from .ros_connection import ros_connection, RCLPY_AVAILABLE

logger = logging.getLogger("ros_subscriber")


class ROSSubscriber:
    """
    Subscribes to ROS 2 topics and buffers incoming telemetry in a thread-safe dict.
    Provides fallback simulation data generation if running without physical hardware.
    """
    _instance: Optional["ROSSubscriber"] = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super(ROSSubscriber, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        if hasattr(self, "_initialized") and self._initialized:
            return
        self._initialized = True
        self._lock = threading.Lock()
        
        # Subscriptions dict
        self._subscriptions = {}
        
        # Topic health tracking (message count, last timestamp, estimated Hz)
        self._topic_stats: Dict[str, Dict[str, Any]] = {
            "/cmd_vel": {"count": 0, "last_time": 0.0, "hz": 0.0, "latency_ms": 12.0},
            "/yolo_detector": {"count": 0, "last_time": 0.0, "hz": 30.0, "latency_ms": 18.0},
            "/odom_filtered": {"count": 0, "last_time": 0.0, "hz": 50.0, "latency_ms": 8.0},
            "/sensors/battery_state": {"count": 0, "last_time": 0.0, "hz": 10.0, "latency_ms": 4.0},
            "/sonar/bathymetry_raw": {"count": 0, "last_time": 0.0, "hz": 20.0, "latency_ms": 15.0},
        }

        # Cached telemetry state matching frontend cockpit widgets
        self._telemetry: Dict[str, Any] = {
            "timestamp": time.time(),
            "emergency_armed": False,
            "mode": "MANUAL",  # MANUAL, AUTO_WAYPOINT, DOCKING, HOLD
            "subsystems": {
                "gps": True,
                "compass": True,
                "vision": True,
                "lidar": True,
            },
            "navigation": {
                "latitude": -7.2825,      # ITS Surabaya Marine Basin baseline
                "longitude": 112.7953,
                "altitude": 1.2,
                "heading_deg": 214.0,     # HDG 214° SW
                "heading_label": "214° SW",
                "surge_velocity": 1.45,   # m/s
                "sway_velocity": 0.02,    # m/s
                "yaw_rate": -0.04,        # rad/s
                "knots": round(1.45 * 1.94384, 2),
                "roll": 0.5,
                "pitch": -0.8,
                "yaw": -2.55,
                "target_surge_limit": 1.45,
            },
            "thrusters": {
                "port_rpm": 1450,
                "stbd_rpm": 1479,
                "port_percent": 72,
                "stbd_percent": 74,
                "power_watts": 385.0,
            },
            "battery": {
                "voltage": 25.2,
                "current": 15.3,
                "percentage": 96.0,
                "temperature": 32.4,
                "type": "LiPo 6S",
                "capacity_ah": 22.0,
                "health": "Optimal",
            },
            "environment": {
                "water_temp_c": 28.4,
                "ambient_temp_c": 31.0,
                "pod_temp_c": 36.2,
                "humidity_percent": 68.5,
                "ph_level": 7.82,
                "cooling_flow_lpm": 3.4,
                "cooling_status": "NORMAL",
            },
            "sonar": {
                "depth_meters": 4.85,
                "frequency_khz": 200,
                "bottom_echo_db": -14.2,
                "profile_status": "CHIRP_LOCKED",
            },
            "detections": {
                "active_target": "Buoy_04",
                "confidence": 0.94,
                "target_distance_m": 12.8,
                "target_bearing_deg": -15.2,
            },
            "mission": {
                "current_id": "NONE",
                "status": "IDLE",
                "progress_percent": 0.0,
                "active_waypoint_index": 0,
                "total_waypoints": 0,
                "distance_to_wp_m": 0.0,
            }
        }

    def setup_subscribers(self):
        """Initializes ROS 2 subscribers if node is available."""
        node = ros_connection.get_node()
        if not node or not RCLPY_AVAILABLE:
            logger.info("ROSSubscriber: Running simulated subscriber loop.")
            return

        try:
            from geometry_msgs.msg import Twist
            from nav_msgs.msg import Odometry
            from sensor_msgs.msg import BatteryState
            from std_msgs.msg import Bool, String

            # Odometry subscription
            self._subscriptions["odom"] = node.create_subscription(
                Odometry,
                "/odom_filtered",
                self._odom_callback,
                10
            )

            # Battery state subscription
            self._subscriptions["battery"] = node.create_subscription(
                BatteryState,
                "/sensors/battery_state",
                self._battery_callback,
                10
            )

            # Robot mode subscription
            self._subscriptions["mode"] = node.create_subscription(
                String,
                "/robot/mode",
                self._mode_callback,
                10
            )

            # Cmd vel monitoring
            self._subscriptions["cmd_vel"] = node.create_subscription(
                Twist,
                "/cmd_vel",
                self._cmd_vel_callback,
                10
            )

            # Mission state
            self._subscriptions["mission"] = node.create_subscription(
                String,
                "/robot/mission",
                self._mission_callback,
                10
            )

            logger.info("ROS 2 subscribers attached successfully.")
        except Exception as e:
            logger.error(f"Failed to setup ROS 2 subscribers: {e}")

    # --- ROS Callbacks ---
    def _update_stats(self, topic: str):
        now = time.time()
        stat = self._topic_stats.get(topic)
        if stat:
            stat["count"] += 1
            dt = now - stat["last_time"]
            if dt > 0:
                inst_hz = 1.0 / dt
                stat["hz"] = round(0.8 * stat["hz"] + 0.2 * inst_hz, 1)
            stat["last_time"] = now

    def _odom_callback(self, msg):
        self._update_stats("/odom_filtered")
        with self._lock:
            x_dot = msg.twist.twist.linear.x
            yaw_rate = msg.twist.twist.angular.z
            nav = self._telemetry["navigation"]
            nav["surge_velocity"] = round(x_dot, 2)
            nav["yaw_rate"] = round(yaw_rate, 2)
            nav["knots"] = round(x_dot * 1.94384, 2)
            
            # Thruster RPM estimation
            port_rpm = int((x_dot / 2.0) * 2000)
            self._telemetry["thrusters"]["port_rpm"] = max(0, port_rpm)
            self._telemetry["thrusters"]["stbd_rpm"] = max(0, int(port_rpm * 1.02))
            self._telemetry["thrusters"]["port_percent"] = min(100, int((port_rpm / 2000) * 100))
            self._telemetry["thrusters"]["stbd_percent"] = min(100, int((port_rpm * 1.02 / 2000) * 100))
            self._telemetry["timestamp"] = time.time()

    def _battery_callback(self, msg):
        self._update_stats("/sensors/battery_state")
        with self._lock:
            bat = self._telemetry["battery"]
            bat["voltage"] = round(float(msg.voltage), 2)
            bat["percentage"] = round(float(msg.percentage * 100.0 if msg.percentage <= 1.0 else msg.percentage), 1)
            bat["current"] = round(float(msg.current), 2)
            self._telemetry["timestamp"] = time.time()

    def _mode_callback(self, msg):
        with self._lock:
            self._telemetry["mode"] = str(msg.data)
            self._telemetry["timestamp"] = time.time()

    def _cmd_vel_callback(self, msg):
        self._update_stats("/cmd_vel")

    def _mission_callback(self, msg):
        with self._lock:
            self._telemetry["mission"]["status"] = str(msg.data)
            self._telemetry["timestamp"] = time.time()

    # --- Simulated Telemetry Updater ---
    def update_simulated_step(self, dt: float = 0.1):
        """Generates realistic slight sensor variations when running without hardware."""
        with self._lock:
            now = time.time()
            self._telemetry["timestamp"] = now
            nav = self._telemetry["navigation"]
            
            # Subtle pitch/roll water oscillations
            t = now * 1.5
            nav["roll"] = round(0.8 * math.sin(t) + 0.2 * math.sin(2.3 * t), 2)
            nav["pitch"] = round(-0.5 * math.cos(t * 0.9) + 0.2 * math.cos(3.1 * t), 2)
            
            # Subtle jitter on depth and sensors
            env = self._telemetry["environment"]
            env["pod_temp_c"] = round(36.0 + 0.5 * math.sin(now * 0.05), 1)
            env["humidity_percent"] = round(68.5 + 0.8 * math.sin(now * 0.02), 1)
            env["ph_level"] = round(7.82 + 0.02 * math.sin(now * 0.03), 2)

            sonar = self._telemetry["sonar"]
            sonar["depth_meters"] = round(4.85 + 0.05 * math.sin(now * 0.2), 2)

    # --- Getters & Setters ---
    def get_telemetry(self) -> Dict[str, Any]:
        """Returns deep copy of current vehicle telemetry."""
        with self._lock:
            return copy.deepcopy(self._telemetry)

    def get_topic_stats(self) -> Dict[str, Dict[str, Any]]:
        """Returns topic health and frequency information."""
        with self._lock:
            return copy.deepcopy(self._topic_stats)

    def update_telemetry_field(self, section: str, key: str, value: Any):
        """Thread-safe update of specific telemetry attributes."""
        with self._lock:
            if section in self._telemetry and isinstance(self._telemetry[section], dict):
                self._telemetry[section][key] = value
            else:
                self._telemetry[key] = value
            self._telemetry["timestamp"] = time.time()


ros_subscriber = ROSSubscriber()
