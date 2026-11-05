"""
ROS 2 Publisher Module for LSS-02 USV Cockpit.
Handles velocity commands, emergency cutoff, mode switching, and subsystem controls.
"""
import logging
import threading
from typing import Optional

from .ros_connection import ros_connection, RCLPY_AVAILABLE
from .ros_subscriber import ros_subscriber

try:
    from config import settings
except ImportError:
    from ..config import settings

logger = logging.getLogger("ros_publisher")


class ROSPublisher:
    """
    Manages publishers to send actuator commands, subsystem controls,
    and emergency signals to the USV.
    """
    _instance: Optional["ROSPublisher"] = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super(ROSPublisher, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        if hasattr(self, "_initialized") and self._initialized:
            return
        self._initialized = True
        self._lock = threading.Lock()
        self._publishers = {}

    def setup_publishers(self):
        """Initializes ROS 2 publishers if node is available."""
        node = ros_connection.get_node()
        if not node or not RCLPY_AVAILABLE:
            logger.info("ROSPublisher: Running in simulated publisher mode.")
            return

        try:
            from geometry_msgs.msg import Twist
            from std_msgs.msg import Bool, String

            self._publishers["cmd_vel"] = node.create_publisher(Twist, "/cmd_vel", 10)
            self._publishers["mode"] = node.create_publisher(String, "/robot/mode", 10)
            self._publishers["reset_mission"] = node.create_publisher(Bool, "/mission/reset", 10)
            self._publishers["control_gps"] = node.create_publisher(Bool, "/robot/control/gps", 10)
            self._publishers["control_compass"] = node.create_publisher(Bool, "/robot/control/compass", 10)
            self._publishers["control_vision"] = node.create_publisher(Bool, "/robot/control/vision", 10)
            self._publishers["lidar"] = node.create_publisher(Bool, "/robot/lidar", 10)
            self._publishers["emergency"] = node.create_publisher(Bool, "/emergency/disarm", 10)

            logger.info("ROS 2 publishers established successfully.")
        except Exception as e:
            logger.error(f"Failed to setup ROS 2 publishers: {e}")

    def publish_cmd_vel(self, linear_x: float, angular_z: float) -> bool:
        """
        Publishes velocity command (linear surge m/s, angular yaw rad/s).
        Clamps values to configured safety limits.
        """
        # Safety limits check
        clamped_linear = max(0.0, min(float(linear_x), settings.MAX_LINEAR_VELOCITY))
        clamped_angular = max(
            -settings.MAX_ANGULAR_VELOCITY,
            min(float(angular_z), settings.MAX_ANGULAR_VELOCITY)
        )

        # Update telemetry store
        ros_subscriber.update_telemetry_field("navigation", "surge_velocity", round(clamped_linear, 2))
        ros_subscriber.update_telemetry_field("navigation", "yaw_rate", round(clamped_angular, 2))
        ros_subscriber.update_telemetry_field("navigation", "knots", round(clamped_linear * 1.94384, 2))
        
        # Calculate RPM gauges
        port_rpm = int((clamped_linear / 2.0) * 2000)
        ros_subscriber.update_telemetry_field("thrusters", "port_rpm", port_rpm)
        ros_subscriber.update_telemetry_field("thrusters", "stbd_rpm", int(port_rpm * 1.02))
        ros_subscriber.update_telemetry_field("thrusters", "port_percent", min(100, int((port_rpm / 2000) * 100)))
        ros_subscriber.update_telemetry_field("thrusters", "stbd_percent", min(100, int((port_rpm * 1.02 / 2000) * 100)))

        pub = self._publishers.get("cmd_vel")
        if pub and RCLPY_AVAILABLE:
            try:
                from geometry_msgs.msg import Twist
                msg = Twist()
                msg.linear.x = clamped_linear
                msg.angular.z = clamped_angular
                pub.publish(msg)
                return True
            except Exception as e:
                logger.error(f"Error publishing to /cmd_vel: {e}")
                return False

        logger.debug(f"[SIM] Published cmd_vel: surge={clamped_linear} m/s, yaw={clamped_angular} rad/s")
        return True

    def publish_emergency_disarm(self, disarm: bool) -> bool:
        """Triggers immediate thruster cut and emergency disarm."""
        ros_subscriber.update_telemetry_field("", "emergency_armed", disarm)
        if disarm:
            self.publish_cmd_vel(0.0, 0.0)
            ros_subscriber.update_telemetry_field("", "mode", "EMERGENCY_DISARM")

        pub = self._publishers.get("emergency")
        if pub and RCLPY_AVAILABLE:
            try:
                from std_msgs.msg import Bool
                msg = Bool()
                msg.data = bool(disarm)
                pub.publish(msg)
                return True
            except Exception as e:
                logger.error(f"Error publishing emergency disarm: {e}")
                return False
        return True

    def publish_mode(self, mode: str) -> bool:
        """Sets vehicle operating mode (e.g. MANUAL, AUTO_WAYPOINT, DOCKING, HOLD)."""
        ros_subscriber.update_telemetry_field("", "mode", mode)
        pub = self._publishers.get("mode")
        if pub and RCLPY_AVAILABLE:
            try:
                from std_msgs.msg import String
                msg = String()
                msg.data = str(mode)
                pub.publish(msg)
                return True
            except Exception as e:
                logger.error(f"Error publishing mode: {e}")
                return False
        return True

    def toggle_subsystem(self, system_name: str, enabled: bool) -> bool:
        """Toggles USV sensor / control subsystems (gps, compass, vision, lidar)."""
        ros_subscriber.update_telemetry_field("subsystems", system_name, enabled)
        pub_key = f"control_{system_name}" if system_name != "lidar" else "lidar"
        pub = self._publishers.get(pub_key)
        if pub and RCLPY_AVAILABLE:
            try:
                from std_msgs.msg import Bool
                msg = Bool()
                msg.data = bool(enabled)
                pub.publish(msg)
                return True
            except Exception as e:
                logger.error(f"Error publishing subsystem {system_name}: {e}")
                return False
        return True

    def reset_mission(self) -> bool:
        """Publishes reset signal to `/mission/reset`."""
        pub = self._publishers.get("reset_mission")
        if pub and RCLPY_AVAILABLE:
            try:
                from std_msgs.msg import Bool
                msg = Bool()
                msg.data = True
                pub.publish(msg)
                return True
            except Exception as e:
                logger.error(f"Error publishing reset_mission: {e}")
                return False
        return True


ros_publisher = ROSPublisher()
