"""
ROS 2 Parameter Management Module for LSS-02 USV.
Allows runtime reading, updating, and synchronizing of navigation and safety parameters.
"""
import copy
import logging
import threading
from typing import Any, Dict, Optional

from .ros_connection import ros_connection, RCLPY_AVAILABLE

try:
    from config import settings
except ImportError:
    from ..config import settings

logger = logging.getLogger("ros_parameter")


class ROSParameterManager:
    """
    Manages USV operational parameters dynamically at runtime.
    Synchronizes with ROS 2 Node parameters when available.
    """
    _instance: Optional["ROSParameterManager"] = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super(ROSParameterManager, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        if hasattr(self, "_initialized") and self._initialized:
            return
        self._initialized = True
        self._lock = threading.Lock()

        # Dynamic parameter registry with default operational parameters
        self._parameters: Dict[str, Any] = {
            "cruise_speed": float(settings.DEFAULT_CRUISE_SPEED),
            "acceptance_radius": float(settings.DEFAULT_ACCEPTANCE_RADIUS),
            "max_linear_velocity": float(settings.MAX_LINEAR_VELOCITY),
            "max_angular_velocity": float(settings.MAX_ANGULAR_VELOCITY),
            "frame_id": str(settings.DEFAULT_FRAME_ID),
            "watchdog_timeout_sec": float(settings.WATCHDOG_TIMEOUT_SEC),
            "critical_battery_percent": float(settings.CRITICAL_BATTERY_PERCENT),
            "telemetry_stream_hz": float(settings.TELEMETRY_STREAM_HZ),
            "pid_heading_kp": 1.25,
            "pid_heading_kd": 0.35,
            "pid_surge_kp": 1.0,
            "geofence_radius_meters": 150.0,
        }

    def setup_parameters(self):
        """Declares ROS 2 parameters on the node if running with rclpy."""
        node = ros_connection.get_node()
        if not node or not RCLPY_AVAILABLE:
            logger.info("ROSParameterManager: Using internal in-memory parameters.")
            return

        try:
            for name, val in self._parameters.items():
                if not node.has_parameter(name):
                    node.declare_parameter(name, val)
            logger.info("ROS 2 parameters declared on node successfully.")
        except Exception as e:
            logger.error(f"Error declaring ROS 2 parameters: {e}")

    def get_all(self) -> Dict[str, Any]:
        """Returns all runtime parameters."""
        with self._lock:
            return copy.deepcopy(self._parameters)

    def get(self, key: str, default: Any = None) -> Any:
        """Retrieves a single parameter value."""
        with self._lock:
            return self._parameters.get(key, default)

    def set(self, key: str, value: Any) -> bool:
        """Sets a parameter value both locally and on the ROS 2 node."""
        with self._lock:
            if key not in self._parameters:
                logger.warning(f"Parameter '{key}' not in known registry. Adding new entry.")
            self._parameters[key] = value

        node = ros_connection.get_node()
        if node and RCLPY_AVAILABLE:
            try:
                from rclpy.parameter import Parameter
                param = Parameter(key, value=value)
                node.set_parameters([param])
            except Exception as e:
                logger.error(f"Error updating ROS parameter {key}: {e}")
                return False

        logger.info(f"Updated parameter: {key} = {value}")
        return True

    def update_bulk(self, updates: Dict[str, Any]) -> Dict[str, Any]:
        """Updates multiple parameters in a single batch."""
        success = {}
        for k, v in updates.items():
            self.set(k, v)
            success[k] = v
        return success


ros_parameter_manager = ROSParameterManager()
