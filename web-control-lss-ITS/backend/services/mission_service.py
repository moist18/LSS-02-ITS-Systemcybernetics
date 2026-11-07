"""
Mission Management Service for LSS-02 Autonomous Marine Surface Vehicle.
Handles waypoint navigation queue, mission state transitions, progress tracking,
and integration with ROS navigation planners.
"""
import copy
import logging
import math
import time
from typing import Any, Dict, List, Optional

try:
    from ros.ros_publisher import ros_publisher
    from ros.ros_subscriber import ros_subscriber
    from ros.ros_parameter import ros_parameter_manager
    from ros.ros_parameter_client import ros2_waypoint_param_client
    from config import settings
except ImportError:
    from ..ros.ros_publisher import ros_publisher
    from ..ros.ros_subscriber import ros_subscriber
    from ..ros.ros_parameter import ros_parameter_manager
    from ..ros.ros_parameter_client import ros2_waypoint_param_client
    from ..config import settings

logger = logging.getLogger("mission_service")


class Waypoint:
    def __init__(
        self,
        wp_id: int,
        lat: float,
        lon: float,
        target_speed: float = 1.5,
        acceptance_radius: float = 3.5,
        action: str = "PASS"
    ):
        self.wp_id = wp_id
        self.lat = lat
        self.lon = lon
        self.target_speed = target_speed
        self.acceptance_radius = acceptance_radius
        self.action = action  # PASS, SURVEY_SURFACE, SURVEY_UNDERWATER, DOCK, LOITER

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.wp_id,
            "lat": self.lat,
            "lon": self.lon,
            "target_speed": self.target_speed,
            "acceptance_radius": self.acceptance_radius,
            "action": self.action,
        }


class MissionService:
    """
    Manages autonomous waypoint missions, tracks progress,
    and updates vehicle state.
    """
    _instance: Optional["MissionService"] = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super(MissionService, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        if hasattr(self, "_initialized") and self._initialized:
            return
        self._initialized = True
        
        self.mission_id: str = "MISSION_DEFAULT_01"
        self.status: str = "IDLE"  # IDLE, RUNNING, PAUSED, COMPLETED, ABORTED
        self.waypoints: List[Waypoint] = []
        self.active_index: int = 0
        self.start_time: Optional[float] = None
        self.completed_time: Optional[float] = None

        # Prepopulate with standard test circuit in ITS Marine Basin
        self._init_default_circuit()

    def _init_default_circuit(self):
        """Initializes standard demonstration waypoints."""
        base_lat = -7.286340
        base_lon = 112.795933
        default_points = [
            (base_lat + 0.0003, base_lon + 0.0002, "SURVEY_SURFACE"),
            (base_lat + 0.0005, base_lon + 0.0006, "SURVEY_UNDERWATER"),
            (base_lat + 0.0002, base_lon + 0.0008, "PASS"),
            (base_lat, base_lon, "DOCK"),
        ]
        self.waypoints = [
            Waypoint(i + 1, lat, lon, target_speed=1.5, action=act)
            for i, (lat, lon, act) in enumerate(default_points)
        ]
        self._sync_to_telemetry()

    def set_waypoints(
        self,
        points: List[Dict[str, Any]],
        sync_to_ros: bool = True,
        trajectory: str = "A",
        radius: Optional[float] = None
    ) -> Dict[str, Any]:
        """
        Replaces current waypoint plan with new list and optionally pushes parameters to ROS 2.
        """
        self.waypoints = []
        for i, pt in enumerate(points):
            wp = Waypoint(
                wp_id=pt.get("id", i + 1),
                lat=float(pt["lat"]),
                lon=float(pt["lon"]),
                target_speed=float(pt.get("target_speed", settings.DEFAULT_CRUISE_SPEED)),
                acceptance_radius=float(pt.get("acceptance_radius", settings.DEFAULT_ACCEPTANCE_RADIUS)),
                action=pt.get("action", "PASS")
            )
            self.waypoints.append(wp)
        self.active_index = 0
        self.status = "IDLE"
        self._sync_to_telemetry()
        logger.info(f"Loaded {len(self.waypoints)} waypoints for mission {self.mission_id}")

        ros_result = None
        if sync_to_ros:
            ros_result = self.sync_waypoints_to_ros(trajectory=trajectory, radius=radius)

        return {
            "success": True,
            "count": len(self.waypoints),
            "trajectory": trajectory,
            "ros_sync": ros_result
        }

    def sync_waypoints_to_ros(self, trajectory: str = "A", radius: Optional[float] = None) -> Dict[str, Any]:
        """Uploads current mission waypoints as parameters to /gps_navigation."""
        wp_dicts = [wp.to_dict() for wp in self.waypoints]
        return ros2_waypoint_param_client.upload_waypoints_as_params(
            waypoints=wp_dicts,
            trajectory=trajectory,
            radius=radius
        )

    def start_mission(self) -> Dict[str, Any]:
        """Starts or resumes execution of loaded mission."""
        if not self.waypoints:
            return {"success": False, "message": "Cannot start mission: No waypoints loaded."}

        self.status = "RUNNING"
        self.start_time = time.time()
        ros_publisher.publish_mode("AUTO_WAYPOINT")
        self._sync_to_telemetry()
        logger.info(f"Mission {self.mission_id} started.")
        return {"success": True, "status": self.status, "active_waypoint": self.active_index + 1}

    def pause_mission(self) -> Dict[str, Any]:
        """Pauses autonomous navigation, holds current position."""
        if self.status == "RUNNING":
            self.status = "PAUSED"
            ros_publisher.publish_cmd_vel(0.0, 0.0)
            ros_publisher.publish_mode("HOLD")
            self._sync_to_telemetry()
            logger.info(f"Mission {self.mission_id} paused.")
            return {"success": True, "status": self.status}
        return {"success": False, "message": "Mission is not currently running."}

    def abort_mission(self) -> Dict[str, Any]:
        """Aborts mission immediately and cuts thrust."""
        self.status = "ABORTED"
        ros_publisher.publish_cmd_vel(0.0, 0.0)
        ros_publisher.publish_mode("MANUAL")
        self._sync_to_telemetry()
        logger.warning(f"Mission {self.mission_id} ABORTED.")
        return {"success": True, "status": self.status}

    def reset_mission(self) -> Dict[str, Any]:
        """Resets waypoint index to 0 and re-arms status."""
        self.active_index = 0
        self.status = "IDLE"
        self.start_time = None
        self.completed_time = None
        ros_publisher.reset_mission()
        ros_publisher.publish_mode("MANUAL")
        self._sync_to_telemetry()
        logger.info(f"Mission {self.mission_id} reset to start.")
        return {"success": True, "status": self.status}

    def get_status(self) -> Dict[str, Any]:
        """Returns comprehensive mission diagnostics."""
        total = len(self.waypoints)
        progress = (self.active_index / total * 100.0) if total > 0 else 0.0
        return {
            "mission_id": self.mission_id,
            "status": self.status,
            "active_waypoint_index": self.active_index,
            "total_waypoints": total,
            "progress_percent": round(progress, 1),
            "waypoints": [wp.to_dict() for wp in self.waypoints],
            "start_time": self.start_time,
            "elapsed_seconds": round(time.time() - self.start_time, 1) if self.start_time else 0.0,
        }

    def _sync_to_telemetry(self):
        """Synchronizes mission progress indicators into ros_subscriber cache."""
        total = len(self.waypoints)
        progress = (self.active_index / total * 100.0) if total > 0 else 0.0
        ros_subscriber.update_telemetry_field("mission", "current_id", self.mission_id)
        ros_subscriber.update_telemetry_field("mission", "status", self.status)
        ros_subscriber.update_telemetry_field("mission", "progress_percent", round(progress, 1))
        ros_subscriber.update_telemetry_field("mission", "active_waypoint_index", self.active_index)
        ros_subscriber.update_telemetry_field("mission", "total_waypoints", total)


mission_service = MissionService()
