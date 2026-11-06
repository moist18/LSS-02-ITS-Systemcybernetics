"""
ROS 2 Waypoint Parameter Client for LSS-02 USV Cockpit.
Manages sending and querying waypoint parameters directly to the /gps_navigation
node using standard ROS 2 rcl_interfaces SetParameters & GetParameters services.
"""
import copy
import logging
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

from .ros_connection import ros_connection, RCLPY_AVAILABLE

logger = logging.getLogger("ros_parameter_client")


class ROS2WaypointParamClient:
    """
    Client for managing autonomous waypoints via ROS 2 parameters on /gps_navigation.
    Supports trajectory switching ('A' or 'B'), coordinate validation, and simulation fallback.
    """
    _instance: Optional["ROS2WaypointParamClient"] = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super(ROS2WaypointParamClient, cls).__new__(cls)
        return cls._instance

    def __init__(self, target_node_name: str = "gps_navigation"):
        if hasattr(self, "_initialized") and self._initialized:
            return
        self._initialized = True
        self._lock = threading.Lock()
        self.target_node_name = target_node_name

        # In-memory parameter cache (useful for simulation or status polling)
        self._parameter_cache: Dict[str, Dict[str, Any]] = {
            "A": {
                "count": 0,
                "radius": 2.0,
                "waypoints": [],
                "last_synced": None,
                "ros_synced": False
            },
            "B": {
                "count": 0,
                "radius": 2.0,
                "waypoints": [],
                "last_synced": None,
                "ros_synced": False
            }
        }

    def validate_coordinates(self, lat: float, lon: float) -> Tuple[bool, str]:
        """Validates that latitude and longitude conform to standard WGS84 bounds."""
        if not (-90.0 <= lat <= 90.0):
            return False, f"Invalid latitude: {lat}. Must be between -90 and 90."
        if not (-180.0 <= lon <= 180.0):
            return False, f"Invalid longitude: {lon}. Must be between -180 and 180."
        return True, "OK"

    def upload_waypoints_as_params(
        self,
        waypoints: List[Dict[str, Any]],
        trajectory: str = "A",
        radius: Optional[float] = None,
        timeout_sec: float = 3.0
    ) -> Dict[str, Any]:
        """
        Uploads a list of waypoints as ROS 2 parameters to /gps_navigation.
        
        Args:
            waypoints: List of dicts containing 'lat', 'lon', and optional 'alt'.
            trajectory: Trajectory identifier ('A' or 'B').
            radius: Optional acceptance radius parameter (navigation.radius).
            timeout_sec: Maximum wait time for ROS 2 service response.
            
        Returns:
            Dict containing execution status, count, and sync details.
        """
        trajectory = trajectory.upper().strip()
        if trajectory not in ["A", "B"]:
            trajectory = "A"

        # 1. Validate all waypoint coordinates
        validated_waypoints = []
        for idx, wp in enumerate(waypoints):
            try:
                lat = float(wp["lat"])
                lon = float(wp["lon"])
                alt = float(wp.get("alt", wp.get("altitude", 0.0)))
            except (KeyError, ValueError, TypeError) as e:
                return {
                    "success": False,
                    "message": f"Waypoint index {idx} has invalid format: {e}",
                    "ros_synced": False
                }

            is_valid, err_msg = self.validate_coordinates(lat, lon)
            if not is_valid:
                return {"success": False, "message": err_msg, "ros_synced": False}

            validated_waypoints.append({
                "latitude": lat,
                "longitude": lon,
                "altitude": alt,
                "action": wp.get("action", "PASS"),
                "target_speed": float(wp.get("target_speed", 1.5))
            })

        count = len(validated_waypoints)

        # 2. Update internal cache first
        with self._lock:
            self._parameter_cache[trajectory] = {
                "count": count,
                "radius": float(radius) if radius is not None else self._parameter_cache[trajectory]["radius"],
                "waypoints": copy.deepcopy(validated_waypoints),
                "last_synced": time.time(),
                "ros_synced": False
            }

        # 3. Check if ROS 2 (rclpy) is available
        node = ros_connection.get_node()
        if not RCLPY_AVAILABLE or not node or ros_connection.is_simulated:
            logger.info(
                f"[SIMULATION] Uploaded {count} waypoints to trajectory {trajectory} "
                f"(ROS 2 simulated/offline)."
            )
            return {
                "success": True,
                "count": count,
                "trajectory": trajectory,
                "ros_synced": False,
                "mode": "SIMULATION",
                "message": "Parameters updated in local gateway cache (ROS 2 running in simulated mode)."
            }

        # 4. Communicate with ROS 2 via /<target_node>/set_parameters service
        try:
            from rcl_interfaces.msg import Parameter as MsgParameter, ParameterType, ParameterValue
            from rcl_interfaces.srv import SetParameters

            service_name = f"/{self.target_node_name}/set_parameters"
            client = node.create_client(SetParameters, service_name)

            if not client.wait_for_service(timeout_sec=timeout_sec):
                logger.warning(
                    f"ROS 2 Service '{service_name}' not available after {timeout_sec}s. "
                    "Saved to cache only."
                )
                return {
                    "success": True,
                    "count": count,
                    "trajectory": trajectory,
                    "ros_synced": False,
                    "mode": "OFFLINE_CACHE",
                    "message": f"Target node '{self.target_node_name}' is offline. Saved to local cache."
                }

            param_list: List[MsgParameter] = []
            prefix = f"navigation.waypoints_{trajectory}"

            # Set count parameter
            param_list.append(MsgParameter(
                name=f"{prefix}.count",
                value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=count)
            ))

            # Set radius if provided
            if radius is not None:
                param_list.append(MsgParameter(
                    name="navigation.radius",
                    value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(radius))
                ))

            # Set individual waypoint parameters
            for i, wp in enumerate(validated_waypoints):
                wp_prefix = f"{prefix}.waypoint_{i}"
                param_list.append(MsgParameter(
                    name=f"{wp_prefix}.latitude",
                    value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=wp["latitude"])
                ))
                param_list.append(MsgParameter(
                    name=f"{wp_prefix}.longitude",
                    value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=wp["longitude"])
                ))
                param_list.append(MsgParameter(
                    name=f"{wp_prefix}.altitude",
                    value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=wp["altitude"])
                ))

            # Send asynchronous request
            req = SetParameters.Request()
            req.parameters = param_list
            future = client.call_async(req)

            # Wait for response with timeout
            start_time = time.time()
            while not future.done() and (time.time() - start_time) < timeout_sec:
                time.sleep(0.02)

            if future.done():
                response = future.result()
                failures = [r.reason for r in response.results if not r.successful]
                if not failures:
                    with self._lock:
                        self._parameter_cache[trajectory]["ros_synced"] = True

                    logger.info(
                        f"Successfully synchronized {count} waypoints to ROS 2 node "
                        f"'/{self.target_node_name}' on trajectory {trajectory}."
                    )
                    return {
                        "success": True,
                        "count": count,
                        "trajectory": trajectory,
                        "ros_synced": True,
                        "mode": "ROS2_DIRECT",
                        "message": f"Uploaded {count} waypoints to /{self.target_node_name} successfully."
                    }
                else:
                    logger.error(f"ROS 2 rejected parameters: {failures}")
                    return {
                        "success": False,
                        "count": count,
                        "trajectory": trajectory,
                        "ros_synced": False,
                        "message": f"Node rejected parameter update: {'; '.join(failures)}"
                    }
            else:
                logger.warning("Timeout waiting for ROS 2 set_parameters response.")
                return {
                    "success": True,
                    "count": count,
                    "trajectory": trajectory,
                    "ros_synced": False,
                    "mode": "TIMEOUT_FALLBACK",
                    "message": "Parameters sent but response timed out. Check node status."
                }

        except Exception as e:
            logger.error(f"Error in upload_waypoints_as_params: {e}", exc_info=True)
            return {
                "success": False,
                "message": f"Exception occurred while setting ROS 2 parameters: {str(e)}",
                "ros_synced": False
            }

    def get_cached_waypoints_param(self, trajectory: str = "A") -> Dict[str, Any]:
        """Returns the cached parameters for the specified trajectory."""
        trajectory = trajectory.upper().strip()
        with self._lock:
            data = self._parameter_cache.get(trajectory, {})
            return copy.deepcopy(data)


ros2_waypoint_param_client = ROS2WaypointParamClient()
