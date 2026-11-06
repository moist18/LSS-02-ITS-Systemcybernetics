"""
Mission API Endpoints for LSS-02 USV Cockpit.
Controls autonomous mission planning, waypoint loading, and mission execution.
"""
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field
from typing import Any, Dict, List, Optional

try:
    from services.mission_service import mission_service
    from ros.ros_parameter_client import ros2_waypoint_param_client
except ImportError:
    from ..services.mission_service import mission_service
    from ..ros.ros_parameter_client import ros2_waypoint_param_client

router = APIRouter(prefix="/api/mission", tags=["Mission"])


class WaypointItem(BaseModel):
    id: Optional[int] = None
    lat: float = Field(..., description="Latitude coordinate (-90 to 90)")
    lon: float = Field(..., description="Longitude coordinate (-180 to 180)")
    alt: Optional[float] = Field(0.0, description="Altitude coordinate")
    target_speed: Optional[float] = Field(1.5, description="Target surge speed in m/s")
    acceptance_radius: Optional[float] = Field(3.5, description="Acceptance radius in meters")
    action: Optional[str] = Field("PASS", description="Action: PASS, SURVEY_SURFACE, SURVEY_UNDERWATER, DOCK")


class WaypointList(BaseModel):
    trajectory: Optional[str] = Field("A", description="Target trajectory ('A' or 'B') for ROS 2 navigation")
    radius: Optional[float] = Field(None, description="Global acceptance radius for gps_navigation")
    sync_to_ros: Optional[bool] = Field(True, description="Whether to automatically push parameters to /gps_navigation")
    waypoints: List[WaypointItem]


@router.get("/status")
async def get_mission_status():
    """Retrieves current autonomous mission status, active waypoint, and progress."""
    return mission_service.get_status()


@router.post("/start")
async def start_mission():
    """Starts or resumes autonomous waypoint mission."""
    result = mission_service.start_mission()
    if not result.get("success"):
        raise HTTPException(status_code=400, detail=result.get("message"))
    return result


@router.post("/pause")
async def pause_mission():
    """Pauses autonomous waypoint navigation and holds position."""
    result = mission_service.pause_mission()
    if not result.get("success"):
        raise HTTPException(status_code=400, detail=result.get("message"))
    return result


@router.post("/abort")
async def abort_mission():
    """Aborts current mission and transitions vehicle to MANUAL standby."""
    return mission_service.abort_mission()


@router.post("/reset")
async def reset_mission():
    """Resets waypoint index and restarts mission state machine."""
    return mission_service.reset_mission()


@router.get("/waypoints")
async def get_waypoints():
    """Returns currently loaded list of waypoints."""
    return {"waypoints": [wp.to_dict() for wp in mission_service.waypoints]}


@router.post("/waypoints")
async def load_waypoints(payload: WaypointList):
    """
    Loads a new series of waypoints for autonomous execution and pushes them
    as ROS 2 parameters to the /gps_navigation node.
    """
    points = [wp.model_dump() for wp in payload.waypoints]
    result = mission_service.set_waypoints(
        points=points,
        sync_to_ros=payload.sync_to_ros if payload.sync_to_ros is not None else True,
        trajectory=payload.trajectory or "A",
        radius=payload.radius
    )

    return {
        "success": True,
        "count": len(points),
        "trajectory": payload.trajectory or "A",
        "ros_sync": result.get("ros_sync"),
        "waypoints": [wp.to_dict() for wp in mission_service.waypoints]
    }


@router.get("/waypoints/ros")
async def get_ros_waypoints(trajectory: str = "A"):
    """
    Retrieves currently synchronized waypoint parameters for trajectory 'A' or 'B'.
    """
    cached_data = ros2_waypoint_param_client.get_cached_waypoints_param(trajectory=trajectory)
    return {
        "trajectory": trajectory.upper(),
        "cached_parameters": cached_data
    }


@router.post("/waypoints/sync")
async def sync_waypoints_to_ros(trajectory: str = "A", radius: Optional[float] = None):
    """
    Manually triggers resynchronization of currently loaded mission waypoints
    to ROS 2 node parameters.
    """
    sync_result = mission_service.sync_waypoints_to_ros(trajectory=trajectory, radius=radius)
    return {
        "trajectory": trajectory.upper(),
        "result": sync_result
    }
