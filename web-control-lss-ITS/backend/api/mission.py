"""
Mission API Endpoints for LSS-02 USV Cockpit.
Controls autonomous mission planning, waypoint loading, and mission execution.
"""
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field
from typing import Any, Dict, List, Optional

try:
    from services.mission_service import mission_service
except ImportError:
    from ..services.mission_service import mission_service

router = APIRouter(prefix="/api/mission", tags=["Mission"])


class WaypointItem(BaseModel):
    id: Optional[int] = None
    lat: float = Field(..., description="Latitude coordinate")
    lon: float = Field(..., description="Longitude coordinate")
    target_speed: Optional[float] = Field(1.5, description="Target surge speed in m/s")
    acceptance_radius: Optional[float] = Field(3.5, description="Acceptance radius in meters")
    action: Optional[str] = Field("PASS", description="Action: PASS, SURVEY_SURFACE, SURVEY_UNDERWATER, DOCK")


class WaypointList(BaseModel):
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
    """Loads a new series of waypoints for autonomous execution."""
    points = [wp.model_dump() for wp in payload.waypoints]
    mission_service.set_waypoints(points)
    return {
        "success": True,
        "count": len(points),
        "waypoints": [wp.to_dict() for wp in mission_service.waypoints]
    }
