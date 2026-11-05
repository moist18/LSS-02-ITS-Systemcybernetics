"""
Configuration API Endpoints for LSS-02 USV Cockpit.
Provides dynamic parameter tuning and system settings inspection.
"""
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Any, Dict

try:
    from ros.ros_parameter import ros_parameter_manager
    from ros.ros_connection import ros_connection
    from config import settings
except ImportError:
    from ..ros.ros_parameter import ros_parameter_manager
    from ..ros.ros_connection import ros_connection
    from ..config import settings

router = APIRouter(prefix="/api/config", tags=["Configuration"])


class ParameterUpdate(BaseModel):
    key: str
    value: Any


class BulkParameters(BaseModel):
    parameters: Dict[str, Any]


@router.get("/")
async def get_configuration():
    """Returns dynamic runtime parameters and ROS connection status."""
    return {
        "parameters": ros_parameter_manager.get_all(),
        "ros_status": ros_connection.get_status(),
        "app_version": settings.VERSION,
    }


@router.put("/parameter")
async def update_single_parameter(payload: ParameterUpdate):
    """Updates a single runtime parameter."""
    success = ros_parameter_manager.set(payload.key, payload.value)
    return {"success": success, "key": payload.key, "value": payload.value}


@router.put("/bulk")
async def update_bulk_parameters(payload: BulkParameters):
    """Updates a dictionary of parameters in one request."""
    updated = ros_parameter_manager.update_bulk(payload.parameters)
    return {"success": True, "updated": updated}


@router.post("/reset")
async def reset_configuration():
    """Resets parameters back to default application settings."""
    defaults = {
        "cruise_speed": float(settings.DEFAULT_CRUISE_SPEED),
        "acceptance_radius": float(settings.DEFAULT_ACCEPTANCE_RADIUS),
        "max_linear_velocity": float(settings.MAX_LINEAR_VELOCITY),
        "max_angular_velocity": float(settings.MAX_ANGULAR_VELOCITY),
        "frame_id": str(settings.DEFAULT_FRAME_ID),
        "watchdog_timeout_sec": float(settings.WATCHDOG_TIMEOUT_SEC),
        "critical_battery_percent": float(settings.CRITICAL_BATTERY_PERCENT),
        "telemetry_stream_hz": float(settings.TELEMETRY_STREAM_HZ),
    }
    ros_parameter_manager.update_bulk(defaults)
    return {"success": True, "parameters": ros_parameter_manager.get_all()}
