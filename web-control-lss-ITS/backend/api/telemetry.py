"""
Telemetry API Endpoints for LSS-02 USV Cockpit.
Provides real-time snapshots of vehicle sensors, navigation, power, and topic status.
"""
from fastapi import APIRouter
from typing import Any, Dict

try:
    from ros.ros_subscriber import ros_subscriber
except ImportError:
    from ..ros.ros_subscriber import ros_subscriber

router = APIRouter(prefix="/api/telemetry", tags=["Telemetry"])


@router.get("/", response_model=Dict[str, Any])
@router.get("/overview", response_model=Dict[str, Any])
async def get_full_telemetry():
    """Returns complete real-time vehicle telemetry snapshot."""
    return ros_subscriber.get_telemetry()


@router.get("/topics", response_model=Dict[str, Any])
async def get_topic_diagnostics():
    """Returns frequency, message count, and latency for monitored ROS 2 topics."""
    return ros_subscriber.get_topic_stats()


@router.get("/navigation")
async def get_navigation_telemetry():
    """Returns vessel position, speed, heading, and attitude."""
    telem = ros_subscriber.get_telemetry()
    return telem.get("navigation", {})


@router.get("/battery")
async def get_battery_telemetry():
    """Returns battery state, voltage, current, and charge percentage."""
    telem = ros_subscriber.get_telemetry()
    return telem.get("battery", {})


@router.get("/environment")
async def get_environmental_telemetry():
    """Returns pod cooling, ambient water temperature, humidity, and pH levels."""
    telem = ros_subscriber.get_telemetry()
    return telem.get("environment", {})


@router.get("/sonar")
async def get_sonar_telemetry():
    """Returns sonar bathymetry depth profile, chirp frequency, and bottom echo echo."""
    telem = ros_subscriber.get_telemetry()
    return telem.get("sonar", {})


@router.get("/detections")
async def get_vision_detections():
    """Returns current active vision detection targets (buoys, docking markers)."""
    telem = ros_subscriber.get_telemetry()
    return telem.get("detections", {})
