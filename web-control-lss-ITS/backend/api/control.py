"""
Control API Endpoints for LSS-02 USV Cockpit.
Handles manual joystick / slider thrust, emergency disarm, subsystem switches, and watchdog keepalive.
"""
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field
from typing import Optional

try:
    from ros.ros_publisher import ros_publisher
    from ros.ros_subscriber import ros_subscriber
    from ros.ros_parameter import ros_parameter_manager
    from services.safety_service import safety_service
except ImportError:
    from ..ros.ros_publisher import ros_publisher
    from ..ros.ros_subscriber import ros_subscriber
    from ..ros.ros_parameter import ros_parameter_manager
    from ..services.safety_service import safety_service

router = APIRouter(prefix="/api/control", tags=["Control"])


class VelocityCommand(BaseModel):
    linear_x: float = Field(..., description="Surge velocity in m/s (0.0 to 4.0)")
    angular_z: float = Field(0.0, description="Yaw rate in rad/s (-1.5 to 1.5)")


class EmergencyCommand(BaseModel):
    disarm: bool = Field(..., description="True to trigger SIL-3 emergency disarm, False to clear")


class ModeCommand(BaseModel):
    mode: str = Field(..., description="Operating mode: MANUAL, AUTO_WAYPOINT, DOCKING, HOLD")


class SubsystemCommand(BaseModel):
    subsystem: str = Field(..., description="Subsystem name: gps, compass, vision, lidar")
    enabled: bool = Field(..., description="Enable or disable subsystem")


class SurgeLimitCommand(BaseModel):
    limit: float = Field(..., ge=0.0, le=4.0, description="Target surge limit in m/s")


@router.post("/cmd_vel")
async def send_velocity_command(cmd: VelocityCommand):
    """Sends linear and angular velocity commands to thruster controller."""
    if safety_service.emergency_active:
        raise HTTPException(
            status_code=400,
            detail="Emergency disarm is active. Release emergency disarm before moving thrusters."
        )
    safety_service.receive_heartbeat()
    success = ros_publisher.publish_cmd_vel(cmd.linear_x, cmd.angular_z)
    return {"success": success, "linear_x": cmd.linear_x, "angular_z": cmd.angular_z}


@router.post("/emergency")
async def toggle_emergency_disarm(cmd: EmergencyCommand):
    """Toggles instant SIL-3 emergency disarm."""
    if cmd.disarm:
        return safety_service.trigger_emergency_disarm()
    else:
        return safety_service.clear_emergency_disarm()


@router.post("/mode")
async def switch_mode(cmd: ModeCommand):
    """Switches vehicle operating mode."""
    allowed = ["MANUAL", "AUTO_WAYPOINT", "DOCKING", "HOLD"]
    if cmd.mode.upper() not in allowed:
        raise HTTPException(
            status_code=400,
            detail=f"Invalid mode '{cmd.mode}'. Allowed: {', '.join(allowed)}"
        )
    success = ros_publisher.publish_mode(cmd.mode.upper())
    return {"success": success, "mode": cmd.mode.upper()}


@router.post("/subsystems")
async def toggle_subsystem(cmd: SubsystemCommand):
    """Enables or disables vehicle navigation subsystems (gps, compass, vision, lidar)."""
    sub = cmd.subsystem.lower()
    if sub not in ["gps", "compass", "vision", "lidar"]:
        raise HTTPException(status_code=400, detail=f"Unknown subsystem '{cmd.subsystem}'")
    success = ros_publisher.toggle_subsystem(sub, cmd.enabled)
    return {"success": success, "subsystem": sub, "enabled": cmd.enabled}


@router.post("/surge_limit")
async def set_surge_limit(cmd: SurgeLimitCommand):
    """Updates target surge limit slider setting."""
    ros_subscriber.update_telemetry_field("navigation", "target_surge_limit", cmd.limit)
    ros_parameter_manager.set("cruise_speed", cmd.limit)
    return {"success": True, "target_surge_limit": cmd.limit}


@router.post("/heartbeat")
async def keepalive_heartbeat():
    """Operator station watchdog keepalive ping."""
    safety_service.receive_heartbeat()
    return {"status": "ok", "safety": safety_service.get_safety_status()}
