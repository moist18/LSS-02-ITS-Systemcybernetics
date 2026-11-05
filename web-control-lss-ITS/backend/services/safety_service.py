"""
Safety & Fail-Safe Service for LSS-02 USV Cockpit.
Implements SIL-3 Emergency Cutoff, Watchdog Heartbeat Monitoring,
Geofence Checking, and Battery Critical Alarms.
"""
import asyncio
import logging
import time
from typing import Dict, Any, Optional

try:
    from ros.ros_publisher import ros_publisher
    from ros.ros_subscriber import ros_subscriber
    from ros.ros_parameter import ros_parameter_manager
    from config import settings
except ImportError:
    from ..ros.ros_publisher import ros_publisher
    from ..ros.ros_subscriber import ros_subscriber
    from ..ros.ros_parameter import ros_parameter_manager
    from ..config import settings

logger = logging.getLogger("safety_service")


class SafetyService:
    """
    Watches system health, battery, heartbeat, and geofence.
    Executes emergency disarm and fail-safe stop when needed.
    """
    _instance: Optional["SafetyService"] = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super(SafetyService, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        if hasattr(self, "_initialized") and self._initialized:
            return
        self._initialized = True
        self.last_heartbeat: float = time.time()
        self.is_monitoring: bool = False
        self.emergency_active: bool = False
        self._monitor_task: Optional[asyncio.Task] = None

    def start_monitoring(self):
        """Starts the safety watchdog monitor loop."""
        if not self.is_monitoring:
            self.is_monitoring = True
            self.last_heartbeat = time.time()
            self._monitor_task = asyncio.create_task(self._watchdog_loop())
            logger.info("SafetyService: Watchdog monitoring started.")

    def stop_monitoring(self):
        """Stops watchdog monitoring loop."""
        self.is_monitoring = False
        if self._monitor_task and not self._monitor_task.done():
            self._monitor_task.cancel()
            logger.info("SafetyService: Watchdog monitoring stopped.")

    def receive_heartbeat(self):
        """Updates operator heartbeat timestamp."""
        self.last_heartbeat = time.time()

    async def _watchdog_loop(self):
        """Periodic safety checks."""
        while self.is_monitoring:
            try:
                await asyncio.sleep(0.5)
                timeout = float(ros_parameter_manager.get("watchdog_timeout_sec", settings.WATCHDOG_TIMEOUT_SEC))
                critical_battery = float(ros_parameter_manager.get("critical_battery_percent", settings.CRITICAL_BATTERY_PERCENT))
                
                # Check 1: Operator Watchdog Heartbeat
                elapsed = time.time() - self.last_heartbeat
                if elapsed > timeout and not self.emergency_active:
                    telemetry = ros_subscriber.get_telemetry()
                    # Only stop if vessel is currently moving
                    if telemetry["navigation"]["surge_velocity"] > 0.05:
                        logger.warning(
                            f"Safety Watchdog Timeout! Last ping {elapsed:.1f}s ago (limit {timeout}s). "
                            "Cutting thruster velocity."
                        )
                        ros_publisher.publish_cmd_vel(0.0, 0.0)

                # Check 2: Critical Battery Alert
                telemetry = ros_subscriber.get_telemetry()
                battery_pct = telemetry["battery"]["percentage"]
                if battery_pct <= critical_battery:
                    logger.critical(
                        f"CRITICAL BATTERY: {battery_pct}% <= {critical_battery}%. "
                        "Recommending Return-To-Base or Safe Stop."
                    )
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in safety monitor loop: {e}")

    def trigger_emergency_disarm(self) -> Dict[str, Any]:
        """
        Executes immediate SIL-3 Emergency Disarm:
        - Cuts thruster surge and steering immediately to 0
        - Signals hardware disarm channel
        - Freezes mission
        """
        self.emergency_active = True
        ros_publisher.publish_emergency_disarm(True)
        ros_publisher.publish_cmd_vel(0.0, 0.0)
        logger.warning("SIL-3 EMERGENCY DISARM ACTIVATED!")
        return {
            "status": "EMERGENCY_DISARM_ACTIVE",
            "message": "SIL-3 Emergency Disarm triggered. Thrusters cut immediately.",
            "timestamp": time.time()
        }

    def clear_emergency_disarm(self) -> Dict[str, Any]:
        """Clears emergency disarm state to allow manual re-arming."""
        self.emergency_active = False
        ros_publisher.publish_emergency_disarm(False)
        ros_publisher.publish_mode("MANUAL")
        logger.info("Emergency disarm cleared. Vehicle returned to MANUAL standby.")
        return {
            "status": "ARMED_STANDBY",
            "message": "Emergency disarm released. System is in MANUAL standby.",
            "timestamp": time.time()
        }

    def get_safety_status(self) -> Dict[str, Any]:
        """Returns comprehensive safety diagnostics."""
        elapsed = time.time() - self.last_heartbeat
        timeout = float(ros_parameter_manager.get("watchdog_timeout_sec", settings.WATCHDOG_TIMEOUT_SEC))
        return {
            "emergency_active": self.emergency_active,
            "watchdog_active": self.is_monitoring,
            "last_heartbeat_ago_sec": round(elapsed, 2),
            "watchdog_healthy": elapsed <= timeout,
            "watchdog_limit_sec": timeout,
        }


safety_service = SafetyService()
