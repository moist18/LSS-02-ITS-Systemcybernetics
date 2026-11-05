"""
API routers package for LSS-02 USV Cockpit.
"""
from .telemetry import router as telemetry_router
from .control import router as control_router
from .mission import router as mission_router
from .configuration import router as config_router

__all__ = [
    "telemetry_router",
    "control_router",
    "mission_router",
    "config_router",
]
