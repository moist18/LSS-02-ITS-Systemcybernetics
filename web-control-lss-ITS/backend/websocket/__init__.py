"""
WebSocket package for LSS-02 USV Cockpit.
"""
from .telemetry_stream import router as ws_router, manager as telemetry_ws_manager

__all__ = ["ws_router", "telemetry_ws_manager"]
