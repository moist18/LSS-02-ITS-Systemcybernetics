"""
LSS-02 USV Marine Command Cockpit - FastAPI Backend Gateway
Integrates ROS 2 Humble node lifecycle, real-time WebSockets, REST APIs,
and safety supervisory services.
"""
import os
import sys

# Auto-switch to system Python 3.10 if ROS 2 Humble is active and current Python is 3.12+ (e.g. radioconda)
if sys.version_info >= (3, 11) and os.environ.get("ROS_DISTRO") == "humble":
    system_python = "/usr/bin/python3"
    if os.path.exists(system_python) and sys.executable != system_python:
        try:
            os.execv(system_python, [system_python] + sys.argv)
        except Exception:
            pass

# Ensure backend directory is in sys.path
backend_dir = os.path.dirname(os.path.abspath(__file__))
if backend_dir not in sys.path:
    sys.path.insert(0, backend_dir)

import logging
from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

from config import settings
from ros import ros_connection, ros_subscriber, ros_publisher, ros_parameter_manager
from services import safety_service, mission_service
from api import telemetry_router, control_router, mission_router, config_router
from websocket import ws_router, telemetry_ws_manager

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
)
logger = logging.getLogger("main")


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Lifecycle manager handling startup and shutdown sequence."""
    logger.info("Initializing LSS-02 Backend Services...")

    # 1. Start ROS 2 Connection & Node
    ros_connection.start()

    # 2. Setup ROS 2 Subscribers, Publishers, and Parameters
    ros_subscriber.setup_subscribers()
    ros_publisher.setup_publishers()
    ros_parameter_manager.setup_parameters()

    # 3. Start Safety Supervisory Watchdog
    safety_service.start_monitoring()

    # 4. Start Real-time Telemetry WebSocket Broadcaster
    telemetry_ws_manager.start_broadcaster()

    logger.info(f"LSS-02 Gateway started successfully on {settings.HOST}:{settings.PORT}")
    yield

    # Shutdown sequence
    logger.info("Shutting down LSS-02 Backend Services...")
    telemetry_ws_manager.stop_broadcaster()
    safety_service.stop_monitoring()
    ros_connection.stop()
    logger.info("All services shut down cleanly.")


app = FastAPI(
    title=settings.PROJECT_NAME,
    version=settings.VERSION,
    description="Backend gateway bridging ROS 2 Humble USV nodes with the Marine Command Cockpit Web UI.",
    lifespan=lifespan,
    docs_url="/docs",
    redoc_url="/redoc"
)

# CORS configuration to allow local & network connections
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.CORS_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Mount REST API Routers
app.include_router(telemetry_router)
app.include_router(control_router)
app.include_router(mission_router)
app.include_router(config_router)

# Mount WebSocket Router
app.include_router(ws_router)


@app.get("/", tags=["Health"])
async def root():
    """Root status summary endpoint."""
    return {
        "project": settings.PROJECT_NAME,
        "version": settings.VERSION,
        "status": "ONLINE",
        "ros_connection": ros_connection.get_status(),
        "safety": safety_service.get_safety_status(),
    }


@app.get("/health", tags=["Health"])
async def health_check():
    """Health check for service monitoring."""
    return {
        "status": "healthy",
        "ros_running": ros_connection.is_running,
        "emergency_active": safety_service.emergency_active,
    }


if __name__ == "__main__":
    uvicorn.run(
        "main:app",
        host=settings.HOST,
        port=settings.PORT,
        reload=settings.DEBUG,
    )
