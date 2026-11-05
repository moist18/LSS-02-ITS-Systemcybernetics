"""
High-Frequency WebSocket Telemetry Streamer for LSS-02 USV Cockpit.
Broadcasts vehicle sensor telemetry, RPMs, battery, and diagnostics at 10-20 Hz.
"""
import asyncio
import json
import logging
from typing import List, Set
from fastapi import APIRouter, WebSocket, WebSocketDisconnect

try:
    from ros.ros_subscriber import ros_subscriber
    from ros.ros_connection import ros_connection
    from ros.ros_parameter import ros_parameter_manager
    from services.safety_service import safety_service
    from config import settings
except ImportError:
    from ..ros.ros_subscriber import ros_subscriber
    from ..ros.ros_connection import ros_connection
    from ..ros.ros_parameter import ros_parameter_manager
    from ..services.safety_service import safety_service
    from ..config import settings

logger = logging.getLogger("telemetry_stream")
router = APIRouter(tags=["WebSocket"])


class ConnectionManager:
    """Manages active WebSocket client connections for real-time streaming."""
    def __init__(self):
        self.active_connections: Set[WebSocket] = set()
        self._lock = asyncio.Lock()
        self._broadcast_task: asyncio.Task = None

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        async with self._lock:
            self.active_connections.add(websocket)
        logger.info(f"WebSocket client connected. Total clients: {len(self.active_connections)}")

    async def disconnect(self, websocket: WebSocket):
        async with self._lock:
            self.active_connections.discard(websocket)
        logger.info(f"WebSocket client disconnected. Total clients: {len(self.active_connections)}")

    async def broadcast_json(self, data: dict):
        """Sends data payload to all connected clients."""
        dead_connections = set()
        async with self._lock:
            connections = list(self.active_connections)

        for conn in connections:
            try:
                await conn.send_json(data)
            except Exception as e:
                logger.debug(f"Failed sending to client, scheduling cleanup: {e}")
                dead_connections.add(conn)

        if dead_connections:
            async with self._lock:
                for dead in dead_connections:
                    self.active_connections.discard(dead)

    def start_broadcaster(self):
        """Starts periodic broadcast task if not already running."""
        if self._broadcast_task is None or self._broadcast_task.done():
            self._broadcast_task = asyncio.create_task(self._stream_loop())
            logger.info("Telemetry streamer broadcast task launched.")

    def stop_broadcaster(self):
        """Stops the broadcast task."""
        if self._broadcast_task and not self._broadcast_task.done():
            self._broadcast_task.cancel()
            logger.info("Telemetry streamer broadcast task stopped.")

    async def _stream_loop(self):
        """Streaming loop broadcasting telemetry at configured rate."""
        while True:
            try:
                hz = float(ros_parameter_manager.get("telemetry_stream_hz", settings.TELEMETRY_STREAM_HZ))
                interval = 1.0 / max(1.0, hz)

                # If running in simulated mode, step simulated physical dynamics
                if ros_connection.is_simulated:
                    ros_subscriber.update_simulated_step(dt=interval)

                if self.active_connections:
                    telemetry = ros_subscriber.get_telemetry()
                    topics = ros_subscriber.get_topic_stats()
                    payload = {
                        "type": "telemetry_update",
                        "data": telemetry,
                        "topics": topics,
                        "emergency": safety_service.emergency_active,
                    }
                    await self.broadcast_json(payload)

                await asyncio.sleep(interval)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in telemetry stream loop: {e}")
                await asyncio.sleep(0.5)


manager = ConnectionManager()


@router.websocket("/ws/telemetry")
async def websocket_telemetry_endpoint(websocket: WebSocket):
    """
    WebSocket endpoint for bidirectional real-time cockpit stream.
    Receives pings/commands from client, pushes vehicle telemetry snapshot.
    """
    await manager.connect(websocket)
    try:
        while True:
            # Listen for client messages (heartbeat pings or commands)
            text_data = await websocket.receive_text()
            try:
                msg = json.loads(text_data)
                msg_type = msg.get("type")
                if msg_type == "ping" or msg_type == "heartbeat":
                    safety_service.receive_heartbeat()
                    await websocket.send_json({"type": "pong", "time": msg.get("time")})
            except json.JSONDecodeError:
                # Raw text ping
                safety_service.receive_heartbeat()
    except WebSocketDisconnect:
        await manager.disconnect(websocket)
    except Exception as e:
        logger.error(f"WebSocket exception: {e}")
        await manager.disconnect(websocket)
