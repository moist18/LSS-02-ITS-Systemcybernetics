"""
ROS 2 Connection and Node Lifecycle Manager for LSS-02 USV.
Handles rclpy initialization, node creation, multi-threaded spinning,
and transparent fallback to simulated mode if ROS 2 is not present.
"""
import logging
import threading
import time
from typing import Optional

try:
    from config import settings
except ImportError:
    from ..config import settings

logger = logging.getLogger("ros_connection")
logging.basicConfig(level=logging.INFO)

RCLPY_AVAILABLE = False
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    RCLPY_AVAILABLE = True
except Exception as e:
    logger.warning(
        f"rclpy not available in current runtime ({e}). "
        "Operating in simulated/gateway bridge mode."
    )


class ROSConnection:
    """
    Singleton class managing the ROS 2 node lifecycle and executor thread.
    """
    _instance: Optional["ROSConnection"] = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super(ROSConnection, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        if hasattr(self, "_initialized") and self._initialized:
            return
        self._initialized = True
        self.node: Optional[object] = None
        self.executor: Optional[object] = None
        self.thread: Optional[threading.Thread] = None
        self.is_running: bool = False
        self.is_simulated: bool = not RCLPY_AVAILABLE
        self.connected_since: Optional[float] = None
        self._lock = threading.Lock()

    def start(self):
        """Initializes ROS 2 node and starts executor in background thread."""
        with self._lock:
            if self.is_running:
                logger.info("ROSConnection is already running.")
                return

            if RCLPY_AVAILABLE:
                try:
                    if not rclpy.ok():
                        rclpy.init(args=None)
                    
                    self.node = Node(settings.ROS_NODE_NAME)
                    self.executor = MultiThreadedExecutor()
                    self.executor.add_node(self.node)

                    self.is_running = True
                    self.is_simulated = False
                    self.connected_since = time.time()

                    self.thread = threading.Thread(
                        target=self._spin_worker,
                        daemon=True,
                        name="ROS2_Executor_Thread"
                    )
                    self.thread.start()
                    logger.info(f"ROS 2 Node '{settings.ROS_NODE_NAME}' initialized successfully.")
                    return
                except Exception as e:
                    logger.error(f"Failed to initialize rclpy node: {e}. Switching to simulation fallback.")
                    self.is_simulated = True

            # Simulation / Standalone Mode
            self.is_running = True
            self.is_simulated = True
            self.connected_since = time.time()
            logger.info("ROSConnection initialized in SIMULATION / STANDALONE mode.")

    def _spin_worker(self):
        """Worker loop running the ROS 2 executor."""
        logger.info("ROS 2 background spin thread started.")
        try:
            if self.executor:
                self.executor.spin()
        except Exception as e:
            logger.error(f"Error in ROS 2 executor spin loop: {e}")
        finally:
            logger.info("ROS 2 background spin thread terminated.")

    def stop(self):
        """Gracefully halts executor and destroys node."""
        with self._lock:
            if not self.is_running:
                return

            self.is_running = False
            if RCLPY_AVAILABLE and self.executor and self.node:
                try:
                    self.executor.shutdown()
                    self.node.destroy_node()
                    if rclpy.ok():
                        rclpy.shutdown()
                    logger.info("ROS 2 Node and Executor stopped cleanly.")
                except Exception as e:
                    logger.error(f"Error shutting down ROS 2: {e}")
            self.node = None
            self.executor = None
            self.connected_since = None

    def get_node(self):
        """Returns the active rclpy node or None if in simulation mode."""
        return self.node

    def get_status(self) -> dict:
        """Returns connection diagnostics and node health."""
        uptime = round(time.time() - self.connected_since, 2) if self.connected_since else 0.0
        return {
            "is_running": self.is_running,
            "is_simulated": self.is_simulated,
            "node_name": settings.ROS_NODE_NAME,
            "domain_id": settings.ROS_DOMAIN_ID,
            "uptime_seconds": uptime,
            "rclpy_available": RCLPY_AVAILABLE,
        }


ros_connection = ROSConnection()
