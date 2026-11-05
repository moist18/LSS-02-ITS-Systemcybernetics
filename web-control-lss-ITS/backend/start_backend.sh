#!/usr/bin/env bash
# ==============================================================================
# LSS-02 USV Marine Command Cockpit - Backend Launcher
# Automatically sources ROS 2 Humble environment and starts FastAPI Gateway.
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Source ROS 2 Humble if installed
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "[INFO] Sourcing ROS 2 Humble environment..."
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
else
    echo "[WARN] ROS 2 Humble not found at /opt/ros/humble/setup.bash. Running in simulated fallback mode."
fi

# Select Python executable (prefer /usr/bin/python3 which has ROS2 rclpy bindings)
if [ -x "/usr/bin/python3" ]; then
    PYTHON_BIN="/usr/bin/python3"
else
    PYTHON_BIN="python3"
fi

echo "[INFO] Using Python: $($PYTHON_BIN --version) at $PYTHON_BIN"
echo "[INFO] Starting LSS-02 FastAPI Backend on host 0.0.0.0 port 8000..."

exec "$PYTHON_BIN" -m uvicorn main:app --host 0.0.0.0 --port 8000 --reload
