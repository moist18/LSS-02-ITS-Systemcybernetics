#!/bin/bash

echo "=== MAVROS Timing Debug Script ==="
echo ""

# Check serial port lock
echo "1. Checking serial port /dev/ttyUSB0..."
lsof /dev/ttyUSB0 2>/dev/null
if [ $? -eq 0 ]; then
    echo "   ⚠️  Port is locked by process above"
else
    echo "   ✅ Port is free"
fi
echo ""

# Check if ArduPilot is responsive
echo "2. Checking ArduPilot heartbeat directly..."
timeout 5s mavproxy.py --master=/dev/ttyUSB0 --baudrate=57600 --cmd="status" 2>&1 | head -20
echo ""

# Monitor MAVROS initialization
echo "3. Starting MAVROS and monitoring plugin load time..."
ros2 launch launcher mavros.launch.py &
MAVROS_PID=$!

START_TIME=$(date +%s)

# Wait for heartbeat
echo "   Waiting for /mavros/state to show connected..."
while true; do
    CONNECTED=$(ros2 topic echo /mavros/state --once 2>/dev/null | grep "connected: true")
    CURRENT_TIME=$(date +%s)
    ELAPSED=$((CURRENT_TIME - START_TIME))

    if [ ! -z "$CONNECTED" ]; then
        echo "   ✅ Heartbeat received after ${ELAPSED} seconds"
        break
    fi

    if [ $ELAPSED -gt 99999 ]; then
        echo "   ❌ Timeout after 99999 seconds"
        break
    fi

    echo "   Still waiting... (${ELAPSED}s)"
    sleep 2
done

# Cleanup
kill $MAVROS_PID 2>/dev/null
echo ""
echo "=== Debug Complete ==="
