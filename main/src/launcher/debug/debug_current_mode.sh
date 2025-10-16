#!/bin/bash

echo "=== Checking Current Flight Controller Mode ==="
echo ""

echo "1. Current MAVROS State:"
ros2 topic echo /mavros/state --once 2>&1
echo ""

echo "2. If MAVROS not running, trying direct serial connection:"
timeout 3 mavproxy.py --master=/dev/ttyUSB0 --baudrate=57600 --cmd="mode" 2>&1 | grep -i mode || echo "Cannot connect to flight controller"
echo ""

echo "=== Common ArduPilot/ArduRover Modes ==="
echo "MANUAL     - Full manual control"
echo "ACRO       - Acrobatic mode"
echo "STEERING   - Manual steering, speed control"
echo "HOLD       - Hold position"
echo "LOITER     - Loiter at position"
echo "FOLLOW     - Follow mode"
echo "SIMPLE     - Simple mode"
echo "AUTO       - Autonomous mission"
echo "RTL        - Return to Launch"
echo "SMART_RTL  - Smart Return to Launch"
echo "GUIDED     - Guided mode (for waypoint nav)"
echo "INITIALISING - Boot sequence"
echo ""

echo "=== Failsafe/Emergency Modes ==="
echo "BRAKE      - Emergency brake"
echo "SAFE       - Safe mode"
echo "E-STOP     - Emergency stop"
echo ""

echo "If you see weird code like 'E20' or number, it might be custom mode code"
echo "Please share the exact output from 'ros2 topic echo /mavros/state'"
