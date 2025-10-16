#!/bin/bash

echo "=== E20 Mode Diagnostic ==="
echo ""

echo "1. MAVROS State Check:"
ros2 topic echo /mavros/state --once
echo ""

echo "2. Battery Status:"
ros2 topic echo /mavros/battery --once | head -10
echo ""

echo "3. RC Status:"
ros2 topic echo /mavros/rc/in --once | head -15
echo ""

echo "4. GPS Status:"
ros2 topic echo /mavros/global_position/global --once
echo ""

echo "5. System Status:"
ros2 topic echo /mavros/state/system_status --once
echo ""

echo "6. Flight Controller Parameters (failsafe settings):"
ros2 param get /mavros/param FS_EKF_ACTION 2>/dev/null || echo "Parameter not available"
ros2 param get /mavros/param FS_EKF_THRESH 2>/dev/null || echo "Parameter not available"
ros2 param get /mavros/param FS_BATT_ENABLE 2>/dev/null || echo "Parameter not available"
ros2 param get /mavros/param FS_GCS_ENABLE 2>/dev/null || echo "Parameter not available"
echo ""

echo "=== Possible Solutions ==="
echo "1. RC Signal: Check RC transmitter is ON and in range"
echo "2. Battery: Check voltage (should be >11V for 3S, >14V for 4S)"
echo "3. GPS: Ensure GPS has satellite lock (outdoors, clear sky)"
echo "4. Reset E-Stop: Cycle power or use GCS to clear failsafe"
echo ""
