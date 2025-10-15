#!/bin/bash

echo "================================================"
echo "Vision Navigation YAW Debugging Script"
echo "================================================"
echo ""
echo "This script will:"
echo "1. Enable vision control"
echo "2. Set mission to 'Floating Ball'"
echo "3. Monitor key topics"
echo ""
echo "After running this, set your remote to AUTO mode"
echo "and observe the logs to see why yaw gets stuck at 1500"
echo ""
echo "================================================"
echo ""

# Enable vision control
echo "▶ Setting vision control to TRUE..."
ros2 topic pub /robot/control/vision std_msgs/msg/Bool "{data: true}" --once
sleep 0.5

# Disable GPS control
echo "▶ Setting GPS control to FALSE..."
ros2 topic pub /robot/control/gps std_msgs/msg/Bool "{data: false}" --once
sleep 0.5

# Set mission to Floating Ball
echo "▶ Setting mission to 'Floating Ball'..."
ros2 topic pub /robot/mission std_msgs/msg/String "{data: 'Floating Ball'}" --once
sleep 0.5

echo ""
echo "✅ Configuration complete!"
echo ""
echo "================================================"
echo "Key Topics to Monitor:"
echo "================================================"
echo ""
echo "Vision Detection:"
echo "  ros2 topic echo /detect/object/bool"
echo ""
echo "Raw Yaw from Vision:"
echo "  ros2 topic echo /robot/vision/raw_yaw"
echo ""
echo "Current Mission:"
echo "  ros2 topic echo /robot/mission"
echo ""
echo "Vision Control Status:"
echo "  ros2 topic echo /robot/control/vision"
echo ""
echo "RC Override (what gets sent to MAVROS):"
echo "  ros2 topic echo /mavros/rc/override"
echo ""
echo "================================================"
echo "Watch the logs from vision_node and vision_navigation_node"
echo "They will now show detailed debugging information:"
echo "================================================"
echo ""
echo "Vision Node will show:"
echo "  - Mission status"
echo "  - Object detection status"
echo "  - Raw yaw values"
echo "  - Processing details"
echo ""
echo "Vision Nav Node will show:"
echo "  - Object detection status"
echo "  - Raw yaw received"
echo "  - Calculated RC yaw"
echo "  - Mission and control states"
echo ""
