#!/bin/bash

# Setup script for ASV-2025 ROS2 environment
# This script sources the libs workspace first, then the main workspace

echo "Setting up ASV-2025 ROS2 Environment..."

# Source ROS2 Humble base
source /opt/ros/humble/setup.bash

# Source libs workspace first (contains mavros_msgs and other dependencies)
if [ -f "/home/arielsulton/ASV-2025/libs/install/setup.bash" ]; then
    echo "Sourcing libs workspace..."
    source /home/arielsulton/ASV-2025/libs/install/setup.bash
else
    echo "Warning: libs workspace not found at /home/arielsulton/ASV-2025/libs/install/setup.bash"
fi

# Source main workspace
if [ -f "/home/arielsulton/ASV-2025/main/install/setup.bash" ]; then
    echo "Sourcing main workspace..."
    source /home/arielsulton/ASV-2025/main/install/setup.bash
else
    echo "Warning: main workspace not found. Run 'colcon build' first."
fi

echo "Environment setup complete!"

# Test if mavros_msgs is available
echo "Testing mavros_msgs availability..."
python3 -c "
try:
    from mavros_msgs.msg import OverrideRCIn
    print('✓ mavros_msgs.msg.OverrideRCIn imported successfully')
except ImportError as e:
    print('✗ Failed to import mavros_msgs.msg.OverrideRCIn:', e)
"