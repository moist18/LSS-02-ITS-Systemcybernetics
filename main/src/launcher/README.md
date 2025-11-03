# Launcher Package

ROS 2 launch files and configurations for ASV mission execution.

## TMUX Cheatsheet

### Basic Commands
- `tmux` - Start tmux
- `tmux a` - Attach to tmux
- `ctrl+d` - Exit tmux

### Session Management
- `ctrl+b d` - Detach session
- `ctrl+b c` - Create new session
- `ctrl+b n` - Next session
- `ctrl+b p` - Previous session
- `ctrl+b w` - Switch to window

## Running Launch Files

Launch files in 2 terminals using tmux:

1. **MAVROS (Terminal 1)**
   ```bash
   ros2 launch launcher mavros.launch.py
   ```

2. **Mission (Terminal 2)**
   ```bash
   ros2 launch launcher {mission}.launch.py
   ```

### Available Missions
- `gps_rtk.launch.py` - GPS RTK mission (14 waypoints)
- `gps_vision.launch.py` - GPS Vision mission (8 waypoints)
- `gps_compass5.launch.py` - GPS Compass5 mission (5 waypoints)
- `gps_compass6.launch.py` - GPS Compass6 mission (6 waypoints)
