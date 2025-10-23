# Bashrc Configuration Files

Bash configuration files for ASV-2025 deployment with rosbridge WebSocket gateway.

## `.bashrc-asv` - Robot Configuration

### Automatic DDS Setup
CycloneDDS automatically configured for **loopback only** on terminal startup.

```bash
# DDS Config (auto-generated on bashrc load)
# Interface: lo (loopback only - WiFi crash protection)
# File: ~/.config/cyclonedds/cyclonedds.xml
```

### `ros_net_tune`
Network performance optimization (requires sudo).

```bash
ros_net_tune
```

## `.bashrc-laptop` - Laptop Functions

### `test_rosbridge()`
Test rosbridge WebSocket connection to robot.

```bash
test_rosbridge              # Test connection to ASV_IP:9090
```

### `ros_net_tune`
Network performance optimization (requires sudo).

```bash
ros_net_tune
```

## Installation

### ASV Robot

**Install rosbridge_server:**
```bash
sudo apt update
sudo apt install ros-humble-rosbridge-server
```

**Install bashrc configuration:**
```bash
cp bashrc/.bashrc-asv ~/.bashrc
source ~/.bashrc
```

### Laptop

**Install Web-GCS dependencies:**
```bash
cd ~/ASV-2025/laptop/web-gcs/backend
pip3 install -r requirements.txt
```

**Install bashrc configuration:**
```bash
cp bashrc/.bashrc-laptop ~/.bashrc
source ~/.bashrc
```

**Create .env file:**
```bash
cd ~/ASV-2025/laptop/web-gcs/backend
cp .env.example .env
# Edit ROBOT_HOST=192.168.0.11 if needed
```

## Usage

### Single Command Launch (Recommended)

**ASV Robot (1 terminal):**
```bash
# Launch mission with rosbridge included
ros2 launch launcher gps_vision.launch.py     # GPS Vision mission
ros2 launch launcher gps_rtk.launch.py        # GPS RTK mission
ros2 launch launcher gps_compass.launch.py    # GPS Compass mission
```

**Laptop (1 terminal):**
```bash
bun start                                     # frontend
uvicorn main:app --host 0.0.0.0 --port 8000   # backend
# Open browser: http://localhost:3000 (frontend)
```

## Architecture

```
ASV Robot (192.168.0.11)
  ├─ Core Nodes (DDS on lo only)     → Safe from WiFi drops
  └─ rosbridge (DDS lo + WS 0.0.0.0) → WebSocket gateway

Laptop (192.168.0.x)
  └─ Web-GCS Backend (roslibpy)      → WebSocket client (No DDS!)
```

**Key Features:**
- **DDS Isolation**: Core nodes use loopback only
- **WebSocket Bridge**: rosbridge translates DDS ↔ JSON
- **Auto-reconnect**: Backend reconnects when rosbridge available
- **WiFi Resilient**: No DDS crashes on network issues

---

**Environment Variables:**
- ASV: `ROS_DOMAIN_ID=7`, `CYCLONEDDS_URI=file://~/.config/cyclonedds/cyclonedds.xml`
- Laptop: `ASV_IP="192.168.0.11"`, `ROSBRIDGE_URL="ws://192.168.0.11:9090"`
