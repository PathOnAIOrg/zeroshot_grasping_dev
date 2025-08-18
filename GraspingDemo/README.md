# GraspingDemo - SO-101 Robot Control System

Streamlined web-based control system for SO-101 6-DOF robot arm with RealSense camera integration.

## Features

- **Web Control Interface** - Browser-based robot control
- **RealSense Integration** - Depth camera and point cloud processing  
- **ThinkGrasp AI** - Advanced grasp detection
- **Trajectory Recording** - Record and replay robot movements
- **Real-time Visualization** - 3D point cloud and robot state

## Quick Start

```bash
# 1. Install dependencies
pip install -r requirements.txt
pip install feetech-servo-sdk dynamixel-sdk

# 2. Launch web interface
cd web_control
python app.py

# 3. Open browser to http://localhost:5000
```

## Project Structure

```
GraspingDemo/
├── web_control/             # Main application
│   ├── app.py              # Flask server
│   ├── templates/          # Web interfaces
│   ├── static/             # CSS, JS, assets
│   └── *.py                # Modules (camera, grasp, visualization)
├── so101_grasp/            # Core robot control
│   ├── robot/              # Robot client & kinematics
│   └── utils/              # Configuration
├── examples/               # Usage examples
├── scripts/                # Utility scripts
└── trajectories/           # Saved trajectories
```

## Installation

### Requirements
- Python 3.8+
- Ubuntu 20.04/22.04 or macOS
- Intel RealSense camera
- SO-101 robot

### Setup

```bash
# Clone repository
git clone <repository>
cd GraspingDemo

# Install packages
pip install -r requirements.txt
pip install feetech-servo-sdk dynamixel-sdk

# Set robot port permissions (Linux)
sudo chmod a+rw /dev/ttyACM0
```

## Usage

### Web Interface

Three interfaces available:

1. **Main** (`http://localhost:5000`) - Basic robot control
2. **Modern** (`http://localhost:5000/modern`) - Advanced features with 3D viz
3. **Unified** (`http://localhost:5000/unified`) - Combined robot & camera control

### Launch Scripts

```bash
# Web UI
./run_web_ui.sh

# Basic robot control
./run_robot.sh basic

# ROS2 integration (optional)
./ros2_control.sh reader
```

### Test Connection

```bash
python scripts/test_connection.py
```

## API Reference

### Robot Control
- `POST /api/connect` - Connect robot
- `POST /api/home` - Home position
- `POST /api/move_to_position` - Move joints
- `POST /api/cartesian_move` - Move to XYZ
- `POST /api/gripper/<open|close>` - Gripper control

### Camera
- `POST /api/camera/connect` - Connect camera
- `GET /api/camera/rgb` - RGB stream
- `GET /api/camera/pointcloud` - Point cloud

### Grasp Detection
- `POST /api/grasp/detect` - Detect grasps
- `POST /api/grasp/execute` - Execute grasp

### Trajectories
- `POST /api/record/start` - Start recording
- `POST /api/record/stop` - Save trajectory
- `POST /api/replay` - Replay trajectory

## Examples

### Basic Control
```python
from so101_grasp.robot import SO101ClientRawSimple

# Connect
robot = SO101ClientRawSimple(port="/dev/ttyACM0")

# Read position
pos = robot.read_joints()

# Move
robot.write_joints([0, 0, 0, 0, 0, 0])
```

### Camera Capture
```python
import pyrealsense2 as rs

pipeline = rs.pipeline()
pipeline.start()
frames = pipeline.wait_for_frames()
```

## Troubleshooting

### Robot Not Found
```bash
ls /dev/tty* | grep -E "(ACM|USB)"
```

### Camera Not Detected
```bash
# Check connection
lsusb | grep Intel

# Test with viewer
realsense-viewer
```

### Permission Issues
```bash
sudo usermod -a -G dialout $USER
sudo chmod a+rw /dev/ttyACM0
```

## License

Apache License 2.0