# GraspingDemo - SO-101 Robot Control System

A complete robotic grasping system with web interface, computer vision, and motion planning for the SO-101 6-DOF robot arm.

## Features

- **Web Control Interface** - Browser-based robot control with 3D visualization
- **Computer Vision** - RealSense depth camera integration with point cloud processing
- **Grasp Detection** - ThinkGrasp AI-powered grasp pose prediction
- **Motion Planning** - Collision-free trajectory planning with IK solver
- **Hand-Eye Calibration** - Camera-robot coordinate transformation
- **LeRobot Integration** - Learning from demonstration capabilities

## Quick Start

```bash
# 1. Install dependencies
pip install -r requirements.txt

# 2. Connect hardware
# - SO-101 robot to USB port (usually /dev/ttyACM0 or /dev/ttyUSB0)
# - Intel RealSense camera to USB 3.0 port

# 3. Launch web interface
cd web_control
python app.py

# 4. Open browser to http://localhost:5000
```

## Project Structure

```
GraspingDemo/
├── web_control/              # Web interface and APIs
│   ├── app.py               # Flask server with all endpoints
│   ├── templates/           # HTML interfaces
│   ├── static/              # CSS, JS, 3D models
│   └── *.py                 # Camera, visualization, planning modules
├── so101_grasp/             # Core robot control library
│   ├── robot/               # Kinematics, motion planning, client
│   ├── vision/              # Camera and point cloud processing
│   ├── api/                 # ThinkGrasp integration
│   └── utils/               # Configuration and transforms
├── examples/                # Usage examples
├── scripts/                 # Utility scripts
├── lerobot/                 # LeRobot teleoperation
└── captures/                # Saved point clouds and trajectories
```

## Installation

### Requirements
- Python 3.8+
- Ubuntu 20.04/22.04/24.04 or macOS
- Intel RealSense D435/D435i/D415 camera
- SO-101 robot with Feetech servos

### Install Steps

```bash
# Clone repository
git clone <repository>
cd GraspingDemo

# Install Python packages
pip install flask numpy opencv-python pyrealsense2 plotly open3d
pip install feetech-servo-sdk dynamixel-sdk
pip install torch torchvision  # For AI features



# Set permissions for robot port
sudo chmod a+rw /dev/ttyACM0  # Linux
# On macOS: /dev/tty.usbmodem*
```

## Web Interface Usage

### Starting the Server
```bash
cd web_control
python app.py
# Server runs on http://localhost:5000
```

### Available Interfaces

1. **Main Control** (`http://localhost:5000`)
   - Connect/disconnect robot
   - Enable/disable motors
   - Home position control
   - Trajectory recording/replay

2. **Modern Interface** (`http://localhost:5000/modern`)
   - Cartesian control (X,Y,Z position)
   - Gripper rotation
   - Inverse kinematics
   - 3D robot visualization

3. **Unified Interface** (`http://localhost:5000/unified`)
   - Camera view with point cloud
   - Grasp detection and execution
   - Combined robot and vision control

4. **Camera Interface** (`http://localhost:5000/camera`)
   - RGB/Depth streaming
   - Point cloud capture
   - Hand-eye calibration

## API Endpoints

### Robot Control
- `POST /api/connect` - Connect to robot
- `POST /api/disconnect` - Disconnect robot
- `GET /api/status` - Get robot status
- `POST /api/enable_torque` - Enable motors
- `POST /api/home` - Go to home position
- `POST /api/move_to_position` - Move joints
- `POST /api/cartesian_move` - Move to XYZ position
- `POST /api/gripper/<open|close>` - Control gripper

### Camera Control
- `POST /api/camera/connect` - Connect camera
- `GET /api/camera/rgb` - Get RGB stream
- `GET /api/camera/depth` - Get depth stream
- `GET /api/camera/pointcloud` - Get point cloud
- `POST /api/camera/capture` - Save capture

### Grasp Detection
- `POST /api/grasp/detect` - Detect grasp poses
- `POST /api/grasp/execute` - Execute grasp
- `POST /api/grasp/visualize` - Visualize grasps

### Trajectory Management
- `POST /api/record/start` - Start recording
- `POST /api/record/keyframe` - Add keyframe
- `POST /api/record/stop` - Stop and save
- `GET /api/trajectories` - List saved trajectories
- `POST /api/replay` - Replay trajectory

## Command Line Tools

### Test Connection
```bash
python scripts/test_connection.py
```

### Calibrate Robot
```bash
python scripts/calibrate_robot.py --port /dev/ttyACM0
```

### Basic Control Examples
```bash
# Test robot movement
python examples/basic_control.py

# Record and replay trajectories
python examples/keyframe_recorder.py

# Test kinematics
python examples/test_kinematics.py
```

### Camera Tools
```bash
# Capture point cloud
python so101_grasp/tools/capture_pointcloud.py

# Test camera
python scripts/test_realsense.py
```

## Configuration

Edit configuration files in `config/`:
- `robot_config.yaml` - Robot parameters, joint limits
- `camera_config.yaml` - Camera settings
- `grasp_config.yaml` - Grasp planning parameters

Or use environment variables:
```bash
export ROBOT_PORT=/dev/ttyACM0
export CAMERA_WIDTH=640
export CAMERA_HEIGHT=480
```

## ThinkGrasp Integration

The system uses ThinkGrasp for AI-powered grasp detection:

```python
# Automatic in web interface, or manual:
from so101_grasp.api import grasp_predictor

# Get grasp from point cloud
result = grasp_predictor.predict(points, colors, masks)
grasp_pose = result['grasp_pose']
confidence = result['confidence']
```

## Troubleshooting

### Robot Not Found
```bash
# Find robot port
ls /dev/tty* | grep -E "(ACM|USB)"
# or
python -m lerobot.find_port
```

### Camera Not Detected
```bash
# Test with RealSense viewer
realsense-viewer

# Check USB 3.0 connection
lsusb | grep Intel
```

### Permission Errors
```bash
# Linux
sudo usermod -a -G dialout $USER
sudo chmod a+rw /dev/ttyACM0

# Logout and login again
```

### Motor Issues
```bash
# Release all motors
python scripts/tools/disable_torque.py

# Reset to home
python examples/basic_control.py
```

## Development

### Running Tests
```bash
python -m pytest tests/
```

### Adding New Features
1. Add endpoint in `web_control/app.py`
2. Add UI in `web_control/templates/`
3. Add robot control in `so101_grasp/robot/`

### Code Structure
- **Web Layer**: Flask routes and WebSocket handlers
- **API Layer**: RESTful endpoints for all operations
- **Control Layer**: Robot kinematics and motion planning
- **Vision Layer**: Camera and point cloud processing
- **AI Layer**: ThinkGrasp integration for grasp detection

## Demo Videos

### Hand-Eye Calibration
- [Hand-eye calibration demo](https://www.loom.com/share/31e41d660eb142b796f7d9ee47fdbe7a)

### Grasping Demonstrations
- [Grasp demo 1](https://www.loom.com/share/3ee0907c5f69443aad677933d6fe5c82)
- [Grasp demo 2](https://www.loom.com/share/ab68494d061c4371900a20091ee867a7)

### Manipulation Demonstrations
- [Manipulation demo 1](https://www.loom.com/share/98f073d6da48446982a7f75057f021ab)
- [Manipulation demo 2](https://www.loom.com/share/74cc9401edb84139bc7283ca1b34c31b)

### Trajectory Replay
- [Replay demo](https://www.loom.com/share/d6c48b19ec5b417e90cadcb8e4c9bc76)

## License

Apache License 2.0