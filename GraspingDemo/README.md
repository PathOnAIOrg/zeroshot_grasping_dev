# SO-101 Robot Grasping System

Intelligent grasping system for SO-101 robotic arm with computer vision, grasp prediction, and autonomous manipulation.

## 🚀 Quick Start


brew install librealsense

# 1. Test system connections
python scripts/test_connection.py

# 2. Calibrate robot
python scripts/calibrate_robot.py

# 3. Run basic control example
python examples/basic_control.py
```

## 📁 Project Structure

```
├── so101_grasp/                 # Core system package
│   ├── robot/                   # Robot control & calibration
│   ├── vision/                  # Camera & point cloud processing
│   ├── planning/                # Grasp prediction & API client
│   ├── control/                 # Trajectory execution & safety
│   ├── visualization/           # 3D visualization
│   └── utils/                   # Configuration & utilities
├── scripts/                     # Executable scripts
│   ├── test_connection.py       # Test robot & camera
│   ├── calibrate_robot.py       # Robot calibration
│   └── tools/                   # Utility tools
├── config/                      # Configuration files
├── examples/                    # Usage examples
├── lerobot/                     # LeRobot framework
└── third_party/                 # External dependencies
```

## 🛠️ Installation

### Prerequisites
- Python 3.8+
- SO-101 robotic arm with Feetech servos
- Intel RealSense depth camera (D435i recommended)

### Setup

```bash
# Clone and install
git clone <repository-url>
cd so101_grasping_system
pip install -r requirements.txt
pip install -e .

# Additional packages
pip install feetech-servo-sdk dynamixel-sdk meshcat

# LeRobot framework
cd lerobot && pip install -e . --no-deps
pip install draccus==0.10.0 pyserial huggingface-hub termcolor


# Optional: ROS integration (see ROS Setup Guide below)
sudo apt install python3-rosdep  # if using ROS
```

## 🤖 ROS Setup (Optional)

Some advanced features may require ROS. Follow these steps if you encounter ROS-related issues:

### 1. Check if ROS is installed
```bash
echo $ROS_DISTRO
```
If it prints nothing, ROS is likely not sourced.

### 2. Install ROS 2 Jazzy (Ubuntu 24.04)
For Ubuntu 24.04, first add the ROS 2 repository:
```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
```

Then install ROS 2 Jazzy:
```bash
sudo apt install ros-jazzy-desktop
```

### 3. Install rosdep
For Ubuntu (assuming ROS is installed via apt):
```bash
sudo apt update
sudo apt install python3-rosdep
```
For other systems, or if using pip:
```bash
pip3 install rosdep
```

### 4. Initialize rosdep (only once)
After installing:
```bash
sudo rosdep init
rosdep update
```

### 5. Ensure your shell sources ROS setup
Add the following to your shell configuration:
```bash
# For bash (~/.bashrc)
source /opt/ros/<your_ros_distro>/setup.bash

# For zsh (~/.zshrc)  
source /opt/ros/<your_ros_distro>/setup.zsh
```
Replace `<your_ros_distro>` with `noetic`, `foxy`, `humble`, etc. Then reload:
```bash
source ~/.bashrc  # or ~/.zshrc
```

### 6. Verify installation
```bash
which rosdep
rosdep --version
```

📖 **For complete ROS setup guide, see [README_ROS_SETUP.md](README_ROS_SETUP.md)**

## 🔧 Configuration

Configuration files in `config/`:
- `robot_config.yaml` - Robot parameters and limits
- `camera_config.yaml` - Camera settings and filters  
- `grasp_config.yaml` - Grasp planning parameters


## 📚 Usage Guide
### 1. Robot Calibration

Calibrate robot joint positions:
```bash
sudo chmod a+rw /dev/ttyACM0
python -m lerobot.calibrate \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM0 
    
    
    
python scripts/calibrate_robot.py --port /dev/ttyACM0
```

Options:
- `--port`: Specify robot port
- `--force`: Force recalibration
- `--config-path`: Custom calibration file path

### 2. System Testing

Test all hardware connections:
```bash
python scripts/test_connection.py
```

This will:
- Detect robot on available ports
- Test camera connection and capture
- Verify all systems are ready



### 3. Basic Examples

```bash
# Basic robot movement
python examples/basic_control.py

# Grasp demonstration
python examples/grasp_example.py

# Manual torque control (for troubleshooting)
python scripts/tools/disable_torque.py
```

## 🤖 API Reference

### Robot Control

```python
from so101_grasp.robot import SO101Client

# Initialize robot
client = SO101Client(port="/dev/ttyACM0", follower=True)

# Read joint positions
positions = client.read_joints()

# Move to position
target = [0.0, -0.5, 1.0, 0.0, 0.0, 0.0]
client.write_joints(target)

# Smooth interpolation
client.interpolate_waypoint(start_pos, end_pos, steps=50)
```

### Vision System

```python
from so101_grasp.vision import CameraController, PointCloudProcessor

# Initialize camera
camera = CameraController()
camera.connect()

# Capture RGB-D data
color, depth, intrinsics = camera.capture_rgbd()

# Process point cloud
processor = PointCloudProcessor()
points, colors = processor.rgbd_to_pointcloud(color, depth, intrinsics)
```


### Configuration Management

```python
from so101_grasp.utils import ConfigManager

config = ConfigManager()
robot_config = config.get_robot_config()
config.update_robot_port("/dev/ttyACM1")
```

## 🔍 Troubleshooting

### Robot Issues
- **Not found**: Run `python -m lerobot.find_port` to check ports
- **Too stiff**: Run `python scripts/tools/disable_torque.py`

### Camera Issues  
- **Not detected**: Check USB 3.0 connection, test with `realsense-viewer`
- **Poor depth**: Adjust lighting and camera position

### API Issues
- **Authentication failed**: Check GENERAL_BIONIX_API_KEY environment variable
- **Grasp prediction fails**: Ensure point cloud is 480x640 resolution, downsampled to 480x160

## 🏗️ Architecture

```
Camera → Point Cloud → API Grasp Prediction → Motion Planning → Robot Execution
   ↓           ↓               ↓                    ↓              ↓
Config → Calibration → Transform to Robot Frame → Safety Check → Feedback
```

Key modules:
- **Robot**: Hardware control and calibration
- **Vision**: Camera and point cloud processing  
- **Planning**: General Bionix API integration for grasp prediction
- **Control**: Trajectory execution and safety checks

## 📄 License

Apache License 2.0 - See LICENSE file for details.