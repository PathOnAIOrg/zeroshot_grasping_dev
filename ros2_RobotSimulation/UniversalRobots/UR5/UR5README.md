# UR5 Robot Simulation with Camera Setup

This repository contains a customized UR5 robot simulation setup with multiple camera views for ROS2 Foxy. The simulation includes both top-down and side-view cameras for comprehensive robot observation.

## Table of Contents
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Building the Workspace](#building-the-workspace)
- [Launch Files](#launch-files)
- [Camera Configuration](#camera-configuration)
- [Usage](#usage)
- [Troubleshooting](#troubleshooting)

## Prerequisites

Before installing this simulation, ensure you have:

- Ubuntu 20.04 LTS
- ROS2 Foxy installed
- Gazebo 11 installed
- Basic knowledge of ROS2 and Gazebo

### Install Gazebo
```bash
# Install Gazebo 11
sudo apt install gazebo11 libgazebo11-dev

# Install Gazebo ROS2 packages
sudo apt install ros-foxy-gazebo-ros-pkgs ros-foxy-gazebo-plugins
```

## Installation

### 1. Clone the Repository
```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone the repository
git clone <repository-url>
# Or if you have the files locally, copy them to the workspace
cp -r /path/to/ros2_RobotSimulation ~/ros2_ws/src/
```

### 2. Install Dependencies
```bash
# Install required packages
sudo apt install ros-foxy-cv-bridge ros-foxy-rqt-image-view ros-foxy-image-view
sudo apt install ros-foxy-moveit-ros-move-group ros-foxy-moveit-ros-planning-interface
sudo apt install ros-foxy-controller-manager ros-foxy-ros2-control ros-foxy-ros2-controllers
```

## Building the Workspace

```bash
cd ~/ros2_ws

# Install colcon if not already installed
sudo apt install python3-colcon-common-extensions

# Build the workspace
colcon build --packages-select ur5_ros2_gazebo ur5_ros2_moveit2

# Source the workspace
source install/setup.bash
```

## Launch Files

This setup includes two main launch files:

### 1. UR5 Camera Simulation (`ur5_camera_simulation.launch.py`)
- **Location**: `UniversalRobots/UR5/ur5_ros2_gazebo/launch/ur5_camera_simulation.launch.py`
- **Purpose**: Launches the UR5 robot simulation with multiple cameras
- **Features**:
  - UR5 robot in Gazebo
  - Top-down camera
  - Side-view camera
  - Robot state publisher
  - Joint controllers

### 2. UR5 Camera Interface (`ur5_camera_interface.launch.py`)
- **Location**: `UniversalRobots/UR5/ur5_ros2_moveit2/launch/ur5_camera_interface.launch.py`
- **Purpose**: Launches MoveIt2 interface with camera views
- **Features**:
  - MoveIt2 planning interface
  - RViz visualization
  - Camera feeds
  - Robot control interface

## Camera Configuration

### Camera Positions

#### Top-Down Camera
- **Position**: `<pose>0.5 0 1.5 -3 1.5 -3</pose>`
- **Purpose**: Bird's-eye view of the robot workspace
- **Topics**:
  - Image: `/topdown_camera_sensor/image_raw`
  - Info: `/topdown_camera_sensor/camera_info`

#### Side-View Camera
- **Position**: `<pose>0.4 0.8 0.1 0 0 -1.5</pose>`
- **Purpose**: Side view for detailed robot observation
- **Topics**:
  - Image: `/sideview_camera/image_raw`
  - Info: `/sideview_camera/camera_info`

### Camera Parameters
- **Resolution**: 1280x720
- **Frame Rate**: 30 FPS
- **Field of View**: 90 degrees
- **Format**: RGB8

## Usage

### 1. Launch Camera Simulation

```bash
# Navigate to workspace
cd ~/ros2_ws

# Source the workspace
source install/setup.bash

# Launch the camera simulation
ros2 launch ur5_ros2_gazebo ur5_camera_simulation.launch.py
```

When prompted:
- **Cell Layout**: Choose option 1 (UR5 ROBOT alone) or option 2 (UR5 ROBOT on pedestal)
- **End-Effector**: Choose option 1 (No end-effector)

### 2. View Camera Feeds

#### Using rqt_image_view
```bash
# View top-down camera
ros2 run rqt_image_view rqt_image_view
# Select: /topdown_camera_sensor/image_raw

# View side-view camera
ros2 run rqt_image_view rqt_image_view
# Select: /sideview_camera/image_raw
```

#### Using Python Script
```bash
# Run the topdown-camera viewer
python3 scripts/camera_viewer.py
```

### 3. Launch MoveIt2 Interface

```bash
# Launch MoveIt2 with camera interface
ros2 launch ur5_ros2_moveit2 ur5_camera_interface.launch.py
```

### 4. Check Available Topics

```bash
# List all camera topics
ros2 topic list | grep camera

# Check camera info
ros2 topic echo /topdown_camera_sensor/camera_info
ros2 topic echo /sideview_camera/camera_info

# Check image publishing
ros2 topic hz /topdown_camera_sensor/image_raw
```

## Customization

### Modifying Camera Positions

Edit the world file: `UniversalRobots/UR5/ur5_ros2_gazebo/worlds/ur5_camera.world`

#### Top-Down Camera
```xml
<!-- Top-Down Camera -->
<model name="topdown_camera">
    <static>true</static>
    <pose>0.5 0 1.5 -3 1.5 -3</pose>  <!-- x y z roll pitch yaw -->
    <!-- ... rest of camera configuration ... -->
</model>
```

#### Side-View Camera
```xml
<!-- Side-View Camera -->
<model name="sideview_camera">
    <static>true</static>
    <pose>0.4 0.8 0.1 0 0 -1.5</pose>  <!-- x y z roll pitch yaw -->
    <!-- ... rest of camera configuration ... -->
</model>
```

### Adding New Cameras

To add a new camera, copy the camera model structure and modify:
1. Model name
2. Position (`<pose>`)
3. Camera name in plugin
4. Topic names


## File Structure

```
ros2_RobotSimulation/
├── UniversalRobots/
│   └── UR5/
│       ├── ur5_ros2_gazebo/
│       │   ├── launch/
│       │   │   └── ur5_camera_simulation.launch.py
│       │   └── worlds/
│       │       └── ur5_camera.world
│       └── ur5_ros2_moveit2/
│           └── launch/
│               └── ur5_camera_interface.launch.py
└── scripts/
    └── camera_viewer.py
```

## Contributing

To contribute to this project:
1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## License

This project is licensed under the Apache-2.0 License.

## Support

For issues and questions:
1. Check the troubleshooting section
2. Review the ROS2 and Gazebo documentation
3. Open an issue on the repository

## Acknowledgments

- IFRA Group at Cranfield University for the original ROS2 Robot Simulation
- Universal Robots for the UR5 robot model
- ROS2 and Gazebo communities for the simulation framework 