# mycobot_ros2 #
![OS](https://img.shields.io/ubuntu/v/ubuntu-wallpapers/noble)
![ROS_2](https://img.shields.io/ros/v/jazzy/rclcpp)

## Overview
This repository contains ROS 2 packages for simulating and controlling the myCobot robotic arm using ROS 2 Control and MoveIt 2. It provides support for Gazebo simulation and visualization in RViz. Gazebo simulation also includes simulated 3D point cloud data from the depth camera (RGBD) sensor plugin for vision.

![Gazebo Pick and Place Task Simulation](https://automaticaddison.com/wp-content/uploads/2024/12/pick-place-gazebo-800-fast.gif)

![Pick and Place with Perception](https://automaticaddison.com/wp-content/uploads/2024/12/pick-place-demo-rviz-800-fast.gif)

## Features
- Gazebo simulation of the myCobot robotic arm
- RViz visualization for robot state and motion planning
- MoveIt 2 integration for motion planning and control
- Pick and place task implementation using the MoveIt Task Constructor (MTC)
- 3D perception and object segmentation using point cloud data
- Automatic planning scene generation from perceived objects
- Support for various primitive shapes (cylinders, boxes) in object detection
- Integration with tf2 for coordinate transformations
- Custom service for retrieving planning scene information
- Advanced object detection algorithms:
  - RANSAC (Random Sample Consensus) for robust model fitting
  - Hough transform for shape recognition
- CPU-compatible implementation, no GPU required. 
- Real-time perception and planning capabilities for responsive robot operation

![Setup Planning Scene](https://automaticaddison.com/wp-content/uploads/2024/12/creating-planning-scene-800.gif)


## Quick Install (PathOn AI)
```bash
cd mycobot_ros2     # go to the root directory of mycobot_ros2 folder


# install required packages
sudo apt update
sudo apt install ros-jazzy-moveit-ros-planning-interface
sudo apt install ros-jazzy-urdf-tutorial
sudo apt install ros-jazzy-controller-manager
sudo apt install ros-jazzy-moveit-msgs
sudo apt install ros-jazzy-moveit-common ros-jazzy-moveit-core
sudo apt install ros-jazzy-control-msgs

# install the MoveIt Task Constructor Package
# refer to https://automaticaddison.com/how-to-set-up-the-moveit-2-task-constructor-ros-2-jazzy/
cd mycobot_ros2
git clone https://github.com/moveit/moveit_task_constructor.git -b jazzy
cd moveit_task_constructor
git reset --hard 3b2a436f0a7e8dbb4d347960430bc26183d99535
colcon build

```

## Run Demos
```bash

# remember to source your terminal before running
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

cd opensource_dev/mycobot_ros2
bash ./mycobot_mtc_pick_place_demo/scripts/robot.sh         # launch gazebo and rviz (moveit), tested
bash ./mycobot_mtc_pick_place_demo/scripts/pointcloud.sh    # launch point cloud related modules, to be tested
```
```
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
# if you add more ros-related code in the mycobot_ros2 folder, remember to colcon build it
colcon build --symlink-install
source install/setup.bash    # use source install/setup.zsh if needed
```

## Getting Started
For a complete step-by-step walkthrough on how to build this repository from scratch, start with this tutorial:
[Create and Visualize a Robotic Arm with URDF](https://automaticaddison.com/create-and-visualize-a-robotic-arm-with-urdf-ros-2-jazzy/)

This guide will take you through the entire process of setting up and understanding the mycobot_ros2 project.

![3D Point Cloud RViz](https://automaticaddison.com/wp-content/uploads/2024/12/800_3d-point-cloud.jpg)

![mycobot280_rviz](./mycobot_description/urdf/mycobot280_rviz.png)
