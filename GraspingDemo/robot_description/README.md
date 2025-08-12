# SO101 Robot Description Files

This folder contains the URDF (Unified Robot Description Format) and STL mesh files for the SO101 robot arm.

## Directory Structure

```
robot_description/
├── urdf/               # URDF/XACRO robot description files
│   ├── so101_base.xacro       # Main robot structure and visual elements
│   ├── so101.urdf.xacro       # Top-level URDF that includes all components
│   ├── so101_gazebo.xacro     # Gazebo simulation parameters
│   └── so101_ros2_control.xacro # ROS2 control configuration
└── meshes/
    └── so101/          # STL mesh files for visualization
        ├── base_*.stl          # Base and mounting components
        ├── sts3215_*.stl       # Servo motor models
        ├── *_arm_*.stl         # Arm segments
        ├── wrist_*.stl         # Wrist components
        └── *_jaw_*.stl         # Gripper jaw parts
```

## Robot Specifications

- **Degrees of Freedom**: 6 (5 arm joints + 1 gripper)
- **Joints**:
  1. Base rotation (revolute)
  2. Shoulder pitch (revolute)
  3. Elbow flexion (revolute)
  4. Wrist pitch (revolute)
  5. Wrist roll (revolute)
  6. Gripper open/close (revolute)

## Usage in Web Control

The URDF and mesh files are automatically loaded by the web control interface to:
- Display the robot model in 3D visualizations
- Show current robot configuration based on joint angles
- Validate grasp poses and collision detection
- Provide visual feedback for robot control

## Coordinate System

The robot uses a standard right-handed coordinate system:
- **X-axis**: Points to the right (red in visualization)
- **Y-axis**: Points down (green in visualization)
- **Z-axis**: Points forward (blue in visualization)
- **Origin**: Robot base center

## File Formats

- **XACRO files**: XML macro files that expand to full URDF
- **STL files**: Binary 3D mesh files for visual representation
- All measurements in the URDF are in meters
- STL files use millimeters as their native unit

## Colors

The robot visualization uses material definitions:
- **3D Printed Parts**: Yellow/orange color (RGB: 1.0, 0.82, 0.12)
- **Servo Motors**: Dark gray color (RGB: 0.1, 0.1, 0.1)

## Credits

Based on the SO-ARM100 model from The Robot Studio, modified for the SO101 configuration.