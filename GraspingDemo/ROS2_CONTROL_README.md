# ROS2 Control for SO-101 Robot

## Requirements

- ROS2 Jazzy
- Python 3.10+

## Overview

This provides ROS2 control for the SO-101 robot, allowing you to:
- Control the real robot through ROS2 topics
- Play back recorded trajectories
- Record new trajectories

## Setup

1. Build the ROS2 package:
```bash
./ros2_control.sh build
```

## Usage

### Method 1: Direct Trajectory Playback

**Terminal 1** - Start hardware interface:
```bash
./ros2_control.sh reader
```

**Terminal 2** - Play trajectory:
```bash
./ros2_control.sh player demo_trajectory_20250811_023603.json
```

### Method 2: Interactive Control

**Terminal 1** - Start hardware interface:
```bash
./ros2_control.sh reader
```

**Terminal 2** - Start controller:
```bash
./ros2_control.sh controller
```

Then use the menu to:
- Enable/disable servos
- Record trajectories
- Play trajectories
- Send to home position

## ROS2 Topics

- `/joint_states` - Current robot joint positions (published by reader)
- `/joint_commands` - Commanded joint positions (subscribed by reader)
- `/robot/cmd_pose` - Twist commands for IK control

## Monitor Topics

View current joint states:
```bash
./ros2_control.sh echo-states
```

View commanded positions:
```bash
./ros2_control.sh echo-commands
```

## Trajectory Files

Trajectories are compatible between:
- GraspingDemo format (from `simple_trajectory_demo.py`)
- ROS2 format (from `joint_controller`)

Both can be played using the `trajectory_player` node.

## Example Workflow

1. Record trajectory with non-ROS2 script:
```bash
./run_robot.sh record
```

2. Play it back through ROS2:
```bash
# Terminal 1
./ros2_control.sh reader

# Terminal 2
./ros2_control.sh player gripper_trajectory_20250811_123456.json
```

## Commands Reference

```bash
./ros2_control.sh help       # Show help
./ros2_control.sh reader     # Start hardware interface
./ros2_control.sh player FILE # Play trajectory
./ros2_control.sh controller # Interactive control
./ros2_control.sh build      # Build packages
./ros2_control.sh topics     # List topics
```

## Notes

- The `joint_state_reader.py` now accepts commands on `/joint_commands`
- All 6 joints are controlled (including gripper)
- Coordinates match the raw servo values (no calibration offsets)
- Compatible with trajectories recorded by `simple_trajectory_demo.py`