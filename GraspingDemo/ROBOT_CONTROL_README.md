# SO-101 Robot Control Guide

## Quick Start

### 1. Basic Control (with calibration)
```bash
./run_robot.sh basic
```
- Uses calibration offsets
- Original coordinate system

### 2. Raw Control (no calibration) 
```bash
./run_robot.sh raw
```
- Direct servo control
- Same coordinates as ROS2 joint_state_reader
- Robot becomes free to move after script ends

### 3. Record & Replay Trajectory
```bash
./run_robot.sh record
```
- Move robot manually and record poses
- Press ENTER to save each pose
- Type 'done' to finish
- Automatically replays the trajectory

### 4. Replay Saved Trajectories
```bash
./run_robot.sh replay
```
- Shows list of saved trajectories
- Choose one to replay
- Adjust playback speed

### 5. Test Gripper
```bash
./run_robot.sh gripper
```
- Test gripper open/close
- Interactive gripper control
- Gripper is joint 5 (6th joint)

### 6. Release Robot (unlock motors)
```bash
./run_robot.sh release
```
- Disables all motor torque
- Use when robot is locked

## File Descriptions

| Script | Purpose |
|--------|---------|
| `basic_control.py` | Test robot with calibration |
| `basic_control_raw.py` | Test robot without calibration (raw servo values) |
| `simple_trajectory_demo.py` | Record and replay robot movements (including gripper) |
| `replay_trajectory.py` | Replay previously saved trajectories |
| `gripper_test.py` | Test gripper functionality |
| `release_torque.py` | Unlock robot motors |

## Trajectories

All recorded trajectories are saved in `trajectories/` folder as JSON files.

## Gripper Information

The gripper is **joint 5** (6th joint, index 5):
- **Negative values** (-3.14 to 0): Gripper CLOSING/CLOSED
- **Positive values** (0 to 3.14): Gripper OPENING/OPEN  
- **0**: Middle position

Example gripper values:
- `-1.5` rad: Closed
- `0.0` rad: Middle
- `1.5` rad: Open

## Notes

- Robot port: `/dev/ttyACM0`
- Uses conda environment: `sim`
- Raw mode matches ROS2 coordinate system exactly
- All trajectories include gripper position (6 joints total)