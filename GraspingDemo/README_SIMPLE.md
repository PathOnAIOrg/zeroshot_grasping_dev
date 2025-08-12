# SO-101 Robot Control Guide

## Quick Start

### Non-ROS2 Version (Direct Control)

#### Record Trajectory
```bash
./run_robot.sh record
```
- Move robot manually
- Press ENTER to save poses
- Type 'done' to finish

#### Replay Trajectory
```bash
./run_robot.sh replay
```
- Select saved trajectory
- Watch robot replay

#### Test Gripper
```bash
./run_robot.sh gripper
```

#### Release Motors
```bash
./run_robot.sh release
```

---

### ROS2 Version (Topic-Based Control)

#### Record Trajectory
```bash
./ros2_direct_run.sh record
```
Same as non-ROS2 version - move manually, press ENTER

#### Replay Trajectory

**Terminal 1:**
```bash
./ros2_direct_run.sh reader
```

**Terminal 2:**
```bash
./ros2_direct_run.sh player trajectory_file.json
```

#### Interactive Control
```bash
./ros2_direct_run.sh controller
```
Then choose:
- 3 = Record trajectory
- 5 = Load trajectory
- 6 = Play trajectory

---

## File Locations

- Trajectories saved in: `trajectories/`
- Non-ROS2: `demo_trajectory_*.json`
- ROS2: `ros2_trajectory_*.json`

## Web UI Control

### Start Web Interface
```bash
cd web_control
./run_web_ui.sh
```
Open browser at: http://localhost:5000

**Features:**
- 🔌 Connect/disconnect robot
- 🎮 Enable/disable motors
- 🤏 Gripper control (open/close/middle)
- 📹 Record trajectories with manual movement
- 🔄 Replay saved trajectories
- 📊 Real-time joint position display

---

## Notes

- Both versions use same coordinate system
- Trajectory files are compatible between versions
- Gripper is joint 5 (6th joint)
- All scripts work with conda `sim` environment
- Web UI provides easiest control interface