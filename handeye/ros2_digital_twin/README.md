# 🤖 SO-101 Digital Twin with RealSense in RViz2

Complete digital twin system for SO-101 robot with RealSense camera integration, supporting both eye-in-hand and eye-to-hand configurations.

## 📁 Project Structure

```
ros2_digital_twin/
├── README.md                     # This file
├── start_digital_twin.py         # 🚀 Main launcher (USE THIS!)
├── sync_real_robot.py            # Real robot synchronization
├── real_robot_sync.py            # Alternative sync implementation
├── connect_camera_to_robot.py    # Eye-in-hand camera connection
├── connect_camera_to_robot_stationary.py  # Eye-to-hand camera connection
│
├── config/                       # Configuration files
│   ├── so101_digital_twin.rviz  # RViz2 configuration
│   └── simple_robot.rviz        # Simple RViz config
│
├── scripts/                      # Shell scripts
│   ├── launch_colored_pointcloud.sh  # RealSense launcher
│   ├── launch_digital_twin.sh        # Alternative launcher
│   ├── test_digital_twin.sh          # System test
│   └── fix_tf_tree.sh               # TF tree fixer
│
├── launch/                       # ROS2 launch files
│   ├── launch_complete.py       # Complete launch script
│   └── launch_so101_real.launch.py  # ROS2 launch file
│
└── utils/                        # Utility scripts
    ├── test_robot_description.py # Test robot_description publishing
    ├── check_robot_setup.py      # Check TF tree
    ├── test_pointcloud.py        # Test point cloud
    └── fix_pointcloud_color_and_transform.py  # Fix point cloud issues
```

## 🚀 Quick Start (RECOMMENDED)

### The Simplest Way That Works

```bash
cd ~/Documents/Github/opensource_dev/handeye/ros2_digital_twin

# With real robot:
python3 start_digital_twin.py /dev/ttyACM0

# Without robot (simulation):
python3 start_digital_twin.py --simulate
```

This will:
1. ✅ Publish robot_description (fixes the error!)
2. ✅ Start robot_state_publisher
3. ✅ Launch RViz2
4. ✅ Sync with real robot (if connected)

## ❌ Why You Got The Error

The error:
```
[FATAL] robot_description parameter must not be empty
```

Happens because `robot_state_publisher` needs the `/robot_description` topic to be published FIRST with the URDF content. The correct order is:

1. **First**: Publish URDF to `/robot_description` topic
2. **Then**: Start `robot_state_publisher`
3. **Finally**: Launch RViz2

## 📋 Prerequisites

1. **ROS2 Workspace with SO-101 URDF**
   ```bash
   # Check if URDF exists:
   ls ~/Documents/Github/opensource_dev/ROS2/lerobot_ws/src/lerobot_description/urdf/so101.urdf.xacro
   ```

2. **Python Dependencies**
   ```bash
   pip install pyrealsense2 numpy scipy
   ```

3. **Hand-Eye Calibration** (for camera integration)
   - Eye-in-hand: `../output/handeye_realsense.npz`
   - Eye-to-hand: `../output/handeye_realsense_stationary.npz`

## 🎯 Complete Setup Instructions

### Step 1: Test Your Setup

```bash
# Run the test script
./scripts/test_digital_twin.sh
```

### Step 2: Test Robot Description Publishing

```bash
# Terminal 1: Publish robot_description
python3 utils/test_robot_description.py

# Terminal 2: After seeing "Published robot_description"
ros2 run robot_state_publisher robot_state_publisher

# Terminal 3: Launch RViz2
rviz2
```

### Step 3: Run Complete Digital Twin

#### Option A: All-in-One (Recommended)
```bash
# With real robot
python3 start_digital_twin.py /dev/ttyACM0

# Simulation mode
python3 start_digital_twin.py --simulate
```

#### Option B: Manual Launch (for debugging)
```bash
# Terminal 1: Main node (publishes robot_description and joint_states)
python3 launch/launch_complete.py --port /dev/ttyACM0

# Terminal 2: Wait 3 seconds, then start robot_state_publisher
ros2 run robot_state_publisher robot_state_publisher

# Terminal 3: RViz2
rviz2 -d config/so101_digital_twin.rviz
```

## 📷 Camera Integration

### For Stationary Camera (Eye-to-Hand)

1. **Calibrate** (if not done):
   ```bash
   cd ../handeye_calibration
   python3 handeye_manual_realsense_stationary.py --port /dev/ttyACM0
   ```

2. **Connect Camera to Robot Base**:
   ```bash
   python3 connect_camera_to_robot_stationary.py
   ```

3. **Launch Colored Point Cloud**:
   ```bash
   ./scripts/launch_colored_pointcloud.sh
   ```

### For Camera on Robot (Eye-in-Hand)

1. **Calibrate** (if not done):
   ```bash
   cd ../handeye_calibration
   python3 handeye_manual_realsense.py --port /dev/ttyACM0
   ```

2. **Connect Camera to Gripper**:
   ```bash
   python3 connect_camera_to_robot.py --ros-args -p gripper_frame:=gripper
   ```

## 🔧 Troubleshooting

### Problem: "robot_description parameter must not be empty"

**Solution**: Use `start_digital_twin.py` which publishes robot_description BEFORE starting robot_state_publisher.

### Problem: Robot not moving in RViz2

**Check**:
```bash
# Is robot connected?
ls /dev/ttyACM*

# Are joint states being published?
ros2 topic echo /joint_states

# Is TF tree complete?
ros2 run tf2_tools view_frames
```

### Problem: No robot model in RViz2

**In RViz2**:
1. Set Fixed Frame to `world` or `base`
2. Add Display → RobotModel
3. Set Robot Description Topic to `/robot_description`

### Problem: Point cloud not colored

**Solution**:
```bash
# Use the optimized launch script
./scripts/launch_colored_pointcloud.sh
```

**In RViz2 PointCloud2 settings**:
- Color Transformer: `RGB8` (not Intensity!)
- Style: `Boxes` or `Spheres`

### Problem: Camera not connected to robot

**For stationary camera**:
```bash
python3 connect_camera_to_robot_stationary.py --ros-args -p base_frame:=base
```

**Check connection**:
```bash
ros2 run tf2_ros tf2_echo base camera_link
```

## 📊 System Architecture

### Data Flow
```
Real Robot (/dev/ttyACM0)
    ↓
sync_real_robot.py (reads joints)
    ↓
/joint_states topic
    ↓
robot_state_publisher (needs /robot_description first!)
    ↓
TF tree
    ↓
RViz2 (visualizes robot model)
    ↑
RealSense Camera (point cloud)
```

### TF Tree Structure

**Eye-to-Hand (Stationary Camera)**:
```
world
└── base (robot base)
    ├── link1 → link2 → ... → gripper
    └── camera_link (stationary, calibrated to base)
        └── camera_color_optical_frame
```

**Eye-in-Hand (Camera on Robot)**:
```
world
└── base
    └── link1 → link2 → ... → gripper
        └── camera_link (moves with gripper)
            └── camera_color_optical_frame
```

## 🎮 Usage Examples

### Basic Digital Twin
```bash
# Just visualize robot movement
python3 start_digital_twin.py --simulate
```

### Real Robot with Camera
```bash
# Terminal 1: Digital twin
python3 start_digital_twin.py /dev/ttyACM0

# Terminal 2: Camera connection
python3 connect_camera_to_robot_stationary.py

# Terminal 3: RealSense
./scripts/launch_colored_pointcloud.sh
```

### Full System Test
```bash
# Run complete setup test
./scripts/test_digital_twin.sh

# If all tests pass, launch:
./scripts/launch_digital_twin.sh /dev/ttyACM0
```

## 📝 Key Scripts Explained

| Script | Purpose | When to Use |
|--------|---------|-------------|
| `start_digital_twin.py` | Main launcher, handles timing correctly | **Always use this first!** |
| `sync_real_robot.py` | Reads robot joints, publishes to ROS2 | Included in start_digital_twin.py |
| `connect_camera_to_robot_stationary.py` | Connects stationary camera | After calibration |
| `utils/test_robot_description.py` | Tests URDF publishing | For debugging |

## 🔄 Updates & Fixes

### Recent Fixes
- ✅ Fixed robot_state_publisher timing issue
- ✅ Added proper QoS settings for robot_description
- ✅ Created organized folder structure
- ✅ Fixed stationary camera calibration
- ✅ Added colored point cloud support

### Known Issues
- SO101Client may need to be installed separately
- RealSense SDK required for camera features

## 📚 Additional Resources

- [Hand-Eye Calibration Guide](../handeye_calibration/README.md)
- [Camera Calibration](../camera_calibration/README.md)
- [SO-101 Robot Control](../../GraspingDemo/README.md)

## 💡 Tips

1. **Always run `start_digital_twin.py` first** - it handles the correct startup sequence
2. **For debugging**, use manual launch to see each component's output
3. **Check TF tree** with `ros2 run tf2_tools view_frames` if things don't appear
4. **Source your workspace** if xacro processing fails

## 🆘 Need Help?

If the digital twin doesn't work:

1. Run the test script: `./scripts/test_digital_twin.sh`
2. Check robot connection: `ls /dev/ttyACM*`
3. Verify URDF exists: `ls ~/Documents/Github/opensource_dev/ROS2/lerobot_ws/src/lerobot_description/urdf/`
4. Test in simulation mode first: `python3 start_digital_twin.py --simulate`

---
*This digital twin system provides real-time synchronization between your physical SO-101 robot and its RViz2 visualization, with full RealSense integration for both mounted and stationary camera configurations.*