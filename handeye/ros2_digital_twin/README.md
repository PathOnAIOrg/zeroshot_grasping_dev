# 🤖 SO-101 Digital Twin with RealSense in RViz2

Complete digital twin system for SO-101 robot with RealSense camera integration, supporting both eye-in-hand and eye-to-hand configurations.

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
1. ✅ Publish robot_description (fixes the "robot_description parameter must not be empty" error!)
2. ✅ Start robot_state_publisher
3. ✅ Launch RViz2
4. ✅ Sync with real robot (if connected)

## ❌ Common Error and Solution

**Error:**
```
[FATAL] robot_description parameter must not be empty
```

**Why it happens:** `robot_state_publisher` needs `/robot_description` to be published FIRST with proper QoS settings.

**Solution:** Use `start_digital_twin.py` which handles the correct startup sequence automatically.

## 📋 Prerequisites

1. **ROS2 Workspace with SO-101 URDF**
   ```bash
   ls ~/Documents/Github/opensource_dev/ROS2/lerobot_ws/src/lerobot_description/urdf/so101.urdf.xacro
   ```

2. **Python Dependencies**
   ```bash
   pip install pyrealsense2 numpy scipy
   ```

3. **Hand-Eye Calibration Files**
   - For stationary camera: `../output/handeye_realsense_stationary.npz`
   - For mounted camera: `../output/handeye_realsense.npz`

## 📁 Project Structure

```
ros2_digital_twin/
├── start_digital_twin.py         # 🚀 Main launcher (USE THIS!)
├── sync_real_robot.py            # Real robot synchronization
├── connect_camera_to_robot.py    # Eye-in-hand camera connection
├── connect_camera_to_robot_stationary.py  # Eye-to-hand camera connection
│
├── config/                       # Configuration files
│   └── so101_digital_twin.rviz  # RViz2 configuration
│
├── scripts/                      # Shell scripts
│   ├── launch_colored_pointcloud.sh  # RealSense with color
│   └── test_digital_twin.sh          # System test
│
├── launch/                       # ROS2 launch files
│   └── launch_complete.py       # Complete launch script
│
└── utils/                        # Utility scripts
    └── test_robot_description.py # Test URDF publishing
```

## 🔧 Key Components

### 1. **start_digital_twin.py** (Main Entry Point)
- Handles correct startup sequence
- Publishes robot_description BEFORE robot_state_publisher
- Launches all components in order
- Works with real robot or simulation

### 2. **sync_real_robot.py**
- Connects to physical SO-101 robot
- Reads joint positions at 30 Hz
- Publishes `/joint_states` and `/robot_description`
- Includes proper QoS settings for robot_state_publisher

### 3. **connect_camera_to_robot_stationary.py**
- For stationary camera setup (eye-to-hand)
- Publishes transform from camera to robot base
- Uses hand-eye calibration results

## 📊 Visualization in RViz2

### Manual Setup

1. **Add Robot Model:**
   - Add Display → RobotModel
   - Topic: `/robot_description`

2. **Add Point Cloud:**
   - Add Display → PointCloud2
   - Topic: `/camera/depth/color/points`

3. **Add TF:**
   - Add Display → TF
   - Show all frames

4. **Add Camera Image:**
   - Add Display → Image
   - Topic: `/camera/color/image_raw`

### Custom RViz Config

The provided config file (`config/so101_digital_twin.rviz`) includes:
- Robot model visualization
- Point cloud from RealSense
- TF tree showing all transforms
- Camera image feed
- Axes for reference frames
- Proper view settings

## 🎯 Coordinate Frames

```
base_link (Robot Base)
    ├── shoulder_pan_link
    │   └── shoulder_lift_link
    │       └── elbow_flex_link
    │           └── wrist_flex_link
    │               └── wrist_roll_link (End-Effector)
    │                   ├── gripper_link
    │                   └── camera_link (From Hand-Eye Calibration)
    │                       └── camera_color_optical_frame
```

## 💻 Using the Transform in Code

```python
import rclpy
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped

# Initialize ROS2
rclpy.init()
node = rclpy.create_node('transform_example')

# TF listener
tf_buffer = Buffer()
tf_listener = TransformListener(tf_buffer, node)

# Transform point from camera to robot base
def transform_point(point_in_camera):
    # Create point in camera frame
    point_stamped = PointStamped()
    point_stamped.header.frame_id = 'camera_color_optical_frame'
    point_stamped.header.stamp = node.get_clock().now().to_msg()
    point_stamped.point.x = point_in_camera[0]
    point_stamped.point.y = point_in_camera[1]
    point_stamped.point.z = point_in_camera[2]
    
    try:
        # Transform to base frame
        point_in_base = tf_buffer.transform(
            point_stamped, 
            'base_link',
            timeout=rclpy.duration.Duration(seconds=1.0)
        )
        
        return [
            point_in_base.point.x,
            point_in_base.point.y,
            point_in_base.point.z
        ]
    except Exception as e:
        print(f"Transform failed: {e}")
        return None

# Example: transform detected object position
camera_point = [0.1, 0.05, 0.3]  # 30cm in front of camera
base_point = transform_point(camera_point)
print(f"Point in base frame: {base_point}")
```

## 🔍 Troubleshooting

### Problem: "robot_description parameter must not be empty"
**Solution:** Use `start_digital_twin.py` which publishes robot_description BEFORE starting robot_state_publisher.

### Problem: Robot model not showing in RViz2
```bash
# Check robot description is published
ros2 topic echo /robot_description

# Check joint states
ros2 topic echo /joint_states
```
**In RViz2:** Set Fixed Frame to `world` or `base`, add RobotModel display

### Problem: Camera/Point cloud not showing
```bash
# Check if RealSense is publishing
ros2 topic list | grep camera

# Launch colored point cloud
./scripts/launch_colored_pointcloud.sh
```

### Problem: Transform not found
```bash
# Check TF tree
ros2 run tf2_tools view_frames

# For stationary camera, connect it:
python3 connect_camera_to_robot_stationary.py
```

## 📋 Launch Parameters

### start_digital_twin.py (Main Launcher)
```bash
# With real robot
python3 start_digital_twin.py /dev/ttyACM0

# Simulation mode
python3 start_digital_twin.py --simulate
```

### sync_real_robot.py
```bash
python3 sync_real_robot.py --port /dev/ttyACM0 --simulate
```

### connect_camera_to_robot_stationary.py
```bash
python3 connect_camera_to_robot_stationary.py --ros-args \
    -p calibration_file:=../output/handeye_realsense_stationary.npz \
    -p base_frame:=base \
    -p camera_frame:=camera_link
```

## 🎬 Complete Workflow

### Option A: Automatic (Recommended)
```bash
# All-in-one launcher
cd ~/Documents/Github/opensource_dev/handeye/ros2_digital_twin
python3 start_digital_twin.py /dev/ttyACM0  # or --simulate
```

### Option B: Manual (for debugging)
```bash
# Terminal 1: Main node (publishes robot_description first!)
python3 launch/launch_complete.py --port /dev/ttyACM0

# Terminal 2: Wait 3 seconds, then robot_state_publisher
ros2 run robot_state_publisher robot_state_publisher

# Terminal 3: Camera connection (for stationary camera)
python3 connect_camera_to_robot_stationary.py

# Terminal 4: RealSense with color
./scripts/launch_colored_pointcloud.sh

# Terminal 5: RViz2
rviz2 -d config/so101_digital_twin.rviz
```

## 📷 Camera Integration

### For Stationary Camera (Eye-to-Hand)
```bash
# 1. Calibrate (if not done)
cd ../handeye_calibration
python3 handeye_manual_realsense_stationary.py --port /dev/ttyACM0

# 2. Connect camera to robot base
cd ../ros2_digital_twin
python3 connect_camera_to_robot_stationary.py

# 3. Launch colored point cloud
./scripts/launch_colored_pointcloud.sh
```

### For Camera on Robot (Eye-in-Hand)
```bash
# 1. Calibrate (if not done)
cd ../handeye_calibration
python3 handeye_manual_realsense.py --port /dev/ttyACM0

# 2. Connect camera to gripper
cd ../ros2_digital_twin
python3 connect_camera_to_robot.py --ros-args -p gripper_frame:=gripper
```

## 🚧 Advanced Features

### Record and Playback
```bash
# Record bag file
ros2 bag record -a -o so101_demo

# Playback
ros2 bag play so101_demo
```

### Add Object Detection
Integrate your object detection to visualize detected objects:
```python
# Publish detected objects as markers
from visualization_msgs.msg import MarkerArray, Marker

marker_pub = node.create_publisher(MarkerArray, '/detected_objects', 10)
```

### Grasp Planning Visualization
Visualize planned grasps in RViz:
```python
# Publish grasp poses
from geometry_msgs.msg import PoseArray

grasp_pub = node.create_publisher(PoseArray, '/grasp_poses', 10)
```

## 🎯 Quick Reference

### Eye-in-Hand vs Eye-to-Hand

| Aspect | Eye-in-Hand | Eye-to-Hand |
|--------|-------------|-------------|
| Camera Position | Mounted on gripper | Fixed in workspace |
| Camera Movement | Moves with robot | Stationary |
| Calibration Script | `handeye_manual_realsense.py` | `handeye_manual_realsense_stationary.py` |
| Connection Script | `connect_camera_to_robot.py` | `connect_camera_to_robot_stationary.py` |
| Transform | Camera → Gripper | Camera → Base |
| Output File | `handeye_realsense.npz` | `handeye_realsense_stationary.npz` |

### Essential Commands

```bash
# Test setup
./scripts/test_digital_twin.sh

# Launch digital twin
python3 start_digital_twin.py /dev/ttyACM0  # Real robot
python3 start_digital_twin.py --simulate     # Simulation

# Camera only
./scripts/launch_colored_pointcloud.sh

# Connect stationary camera
python3 connect_camera_to_robot_stationary.py
```

## 📚 References

- [ROS2 TF2 Documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html)
- [RealSense ROS2 Wrapper](https://github.com/IntelRealSense/realsense-ros)
- [RViz2 User Guide](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html)
- [Parent README](../README.md) - Complete calibration guide