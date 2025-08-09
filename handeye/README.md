# 🎯 Hand-Eye Calibration System

Complete calibration system for robot-camera coordination, including camera intrinsic calibration and hand-eye calibration for the SO-101 robot.

## 📁 Project Structure

```
handeye/
├── camera_calibration/       # Camera intrinsic calibration
│   ├── capture_calibration_images.py     # Webcam image capture
│   ├── capture_realsense.py              # Auto-detect camera type
│   ├── capture_realsense_dedicated.py    # RealSense-specific capture
│   ├── calibrate.py                      # Calibrate from existing images
│   └── calibrate_from_capture.py         # Run calibration on captured images
│
├── handeye_calibration/      # Robot-camera calibration
│   ├── handeye_calibration_so101.py      # Automatic calibration
│   ├── handeye_manual_calibration.py     # Manual mode for webcam
│   └── handeye_manual_realsense.py       # Manual mode for RealSense
│
├── utils/                    # Utility scripts
│   └── test_checkerboard_detection.py    # Test pattern detection
│
├── calibration_images/       # Captured calibration images
├── output/                   # Calibration results
│   ├── calibration_data.npz             # Camera intrinsics
│   ├── handeye_realsense.npz           # Hand-eye transformation
│   └── handeye_realsense.json          # Human-readable results
│
└── config/                   # Configuration files
    └── robot_config.yaml     # Robot settings
```

## 🚀 Quick Start Guide

### Prerequisites

```bash
# Install required packages
pip install opencv-python numpy pyrealsense2

# For robot control (if not already installed)
pip install lerobot
```

### Step 1: Camera Intrinsic Calibration

First, calibrate your camera to get intrinsic parameters (focal length, distortion coefficients).

#### For RealSense Camera:
```bash
cd handeye/camera_calibration

# Capture calibration images
python capture_realsense.py

# Run calibration
python calibrate_from_capture.py
```

#### For Regular Webcam:
```bash
cd handeye/camera_calibration

# Capture calibration images
python capture_calibration_images.py

# Run calibration
python calibrate_from_capture.py
```

**Tips for capturing:**
- Press **SPACE** to capture image
- Press **C** to toggle checkerboard detection
- Press **Q** to quit
- Capture 15-20 images from different angles and distances

### Step 2: Test Checkerboard Detection

Identify your checkerboard pattern size:

```bash
cd handeye/utils
python test_checkerboard_detection.py
```

This will show you the pattern size (e.g., 9×6 = 9 columns, 6 rows of internal corners).

### Step 3: Robot Calibration

Calibrate the SO-101 robot joint limits:

```bash
python -m lerobot.calibrate \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM0
```

### Step 4: Hand-Eye Calibration

Now perform the hand-eye calibration to find the transformation between camera and robot.

#### Option A: Manual Mode (Recommended)

You control the robot movement:

```bash
cd handeye/handeye_calibration

# For RealSense camera
python handeye_manual_realsense.py --port /dev/ttyACM0

# With specific pattern (if auto-detect fails)
python handeye_manual_realsense.py --port /dev/ttyACM0 --pattern 9x6

# For webcam
python handeye_manual_calibration.py --port /dev/ttyACM0
```

**Process:**
1. Robot torque is disabled automatically
2. Move robot manually to different positions
3. Position checkerboard in camera view
4. Press **SPACE** when "CHECKERBOARD READY" appears
5. Capture 10-15 different positions
6. Press **F** to finish calibration
7. Press **D** to delete last capture
8. Press **ESC** to cancel

#### Option B: Automatic Mode

Let the program control robot movement:

```bash
cd handeye/handeye_calibration
python handeye_calibration_so101.py --port /dev/ttyACM0
```

The robot will move through predefined positions automatically.

## 📊 Calibration Results

After successful calibration, you'll find:

### Camera Calibration Output:
- `output/calibration_data.npz` - Camera matrix and distortion coefficients
- `output/calibration_comparison.png` - Before/after undistortion comparison

### Hand-Eye Calibration Output:
- `output/handeye_realsense.npz` - 4×4 transformation matrix (camera to gripper)
- `output/handeye_realsense.json` - Human-readable format with all positions

## 💻 Using Calibration in Your Code

### Load Camera Calibration:
```python
import numpy as np

# Load camera intrinsics
data = np.load('output/calibration_data.npz')
camera_matrix = data['mtx']
dist_coeffs = data['dist']
```

### Load Hand-Eye Calibration:
```python
import numpy as np

# Load hand-eye transformation
data = np.load('output/handeye_realsense.npz')
T_cam2gripper = data['T_cam2gripper']  # 4x4 transformation matrix

# Transform point from camera to robot base
def camera_to_robot_base(point_in_camera, robot_pose, T_cam2gripper):
    """
    Transform 3D point from camera to robot base coordinates.
    
    Args:
        point_in_camera: [x, y, z] in camera frame (mm)
        robot_pose: 4x4 current robot end-effector pose
        T_cam2gripper: 4x4 camera to gripper transformation
    
    Returns:
        [x, y, z] in robot base frame
    """
    # Convert to homogeneous coordinates
    point_cam = np.array([point_in_camera[0], point_in_camera[1], 
                          point_in_camera[2], 1.0])
    
    # Transform: base <- gripper <- camera
    point_base = robot_pose @ T_cam2gripper @ point_cam
    
    return point_base[:3]
```

## 🔧 Troubleshooting

### Robot Connection Issues

**Robot won't connect:**
```bash
# Find the correct port
ls /dev/tty* | grep -E 'ACM|USB'

# Check permissions
sudo chmod 666 /dev/ttyACM0

# Add user to dialout group
sudo usermod -a -G dialout $USER
# Then logout and login again
```

**Robot won't move manually:**
- The script attempts to disable torque automatically
- If robot still resists:
  ```bash
  # Run without robot connection
  python handeye_manual_realsense.py --no-robot
  ```

### Camera Issues

**RealSense not detected:**
```bash
# Install RealSense SDK
pip install pyrealsense2

# Test camera
realsense-viewer
```

**Checkerboard not detected:**
1. Run pattern detection test:
   ```bash
   python utils/test_checkerboard_detection.py
   ```
2. Ensure good lighting (no shadows)
3. Keep checkerboard flat
4. Try different distances (300-800mm)
5. Specify pattern manually:
   ```bash
   python handeye_manual_realsense.py --pattern 9x6
   ```

### Calibration Issues

**Poor calibration quality:**
- Use more calibration positions (15-20)
- Vary robot poses more (different heights, angles)
- Ensure checkerboard is clearly visible in all captures
- Check for motion blur (move slowly)

## 📋 Command Line Options

### Camera Calibration:
```bash
# All capture scripts support:
--camera_id 0    # Camera device ID
```

### Hand-Eye Calibration:
```bash
# Manual calibration options:
--port /dev/ttyACM0      # Robot serial port
--pattern 9x6            # Checkerboard pattern (cols×rows)
--min-positions 3        # Minimum calibration positions
--max-positions 30       # Maximum calibration positions
--no-robot              # Run without robot (camera only)

# Automatic calibration options:
--port /dev/ttyACM0      # Robot serial port
--camera 0               # Camera device ID
--positions 15           # Number of positions
--skip-robot            # Test camera only
```

## 📐 Checkerboard Patterns

Common checkerboard patterns (internal corners):

| Pattern | Description | Use Case |
|---------|-------------|----------|
| 9×6 | Standard OpenCV pattern | Most common |
| 7×4 | Smaller pattern | Limited space |
| 8×6 | Medium pattern | Good balance |
| 11×8 | Large pattern | High accuracy |

**Note:** Count the INTERNAL corners (where black squares meet), not the squares themselves.

## 🎯 Tips for Best Results

### Camera Calibration:
1. Capture 15-20 images minimum
2. Cover entire field of view
3. Use different angles (tilt checkerboard)
4. Vary distance to camera
5. Avoid motion blur

### Hand-Eye Calibration:
1. Use 10-15 different robot positions
2. Vary both position AND orientation
3. Keep checkerboard visible but at different distances
4. Move robot slowly to avoid vibration
5. Ensure good lighting throughout

### Validation:
- RMS error < 1.0 pixels is good
- RMS error < 0.5 pixels is excellent
- Check calibration by projecting known points

## 📚 File Descriptions

### Camera Calibration Scripts:
| File | Description |
|------|-------------|
| `capture_realsense.py` | Auto-detect camera type (RealSense/webcam) |
| `capture_realsense_dedicated.py` | RealSense using pyrealsense2 SDK |
| `capture_calibration_images.py` | Standard webcam capture |
| `calibrate_from_capture.py` | Run calibration on captured images |
| `calibrate.py` | Calibrate from existing image files |

### Hand-Eye Calibration Scripts:
| File | Description |
|------|-------------|
| `handeye_calibration_so101.py` | Automatic calibration (program moves robot) |
| `handeye_manual_calibration.py` | Manual calibration for webcam |
| `handeye_manual_realsense.py` | Manual calibration for RealSense |

### Utility Scripts:
| File | Description |
|------|-------------|
| `test_checkerboard_detection.py` | Test and identify checkerboard pattern |

## 🔍 Understanding the Output

### Camera Calibration Output:
```python
# calibration_data.npz contains:
mtx:          # 3x3 camera matrix (fx, fy, cx, cy)
dist:         # Distortion coefficients (k1, k2, p1, p2, k3)
rvecs:        # Rotation vectors for each calibration image
tvecs:        # Translation vectors for each calibration image
pattern_size: # Detected checkerboard pattern
rms_error:    # Reprojection error
```

### Hand-Eye Calibration Output:
```python
# handeye_realsense.npz contains:
T_cam2gripper:  # 4x4 transformation matrix
robot_poses:    # List of robot poses during calibration
robot_joints:   # Joint positions for each calibration
camera_matrix:  # Camera intrinsic matrix used
dist_coeffs:    # Distortion coefficients used
```

## 🎬 Complete Workflow Example

```bash
# 1. Navigate to project
cd /path/to/handeye

# 2. Calibrate camera
cd camera_calibration
python capture_realsense.py  # Capture 20 images
python calibrate_from_capture.py

# 3. Test pattern detection
cd ../utils
python test_checkerboard_detection.py
# Note the pattern size (e.g., 9x6)

# 4. Calibrate robot (if needed)
python -m lerobot.calibrate \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM0

# 5. Perform hand-eye calibration
cd ../handeye_calibration
python handeye_manual_realsense.py \
    --port /dev/ttyACM0 \
    --pattern 9x6

# 6. Check results
cd ../output
ls -la  # View generated files
```

## 📧 Support

For issues or questions:
1. Check the troubleshooting section
2. Ensure all prerequisites are installed
3. Verify hardware connections
4. Review error messages in terminal

## 📄 License

This project is part of the SO-101 grasping system.