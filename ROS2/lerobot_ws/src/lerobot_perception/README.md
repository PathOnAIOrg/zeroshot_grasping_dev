# Lerobot Perception Package

A ROS2 perception package for object detection, point cloud generation, and grasp planning for the Lerobot SO-101.

## 🚀 Features

- **Object Detection**: Color-based segmentation using HSV color space
- **Point Cloud Generation**: 3D point clouds from RGB-D camera data
- **Multiple Object Support**: Distinguish objects by color and unique IDs
- **RViz Visualization**: Real-time visualization of detected objects

## 📦 Dependencies

### System Dependencies
- ROS2 Jazzy
- Python 3.10+
- OpenCV
- NumPy
- Open3D

### Python Dependencies

#### Option 1: Virtual Environment (Recommended)
```bash
# Create virtual environment
python3 -m venv lerobot_perception_venv

# Activate virtual environment
source lerobot_perception_venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Verify installation
python -c "import cv2, numpy, open3d, cv_bridge; print('All dependencies installed successfully!')"
```

#### Option 2: System-wide Installation
```bash
pip3 install -r requirements.txt

# Verify installation
python3 -c "import cv2, numpy, open3d, cv_bridge; print('All dependencies installed successfully!')"
```

**Note:** When using a virtual environment, make sure to activate it before building and running the ROS2 package. To deactivate the virtual environment when done, use `deactivate`.

## 🏗️ Building

**If using virtual environment, activate it first:**
```bash
source lerobot_perception_venv/bin/activate
```

**Build the package:**
```bash
cd ~/ros2_ws
colcon build --packages-select lerobot_perception
source install/setup.bash
```

## 🎯 Usage

### Launch Object Detection Node

```bash
# Terminal 1: Launch gazebo
ros2 launch lerobot_description so101_gazebo.launch.py

# Terminal 2: Launch object detection
ros2 launch lerobot_perception object_detection.launch.py
```

**Topics:**
- `/detected_objects/pointcloud` (sensor_msgs/PointCloud2): Point clouds of all detected objects
- `/detected_objects/poses` (geometry_msgs/PoseArray): 3D poses of detected objects
- `/detected_objects/markers` (visualization_msgs/MarkerArray): RViz markers for visualization
- `/detected_objects/red/pointcloud` (sensor_msgs/PointCloud2): Red object point cloud
- `/detected_objects/green/pointcloud` (sensor_msgs/PointCloud2): Green object point cloud
- `/detected_objects/blue/pointcloud` (sensor_msgs/PointCloud2): Blue object point cloud
- `/detected_objects/purple/pointcloud` (sensor_msgs/PointCloud2): Purple object point cloud

## 🎨 Object Detection

### Supported Colors
- **Red**: HSV range (0, 100, 100) to (10, 255, 255)
- **Green**: HSV range (40, 100, 100) to (80, 255, 255)
- **Blue**: HSV range (100, 100, 100) to (130, 255, 255)
- **Purple**: HSV range (130, 100, 100) to (160, 255, 255)

### Object Properties
- **Unique IDs**: Each object gets a unique identifier (e.g., `red_1`, `green_2`)
- **3D Centers**: Calculated from point cloud centroids
- **Bounding Boxes**: 2D bounding boxes from image processing
- **Point Clouds**: 3D point clouds in world frame



## 🛠️ Configuration

### Launch Parameters
- `camera_topic_prefix`: Camera topic prefix (default: `/camera`)
- `rgb_topic`: RGB image topic (default: `rgb/image_raw`)
- `depth_topic`: Depth image topic (default: `depth/image_raw`)
- `camera_info_topic`: Camera info topic (default: `rgb/camera_info`)
- `min_area`: Minimum object area in pixels (default: 1000)
- `max_area`: Maximum object area in pixels (default: 50000)



## 📊 Visualization

### RViz Setup
1. Add PointCloud2 display for `/detected_objects/pointcloud`
2. Add PoseArray display for `/detected_objects/poses`
3. Add MarkerArray display for `/detected_objects/markers`
4. Set Fixed Frame to `world`



## 🔧 Troubleshooting

### Common Issues
1. **No Point Cloud Data**: Check camera topics and depth values
2. **Upside-down Point Clouds**: Verify coordinate transformations
3. **No Objects Detected**: Check color ranges and lighting conditions

### Debug Commands
```bash
# Check available topics
ros2 topic list

# Monitor point cloud data
ros2 topic echo /detected_objects/pointcloud

# Check TF frames
ros2 run tf2_tools view_frames

# Monitor object detection
ros2 topic echo /detected_objects/poses
```

## 📁 Package Structure

```
lerobot_perception/
├── scripts/
│   └── object_detection_node.py      # Main object detection node
├── launch/
│   └── object_detection.launch.py    # Object detection launch file
├── requirements.txt                  # Python dependencies
├── CMakeLists.txt                   # Build configuration
├── package.xml                      # Package metadata
└── README.md                        # This file
```

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## 📄 License

This package is part of the Lerobot project and follows the same license terms.
