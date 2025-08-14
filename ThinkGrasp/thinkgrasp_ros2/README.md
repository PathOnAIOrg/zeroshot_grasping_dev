# ThinkGrasp ROS2 Package

ROS2 wrapper for the ThinkGrasp intelligent grasp detection system. This package provides ROS2 interfaces to use ThinkGrasp's vision-language model for robotic grasping.

## Features

- **Service-based API**: Call ThinkGrasp as a ROS2 service
- **Real-time visualization**: Visualize grasp poses in RViz
- **Camera integration**: Works with RealSense, Kinect, and other RGB-D cameras
- **Multi-modal input**: Combines RGB-D images with natural language instructions
- **Web visualization**: Access results through web interface

## Installation

### Prerequisites

1. Install ROS2 (Humble or newer)
2. Install ThinkGrasp dependencies (see main README)
3. Start the ThinkGrasp API server:
```bash
cd /path/to/ThinkGrasp
python realarm310.py
```

### Build the Package

```bash
# Create workspace
mkdir -p ~/thinkgrasp_ws/src
cd ~/thinkgrasp_ws/src

# Clone or link the package
ln -s /path/to/ThinkGrasp/thinkgrasp_ros2 .

# Build
cd ~/thinkgrasp_ws
colcon build --packages-select thinkgrasp_ros2

# Source
source install/setup.bash
```

## Usage

### 1. Start ThinkGrasp API Server

First, start the ThinkGrasp Flask API server:
```bash
cd /path/to/ThinkGrasp
python realarm310.py
```

### 2. Launch ROS2 Nodes

#### With RealSense Camera:
```bash
ros2 launch thinkgrasp_ros2 thinkgrasp_with_realsense.launch.py
```

#### Without Camera (using existing topics):
```bash
ros2 launch thinkgrasp_ros2 thinkgrasp.launch.py
```

#### With custom API host/port:
```bash
ros2 launch thinkgrasp_ros2 thinkgrasp.launch.py api_host:=192.168.1.100 api_port:=5010
```

### 3. Test with Client

#### From image files:
```bash
ros2 run thinkgrasp_ros2 thinkgrasp_client_node.py /path/to/rgb.png /path/to/depth.png "grasp the red cube"
```

#### From ROS topics:
```bash
ros2 run thinkgrasp_ros2 thinkgrasp_client_node.py "pick up the bottle"
```

### 4. Service Interface

You can also call the service directly:

```bash
ros2 service call /detect_grasp thinkgrasp_ros2/srv/DetectGrasp "{
  grasp_instruction: 'grasp the object',
  save_visualization: true
}"
```

## Nodes

### thinkgrasp_service_node
Main service node that interfaces with ThinkGrasp API.

**Subscriptions:**
- None (service-based)

**Publications:**
- `/thinkgrasp/grasps` (GraspArray): All detected grasps
- `/thinkgrasp/markers` (MarkerArray): Visualization markers
- `/thinkgrasp/point_cloud` (PointCloud2): Scene point cloud

**Services:**
- `/detect_grasp` (DetectGrasp): Main grasp detection service

**Parameters:**
- `api_host` (string): ThinkGrasp API host address
- `api_port` (int): ThinkGrasp API port
- `publish_markers` (bool): Enable marker publishing
- `marker_lifetime` (float): Marker display duration

### image_capture_node
Captures and saves RGB-D images from camera topics.

**Subscriptions:**
- RGB image topic (configurable)
- Depth image topic (configurable)

**Services:**
- `/capture_images`: Manual image capture

**Parameters:**
- `rgb_topic` (string): RGB image topic name
- `depth_topic` (string): Depth image topic name
- `capture_dir` (string): Directory to save captures
- `auto_capture_rate` (float): Auto-capture rate in Hz (0 = disabled)

### grasp_visualizer_node
Enhanced visualization of grasp poses in RViz.

**Subscriptions:**
- `/thinkgrasp/grasps` (GraspArray): Grasp poses to visualize

**Publications:**
- `/thinkgrasp/visualization_markers` (MarkerArray): Enhanced markers

## Custom Messages and Services

### DetectGrasp.srv
```
# Request
sensor_msgs/Image rgb_image
sensor_msgs/Image depth_image
string grasp_instruction
bool save_visualization
---
# Response
bool success
string message
geometry_msgs/Pose grasp_pose
float64 grasp_depth
float64 confidence_score
string timestamp
string visualization_url
```

### GraspArray.msg
```
std_msgs/Header header
geometry_msgs/PoseArray grasp_poses
float64[] scores
int32 selected_index
```

## Visualization in RViz

1. Start RViz:
```bash
rviz2
```

2. Add displays:
- Add **MarkerArray** display
  - Topic: `/thinkgrasp/markers` or `/thinkgrasp/visualization_markers`
- Add **PointCloud2** display
  - Topic: `/thinkgrasp/point_cloud`
- Add **Image** displays for RGB and depth

3. Set fixed frame to `camera_link` or your camera frame

## Integration Examples

### Python Example
```python
import rclpy
from rclpy.node import Node
from thinkgrasp_ros2.srv import DetectGrasp
from sensor_msgs.msg import Image

class GraspingNode(Node):
    def __init__(self):
        super().__init__('grasping_node')
        self.client = self.create_client(DetectGrasp, 'detect_grasp')
        
    def request_grasp(self, rgb_msg, depth_msg, instruction):
        request = DetectGrasp.Request()
        request.rgb_image = rgb_msg
        request.depth_image = depth_msg
        request.grasp_instruction = instruction
        request.save_visualization = True
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        if response.success:
            return response.grasp_pose
        else:
            self.get_logger().error(f"Grasp detection failed: {response.message}")
            return None
```

### Launch File Example
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='thinkgrasp_ros2',
            executable='thinkgrasp_service_node.py',
            parameters=[{
                'api_host': 'localhost',
                'api_port': 5010
            }]
        )
    ])
```

## Troubleshooting

### API Connection Issues
- Ensure ThinkGrasp API server is running (`python realarm310.py`)
- Check firewall settings
- Verify API host and port parameters

### No Grasps Detected
- Check camera data quality
- Ensure adequate lighting
- Verify depth image alignment
- Try different grasp instructions

### Visualization Issues
- Set correct fixed frame in RViz
- Check marker topic subscriptions
- Verify TF tree is complete

## Configuration Files

- `config/thinkgrasp_params.yaml`: Default parameters
- `launch/thinkgrasp.launch.py`: Main launch file
- `launch/thinkgrasp_with_realsense.launch.py`: Launch with RealSense

## Web Interface

Access the web visualization at:
```
http://localhost:5010/viewer
```

## License

MIT License - See LICENSE file for details.