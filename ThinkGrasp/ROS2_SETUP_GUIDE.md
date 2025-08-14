# ThinkGrasp ROS2 Setup Guide

## ✅ Quick Start - ROS2 Works!

The ROS2 integration is now working! Use the provided runner script:

```bash
# 1. Start the ThinkGrasp API server
cd /path/to/ThinkGrasp
python realarm310.py

# 2. In another terminal, run ROS2 node
./ros2_run.sh simple_client_node /path/to/rgb.png /path/to/depth.png "grasp instruction"

# Example:
./ros2_run.sh simple_client_node outputs/20250814_002629_010/input_rgb.png outputs/20250814_002629_010/input_depth.png "grasp the item"
```

## Working ROS2 Nodes

### simple_client_node
A simplified ROS2 client that works without custom service definitions:

```bash
# Run with the helper script
./ros2_run.sh simple_client_node rgb.png depth.png "grasp the object"

# Or directly with Python
export PYTHONPATH=/home/pathonai/thinkgrasp_ros2_ws/install/thinkgrasp_ros2/lib/python3.12/site-packages:$PYTHONPATH
python3 /home/pathonai/thinkgrasp_ros2_ws/install/thinkgrasp_ros2/lib/thinkgrasp_ros2/simple_client_node rgb.png depth.png "grasp"
```

Features:
- ✅ Works without custom services
- ✅ Publishes results to `/thinkgrasp/result` topic
- ✅ Full ROS2 logging
- ✅ Proper node lifecycle

## Alternative Methods

### Direct Python Client
For simplicity without ROS2:

```bash
python simple_grasp_client.py /path/to/rgb.png /path/to/depth.png "grasp instruction"
```

### Automatic Fallback Script
Tries ROS2 first, falls back to Python:

```bash
./run_ros2_client.sh /path/to/rgb.png /path/to/depth.png "grasp instruction"
```

## Full ROS2 Setup

### Prerequisites

1. Install ROS2 (Humble, Iron, Jazzy, or Foxy)
2. Install colcon build tools:
```bash
sudo apt install python3-colcon-common-extensions
```

### Building the ROS2 Package

```bash
# 1. Create a ROS2 workspace
mkdir -p ~/thinkgrasp_ws/src
cd ~/thinkgrasp_ws/src

# 2. Link or copy the thinkgrasp_ros2 package
ln -s /path/to/ThinkGrasp/thinkgrasp_ros2 .

# 3. Build the package
cd ~/thinkgrasp_ws
colcon build --packages-select thinkgrasp_ros2

# 4. Source the workspace
source install/setup.bash
```

### Running the ROS2 Nodes

#### Terminal 1: Start ThinkGrasp API Server
```bash
cd /path/to/ThinkGrasp
python realarm310.py
```

#### Terminal 2: Start ROS2 Service Node
```bash
# Source ROS2 and workspace
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/thinkgrasp_ws/install/setup.bash

# Run the service node
ros2 run thinkgrasp_ros2 thinkgrasp_service_node.py
```

#### Terminal 3: Test with Client
```bash
# Source ROS2 and workspace
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/thinkgrasp_ws/install/setup.bash

# From image files
ros2 run thinkgrasp_ros2 thinkgrasp_client_node.py /path/to/rgb.png /path/to/depth.png "grasp instruction"

# Or from ROS topics (requires camera node running)
ros2 run thinkgrasp_ros2 thinkgrasp_client_node.py "grasp instruction"
```

## Troubleshooting

### ROS2 Package Not Found

If you get "Package 'thinkgrasp_ros2' not found", try:

1. Make sure you've built the package:
```bash
cd ~/thinkgrasp_ws
colcon build --packages-select thinkgrasp_ros2
```

2. Source the workspace in each new terminal:
```bash
source ~/thinkgrasp_ws/install/setup.bash
```

3. Check if the package is installed:
```bash
ros2 pkg list | grep thinkgrasp
```

### ModuleNotFoundError: No module named 'thinkgrasp_ros2.srv'

This means the service definitions haven't been built. Rebuild the package:
```bash
cd ~/thinkgrasp_ws
rm -rf build install log
colcon build --packages-select thinkgrasp_ros2
source install/setup.bash
```

### Connection Refused

Make sure the ThinkGrasp API server is running:
```bash
cd /path/to/ThinkGrasp
python realarm310.py
```

Check if it's accessible:
```bash
curl http://localhost:5010/list_results
```

### Alternative: Direct Python Import

If ROS2 setup is problematic, you can also import and use the ThinkGrasp API directly in your Python code:

```python
import requests
import json

def call_thinkgrasp_api(rgb_path, depth_path, instruction):
    url = "http://localhost:5010/grasp_pose"
    payload = {
        'image_path': rgb_path,
        'depth_path': depth_path,
        'text_path': '/tmp/instruction.txt'
    }
    
    # Write instruction to temp file
    with open('/tmp/instruction.txt', 'w') as f:
        f.write(instruction)
    
    response = requests.post(url, json=payload)
    if response.status_code == 200:
        return response.json()
    else:
        raise Exception(f"API error: {response.text}")

# Example usage
result = call_thinkgrasp_api(
    "/path/to/rgb.png",
    "/path/to/depth.png", 
    "grasp the red cube"
)
print(f"Grasp position: {result['xyz']}")
```

## Viewing Results

After running grasp detection, view results at:
```
http://localhost:5010/viewer
```

Select your result from the sidebar to see:
- 3D point cloud visualization
- Grasp poses
- Input images
- Analysis details