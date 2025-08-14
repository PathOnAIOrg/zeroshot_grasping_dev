# ThinkGrasp Quick Start Guide

## 🚀 Fastest Way to Get Started

### Step 1: Start the ThinkGrasp API Server

```bash
cd /path/to/ThinkGrasp
python realarm310.py
```

The server will start on `http://localhost:5010`

### Step 2: Run Grasp Detection

Use the simple client (no ROS2 required):

```bash
python simple_grasp_client.py /path/to/rgb.png /path/to/depth.png "grasp instruction"
```

Example:
```bash
python simple_grasp_client.py outputs/20250814_002629_010/input_rgb.png \
                              outputs/20250814_002629_010/input_depth.png \
                              "grasp the item"
```

### Step 3: View Results

Open your browser and go to:
```
http://localhost:5010/viewer
```

Select your result from the sidebar to see:
- 3D point cloud visualization
- Grasp poses with interactive controls
- Input images (RGB, depth, cropping box)
- Analysis details

## 📝 Available Clients

### 1. Simple Client (Recommended)
No dependencies except Python requests:
```bash
python simple_grasp_client.py rgb.png depth.png "instruction"
```

Options:
- `--host HOST`: API server host (default: localhost)
- `--port PORT`: API server port (default: 5010)  
- `--json`: Output raw JSON response
- `--save FILE`: Save results to JSON file

### 2. Test API Client
For testing and validation:
```bash
python test_thinkgrasp_api.py rgb.png depth.png "instruction"
```

### 3. Direct API Call
Using curl:
```bash
# First, create instruction file
echo "grasp the red cube" > /tmp/instruction.txt

# Call API
curl -X POST http://localhost:5010/grasp_pose \
  -H "Content-Type: application/json" \
  -d '{
    "image_path": "/path/to/rgb.png",
    "depth_path": "/path/to/depth.png",
    "text_path": "/tmp/instruction.txt"
  }'
```

### 4. Python Code Integration
```python
import requests
import json

def grasp_detection(rgb_path, depth_path, instruction):
    # Write instruction to file
    with open('/tmp/instruction.txt', 'w') as f:
        f.write(instruction)
    
    # Call API
    response = requests.post('http://localhost:5010/grasp_pose', json={
        'image_path': rgb_path,
        'depth_path': depth_path,
        'text_path': '/tmp/instruction.txt'
    })
    
    if response.status_code == 200:
        result = response.json()
        print(f"Grasp position: {result['xyz']}")
        print(f"Rotation: {result['rot']}")
        return result
    else:
        raise Exception(f"API error: {response.text}")

# Example usage
result = grasp_detection(
    '/path/to/rgb.png',
    '/path/to/depth.png',
    'grasp the bottle'
)
```

## 🔍 Output Format

The API returns:
```json
{
  "xyz": [x, y, z],           // Grasp position
  "rot": [[...], [...], [...]],  // 3x3 rotation matrix
  "dep": 0.04,                // Grasp depth
  "timestamp": "20250814_...", // Unique timestamp
  "summary_file": "outputs/.../summary.json",
  "message": "Use http://localhost:5010/viewer to visualize results"
}
```

## 📊 Visualization Features

The web viewer provides:
- **3D Point Cloud**: Interactive visualization
- **Grasp Poses**: All detected grasps with scores
- **Selected Grasp**: Highlighted in red
- **Input Images**: RGB, depth, and cropping box
- **Multi-Panel View**: Step-by-step processing stages
- **Controls**:
  - 🔄 Reset camera
  - 👁️ Toggle point cloud
  - 🤏 Toggle gripper
  - 📊 Show all grasps
  - 🔗 Wireframe mode

## 🛠️ Troubleshooting

### API Server Won't Start
- Check if port 5010 is available: `lsof -i :5010`
- Ensure all dependencies are installed
- Check OpenAI API key is set: `echo $OPENAI_API_KEY`

### No Grasp Poses Detected
- Verify depth image quality
- Ensure good lighting for RGB image
- Try different grasp instructions
- Check that objects are visible and graspable

### Connection Refused
- Make sure API server is running
- Check firewall settings
- Verify host and port are correct

### Visualization Not Loading
- Clear browser cache
- Try different browser
- Check browser console for errors
- Ensure JavaScript is enabled

## 📁 Output Files

Each run creates a timestamped folder with:
- `input_rgb.png` - Original RGB image
- `input_depth.png` - Original depth image
- `input_depth_visualization.png` - Colorized depth
- `input_instruction.txt` - Grasp instruction
- `cropping_box_visualization.png` - Detection area
- `point_cloud.ply` - 3D point cloud
- `point_cloud.json` - Web-compatible point cloud
- `chosen_grasp.ply` - Selected grasp mesh
- `all_grasps.json` - All detected grasps
- `final_grasp_results.json` - Numerical results
- `openai_response.txt` - Vision model output
- `summary.json` - Complete summary

## 🎯 Tips for Best Results

1. **Image Quality**: Use well-lit, clear images
2. **Depth Alignment**: Ensure RGB and depth are aligned
3. **Clear Instructions**: Be specific about target object
4. **Object Visibility**: Ensure target is not occluded
5. **Appropriate Scale**: Objects should be graspable size

## 🔗 ROS2 Integration

For ROS2 integration, see [ROS2_SETUP_GUIDE.md](ROS2_SETUP_GUIDE.md)

Note: The simple clients work without ROS2, making them ideal for testing and development.