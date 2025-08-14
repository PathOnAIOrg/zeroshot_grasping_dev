# ThinkGrasp - Simple Usage Guide

This is a simplified guide for using ThinkGrasp's API server and clients. For the full research documentation, see the main [README.md](README.md).

## 🚀 Quick Start (3 Steps)

### Step 1: Start Server
```bash
python realarm310.py
```

### Step 2: Run Detection
```bash
python simple_grasp_client.py rgb.png depth.png "grasp the object"
```

### Step 3: View Results
Open browser: `http://localhost:5010/viewer`

## 📦 Available Tools

### Main Components
- **`realarm310.py`** - API server (Flask)
- **`simple_grasp_client.py`** - Easy-to-use client
- **`test_thinkgrasp_api.py`** - Testing tool
- **`results_viewer.html`** - Web visualization

### Key Features
- ✅ No ROS2 build required
- ✅ Simple Python API
- ✅ Web-based 3D visualization
- ✅ Timestamped results
- ✅ Input image preservation

## 📊 Output Files

Each run creates `outputs/TIMESTAMP/` with:
- Input images (RGB, depth, instruction)
- Point clouds (PLY and JSON)
- Grasp poses and results
- Visualization files

## 🛠️ Troubleshooting

### Common Issues

**Server won't start:**
```bash
# Check port
lsof -i :5010
# Check API key
echo $OPENAI_API_KEY
```

**No grasps detected:**
- Check image quality
- Verify depth alignment
- Try clearer instructions

**Client connection error:**
- Ensure server is running
- Check firewall settings

## 📝 Example Usage

### Basic Example
```bash
# Terminal 1 - Start server
python realarm310.py

# Terminal 2 - Run detection  
python simple_grasp_client.py \
    test_images/rgb.png \
    test_images/depth.png \
    "pick up the red cube"
```

### With Options
```bash
# Save results to JSON
python simple_grasp_client.py rgb.png depth.png "grasp" --save results.json

# Use remote server
python simple_grasp_client.py rgb.png depth.png "grasp" --host 192.168.1.100
```

### Python Integration
```python
import requests

# Quick function
def detect_grasp(rgb, depth, text):
    with open('/tmp/inst.txt', 'w') as f:
        f.write(text)
    
    resp = requests.post('http://localhost:5010/grasp_pose', json={
        'image_path': rgb,
        'depth_path': depth,
        'text_path': '/tmp/inst.txt'
    })
    return resp.json()

# Use it
result = detect_grasp('rgb.png', 'depth.png', 'grasp the bottle')
print(f"Position: {result['xyz']}")
```

## 🔗 Related Files

- **[QUICK_START.md](QUICK_START.md)** - Detailed guide
- **[ROS2_SETUP_GUIDE.md](ROS2_SETUP_GUIDE.md)** - ROS2 integration
- **[API_REFERENCE.md](API_REFERENCE.md)** - API documentation

---

For the full research implementation and paper details, see the main [README.md](README.md).