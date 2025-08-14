# ThinkGrasp ROS2 Quick Reference

## ✅ ROS2 is Working!

### Quick Command
```bash
./ros2_run.sh simple_client_node outputs/20250814_002629_010/input_rgb.png outputs/20250814_002629_010/input_depth.png "grasp the item"
```

### Important: Correct Usage
✅ **CORRECT:**
```bash
./ros2_run.sh simple_client_node rgb.png depth.png "instruction"
```

❌ **WRONG (extra argument):**
```bash
./ros2_run.sh simple_client_node thinkgrasp_client_node.py rgb.png depth.png "instruction"
```

## Available Scripts

| Script | Purpose | Usage |
|--------|---------|-------|
| `ros2_run.sh` | Run ROS2 nodes | `./ros2_run.sh simple_client_node <rgb> <depth> <text>` |
| `simple_grasp_client.py` | Direct Python client | `python simple_grasp_client.py <rgb> <depth> <text>` |
| `run_ros2_client.sh` | Auto-fallback script | `./run_ros2_client.sh <rgb> <depth> <text>` |

## ROS2 Node Features

The `simple_client_node`:
- ✅ Full ROS2 integration
- ✅ Publishes to `/thinkgrasp/result` topic  
- ✅ ROS2 logging with timestamps
- ✅ Works without custom services
- ✅ Proper error handling

## Example Workflow

```bash
# Terminal 1: Start API server
python realarm310.py

# Terminal 2: Run ROS2 grasp detection
./ros2_run.sh simple_client_node \
    outputs/20250814_002629_010/input_rgb.png \
    outputs/20250814_002629_010/input_depth.png \
    "grasp the item"

# Terminal 3: View results
# Open browser: http://localhost:5010/viewer
```

## Output Format

ROS2 node publishes JSON to `/thinkgrasp/result`:
```json
{
  "xyz": [-0.0705, 0.0008, 0.3490],
  "rot": [[...], [...], [...]],
  "dep": 0.04,
  "timestamp": "20250814_011707_496"
}
```

## Important Notes

⚠️ **DO NOT use sudo** - It causes permission issues with temporary files
✅ Run normally: `./ros2_run.sh simple_client_node rgb.png depth.png "text"`
❌ Not with sudo: `sudo ./ros2_run.sh ...`

## Troubleshooting

### "RGB image not found" 
- Check you're not adding extra arguments
- Ensure paths are correct
- Use absolute or relative paths from current directory

### OpenCV imread error / 500 Internal Server Error
- **DO NOT use sudo** when running the ROS2 node
- The API server can't access files created by sudo
- Run normally without sudo

### "Node not found"
- Run from ThinkGrasp directory
- Ensure workspace was built: `cd ~/thinkgrasp_ros2_ws && colcon build`

### API Connection Failed
- Ensure server is running: `python realarm310.py`
- Check port 5010 is available