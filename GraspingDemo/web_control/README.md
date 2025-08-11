# SO-101 Robot Web Control Interface

A modern, user-friendly web interface for controlling the SO-101 robot arm with trajectory recording and playback capabilities.

## Features

### 🎯 Core Functionality
- **Real-time Connection Management** - Connect/disconnect to robot with status monitoring
- **Motor Control** - Enable/disable all servos for manual or programmed movement
- **Gripper Control** - Open, close, or set to middle position with visual feedback
- **Live Joint Monitoring** - Real-time display of all 6 joint positions in radians
- **Home Position** - One-click return to default position
- **Cartesian Position Control** - (🚧 Under Development) Move gripper in 3D space
- **Rotation Control** - (🚧 Under Development) Control gripper orientation

### 📹 Trajectory Recording
- **Manual Teaching** - Disable motors and physically move robot to desired positions
- **Keyframe Capture** - Save important poses during manual movement
- **Progress Tracking** - Visual progress bar and keyframe counter
- **Auto-save** - Trajectories automatically saved with timestamps

### 🔄 Trajectory Playback
- **Trajectory Library** - Browse all saved trajectories with metadata
- **One-click Playback** - Select and replay any saved trajectory
- **Delete Management** - Remove unwanted trajectory files
- **Compatible Format** - Works with trajectories from both ROS2 and non-ROS2 versions

## Quick Start

### Prerequisites
- Python 3.8+
- Conda environment (optional but recommended)
- SO-101 robot connected via USB (/dev/ttyACM0)

### Installation

1. **Navigate to web control directory:**
```bash
cd /home/pathonai/Documents/Github/opensource_dev/GraspingDemo/web_control
```

2. **Run the launcher script:**
```bash
./run_web_ui.sh
```

3. **Open your browser:**
```
http://localhost:5000
```

### Manual Installation (if script fails)

```bash
# Install Flask
pip install flask

# Run the server
python app.py
```

## Usage Guide

### Basic Operation

1. **Connect to Robot**
   - Click "Connect" button
   - Wait for green status indicator
   - Verify gripper status shows (OPEN/CLOSED/MIDDLE)

2. **Enable Motors**
   - Click "Enable Motors" to activate servos
   - Click "Disable Motors" for manual movement
   - Use "Go Home Position" to reset robot

3. **Control Gripper**
   - Open: Fully open gripper
   - Middle: Half-open position
   - Close: Fully closed gripper
   - Visual feedback shows current state

4. **Cartesian Position Control (World Frame)** 🚧 *Under Development*
   - Feature temporarily disabled while kinematics are being calibrated
   - Will enable precise 3D position control when complete

5. **Rotation Control** 🚧 *Under Development*
   - Feature temporarily disabled while kinematics are being calibrated
   - Will enable gripper orientation control when complete

### Recording Trajectories

1. **Start Recording**
   - Click "Start" button (motors auto-disable)
   - Recording indicator appears in header

2. **Teach Positions**
   - Physically move robot to desired pose
   - Click "Keyframe" to save current position
   - Repeat for all desired waypoints (minimum 2)

3. **Save Trajectory**
   - Click "Stop" to finish recording
   - File automatically saved with timestamp
   - Motors re-enable after saving

### Playing Trajectories

1. **Load Trajectory List**
   - Click "Refresh List" to see available files
   - Files show creation time and keyframe count

2. **Select and Play**
   - Click on trajectory to select (highlighted in blue)
   - Click "Play Selected" to execute
   - Robot will interpolate smoothly between keyframes

3. **Delete Unwanted Files**
   - Select trajectory
   - Click "Delete Selected"
   - Confirm deletion in popup

## Architecture

### Frontend (index.html)
- **Modern Design** - Responsive card-based layout
- **Real-time Updates** - 1Hz status polling
- **Visual Feedback** - Progress bars, status indicators, animations
- **Mobile Friendly** - Responsive design works on tablets/phones

### Backend (app.py)
- **Flask Server** - RESTful API endpoints
- **Robot Interface** - Direct serial communication via SO101ClientRawSimple
- **File Management** - JSON trajectory storage and retrieval
- **Thread Safety** - Background threads for non-blocking operations

### API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Serve web interface |
| `/api/status` | GET | Get robot status and positions |
| `/api/connect` | POST | Connect to robot |
| `/api/disconnect` | POST | Disconnect from robot |
| `/api/enable_torque` | POST | Enable all motors |
| `/api/disable_torque` | POST | Disable all motors |
| `/api/home` | POST | Move to home position |
| `/api/gripper/{action}` | POST | Control gripper (open/close/middle) |
| `/api/record/start` | POST | Start trajectory recording |
| `/api/record/keyframe` | POST | Save current keyframe |
| `/api/record/stop` | POST | Stop and save recording |
| `/api/trajectories` | GET | List saved trajectories |
| `/api/replay` | POST | Replay selected trajectory |
| `/api/delete` | POST | Delete trajectory file |
| `/api/gripper_pose` | GET | Get gripper position in cartesian coordinates |
| `/api/cartesian_move` | POST | Move gripper in cartesian direction |
| `/api/move_to_position` | POST | Move gripper to absolute position |
| `/api/rotate_gripper` | POST | Rotate gripper around specified axis |

## File Structure

```
web_control/
├── app.py                 # Flask backend server
├── templates/
│   └── index.html        # Web interface
├── run_web_ui.sh         # Launch script
├── README.md             # This file
└── trajectories/         # Saved trajectory files (created automatically)
```

## Trajectory Format

Trajectories are saved as JSON files with the following structure:

```json
{
  "name": "web_20240315_143022",
  "created": "2024-03-15T14:30:22",
  "num_keyframes": 5,
  "keyframes": [
    {
      "index": 0,
      "pose": [0.0, 0.0, 0.0, 0.0, 0.0, 1.5],
      "timestamp": 1710512422.123
    }
  ]
}
```

## Troubleshooting

### Connection Issues
- **Check USB Connection**: Ensure robot is connected to /dev/ttyACM0
- **Permissions**: May need to run with sudo or add user to dialout group
- **Port Busy**: Kill any other processes using the serial port

### Web Interface Issues
- **Port 5000 Busy**: Change port in app.py or kill existing process
- **No Robot Response**: Check motor power and emergency stop
- **Trajectory Not Playing**: Ensure motors are enabled before playback

### Common Commands

```bash
# Check if robot is connected
ls /dev/ttyACM*

# Add user to dialout group (for USB access)
sudo usermod -a -G dialout $USER

# Kill process on port 5000
sudo lsof -i :5000
kill -9 [PID]

# Monitor Flask logs
python app.py 2>&1 | tee web_control.log
```

## Safety Notes

⚠️ **IMPORTANT SAFETY INFORMATION**

1. **Clear Workspace**: Ensure adequate clearance around robot before operation
2. **Emergency Stop**: Keep hand near power switch during operation
3. **Manual Mode**: Always disable motors before manual movement
4. **Speed Control**: Start with slow movements when testing trajectories
5. **Gripper Force**: Be cautious with gripper close operations

## Integration with ROS2

This web interface can work alongside ROS2 control:

1. **Shared Trajectories**: Files are compatible between systems
2. **Coordinate System**: Uses same raw joint values as joint_state_reader
3. **Parallel Operation**: Can switch between web and ROS2 control

## Development

### Adding New Features

1. **Backend**: Add endpoint in `app.py`
2. **Frontend**: Update UI in `templates/index.html`
3. **API Call**: Add JavaScript function for new endpoint

### Customization

- **Robot Port**: Change `/dev/ttyACM0` in app.py line 37
- **Web Port**: Change port 5000 in app.py line 332
- **Update Rate**: Modify interval in index.html line 643
- **Trajectory Directory**: Update TRAJ_DIR in app.py line 29

## Support

For issues or questions:
1. Check this README first
2. Review troubleshooting section
3. Check robot hardware connections
4. Verify software dependencies

## License

This software is provided as-is for educational and research purposes.

---

**Version**: 1.0.0  
**Last Updated**: 2024  
**Compatible With**: SO-101 Robot, ROS2 Jazzy, Python 3.8+