# SO-101 Robot Control - Quick Start

## Run the Web Interface

```bash
./run_web_ui.sh
```

Then open your browser to: http://localhost:5000

## Features

- **Robot Control**: Real-time joint control with reliability improvements
- **Camera**: RealSense depth camera integration  
- **Point Cloud**: 3D visualization with Plotly
- **Recording**: Trajectory recording and playback
- **Live Streaming**: Real-time robot and point cloud visualization

## Project Structure

```
GraspingDemo/
├── web_control/        # Web interface (Flask app)
├── so101_grasp/       # Robot control library
├── robot_description/ # URDF and mesh files
├── captures/          # Saved camera captures
├── trajectories/      # Recorded trajectories
└── run_web_ui.sh     # Main startup script
```

## Built-in Reliability

The robot client includes automatic:
- Retry mechanisms (up to 5 attempts)
- Position history tracking
- Fallback to last valid positions
- Communication statistics

No configuration needed - it just works!