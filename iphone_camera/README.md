# Robot-Camera-iPhone

A comprehensive Python library for streaming and processing RGB-D data from iPhone devices using the Record3D app. This project enables real-time point cloud visualization, camera pose tracking, and RGB-D data capture for robotics and computer vision applications.

## Features

- **Real-time RGB-D Streaming**: Stream depth and color data from iPhone's TrueDepth camera
- **Camera Pose Tracking**: Track device position and orientation in real-time
- **Point Cloud Visualization**: Interactive 3D visualization using Open3D
- **Multiple Visualization Modes**: Support for different visualization approaches
- **Data Recording**: Save RGB-D sequences and point cloud data
- **Confidence Filtering**: Filter depth data based on confidence maps
- **Cross-platform Support**: Works on macOS, Linux, and Windows

## Prerequisites

### Hardware Requirements
- iPhone with TrueDepth camera (iPhone X or newer)
- USB cable for connection
- Computer with USB port

### Software Requirements
- Python 3.12 
- [Record3D iOS App](https://record3d.app/) (requires $5 USD for USB streaming)
- CMake >= 3.13.0
- iTunes (macOS/Windows) or libusbmuxd (Linux)

### On your iPhone
Download Record3D app on your iPhone, which allows you (among other features) to live-stream RGBD video from iOS devices with TrueDepth camera to a computer.

Connect your iPhone to your computer using a USB 3.0 cable.

In the Record3D app, select USB mode in the settings (a one-time fee of $5 is required). Press the record button to start streaming RGBD data. Once you press the record button, make sure the app remains in the foreground. If it stops working, try restarting the app.


## Installation

### 1. Create Conda Environment
```bash
conda create -y -n record3d python=3.12
conda activate record3d
```

### 2. Install Record3D Library
```bash
git clone https://github.com/marek-simonik/record3d
cd record3d
python setup.py install
cd ..
```

### 3. Install Required Dependencies
```bash
pip install opencv-python
pip install open3d
pip install --upgrade --force-reinstall numpy-quaternion
```

### 4. Linux Users (Additional Setup)
```bash
sudo apt install libusbmuxd-dev
```

## Usage

### Setup iPhone
1. Download and install [Record3D app](https://record3d.app/) on your iPhone
2. Connect iPhone to computer via USB cable
3. Open Record3D app and go to Settings
4. Enable "USB Streaming mode"
5. Press the record button to start streaming

### Available Scripts

#### 1. Point Cloud Streaming with Visualization
```bash
python iphone_point_cloud_streaming_vis.py
```
- Streams real-time point clouds from iPhone
- Displays interactive 3D visualization using Open3D
- Shows camera trajectory and workspace boundaries
- Saves point cloud data for later analysis

#### 2. Camera Pose Streaming with Visualization
```bash
python iphone_camera_pose_streaming_vis.py
```
- Tracks camera position and orientation in real-time
- Visualizes camera trajectory in 3D space
- Shows coordinate frames and workspace boundaries
- Useful for SLAM and pose estimation applications

#### 3. RGB-D Data Recording
```bash
python iphone_save_rgbd.py
```
- Captures and saves RGB-D sequences
- Records camera poses for each frame
- Saves data in numpy format for post-processing
- Useful for dataset creation and offline analysis

#### 4. Advanced Visualization Tools
```bash
python visualization.py
```
- Comprehensive visualization utilities
- Multiple visualization modes (Open3D, Plotly)
- Point cloud animation and video generation
- Debug tools for pose analysis

## API Reference

### Main Classes

#### DemoApp
The main application class that handles device connection and data processing.

**Key Methods:**
- `connect_to_device(dev_idx)`: Connect to iPhone device
- `start_processing_stream()`: Start processing RGB-D stream
- `get_global_xyz()`: Convert depth data to 3D point cloud
- `pose_to_extrinsic_matrix()`: Convert camera pose to transformation matrix

**Key Properties:**
- `session`: Record3D stream session
- `vis`: Open3D visualizer
- `pcd`: Point cloud object
- `init_camera_pose`: Initial camera pose reference

### Data Structures

#### RGB-D Frame
- **RGB Image**: Color image (720x960x3)
- **Depth Image**: Depth map (256x192)
- **Confidence Map**: Depth confidence values
- **Camera Pose**: Position and orientation (quaternion + translation)

#### Point Cloud
- **Points**: 3D coordinates in world frame
- **Colors**: RGB values from color image
- **Transformation**: Applied camera pose transformations

## Configuration

### Resolution Settings
```python
self.rgb_width = 720
self.rgb_height = 960
self.depth_width = 192
self.depth_height = 256
```

### Depth Processing
```python
depth_scale = 1000.0  # Scale factor for depth values
only_confident = False  # Filter by confidence map
```

### Visualization Settings
```python
window_width = 720
window_height = 960
coordinate_frame_size = 0.1
```

## Troubleshooting

### Common Issues

1. **Device Not Found**
   - Ensure iPhone is connected via USB
   - Check that Record3D app is running
   - Verify USB streaming mode is enabled

2. **Connection Errors**
   - Restart Record3D app
   - Reconnect USB cable
   - Check device permissions

3. **Visualization Issues**
   - Update Open3D to latest version
   - Check graphics driver compatibility
   - Reduce point cloud density if performance is poor

4. **Import Errors**
   - Ensure all dependencies are installed
   - Check Python environment activation
   - Verify record3d library installation

### Performance Optimization

- Reduce visualization update frequency
- Downsample point clouds for better performance
- Use confidence filtering to reduce noise
- Close unnecessary applications

## Examples

### Basic Point Cloud Capture
```python
from iphone_point_cloud_streaming_vis import DemoApp

app = DemoApp()
app.connect_to_device(0)  # Connect to first device
app.start_processing_stream()
```

### Custom Visualization
```python
# Add custom geometry to visualization
cube = app.create_wireframe_cube(size=0.5)
app.vis.add_geometry(cube)
```

### Data Export
```python
# Save point cloud data
import open3d as o3d
o3d.io.write_point_cloud("captured_scene.ply", app.pcd)
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- [Robot-Camera-iPhone](https://github.com/haoyu-x/Robot-Camera-iPhone)
- [Record3D](https://record3d.app/)
- [Marek Simonik](https://github.com/marek-simonik)

## Support

For issues and questions:
- Check the troubleshooting section above
- Review the [Record3D documentation](https://github.com/marek-simonik/record3d)
- Open an issue on GitHub

## Changelog

### Version 1.0.0
- Initial release with basic RGB-D streaming
- Point cloud visualization support
- Camera pose tracking
- Data recording capabilities
