#!/bin/bash
# Complete setup script for SO-101 + RealSense in RViz2

echo "=========================================="
echo "🚀 SO-101 + RealSense Complete Setup"
echo "=========================================="

# Terminal 1: RealSense camera
echo "1. Starting RealSense camera..."
gnome-terminal --tab --title="RealSense" -- bash -c "
ros2 launch realsense2_camera rs_launch.py \
    enable_rgbd:=true \
    enable_sync:=true \
    align_depth.enable:=true \
    enable_color:=true \
    enable_depth:=true \
    pointcloud.enable:=true \
    pointcloud.ordered_pc:=true;
exec bash"

sleep 3

# Terminal 2: Robot model
echo "2. Starting SO-101 robot model..."
gnome-terminal --tab --title="Robot Model" -- bash -c "
cd ~/Documents/Github/opensource_dev/ROS2/lerobot_ws && \
source install/setup.bash && \
ros2 launch lerobot_description so101_display.launch.py;
exec bash"

sleep 2

# Terminal 3: Camera transform
echo "3. Adding camera to robot..."
gnome-terminal --tab --title="Camera Transform" -- bash -c "
cd ~/Documents/Github/opensource_dev/handeye/ros2_digital_twin && \
python3 add_camera_to_robot.py;
exec bash"

sleep 2

# Terminal 4: Check setup
echo "4. Checking setup..."
gnome-terminal --tab --title="Check Setup" -- bash -c "
cd ~/Documents/Github/opensource_dev/handeye/ros2_digital_twin && \
python3 check_robot_setup.py;
exec bash"

sleep 2

# Terminal 5: RViz2
echo "5. Starting RViz2..."
echo ""
echo "=========================================="
echo "📋 RViz2 Setup Instructions:"
echo "=========================================="
echo "1. Set Fixed Frame to: 'base' (or 'base_link')"
echo "2. Add Displays:"
echo "   - RobotModel"
echo "   - PointCloud2 → Topic: /camera/camera/depth/color/points"
echo "   - TF"
echo "   - Image → Topic: /camera/camera/color/image_raw"
echo "=========================================="
echo ""

rviz2