#!/bin/bash
# Launch RealSense with proper colored point cloud

echo "=========================================="
echo "🎨 Launching RealSense with COLORED Point Cloud"
echo "=========================================="

# Kill any existing realsense nodes
echo "Stopping any existing RealSense nodes..."
ros2 node list | grep realsense | xargs -I {} ros2 lifecycle set {} shutdown 2>/dev/null
pkill -f realsense2_camera_node 2>/dev/null
sleep 2

echo "Starting RealSense with color-enabled point cloud..."
echo ""

# Launch with proper color settings
ros2 launch realsense2_camera rs_launch.py \
    enable_rgbd:=true \
    enable_sync:=true \
    enable_color:=true \
    enable_depth:=true \
    align_depth.enable:=true \
    pointcloud.enable:=true \
    pointcloud.stream_filter:=2 \
    pointcloud.stream_index_filter:=-1 \
    pointcloud.ordered_pc:=false \
    pointcloud.allow_no_texture_points:=false \
    depth_module.profile:=640x480x30 \
    rgb_camera.profile:=640x480x30 \
    depth_module.color_scheme:=0 \
    initial_reset:=true

# Note about stream_filter values:
# 0 = RS2_STREAM_ANY
# 1 = RS2_STREAM_DEPTH  
# 2 = RS2_STREAM_COLOR (we want this for colored point cloud)
# 3 = RS2_STREAM_INFRARED