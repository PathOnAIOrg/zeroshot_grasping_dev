#!/bin/bash

# ROS2 Client Runner Script
# This script helps run the ROS2 client with proper environment setup

echo "🚀 ThinkGrasp ROS2 Client Runner"
echo "================================"

# Check arguments
if [ "$#" -ne 3 ]; then
    echo "Usage: $0 <rgb_image> <depth_image> <instruction>"
    echo "Example: $0 rgb.png depth.png 'grasp the object'"
    exit 1
fi

RGB_IMAGE="$1"
DEPTH_IMAGE="$2"
INSTRUCTION="$3"

# Convert to absolute paths
RGB_IMAGE=$(realpath "$RGB_IMAGE" 2>/dev/null || echo "$RGB_IMAGE")
DEPTH_IMAGE=$(realpath "$DEPTH_IMAGE" 2>/dev/null || echo "$DEPTH_IMAGE")

echo "RGB Image: $RGB_IMAGE"
echo "Depth Image: $DEPTH_IMAGE"
echo "Instruction: $INSTRUCTION"
echo ""

# Option 1: Try ROS2 if available
if [ -d "$HOME/thinkgrasp_ros2_ws/install" ]; then
    echo "Attempting ROS2 method..."
    cd $HOME/thinkgrasp_ros2_ws
    
    # Source ROS2
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash 2>/dev/null
    elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
        source /opt/ros/jazzy/setup.bash 2>/dev/null
    fi
    
    # Source workspace
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash 2>/dev/null
    fi
    
    # Try to run
    ros2 run thinkgrasp_ros2 thinkgrasp_client_node "$RGB_IMAGE" "$DEPTH_IMAGE" "$INSTRUCTION" 2>/dev/null
    
    if [ $? -eq 0 ]; then
        echo "✅ ROS2 client executed successfully"
        exit 0
    else
        echo "⚠️ ROS2 method failed, falling back to direct Python..."
    fi
fi

# Option 2: Direct Python execution
echo "Using direct Python client..."

# Find the simple client
SIMPLE_CLIENT=""
if [ -f "/home/pathonai/Documents/Github/opensource_dev/ThinkGrasp/simple_grasp_client.py" ]; then
    SIMPLE_CLIENT="/home/pathonai/Documents/Github/opensource_dev/ThinkGrasp/simple_grasp_client.py"
elif [ -f "./simple_grasp_client.py" ]; then
    SIMPLE_CLIENT="./simple_grasp_client.py"
fi

if [ -n "$SIMPLE_CLIENT" ]; then
    echo "Running: python3 $SIMPLE_CLIENT"
    python3 "$SIMPLE_CLIENT" "$RGB_IMAGE" "$DEPTH_IMAGE" "$INSTRUCTION"
    
    if [ $? -eq 0 ]; then
        echo ""
        echo "✅ Grasp detection completed successfully"
        echo "📊 View results at: http://localhost:5010/viewer"
    else
        echo "❌ Error running client"
        exit 1
    fi
else
    echo "❌ Could not find client script"
    echo "Please ensure you're in the ThinkGrasp directory or install it properly"
    exit 1
fi