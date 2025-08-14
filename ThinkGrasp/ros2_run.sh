#!/bin/bash

# ROS2 ThinkGrasp Runner
# This script properly sets up the environment to run ROS2 nodes

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}🚀 ThinkGrasp ROS2 Runner${NC}"
echo "================================"

# Check arguments
if [ "$#" -lt 1 ]; then
    echo "Usage: $0 <node_name> [args...]"
    echo ""
    echo "Available nodes:"
    echo "  simple_client_node <rgb> <depth> <instruction> - Run grasp detection"
    echo ""
    echo "Example:"
    echo "  $0 simple_client_node rgb.png depth.png 'grasp the object'"
    echo ""
    echo "Full example with real paths:"
    echo "  $0 simple_client_node outputs/20250814_002629_010/input_rgb.png outputs/20250814_002629_010/input_depth.png 'grasp the item'"
    exit 1
fi

# Special check for simple_client_node
if [ "$1" = "simple_client_node" ] && [ "$#" -ne 4 ]; then
    echo -e "${RED}Error: simple_client_node requires exactly 3 arguments${NC}"
    echo ""
    echo "Usage: $0 simple_client_node <rgb_image> <depth_image> <instruction>"
    echo ""
    echo "You provided $# arguments:"
    for i in "$@"; do
        echo "  - $i"
    done
    echo ""
    echo "Example:"
    echo "  $0 simple_client_node outputs/20250814_002629_010/input_rgb.png outputs/20250814_002629_010/input_depth.png 'grasp the item'"
    exit 1
fi

NODE_NAME=$1
shift  # Remove first argument

# Set up Python path
export PYTHONPATH=/home/pathonai/thinkgrasp_ros2_ws/install/thinkgrasp_ros2/lib/python3.12/site-packages:$PYTHONPATH

# Set up ROS2 environment if available
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash 2>/dev/null
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
    # Jazzy has issues, but try anyway
    source /opt/ros/jazzy/setup.bash 2>/dev/null
elif [ -f "/opt/ros/iron/setup.bash" ]; then
    source /opt/ros/iron/setup.bash 2>/dev/null
fi

# Determine the node executable path
NODE_PATH="/home/pathonai/thinkgrasp_ros2_ws/install/thinkgrasp_ros2/lib/thinkgrasp_ros2/${NODE_NAME}"

if [ ! -f "$NODE_PATH" ]; then
    echo -e "${RED}Error: Node '${NODE_NAME}' not found${NC}"
    echo "Available nodes:"
    ls /home/pathonai/thinkgrasp_ros2_ws/install/thinkgrasp_ros2/lib/thinkgrasp_ros2/ 2>/dev/null | grep -v __pycache__ | sed 's/^/  /'
    exit 1
fi

# Run the node
echo -e "${YELLOW}Running: ${NODE_NAME}${NC}"
echo "Arguments: $@"
echo ""

python3 "$NODE_PATH" "$@"

EXIT_CODE=$?

if [ $EXIT_CODE -eq 0 ]; then
    echo ""
    echo -e "${GREEN}✅ Node executed successfully${NC}"
else
    echo ""
    echo -e "${RED}❌ Node exited with error code: $EXIT_CODE${NC}"
fi

exit $EXIT_CODE