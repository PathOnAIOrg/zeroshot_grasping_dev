#!/bin/bash

# ROS2 Control Script for SO-101
# This script launches ROS2 nodes for robot control

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}================================${NC}"
echo -e "${BLUE}   SO-101 ROS2 Control System${NC}"
echo -e "${BLUE}================================${NC}"

# Source ROS2 (Jazzy)
source /opt/ros/jazzy/setup.bash

# Check command
CMD=${1:-help}

case $CMD in
    reader)
        echo -e "${GREEN}Starting joint state reader...${NC}"
        cd ~/Documents/Github/opensource_dev/so-arm101-ros2-bridge
        colcon build
        source install/setup.bash
        ros2 run jointstatereader joint_state_reader
        ;;
        
    player)
        echo -e "${GREEN}Starting trajectory player...${NC}"
        TRAJ_FILE=${2:-""}
        cd ~/Documents/Github/opensource_dev/so-arm101-ros2-bridge
        source install/setup.bash
        if [ -z "$TRAJ_FILE" ]; then
            echo "Usage: ./ros2_control.sh player <trajectory_file>"
            echo ""
            echo "Available trajectories:"
            ls ~/Documents/Github/opensource_dev/GraspingDemo/trajectories/*.json 2>/dev/null | xargs -n1 basename
        else
            ros2 run jointstatereader trajectory_player "$TRAJ_FILE"
        fi
        ;;
        
    controller)
        echo -e "${GREEN}Starting joint controller...${NC}"
        cd ~/Documents/Github/opensource_dev/so-arm101-ros2-bridge
        source install/setup.bash
        ros2 run jointstatereader joint_controller
        ;;
        
    build)
        echo -e "${GREEN}Building ROS2 packages...${NC}"
        cd ~/Documents/Github/opensource_dev/so-arm101-ros2-bridge
        # Clean build
        rm -rf build install log
        colcon build --packages-select jointstatereader --symlink-install
        source install/setup.bash
        echo -e "${GREEN}Build complete! Available nodes:${NC}"
        ros2 pkg executables jointstatereader
        ;;
        
    topics)
        echo -e "${GREEN}Listing ROS2 topics...${NC}"
        ros2 topic list
        ;;
        
    echo-states)
        echo -e "${GREEN}Echoing joint states...${NC}"
        ros2 topic echo /joint_states
        ;;
        
    echo-commands)
        echo -e "${GREEN}Echoing joint commands...${NC}"
        ros2 topic echo /joint_commands
        ;;
        
    help|*)
        echo "Usage: ./ros2_control.sh [command] [options]"
        echo ""
        echo "Commands:"
        echo "  reader       - Start joint state reader (hardware interface)"
        echo "  player FILE  - Play trajectory file"
        echo "  controller   - Start interactive controller"
        echo "  build        - Build ROS2 packages"
        echo "  topics       - List ROS2 topics"
        echo "  echo-states  - Echo /joint_states topic"
        echo "  echo-commands - Echo /joint_commands topic"
        echo ""
        echo "Example workflow:"
        echo "  Terminal 1: ./ros2_control.sh reader"
        echo "  Terminal 2: ./ros2_control.sh player demo_trajectory_20250811_023603.json"
        ;;
esac