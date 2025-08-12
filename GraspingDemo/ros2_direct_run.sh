#!/bin/bash

# Direct ROS2 Node Runner for SO-101
# Since ros2 run has issues, we run Python scripts directly

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${BLUE}================================${NC}"
echo -e "${BLUE}   SO-101 ROS2 Control (Direct)${NC}"
echo -e "${BLUE}================================${NC}"

# Base directory
ROS2_BRIDGE_DIR="/home/pathonai/Documents/Github/opensource_dev/so-arm101-ros2-bridge"
SCRIPTS_DIR="${ROS2_BRIDGE_DIR}/src/jointstatereader/jointstatereader"

# Check command
CMD=${1:-help}

case $CMD in
    reader)
        echo -e "${GREEN}Starting joint state reader...${NC}"
        cd ${ROS2_BRIDGE_DIR}
        python3 ${SCRIPTS_DIR}/joint_state_reader.py
        ;;
        
    player)
        echo -e "${GREEN}Starting trajectory player...${NC}"
        TRAJ_FILE=${2:-""}
        cd ${ROS2_BRIDGE_DIR}
        if [ -z "$TRAJ_FILE" ]; then
            echo "Usage: ./ros2_direct_run.sh player <trajectory_file>"
            echo ""
            echo "Available trajectories:"
            ls ~/Documents/Github/opensource_dev/GraspingDemo/trajectories/*.json 2>/dev/null | xargs -n1 basename
        else
            python3 ${SCRIPTS_DIR}/trajectory_player.py "$TRAJ_FILE"
        fi
        ;;
        
    controller)
        echo -e "${GREEN}Starting joint controller...${NC}"
        cd ${ROS2_BRIDGE_DIR}
        python3 ${SCRIPTS_DIR}/joint_controller.py
        ;;
        
    record)
        echo -e "${GREEN}Starting trajectory recorder...${NC}"
        cd ${ROS2_BRIDGE_DIR}
        python3 ${SCRIPTS_DIR}/trajectory_recorder.py
        ;;
        
    test-import)
        echo -e "${GREEN}Testing Python imports...${NC}"
        python3 -c "
import sys
sys.path.insert(0, '${SCRIPTS_DIR}')
try:
    import rclpy
    print('✅ rclpy imported successfully')
except ImportError as e:
    print('❌ Failed to import rclpy:', e)
    
try:
    from sensor_msgs.msg import JointState
    print('✅ sensor_msgs imported successfully')
except ImportError as e:
    print('❌ Failed to import sensor_msgs:', e)
"
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
        echo "Usage: ./ros2_direct_run.sh [command] [options]"
        echo ""
        echo "Commands:"
        echo "  reader       - Start joint state reader (hardware interface)"
        echo "  record       - Record trajectory (like ./run_robot.sh record)"
        echo "  player FILE  - Play trajectory file"
        echo "  controller   - Start interactive controller"
        echo "  test-import  - Test Python ROS2 imports"
        echo "  topics       - List ROS2 topics"
        echo "  echo-states  - Echo /joint_states topic"
        echo "  echo-commands - Echo /joint_commands topic"
        echo ""
        echo "Example workflow:"
        echo "  Terminal 1: ./ros2_direct_run.sh reader"
        echo "  Terminal 2: ./ros2_direct_run.sh player demo_trajectory_20250811_023603.json"
        echo ""
        echo "Note: This script runs Python files directly since 'ros2 run' has issues"
        ;;
esac