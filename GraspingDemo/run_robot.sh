#!/bin/bash

# SO-101 Robot Control Launcher
# Usage: ./run_robot.sh [command]

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Conda environment name
CONDA_ENV="sim"

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Function to print colored messages
print_msg() {
    echo -e "${GREEN}[SO-101]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_header() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}     SO-101 Robot Control System${NC}"
    echo -e "${BLUE}========================================${NC}"
}

# Function to activate conda environment
activate_conda() {
    print_msg "Activating conda environment: ${CONDA_ENV}"
    
    # Try to find conda
    if command -v conda &> /dev/null; then
        eval "$(conda shell.bash hook)"
        conda activate ${CONDA_ENV}
        if [ $? -eq 0 ]; then
            print_msg "Conda environment activated ✓"
        else
            print_error "Failed to activate conda environment '${CONDA_ENV}'"
            print_msg "Make sure the environment exists: conda create -n ${CONDA_ENV} python=3.10"
            exit 1
        fi
    else
        print_error "Conda not found! Please install Anaconda/Miniconda first."
        exit 1
    fi
}

# Function to show help
show_help() {
    print_header
    echo ""
    echo "Usage: ./run_robot.sh [command]"
    echo ""
    echo "Commands:"
    echo "  basic     - Run basic control with calibration"
    echo "  raw       - Run raw control (no calibration)"
    echo "  record    - Record and replay trajectory"
    echo "  replay    - Replay saved trajectories"
    echo "  gripper   - Test gripper open/close"
    echo "  release   - Release/unlock robot motors"
    echo "  help      - Show this help message"
    echo ""
    echo "Examples:"
    echo "  ./run_robot.sh basic"
    echo "  ./run_robot.sh record"
    echo ""
}

# Main script
print_header

# Check if no arguments provided
if [ $# -eq 0 ]; then
    show_help
    exit 0
fi

# Parse command
COMMAND=$1

# Change to script directory
cd "${SCRIPT_DIR}"

# Activate conda environment
activate_conda

# Execute based on command
case ${COMMAND} in
    basic)
        print_msg "Starting basic control (with calibration)..."
        echo ""
        python examples/basic_control.py
        ;;
        
    raw)
        print_msg "Starting raw control (no calibration)..."
        echo ""
        python examples/basic_control_raw.py
        ;;
        
    record)
        print_msg "Starting trajectory recorder (with gripper info)..."
        echo ""
        python examples/trajectory_with_gripper_info.py
        ;;
        
    replay)
        print_msg "Starting trajectory replay..."
        echo ""
        python examples/replay_trajectory.py
        ;;
        
    gripper)
        print_msg "Starting gripper test..."
        echo ""
        python examples/gripper_test.py
        ;;
        
    release)
        print_msg "Releasing robot motors..."
        echo ""
        python release_torque.py
        ;;
        
    help)
        show_help
        ;;
        
    *)
        print_error "Unknown command: ${COMMAND}"
        echo ""
        show_help
        exit 1
        ;;
esac

# Check exit status
if [ $? -eq 0 ]; then
    echo ""
    print_msg "Command completed successfully ✓"
else
    echo ""
    print_error "Command failed!"
fi