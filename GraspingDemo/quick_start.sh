#!/bin/bash

# Quick launcher with menu
# Just run: ./quick_start.sh

# Activate conda
eval "$(conda shell.bash hook)"
conda activate sim

clear
echo "================================"
echo "   SO-101 Robot Control Menu"
echo "================================"
echo ""
echo "1) Basic Control (calibrated)"
echo "2) Raw Control (no calibration)" 
echo "3) Record Trajectory"
echo "4) Replay Trajectory"
echo "5) Test Gripper"
echo "6) Release Motors"
echo "0) Exit"
echo ""
read -p "Choose [0-6]: " choice

case $choice in
    1) python examples/basic_control.py ;;
    2) python examples/basic_control_raw.py ;;
    3) python examples/simple_trajectory_demo.py ;;
    4) python examples/replay_trajectory.py ;;
    5) python examples/gripper_test.py ;;
    6) python release_torque.py ;;
    0) echo "Bye!" ;;
    *) echo "Invalid choice!" ;;
esac