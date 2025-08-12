#!/bin/bash
# Launch SO-101 digital twin synchronized with real robot

echo "=========================================="
echo "🤖 LAUNCHING SO-101 REAL ROBOT DIGITAL TWIN"
echo "=========================================="

# Function to cleanup on exit
cleanup() {
    echo "Cleaning up..."
    pkill -f real_robot_sync.py
    pkill -f robot_state_publisher
    exit 0
}
trap cleanup EXIT

# Check if robot port is provided
ROBOT_PORT=${1:-/dev/ttyACM0}
echo "Using robot port: $ROBOT_PORT"

# Terminal 1: Robot state publisher (URDF to TF)
echo "Starting robot state publisher..."
gnome-terminal --tab --title="Robot State Publisher" -- bash -c "
source /opt/ros/humble/setup.bash
source ~/Documents/Github/opensource_dev/ROS2/lerobot_ws/install/setup.bash
ros2 launch lerobot_description so101_state_publisher.launch.py use_sim_time:=false gui:=false
" &

sleep 2

# Terminal 2: Real robot sync (reads robot and publishes joint states)
echo "Starting real robot synchronization..."
gnome-terminal --tab --title="Real Robot Sync" -- bash -c "
cd $(dirname $0)
python3 real_robot_sync.py --port $ROBOT_PORT
" &

sleep 2

# Terminal 3: Stationary camera connection
echo "Connecting stationary camera..."
gnome-terminal --tab --title="Camera Connection" -- bash -c "
cd $(dirname $0)
python3 connect_camera_to_robot_stationary.py
" &

sleep 2

# Terminal 4: RealSense camera
echo "Starting RealSense camera..."
gnome-terminal --tab --title="RealSense" -- bash -c "
cd $(dirname $0)
./launch_colored_pointcloud.sh
" &

sleep 3

# Terminal 5: RViz2
echo "Launching RViz2..."
gnome-terminal --tab --title="RViz2" -- bash -c "
rviz2 -d $(dirname $0)/config/so101_real_robot.rviz
" &

echo ""
echo "=========================================="
echo "✅ ALL COMPONENTS LAUNCHED!"
echo "=========================================="
echo ""
echo "The digital twin should now be synchronized with your real robot."
echo "Move the real robot and watch it update in RViz2!"
echo ""
echo "Press Ctrl+C to stop all components"

# Keep script running
while true; do
    sleep 1
done