#!/bin/bash
# Quick test script for digital twin

echo "============================================================"
echo "🧪 TESTING SO-101 DIGITAL TWIN SETUP"
echo "============================================================"

# Test 1: Check if workspace exists
echo -n "1. Checking ROS2 workspace... "
if [ -d "/home/pathonai/Documents/Github/opensource_dev/ROS2/lerobot_ws" ]; then
    echo "✅ Found"
else
    echo "❌ Not found"
    echo "   Please ensure lerobot_ws is in the correct location"
    exit 1
fi

# Test 2: Check if URDF exists
echo -n "2. Checking URDF file... "
URDF_LOCATIONS=(
    "/home/pathonai/Documents/Github/opensource_dev/ROS2/lerobot_ws/src/lerobot_description/urdf/so101.urdf.xacro"
    "/home/pathonai/Documents/Github/opensource_dev/ROS2/lerobot_ws/install/lerobot_description/share/lerobot_description/urdf/so101.urdf.xacro"
)
URDF_FOUND=false
for loc in "${URDF_LOCATIONS[@]}"; do
    if [ -f "$loc" ]; then
        echo "✅ Found at $(basename $(dirname $loc))"
        URDF_FOUND=true
        break
    fi
done
if [ "$URDF_FOUND" = false ]; then
    echo "❌ Not found"
    echo "   URDF file missing from expected locations"
fi

# Test 3: Check ROS2 installation
echo -n "3. Checking ROS2... "
if command -v ros2 &> /dev/null; then
    ROS_VERSION=$(ros2 --version 2>&1 | grep -oP 'ros2 \K\w+' || echo "unknown")
    echo "✅ Found ($ROS_VERSION)"
else
    echo "❌ Not found"
    echo "   Please install ROS2 Humble or Jazzy"
    exit 1
fi

# Test 4: Check if robot port exists
echo -n "4. Checking robot port... "
if [ -e "/dev/ttyACM0" ]; then
    echo "✅ Found /dev/ttyACM0"
elif [ -e "/dev/ttyUSB0" ]; then
    echo "⚠️  Found /dev/ttyUSB0 instead"
    echo "   Use: ./launch_digital_twin.sh /dev/ttyUSB0"
else
    echo "⚠️  No robot port found"
    echo "   Will run in simulation mode"
fi

# Test 5: Check Python dependencies
echo -n "5. Checking Python dependencies... "
MISSING_DEPS=""
python3 -c "import pyrealsense2" 2>/dev/null || MISSING_DEPS="$MISSING_DEPS pyrealsense2"
python3 -c "import numpy" 2>/dev/null || MISSING_DEPS="$MISSING_DEPS numpy"
python3 -c "import scipy" 2>/dev/null || MISSING_DEPS="$MISSING_DEPS scipy"

if [ -z "$MISSING_DEPS" ]; then
    echo "✅ All installed"
else
    echo "⚠️  Missing:$MISSING_DEPS"
    echo "   Install with: pip install$MISSING_DEPS"
fi

# Test 6: Check calibration files
echo -n "6. Checking calibration files... "
CALIB_FILES=(
    "../output/handeye_realsense.npz"
    "../output/handeye_realsense_stationary.npz"
)
CALIB_FOUND=false
for file in "${CALIB_FILES[@]}"; do
    if [ -f "$file" ]; then
        echo "✅ Found $(basename $file)"
        CALIB_FOUND=true
        break
    fi
done
if [ "$CALIB_FOUND" = false ]; then
    echo "⚠️  No calibration found"
    echo "   Run calibration first or camera won't be positioned correctly"
fi

echo ""
echo "============================================================"
echo "📋 LAUNCH OPTIONS:"
echo "============================================================"
echo ""
echo "1. WITH REAL ROBOT:"
echo "   ./launch_digital_twin.sh /dev/ttyACM0"
echo ""
echo "2. SIMULATION MODE (no robot):"
echo "   ./launch_digital_twin.sh --simulate"
echo ""
echo "3. MANUAL LAUNCH:"
echo "   Terminal 1: python3 sync_real_robot.py --port /dev/ttyACM0"
echo "   Terminal 2: ros2 run robot_state_publisher robot_state_publisher"
echo "   Terminal 3: rviz2"
echo ""
echo "============================================================"