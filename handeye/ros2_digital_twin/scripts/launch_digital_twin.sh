#!/bin/bash
# Launch SO-101 digital twin with proper workspace sourcing

echo "============================================================"
echo "🤖 LAUNCHING SO-101 DIGITAL TWIN"
echo "============================================================"

# Parse arguments
ROBOT_PORT=${1:-/dev/ttyACM0}
SIMULATE_MODE=""
if [ "$1" == "--simulate" ]; then
    SIMULATE_MODE="--simulate"
    echo "Running in SIMULATION mode"
else
    echo "Using robot port: $ROBOT_PORT"
fi

# Find workspace directory
WORKSPACE_DIR="/home/pathonai/Documents/Github/opensource_dev/ROS2/lerobot_ws"
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo "Error: Workspace not found at $WORKSPACE_DIR"
    exit 1
fi

# Source ROS2 and workspace
echo "Sourcing ROS2 workspace..."
source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash 2>/dev/null
source $WORKSPACE_DIR/install/setup.bash 2>/dev/null

# Check if lerobot_description package exists
if ! ros2 pkg list | grep -q lerobot_description; then
    echo "Warning: lerobot_description package not found"
    echo "Building workspace..."
    cd $WORKSPACE_DIR
    colcon build --packages-select lerobot_description
    source install/setup.bash
fi

# Find URDF file
URDF_FILE="$WORKSPACE_DIR/src/lerobot_description/urdf/so101.urdf.xacro"
if [ ! -f "$URDF_FILE" ]; then
    URDF_FILE="$WORKSPACE_DIR/install/lerobot_description/share/lerobot_description/urdf/so101.urdf.xacro"
fi

if [ ! -f "$URDF_FILE" ]; then
    echo "Error: URDF file not found"
    echo "Looked in:"
    echo "  - $WORKSPACE_DIR/src/lerobot_description/urdf/"
    echo "  - $WORKSPACE_DIR/install/lerobot_description/share/lerobot_description/urdf/"
    exit 1
fi

echo "Found URDF: $URDF_FILE"

# Process xacro to URDF
echo "Processing xacro file..."
TEMP_URDF="/tmp/so101_processed.urdf"
xacro $URDF_FILE -o $TEMP_URDF

if [ ! -f "$TEMP_URDF" ]; then
    echo "Error: Failed to process xacro file"
    exit 1
fi

# Start the digital twin sync script
echo ""
echo "Starting digital twin synchronization..."
echo "============================================================"

cd $(dirname $0)

# Launch in background with proper URDF
python3 - << EOF &
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, String
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np
import sys
import os
import time

sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'GraspingDemo'))

try:
    from so101_grasp.robot.so101_client import SO101Client
except ImportError:
    print("Warning: SO101 client not found. Running in simulation mode.")
    SO101Client = None

class SO101DigitalTwin(Node):
    def __init__(self, robot_port='$ROBOT_PORT', simulate=$([[ -n "$SIMULATE_MODE" ]] && echo "True" || echo "False")):
        super().__init__('so101_digital_twin')
        
        self.robot_port = robot_port
        self.simulate = simulate
        self.publish_rate = 30.0
        
        # Joint names
        self.joint_names = [
            'waist', 'shoulder', 'elbow', 
            'wrist_angle', 'wrist_rotate', 'gripper'
        ]
        
        # Publishers
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Use transient local QoS for robot_description
        from rclpy.qos import QoSProfile, QoSDurabilityPolicy
        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.robot_description_pub = self.create_publisher(String, '/robot_description', qos_profile)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Robot client
        self.robot_client = None
        if not self.simulate:
            self.connect_to_robot()
        
        # Publish URDF
        self.publish_urdf()
        
        # Timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_state)
        self.sim_time = 0.0
        
        self.get_logger().info('Digital twin initialized')
        
    def connect_to_robot(self):
        if SO101Client is None:
            self.simulate = True
            return
        try:
            self.robot_client = SO101Client(
                port=self.robot_port,
                follower=True,
                force_calibration=False
            )
            self.get_logger().info('Connected to real robot!')
        except Exception as e:
            self.get_logger().warn(f'Could not connect: {e}')
            self.simulate = True
            
    def publish_urdf(self):
        with open('$TEMP_URDF', 'r') as f:
            urdf_content = f.read()
        msg = String()
        msg.data = urdf_content
        self.robot_description_pub.publish(msg)
        self.get_logger().info('Published robot description')
        
    def read_joints(self):
        if self.simulate or self.robot_client is None:
            self.sim_time += 1.0 / self.publish_rate
            return [
                0.5 * np.sin(self.sim_time * 0.5),
                0.3 * np.sin(self.sim_time * 0.7),
                0.4 * np.sin(self.sim_time * 0.6),
                0.2 * np.sin(self.sim_time * 0.8),
                0.3 * np.sin(self.sim_time * 0.9),
                0.0
            ]
        try:
            joints = self.robot_client.read_joints()
            return list(joints[:6])
        except:
            return [0.0] * 6
            
    def publish_state(self):
        # Joint states
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        joint_state.position = self.read_joints()
        joint_state.velocity = [0.0] * 6
        joint_state.effort = [0.0] * 6
        self.joint_pub.publish(joint_state)
        
        # Base TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base'
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = SO101DigitalTwin()
    print("Digital twin running!")
    rclpy.spin(node)

if __name__ == '__main__':
    main()
EOF

SYNC_PID=$!

# Wait for robot_description to be published
echo "Waiting for robot_description to be published..."
sleep 3

# Check if robot_description is available
if ros2 topic list | grep -q "/robot_description"; then
    echo "Robot description available"
else
    echo "Warning: robot_description topic not found"
fi

# Launch robot_state_publisher
echo "Starting robot_state_publisher..."
ros2 run robot_state_publisher robot_state_publisher --ros-args -p use_sim_time:=false &
RSP_PID=$!

sleep 2

# Launch RViz2
echo "Starting RViz2..."
rviz2 &
RVIZ_PID=$!

echo ""
echo "============================================================"
echo "✅ DIGITAL TWIN LAUNCHED!"
echo "============================================================"
echo ""
echo "In RViz2:"
echo "  1. Set Fixed Frame to 'world' or 'base'"
echo "  2. Add Display → RobotModel"
echo "  3. Set Robot Description Topic to '/robot_description'"
echo ""
if [[ -n "$SIMULATE_MODE" ]]; then
    echo "🎮 Running in SIMULATION mode"
else
    echo "📡 Syncing with real robot on $ROBOT_PORT"
fi
echo ""
echo "Press Ctrl+C to stop all components"
echo "============================================================"

# Cleanup function
cleanup() {
    echo "Stopping all components..."
    kill $SYNC_PID 2>/dev/null
    kill $RSP_PID 2>/dev/null
    kill $RVIZ_PID 2>/dev/null
    rm -f $TEMP_URDF
    exit 0
}

trap cleanup EXIT INT TERM

# Keep script running
wait