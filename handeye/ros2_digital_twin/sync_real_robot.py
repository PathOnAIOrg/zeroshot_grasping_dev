#!/usr/bin/env python3
"""
Complete solution for syncing real SO-101 robot with RViz2 digital twin.
This combines URDF loading, joint state publishing, and TF broadcasting.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, String
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np
import sys
import os
import threading

# Add parent directory to path for robot imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'GraspingDemo'))

try:
    from so101_grasp.robot.so101_client import SO101Client
except ImportError:
    print("Warning: SO101 client not found. Running in simulation mode.")
    SO101Client = None


class SO101DigitalTwin(Node):
    def __init__(self, robot_port='/dev/ttyACM0', urdf_file=None):
        super().__init__('so101_digital_twin')
        
        # Parameters
        self.declare_parameter('robot_port', robot_port)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('use_real_robot', True)
        self.declare_parameter('urdf_file', '')
        
        self.robot_port = self.get_parameter('robot_port').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.use_real_robot = self.get_parameter('use_real_robot').value
        
        # URDF file path
        if urdf_file:
            self.urdf_file = urdf_file
        else:
            self.urdf_file = self.get_parameter('urdf_file').value
            if not self.urdf_file:
                # Try to find URDF in common locations
                possible_paths = [
                    '/home/pathonai/Documents/Github/opensource_dev/ROS2/lerobot_ws/src/lerobot_description/urdf/so101.urdf.xacro',
                    '/home/pathonai/Documents/Github/opensource_dev/ROS2/lerobot_ws/install/lerobot_description/share/lerobot_description/urdf/so101.urdf.xacro',
                    os.path.expanduser('~/lerobot_ws/src/lerobot_description/urdf/so101.urdf.xacro')
                ]
                for path in possible_paths:
                    if os.path.exists(path):
                        self.urdf_file = path
                        break
        
        # Joint configuration for SO-101
        self.setup_joint_configuration()
        
        # Publishers
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        # Use transient local QoS for robot_description to work with robot_state_publisher
        from rclpy.qos import QoSProfile, QoSDurabilityPolicy
        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.robot_description_pub = self.create_publisher(String, '/robot_description', qos_profile)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Robot client
        self.robot_client = None
        if self.use_real_robot:
            self.connect_to_robot()
        
        # Load and publish URDF
        self.load_and_publish_urdf()
        
        # Timer for publishing joint states
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_robot_state)
        
        # Simulation time
        self.sim_time = 0.0
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('🤖 SO-101 DIGITAL TWIN INITIALIZED')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Robot port: {self.robot_port}')
        self.get_logger().info(f'Real robot: {self.use_real_robot}')
        self.get_logger().info(f'Publish rate: {self.publish_rate} Hz')
        if self.urdf_file:
            self.get_logger().info(f'URDF: {os.path.basename(self.urdf_file)}')
        self.get_logger().info('=' * 60)
    
    def setup_joint_configuration(self):
        """Setup joint names and limits for SO-101."""
        # Joint names matching the URDF
        self.joint_names = [
            'waist',       # Base rotation
            'shoulder',    # Shoulder
            'elbow',       # Elbow  
            'wrist_angle', # Wrist pitch
            'wrist_rotate',# Wrist roll
            'gripper'      # Gripper
        ]
        
        # Alternative names if URDF is different
        self.alt_joint_names = [
            'joint1', 'joint2', 'joint3', 
            'joint4', 'joint5', 'joint6'
        ]
        
        # Joint limits (radians)
        self.joint_limits = [
            (-3.14, 3.14),   # waist
            (-1.57, 1.57),   # shoulder
            (-1.57, 1.57),   # elbow
            (-1.57, 1.57),   # wrist_angle
            (-3.14, 3.14),   # wrist_rotate
            (0.0, 1.0)       # gripper
        ]
    
    def connect_to_robot(self):
        """Connect to the real SO-101 robot."""
        if SO101Client is None:
            self.get_logger().warn('SO101Client not available, using simulation')
            self.use_real_robot = False
            return
            
        try:
            self.get_logger().info(f'Connecting to robot on {self.robot_port}...')
            
            self.robot_client = SO101Client(
                port=self.robot_port,
                follower=True,
                force_calibration=False
            )
            
            self.get_logger().info('✅ Connected to real robot!')
            
        except Exception as e:
            self.get_logger().error(f'Failed to connect: {e}')
            self.get_logger().warn('Using simulation mode')
            self.use_real_robot = False
            self.robot_client = None
    
    def load_and_publish_urdf(self):
        """Load URDF and publish to robot_description topic."""
        if not self.urdf_file:
            self.get_logger().warn('No URDF file specified')
            return
            
        try:
            # Check if it's a xacro file
            if self.urdf_file.endswith('.xacro'):
                # Process xacro to get URDF
                import subprocess
                result = subprocess.run(
                    ['xacro', self.urdf_file],
                    capture_output=True,
                    text=True
                )
                if result.returncode == 0:
                    urdf_content = result.stdout
                else:
                    self.get_logger().error(f'Failed to process xacro: {result.stderr}')
                    return
            else:
                # Read URDF directly
                with open(self.urdf_file, 'r') as f:
                    urdf_content = f.read()
            
            # Publish robot description
            msg = String()
            msg.data = urdf_content
            self.robot_description_pub.publish(msg)
            
            self.get_logger().info('✅ Published robot description')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load URDF: {e}')
    
    def read_robot_joints(self):
        """Read current joint positions from robot or simulation."""
        if not self.use_real_robot or self.robot_client is None:
            # Simulation: smooth motion
            self.sim_time += 1.0 / self.publish_rate
            joints = []
            for i, (min_val, max_val) in enumerate(self.joint_limits):
                if i == 5:  # Gripper stays closed
                    joints.append(0.0)
                else:
                    # Sinusoidal motion within limits
                    range_val = (max_val - min_val) * 0.3
                    center = (max_val + min_val) / 2
                    value = center + range_val * np.sin(self.sim_time * (0.5 + i * 0.1))
                    joints.append(value)
            return joints
        
        try:
            # Read real robot joints
            raw_joints = self.robot_client.read_joints()
            
            # Convert SO-101 format to joint angles
            # SO-101 returns: [x, y, z, rx, ry, gripper]
            # Need to map to actual joint angles
            
            if len(raw_joints) >= 6:
                # This mapping depends on your robot's kinematics
                # Adjust as needed for your SO-101
                joints = [
                    raw_joints[0],  # waist (base rotation)
                    raw_joints[1],  # shoulder
                    raw_joints[2],  # elbow
                    raw_joints[3],  # wrist_angle
                    raw_joints[4],  # wrist_rotate
                    raw_joints[5]   # gripper
                ]
                
                # Clamp to joint limits
                for i, (value, (min_val, max_val)) in enumerate(zip(joints, self.joint_limits)):
                    joints[i] = np.clip(value, min_val, max_val)
                
                return joints
            else:
                self.get_logger().warn(f'Unexpected joint count: {len(raw_joints)}')
                return [0.0] * 6
                
        except Exception as e:
            self.get_logger().error(f'Failed to read joints: {e}')
            return [0.0] * 6
    
    def publish_robot_state(self):
        """Publish joint states and TF transforms."""
        try:
            # Read joint positions
            joint_positions = self.read_robot_joints()
            
            # Create JointState message
            joint_state = JointState()
            joint_state.header = Header()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = self.joint_names[:len(joint_positions)]
            joint_state.position = list(joint_positions)
            joint_state.velocity = [0.0] * len(joint_positions)
            joint_state.effort = [0.0] * len(joint_positions)
            
            # Publish joint states
            self.joint_pub.publish(joint_state)
            
            # Publish base transform (robot in world)
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'world'
            t.child_frame_id = 'base'
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            
            self.tf_broadcaster.sendTransform(t)
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish state: {e}')
    
    def shutdown(self):
        """Clean shutdown."""
        if self.robot_client:
            try:
                self.robot_client.disconnect()
                self.get_logger().info('Disconnected from robot')
            except:
                pass


def main(args=None):
    print("\n" + "=" * 60)
    print("🤖 SO-101 COMPLETE DIGITAL TWIN")
    print("=" * 60)
    
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='/dev/ttyACM0', help='Robot port')
    parser.add_argument('--simulate', action='store_true', help='Simulation mode')
    parser.add_argument('--rate', type=float, default=30.0, help='Publish rate Hz')
    parser.add_argument('--urdf', help='Path to URDF/xacro file')
    
    parsed_args, unknown = parser.parse_known_args()
    
    rclpy.init(args=args)
    
    node = SO101DigitalTwin(
        robot_port=parsed_args.port,
        urdf_file=parsed_args.urdf
    )
    
    if parsed_args.simulate:
        node.use_real_robot = False
    
    print("\n✅ Digital twin running!")
    print("\nIn another terminal, run:")
    print("  ros2 run robot_state_publisher robot_state_publisher")
    print("  rviz2")
    print("\nIn RViz2:")
    print("  - Set Fixed Frame to 'base' or 'world'")
    print("  - Add RobotModel display")
    print("  - Robot Description Topic: /robot_description")
    
    if node.use_real_robot:
        print(f"\n📡 Syncing with real robot on {parsed_args.port}")
    else:
        print("\n🎮 Running in simulation mode")
    
    print("\nPress Ctrl+C to stop")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n⏹️  Stopping...")
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()