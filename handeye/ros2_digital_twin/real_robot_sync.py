#!/usr/bin/env python3
"""
Real-time synchronization of SO-101 robot with RViz2 digital twin.

This script reads the real robot joint positions and publishes them
to ROS2 so the digital twin in RViz2 matches the physical robot.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np
import sys
import os
import time

# Add parent directory to path for robot imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'GraspingDemo'))

try:
    from so101_grasp.robot.so101_client import SO101Client
except ImportError:
    print("Warning: SO101 client not found. Running in simulation mode.")
    SO101Client = None


class RealRobotSync(Node):
    def __init__(self, robot_port='/dev/ttyACM0', simulate=False):
        super().__init__('real_robot_sync')
        
        # Parameters
        self.declare_parameter('robot_port', robot_port)
        self.declare_parameter('publish_rate', 30.0)  # Hz
        self.declare_parameter('simulate', simulate)
        
        self.robot_port = self.get_parameter('robot_port').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.simulate = self.get_parameter('simulate').value
        
        # Joint names for SO-101 (must match URDF)
        self.joint_names = [
            'joint1',  # Base rotation
            'joint2',  # Shoulder
            'joint3',  # Elbow
            'joint4',  # Wrist pitch
            'joint5',  # Wrist roll
            'joint6'   # Gripper
        ]
        
        # Alternative joint names (if URDF uses different names)
        self.alt_joint_names = [
            'base_joint',
            'shoulder_joint', 
            'elbow_joint',
            'wrist_pitch_joint',
            'wrist_roll_joint',
            'gripper_joint'
        ]
        
        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Connect to robot
        self.robot_client = None
        if not self.simulate:
            self.connect_to_robot()
        
        # Timer for publishing joint states
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_joint_states)
        
        # Simulation values
        self.sim_time = 0.0
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('🤖 REAL ROBOT SYNCHRONIZATION STARTED')
        self.get_logger().info('=' * 60)
        if self.simulate:
            self.get_logger().warn('Running in SIMULATION mode (no real robot)')
        else:
            self.get_logger().info(f'Connected to robot on {self.robot_port}')
        self.get_logger().info(f'Publishing joint states at {self.publish_rate} Hz')
        self.get_logger().info('=' * 60)
        
    def connect_to_robot(self):
        """Connect to the real SO-101 robot."""
        if SO101Client is None:
            self.get_logger().warn('SO101Client not available, running in simulation mode')
            self.simulate = True
            return
            
        try:
            self.get_logger().info(f'Connecting to robot on {self.robot_port}...')
            
            # Try to connect
            self.robot_client = SO101Client(
                port=self.robot_port,
                follower=True,
                force_calibration=False
            )
            
            self.get_logger().info('✅ Connected to real robot!')
            
            # Try to disable torque for manual movement
            try:
                if hasattr(self.robot_client, 'robot') and hasattr(self.robot_client.robot, 'bus'):
                    # Don't disable torque - we want to read positions
                    # self.robot_client.robot.bus.disable_torque()
                    self.get_logger().info('Robot ready for position reading')
            except Exception as e:
                self.get_logger().warn(f'Note: {e}')
                
        except Exception as e:
            self.get_logger().error(f'Failed to connect to robot: {e}')
            self.get_logger().warn('Falling back to simulation mode')
            self.simulate = True
            self.robot_client = None
    
    def read_robot_joints(self):
        """Read current joint positions from the real robot."""
        if self.simulate or self.robot_client is None:
            # Simulation: generate smooth motion
            self.sim_time += 1.0 / self.publish_rate
            joints = [
                0.5 * np.sin(self.sim_time * 0.5),  # Base rotation
                0.3 * np.sin(self.sim_time * 0.7),  # Shoulder
                0.4 * np.sin(self.sim_time * 0.6),  # Elbow
                0.2 * np.sin(self.sim_time * 0.8),  # Wrist pitch
                0.3 * np.sin(self.sim_time * 0.9),  # Wrist roll
                0.0  # Gripper (closed)
            ]
            return joints
        
        try:
            # Read real robot joints
            joints = self.robot_client.read_joints()
            
            # SO-101 returns 6 values: [x, y, z, rx, ry, gripper]
            # We need to convert to joint angles for the URDF
            # This is a simplified mapping - adjust based on your robot kinematics
            
            if len(joints) == 6:
                # Map SO-101 values to joint angles
                joint_angles = [
                    joints[0],  # Base rotation (x)
                    joints[1],  # Shoulder (y) 
                    joints[2],  # Elbow (z)
                    joints[3],  # Wrist pitch (rx)
                    joints[4],  # Wrist roll (ry)
                    joints[5]   # Gripper
                ]
                return joint_angles
            else:
                self.get_logger().warn(f'Unexpected joint count: {len(joints)}')
                return [0.0] * 6
                
        except Exception as e:
            self.get_logger().error(f'Failed to read joints: {e}')
            return [0.0] * 6
    
    def publish_joint_states(self):
        """Publish joint states to ROS2."""
        try:
            # Read current joint positions
            joint_positions = self.read_robot_joints()
            
            # Create JointState message
            joint_state = JointState()
            joint_state.header = Header()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            
            # Try primary joint names first
            joint_state.name = self.joint_names[:len(joint_positions)]
            joint_state.position = joint_positions
            
            # Add velocity and effort (optional, set to 0)
            joint_state.velocity = [0.0] * len(joint_positions)
            joint_state.effort = [0.0] * len(joint_positions)
            
            # Publish
            self.joint_pub.publish(joint_state)
            
            # Log occasionally
            if int(time.time()) % 10 == 0:  # Every 10 seconds
                self.get_logger().debug(f'Published joints: {[f"{j:.3f}" for j in joint_positions]}')
                
        except Exception as e:
            self.get_logger().error(f'Failed to publish joint states: {e}')
    
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
    print("🔄 SO-101 REAL-TIME DIGITAL TWIN SYNCHRONIZATION")
    print("=" * 60)
    
    import argparse
    parser = argparse.ArgumentParser(description='Real robot sync for digital twin')
    parser.add_argument('--port', type=str, default='/dev/ttyACM0',
                       help='Robot serial port')
    parser.add_argument('--simulate', action='store_true',
                       help='Run in simulation mode without real robot')
    parser.add_argument('--rate', type=float, default=30.0,
                       help='Publishing rate in Hz')
    
    # Parse known args (ROS2 may have additional args)
    parsed_args, unknown = parser.parse_known_args()
    
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create node with parameters
    node = RealRobotSync(
        robot_port=parsed_args.port,
        simulate=parsed_args.simulate
    )
    
    # Override rate if specified
    if parsed_args.rate != 30.0:
        node.publish_rate = parsed_args.rate
        node.destroy_timer(node.timer)
        node.timer = node.create_timer(1.0 / node.publish_rate, node.publish_joint_states)
    
    print("\n📋 INSTRUCTIONS:")
    print("=" * 60)
    print("1. Start this script to sync real robot")
    print("2. Launch robot visualization:")
    print("   ros2 launch lerobot_description so101_display.launch.py")
    print("3. Open RViz2:")
    print("   rviz2")
    print("4. Add RobotModel display")
    print("5. The digital twin will follow your real robot!")
    print("=" * 60)
    
    if parsed_args.simulate:
        print("\n⚠️  SIMULATION MODE - No real robot connected")
        print("   The digital twin will show simulated motion")
    else:
        print(f"\n✅ Syncing with real robot on {parsed_args.port}")
    
    print("\nPress Ctrl+C to stop")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n⏹️  Stopping synchronization...")
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        print("✅ Synchronization stopped")


if __name__ == '__main__':
    main()