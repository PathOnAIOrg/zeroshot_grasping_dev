#!/usr/bin/env python3
"""
Complete launch script for SO-101 digital twin with proper timing.
This handles robot_description publishing before robot_state_publisher starts.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, String
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np
import sys
import os
import subprocess
import threading
import time
import argparse

# Add parent directory to path for robot imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'GraspingDemo'))

try:
    from so101_grasp.robot.so101_client import SO101Client
except ImportError:
    print("Warning: SO101 client not found. Running in simulation mode.")
    SO101Client = None


class SO101CompleteTwin(Node):
    def __init__(self, robot_port='/dev/ttyACM0', simulate=False):
        super().__init__('so101_complete_twin')
        
        self.robot_port = robot_port
        self.simulate = simulate
        self.publish_rate = 30.0
        
        # Joint names matching URDF
        self.joint_names = [
            'waist', 'shoulder', 'elbow', 
            'wrist_angle', 'wrist_rotate', 'gripper'
        ]
        
        # Joint limits
        self.joint_limits = [
            (-3.14, 3.14),   # waist
            (-1.57, 1.57),   # shoulder
            (-1.57, 1.57),   # elbow
            (-1.57, 1.57),   # wrist_angle
            (-3.14, 3.14),   # wrist_rotate
            (0.0, 1.0)       # gripper
        ]
        
        # Publishers
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Use transient local QoS for robot_description (required for robot_state_publisher)
        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.robot_description_pub = self.create_publisher(String, '/robot_description', qos_profile)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Load and publish URDF first
        self.load_and_publish_urdf()
        
        # Connect to robot
        self.robot_client = None
        if not self.simulate:
            self.connect_to_robot()
        
        # Start publishing joint states
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_state)
        self.sim_time = 0.0
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('🤖 SO-101 COMPLETE DIGITAL TWIN')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Mode: {"SIMULATION" if self.simulate else "REAL ROBOT"}')
        if not self.simulate:
            self.get_logger().info(f'Port: {self.robot_port}')
        self.get_logger().info(f'Rate: {self.publish_rate} Hz')
        self.get_logger().info('=' * 60)
        
        # Flag to indicate URDF is published
        self.urdf_published = False
        
    def load_and_publish_urdf(self):
        """Load and publish URDF with proper error handling."""
        # Find URDF file
        possible_paths = [
            '/home/pathonai/Documents/Github/opensource_dev/ROS2/lerobot_ws/src/lerobot_description/urdf/so101.urdf.xacro',
            '/home/pathonai/Documents/Github/opensource_dev/ROS2/lerobot_ws/install/lerobot_description/share/lerobot_description/urdf/so101.urdf.xacro',
        ]
        
        urdf_file = None
        for path in possible_paths:
            if os.path.exists(path):
                urdf_file = path
                break
        
        if not urdf_file:
            self.get_logger().error('URDF file not found!')
            self.get_logger().info('Looked in:')
            for path in possible_paths:
                self.get_logger().info(f'  - {path}')
            return False
        
        self.get_logger().info(f'Found URDF: {os.path.basename(urdf_file)}')
        
        try:
            # Process xacro
            # First source the workspace
            workspace_setup = '/home/pathonai/Documents/Github/opensource_dev/ROS2/lerobot_ws/install/setup.bash'
            if os.path.exists(workspace_setup):
                source_cmd = f'source {workspace_setup} && '
            else:
                source_cmd = ''
            
            result = subprocess.run(
                f'{source_cmd}xacro {urdf_file}',
                shell=True,
                capture_output=True,
                text=True,
                executable='/bin/bash'
            )
            
            if result.returncode != 0:
                self.get_logger().error(f'Failed to process xacro: {result.stderr}')
                # Try without sourcing
                result = subprocess.run(
                    ['xacro', urdf_file],
                    capture_output=True,
                    text=True
                )
            
            if result.returncode == 0:
                urdf_content = result.stdout
                
                # Publish robot description
                msg = String()
                msg.data = urdf_content
                self.robot_description_pub.publish(msg)
                
                self.get_logger().info('✅ Published robot description successfully!')
                self.urdf_published = True
                
                # Keep publishing for late subscribers
                def republish():
                    for _ in range(10):
                        time.sleep(0.5)
                        self.robot_description_pub.publish(msg)
                
                threading.Thread(target=republish, daemon=True).start()
                return True
            else:
                self.get_logger().error('Failed to process URDF')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Error loading URDF: {e}')
            return False
    
    def connect_to_robot(self):
        """Connect to real robot."""
        if SO101Client is None:
            self.get_logger().warn('SO101Client not available, using simulation')
            self.simulate = True
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
            self.get_logger().warn(f'Could not connect to robot: {e}')
            self.get_logger().info('Falling back to simulation mode')
            self.simulate = True
            self.robot_client = None
    
    def read_joints(self):
        """Read joint positions from robot or generate simulation data."""
        if self.simulate or self.robot_client is None:
            # Simulation: smooth sinusoidal motion
            self.sim_time += 1.0 / self.publish_rate
            joints = []
            for i, (min_val, max_val) in enumerate(self.joint_limits):
                if i == 5:  # Gripper
                    joints.append(0.0)
                else:
                    # Generate smooth motion within limits
                    range_val = (max_val - min_val) * 0.3
                    center = (max_val + min_val) / 2
                    value = center + range_val * np.sin(self.sim_time * (0.5 + i * 0.1))
                    joints.append(value)
            return joints
        
        try:
            # Read real robot joints
            raw_joints = self.robot_client.read_joints()
            if len(raw_joints) >= 6:
                joints = list(raw_joints[:6])
                # Clamp to limits
                for i in range(len(joints)):
                    min_val, max_val = self.joint_limits[i]
                    joints[i] = np.clip(joints[i], min_val, max_val)
                return joints
            else:
                return [0.0] * 6
        except Exception as e:
            self.get_logger().error(f'Failed to read joints: {e}')
            return [0.0] * 6
    
    def publish_state(self):
        """Publish joint states and TF."""
        try:
            # Read joint positions
            joint_positions = self.read_joints()
            
            # Create JointState message
            joint_state = JointState()
            joint_state.header = Header()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = self.joint_names
            joint_state.position = joint_positions
            joint_state.velocity = [0.0] * len(joint_positions)
            joint_state.effort = [0.0] * len(joint_positions)
            
            # Publish joint states
            self.joint_pub.publish(joint_state)
            
            # Publish world to base transform
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


def launch_robot_state_publisher():
    """Launch robot_state_publisher in subprocess."""
    time.sleep(2)  # Wait for robot_description
    print("\n📦 Starting robot_state_publisher...")
    
    # Source workspace and run
    cmd = """
    source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash 2>/dev/null
    source /home/pathonai/Documents/Github/opensource_dev/ROS2/lerobot_ws/install/setup.bash 2>/dev/null
    exec ros2 run robot_state_publisher robot_state_publisher
    """
    
    process = subprocess.Popen(
        cmd,
        shell=True,
        executable='/bin/bash',
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )
    return process


def launch_rviz():
    """Launch RViz2."""
    time.sleep(3)  # Wait for everything to be ready
    print("\n🎨 Starting RViz2...")
    
    config_file = os.path.join(
        os.path.dirname(__file__),
        'config',
        'so101_digital_twin.rviz'
    )
    
    if os.path.exists(config_file):
        cmd = f'rviz2 -d {config_file}'
    else:
        cmd = 'rviz2'
    
    process = subprocess.Popen(
        cmd,
        shell=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )
    return process


def main():
    print("\n" + "=" * 60)
    print("🚀 SO-101 COMPLETE DIGITAL TWIN LAUNCHER")
    print("=" * 60)
    
    parser = argparse.ArgumentParser(description='SO-101 Digital Twin')
    parser.add_argument('--port', default='/dev/ttyACM0', help='Robot serial port')
    parser.add_argument('--simulate', action='store_true', help='Run in simulation mode')
    parser.add_argument('--no-rviz', action='store_true', help='Don\'t launch RViz2')
    
    args, unknown = parser.parse_known_args()
    
    # Initialize ROS2
    rclpy.init(args=unknown)
    
    # Create and start main node
    node = SO101CompleteTwin(
        robot_port=args.port,
        simulate=args.simulate
    )
    
    # Start robot_state_publisher in background
    rsp_process = launch_robot_state_publisher()
    
    # Start RViz2 if requested
    rviz_process = None
    if not args.no_rviz:
        rviz_process = launch_rviz()
    
    print("\n" + "=" * 60)
    print("✅ DIGITAL TWIN RUNNING!")
    print("=" * 60)
    
    if args.simulate:
        print("🎮 Mode: SIMULATION")
    else:
        print(f"📡 Mode: REAL ROBOT on {args.port}")
    
    print("\n📋 RViz2 Setup:")
    print("  1. Fixed Frame: 'world' or 'base'")
    print("  2. Add Display → RobotModel")
    print("  3. Robot Description Topic: '/robot_description'")
    print("\nThe robot model should appear and move!")
    print("\nPress Ctrl+C to stop")
    print("=" * 60)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n⏹️  Shutting down...")
    finally:
        node.shutdown()
        node.destroy_node()
        
        # Terminate subprocesses
        if rsp_process:
            rsp_process.terminate()
        if rviz_process:
            rviz_process.terminate()
        
        rclpy.shutdown()
        print("✅ Shutdown complete")


if __name__ == '__main__':
    main()