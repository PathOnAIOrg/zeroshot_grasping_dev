#!/usr/bin/env python3
"""
Test script to publish robot_description and verify it works with robot_state_publisher.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import String
import subprocess
import os
import time

class TestRobotDescription(Node):
    def __init__(self):
        super().__init__('test_robot_description')
        
        # Create publisher with correct QoS for robot_state_publisher
        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(String, '/robot_description', qos_profile)
        
        # Load URDF
        self.load_and_publish_urdf()
        
    def load_and_publish_urdf(self):
        """Load URDF and publish it."""
        # Find URDF file
        urdf_paths = [
            '/home/pathonai/Documents/Github/opensource_dev/ROS2/lerobot_ws/src/lerobot_description/urdf/so101.urdf.xacro',
            '/home/pathonai/Documents/Github/opensource_dev/ROS2/lerobot_ws/install/lerobot_description/share/lerobot_description/urdf/so101.urdf.xacro',
        ]
        
        urdf_file = None
        for path in urdf_paths:
            if os.path.exists(path):
                urdf_file = path
                break
        
        if not urdf_file:
            self.get_logger().error('❌ URDF file not found!')
            return
        
        self.get_logger().info(f'✅ Found URDF: {os.path.basename(urdf_file)}')
        
        # Process xacro
        try:
            # Try with workspace sourced
            cmd = f"""
            source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash 2>/dev/null
            source /home/pathonai/Documents/Github/opensource_dev/ROS2/lerobot_ws/install/setup.bash 2>/dev/null
            xacro {urdf_file}
            """
            
            result = subprocess.run(
                cmd,
                shell=True,
                capture_output=True,
                text=True,
                executable='/bin/bash'
            )
            
            if result.returncode != 0:
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
                
                # Publish multiple times for late subscribers
                for i in range(5):
                    self.pub.publish(msg)
                    self.get_logger().info(f'📤 Published robot_description (attempt {i+1}/5)')
                    time.sleep(0.5)
                
                self.get_logger().info('✅ Robot description published successfully!')
                self.get_logger().info('\nNow you can run in another terminal:')
                self.get_logger().info('  ros2 run robot_state_publisher robot_state_publisher')
                
            else:
                self.get_logger().error(f'❌ Failed to process xacro: {result.stderr}')
                
        except Exception as e:
            self.get_logger().error(f'❌ Error: {e}')

def main():
    print("=" * 60)
    print("🧪 TESTING ROBOT DESCRIPTION PUBLISHING")
    print("=" * 60)
    
    rclpy.init()
    node = TestRobotDescription()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n✅ Test stopped")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()