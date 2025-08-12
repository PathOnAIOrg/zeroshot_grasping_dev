#!/usr/bin/env python3
"""
Check robot setup and available frames in TF
"""

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import time


class RobotSetupChecker(Node):
    def __init__(self):
        super().__init__('robot_setup_checker')
        
        self.frames = set()
        self.has_robot_description = False
        self.has_joint_states = False
        self.joint_names = []
        
        # Subscribe to TF
        self.tf_sub = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10
        )
        
        self.tf_static_sub = self.create_subscription(
            TFMessage,
            '/tf_static',
            self.tf_callback,
            10
        )
        
        # Subscribe to robot description
        self.robot_desc_sub = self.create_subscription(
            String,
            '/robot_description',
            self.robot_description_callback,
            10
        )
        
        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Timer to report status
        self.timer = self.create_timer(3.0, self.report_status)
        
        self.get_logger().info('🔍 Checking robot setup...')
        
    def tf_callback(self, msg):
        """Collect all TF frames."""
        for transform in msg.transforms:
            self.frames.add(transform.header.frame_id)
            self.frames.add(transform.child_frame_id)
    
    def robot_description_callback(self, msg):
        """Check if robot description is available."""
        if not self.has_robot_description:
            self.has_robot_description = True
            # Check for base_link in URDF
            if 'base_link' in msg.data:
                self.get_logger().info('✅ Robot description contains base_link')
            else:
                self.get_logger().warn('⚠️  Robot description might not have base_link')
                
            # Check for SO-101 specific content
            if 'so101' in msg.data.lower() or 'shoulder' in msg.data.lower():
                self.get_logger().info('✅ SO-101 robot description detected')
            elif 'lerobot' in msg.data.lower():
                self.get_logger().info('✅ LeRobot description detected')
    
    def joint_state_callback(self, msg):
        """Check joint states."""
        if not self.has_joint_states:
            self.has_joint_states = True
            self.joint_names = msg.name
            self.get_logger().info(f'✅ Joint states received: {msg.name}')
    
    def report_status(self):
        """Report current status."""
        print("\n" + "=" * 60)
        print("🤖 ROBOT SETUP STATUS")
        print("=" * 60)
        
        # Check robot description
        if self.has_robot_description:
            print("✅ Robot description: PUBLISHED")
        else:
            print("❌ Robot description: NOT FOUND")
            print("   Fix: ros2 launch lerobot_description so101_display.launch.py")
        
        # Check joint states
        if self.has_joint_states:
            print(f"✅ Joint states: PUBLISHED ({len(self.joint_names)} joints)")
        else:
            print("❌ Joint states: NOT FOUND")
            print("   Fix: ros2 run joint_state_publisher_gui joint_state_publisher_gui")
        
        # Check TF frames
        if self.frames:
            print(f"\n📍 Available TF frames ({len(self.frames)}):")
            
            # Look for important frames
            important_frames = ['base_link', 'world', 'base_footprint', 'base', 
                              'shoulder_pan_link', 'camera_link', 'odom']
            
            found_frames = []
            for frame in important_frames:
                if frame in self.frames:
                    found_frames.append(frame)
                    print(f"   ✅ {frame}")
            
            # List all frames
            print("\n   All frames:")
            for frame in sorted(self.frames):
                if frame not in important_frames:
                    print(f"   - {frame}")
            
            # Suggest fixed frame
            if 'base_link' in self.frames:
                print("\n💡 Use Fixed Frame: base_link")
            elif 'world' in self.frames:
                print("\n💡 Use Fixed Frame: world")
            elif 'base_footprint' in self.frames:
                print("\n💡 Use Fixed Frame: base_footprint")
            elif found_frames:
                print(f"\n💡 Try Fixed Frame: {found_frames[0]}")
            else:
                print("\n⚠️  No standard base frame found!")
                if self.frames:
                    print(f"   Try using: {sorted(self.frames)[0]}")
        else:
            print("\n❌ No TF frames found!")
            print("   This means the robot model is not being published")
        
        print("\n" + "=" * 60)
        print("📋 NEXT STEPS:")
        print("=" * 60)
        
        if not self.has_robot_description:
            print("\n1. Launch robot description:")
            print("   cd ~/Documents/Github/opensource_dev/ROS2/lerobot_ws")
            print("   source install/setup.bash")
            print("   ros2 launch lerobot_description so101_display.launch.py")
        
        if not self.has_joint_states:
            print("\n2. Launch joint state publisher:")
            print("   ros2 run joint_state_publisher_gui joint_state_publisher_gui")
        
        if self.has_robot_description and self.has_joint_states and not self.frames:
            print("\n3. Launch robot state publisher:")
            print("   ros2 run robot_state_publisher robot_state_publisher")
        
        print("=" * 60)


def main(args=None):
    rclpy.init(args=args)
    node = RobotSetupChecker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()