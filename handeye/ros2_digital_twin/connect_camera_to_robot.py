#!/usr/bin/env python3
"""
Connect RealSense camera to SO-101 robot using hand-eye calibration
This creates the missing link between robot and camera TF trees
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import numpy as np
import os
from scipy.spatial.transform import Rotation


class ConnectCameraToRobot(Node):
    def __init__(self):
        super().__init__('connect_camera_to_robot')
        
        # Based on your TF tree, we need to connect:
        # gripper (or link_gripper) -> camera_link
        
        # Try to find the correct gripper frame name
        self.declare_parameter('gripper_frame', 'gripper')  # Could be 'gripper' or 'link_gripper'
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('calibration_file', 'output/handeye_realsense.npz')
        
        self.gripper_frame = self.get_parameter('gripper_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        calibration_file = self.get_parameter('calibration_file').value
        
        # Load hand-eye calibration
        self.transform = self.load_calibration(calibration_file)
        
        # Static transform broadcaster
        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        # Publish the transform
        self.publish_transform()
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('🔗 CONNECTING CAMERA TO ROBOT')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Parent frame (robot): {self.gripper_frame}')
        self.get_logger().info(f'Child frame (camera): {self.camera_frame}')
        self.get_logger().info(f'Translation: {self.transform["translation"]}')
        self.get_logger().info('=' * 60)
        
    def load_calibration(self, filename):
        """Load hand-eye calibration from file."""
        # Try multiple paths
        possible_paths = [
            filename,
            os.path.join(os.path.dirname(__file__), '..', filename),
            os.path.join(os.path.dirname(__file__), '..', 'output', 'handeye_realsense.npz'),
            '/home/pathonai/Documents/Github/opensource_dev/handeye/output/handeye_realsense.npz'
        ]
        
        for filepath in possible_paths:
            if os.path.exists(filepath):
                self.get_logger().info(f'✅ Loading calibration from: {filepath}')
                data = np.load(filepath)
                T = data['T_cam2gripper']
                
                # Extract translation (convert mm to m if needed)
                translation = T[:3, 3]
                if np.max(np.abs(translation)) > 10:  # Likely in mm
                    translation = translation / 1000.0
                    self.get_logger().info(f'   Converted translation from mm to m')
                
                # Extract rotation as quaternion
                r = Rotation.from_matrix(T[:3, :3])
                quaternion = r.as_quat()  # [x, y, z, w]
                
                return {
                    'translation': translation.tolist(),
                    'quaternion': quaternion.tolist()
                }
        
        # Default transform if calibration not found
        self.get_logger().warn('⚠️  Calibration file not found, using default transform')
        self.get_logger().warn('   Camera will be placed 10cm above gripper')
        return {
            'translation': [0.0, 0.0, 0.1],  # 10cm above gripper
            'quaternion': [0.0, 0.0, 0.0, 1.0]  # No rotation
        }
    
    def publish_transform(self):
        """Publish the transform connecting robot to camera."""
        t = TransformStamped()
        
        # Header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.gripper_frame
        t.child_frame_id = self.camera_frame
        
        # Translation
        t.transform.translation.x = self.transform['translation'][0]
        t.transform.translation.y = self.transform['translation'][1]
        t.transform.translation.z = self.transform['translation'][2]
        
        # Rotation
        t.transform.rotation.x = self.transform['quaternion'][0]
        t.transform.rotation.y = self.transform['quaternion'][1]
        t.transform.rotation.z = self.transform['quaternion'][2]
        t.transform.rotation.w = self.transform['quaternion'][3]
        
        # Send transform
        self.static_broadcaster.sendTransform(t)
        
        self.get_logger().info(f'✅ Transform published: {self.gripper_frame} → {self.camera_frame}')


def main(args=None):
    print("\n" + "=" * 60)
    print("🔗 CONNECTING CAMERA TO ROBOT")
    print("=" * 60)
    print("\nThis will connect the RealSense camera TF tree")
    print("to the SO-101 robot TF tree using hand-eye calibration.\n")
    
    rclpy.init(args=args)
    
    # Create node
    node = ConnectCameraToRobot()
    
    print("\nThe following transform has been published:")
    print(f"  {node.gripper_frame} → {node.camera_frame}")
    print("\nYou should now see the camera attached to the robot in RViz2!")
    print("\nVerify with:")
    print("  ros2 run tf2_ros tf2_echo base camera_link")
    print("  ros2 run tf2_tools view_frames")
    print("\nPress Ctrl+C to stop")
    print("=" * 60)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()