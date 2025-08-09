#!/usr/bin/env python3
"""
Connect STATIONARY camera to robot TF tree using eye-to-hand calibration.

This version is for when the camera is fixed in the workspace.
It publishes the transform from camera to robot base.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import numpy as np
from scipy.spatial.transform import Rotation
import os


class ConnectStationaryCamera(Node):
    def __init__(self):
        super().__init__('connect_stationary_camera')
        
        # Parameters
        self.declare_parameter('calibration_file', 
                             '/home/pathonai/Documents/Github/opensource_dev/handeye/output/handeye_realsense_stationary.npz')
        self.declare_parameter('base_frame', 'base')  # Robot base frame
        self.declare_parameter('camera_frame', 'camera_link')
        
        calibration_file = self.get_parameter('calibration_file').value
        self.base_frame = self.get_parameter('base_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        
        # Load calibration
        if not os.path.exists(calibration_file):
            # Try alternate path
            alt_path = calibration_file.replace('stationary', '')
            if os.path.exists(alt_path):
                self.get_logger().warn(f'Using alternate calibration: {alt_path}')
                calibration_file = alt_path
            else:
                self.get_logger().error(f'Calibration file not found: {calibration_file}')
                self.get_logger().warn('Using default transformation (camera 1m from base)')
                
                # Default: camera looking at robot from 1 meter away
                T_cam2base = np.eye(4)
                T_cam2base[0, 3] = 1.0  # 1m in X
                T_cam2base[2, 3] = 0.5  # 0.5m in Z (height)
                
                # Rotate to look at robot
                R = Rotation.from_euler('y', -90, degrees=True).as_matrix()
                T_cam2base[:3, :3] = R
        else:
            data = np.load(calibration_file)
            
            # Check if this is stationary camera calibration
            if 'T_cam2base' in data:
                T_cam2base = data['T_cam2base']
                self.get_logger().info('✅ Loaded stationary camera calibration')
            elif 'T_cam2gripper' in data:
                # This is eye-in-hand calibration, warn user
                self.get_logger().warn('⚠️  This is eye-in-hand calibration, not stationary!')
                self.get_logger().warn('   Using as camera-to-base anyway (may be incorrect)')
                T_cam2base = data['T_cam2gripper']
            else:
                raise ValueError('Invalid calibration file format')
        
        # Create static transform broadcaster
        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        # Publish transform from base to camera
        # (We have camera to base, so we need to invert it)
        T_base2cam = np.linalg.inv(T_cam2base)
        
        # Create transform message
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.camera_frame
        
        # Set translation (base to camera)
        t.transform.translation.x = T_base2cam[0, 3]
        t.transform.translation.y = T_base2cam[1, 3]
        t.transform.translation.z = T_base2cam[2, 3]
        
        # Set rotation (base to camera)
        r = Rotation.from_matrix(T_base2cam[:3, :3])
        q = r.as_quat()  # [x, y, z, w]
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        # Publish transform
        self.static_broadcaster.sendTransform(t)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('📷 STATIONARY CAMERA CONNECTED TO ROBOT')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Published transform: {self.base_frame} → {self.camera_frame}')
        self.get_logger().info(f'Translation: [{T_base2cam[0,3]:.3f}, {T_base2cam[1,3]:.3f}, {T_base2cam[2,3]:.3f}] m')
        
        # Also show camera position in world/base frame
        self.get_logger().info('Camera position in base frame:')
        cam_pos = T_cam2base[:3, 3]
        self.get_logger().info(f'  X: {cam_pos[0]:.3f} m')
        self.get_logger().info(f'  Y: {cam_pos[1]:.3f} m')
        self.get_logger().info(f'  Z: {cam_pos[2]:.3f} m')
        
        # Publish optical frame transform
        self.publish_optical_frames()
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('✅ Stationary camera TF tree connected!')
        self.get_logger().info('The camera can now see objects in robot base frame')
        
    def publish_optical_frames(self):
        """Publish standard camera optical frame transforms."""
        transforms = []
        
        # Camera link to color frame
        t1 = TransformStamped()
        t1.header.stamp = self.get_clock().now().to_msg()
        t1.header.frame_id = self.camera_frame
        t1.child_frame_id = 'camera_color_frame'
        t1.transform.rotation.w = 1.0
        transforms.append(t1)
        
        # Color frame to optical frame (rotate to ROS convention)
        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'camera_color_frame'
        t2.child_frame_id = 'camera_color_optical_frame'
        # Rotation: z forward, x right, y down
        t2.transform.rotation.x = -0.5
        t2.transform.rotation.y = 0.5
        t2.transform.rotation.z = -0.5
        t2.transform.rotation.w = 0.5
        transforms.append(t2)
        
        # Depth frame
        t3 = TransformStamped()
        t3.header.stamp = self.get_clock().now().to_msg()
        t3.header.frame_id = self.camera_frame
        t3.child_frame_id = 'camera_depth_frame'
        t3.transform.rotation.w = 1.0
        transforms.append(t3)
        
        # Depth optical frame
        t4 = TransformStamped()
        t4.header.stamp = self.get_clock().now().to_msg()
        t4.header.frame_id = 'camera_depth_frame'
        t4.child_frame_id = 'camera_depth_optical_frame'
        t4.transform.rotation.x = -0.5
        t4.transform.rotation.y = 0.5
        t4.transform.rotation.z = -0.5
        t4.transform.rotation.w = 0.5
        transforms.append(t4)
        
        self.static_broadcaster.sendTransform(transforms)


def main(args=None):
    print("\n" + "=" * 60)
    print("🔗 CONNECTING STATIONARY CAMERA TO ROBOT TF TREE")
    print("=" * 60)
    
    rclpy.init(args=args)
    
    node = ConnectStationaryCamera()
    
    print("\n📋 TF Tree Structure (Stationary Camera):")
    print("=" * 60)
    print("""
world/map (optional)
└── base (or base_link) ← Robot base frame
    ├── link1
    │   └── link2
    │       └── ...
    │           └── gripper
    └── camera_link ← STATIONARY camera (from calibration)
        ├── camera_color_frame
        │   └── camera_color_optical_frame
        └── camera_depth_frame
            └── camera_depth_optical_frame
""")
    print("=" * 60)
    print("\n✅ The stationary camera can now:")
    print("   1. See point clouds in robot base frame")
    print("   2. Transform detected objects to robot coordinates")
    print("   3. Plan grasps in robot workspace")
    
    print("\n🔍 To verify connection:")
    print("   ros2 run tf2_ros tf2_echo base camera_link")
    
    print("\nPress Ctrl+C to exit (transform will persist)")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()