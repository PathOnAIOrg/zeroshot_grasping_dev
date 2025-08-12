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
        
        # Rotation corrections (for stationary camera looking at robot)
        self.declare_parameter('flip_x', False)  # Flip around X axis
        self.declare_parameter('flip_y', True)  # Flip around Y axis (fixes upside-down point cloud)
        self.declare_parameter('flip_z', False)  # Flip around Z axis
        self.declare_parameter('rotate_x_deg', 0.0)  # Additional rotation around X
        self.declare_parameter('rotate_y_deg', 90.0)  # Additional rotation around Y
        self.declare_parameter('rotate_z_deg', 270.0)  # Additional rotation around Z
        self.declare_parameter('use_raw_calibration', False)  # Use raw calibration without inversion
        
        # Gripper to checkerboard offset (if checkerboard is not at gripper center)
        self.declare_parameter('checkerboard_offset_x', 0.)  # Offset in meters
        self.declare_parameter('checkerboard_offset_y', 0.33)
        self.declare_parameter('checkerboard_offset_z', -0.47)
        
        calibration_file = self.get_parameter('calibration_file').value
        self.base_frame = self.get_parameter('base_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        
        # Load calibration
        self.T_cam2base = self.load_calibration(calibration_file)
        
        # Create static transform broadcaster
        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        # Publish transform from base to camera
        self.publish_transform()
        
        # Publish optical frame transforms
        self.publish_optical_frames()
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('📷 STATIONARY CAMERA CONNECTED TO ROBOT')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Published transform: {self.base_frame} → {self.camera_frame}')
        
        # Print usage tips for debugging
        self.get_logger().info('')
        self.get_logger().info('🔧 Debugging tips if position is wrong:')
        self.get_logger().info('1. Check camera position above - is it where you expect?')
        self.get_logger().info('2. Try different rotation corrections:')
        self.get_logger().info('   --ros-args -p flip_x:=true -p flip_y:=true')
        self.get_logger().info('3. If checkerboard was offset from gripper center, add offset:')
        self.get_logger().info('   --ros-args -p checkerboard_offset_z:=0.1  # 10cm offset')
        self.get_logger().info('4. Verify with: ros2 run tf2_ros tf2_echo base camera_link')
        T_base2cam = np.linalg.inv(self.T_cam2base)
        self.get_logger().info(f'Translation: [{T_base2cam[0,3]:.3f}, {T_base2cam[1,3]:.3f}, {T_base2cam[2,3]:.3f}] m')
        
        # Show detailed transformation info
        self.get_logger().info('')
        self.get_logger().info('📊 Transformation Details:')
        self.get_logger().info('Camera to Base (T_cam2base):')
        cam_pos = self.T_cam2base[:3, 3]
        self.get_logger().info(f'  Translation: [{cam_pos[0]:.3f}, {cam_pos[1]:.3f}, {cam_pos[2]:.3f}] m')
        
        # Extract Euler angles from rotation matrix
        r_cam2base = Rotation.from_matrix(self.T_cam2base[:3, :3])
        euler_cam2base = r_cam2base.as_euler('xyz', degrees=True)
        self.get_logger().info(f'  Rotation (Euler XYZ): [{euler_cam2base[0]:.1f}°, {euler_cam2base[1]:.1f}°, {euler_cam2base[2]:.1f}°]')
        
        self.get_logger().info('')
        self.get_logger().info('Base to Camera (T_base2cam - published):')
        base2cam_pos = T_base2cam[:3, 3]
        self.get_logger().info(f'  Translation: [{base2cam_pos[0]:.3f}, {base2cam_pos[1]:.3f}, {base2cam_pos[2]:.3f}] m')
        
        r_base2cam = Rotation.from_matrix(T_base2cam[:3, :3])
        euler_base2cam = r_base2cam.as_euler('xyz', degrees=True)
        self.get_logger().info(f'  Rotation (Euler XYZ): [{euler_base2cam[0]:.1f}°, {euler_base2cam[1]:.1f}°, {euler_base2cam[2]:.1f}°]')
        self.get_logger().info('=' * 60)
        self.get_logger().info('✅ Stationary camera TF tree connected!')
        self.get_logger().info('The camera can now see objects in robot base frame')
        
    def load_calibration(self, calibration_file):
        """Load eye-to-hand calibration from file."""
        # Try multiple paths
        possible_paths = [
            calibration_file,
            os.path.join(os.path.dirname(__file__), '..', calibration_file),
            os.path.join(os.path.dirname(__file__), '..', 'output', 'handeye_realsense_stationary.npz'),
            '/home/pathonai/Documents/Github/opensource_dev/handeye/output/handeye_realsense_stationary.npz'
        ]
        
        for filepath in possible_paths:
            if os.path.exists(filepath):
                self.get_logger().info(f'✅ Loading calibration from: {filepath}')
                data = np.load(filepath)
                
                # Check if this is stationary camera calibration
                if 'T_cam2base' in data:
                    T_cam2base = data['T_cam2base']
                    self.get_logger().info('   Loaded stationary camera calibration')
                elif 'T_cam2gripper' in data:
                    # This is eye-in-hand calibration, cannot use directly for stationary
                    self.get_logger().error('⚠️  This is eye-in-hand calibration file!')
                    self.get_logger().error('   Cannot use T_cam2gripper as T_cam2base')
                    self.get_logger().error('   Please run stationary calibration:')
                    self.get_logger().error('   python handeye_manual_realsense_stationary.py')
                    raise ValueError('Wrong calibration type: expected eye-to-hand (stationary), got eye-in-hand')
                else:
                    raise ValueError('Invalid calibration file format')
                
                # Extract translation (convert mm to m if needed)
                translation = T_cam2base[:3, 3]
                if np.max(np.abs(translation)) > 10:  # Likely in mm
                    translation = translation / 1000.0
                    T_cam2base[:3, 3] = translation
                    self.get_logger().info('   Converted translation from mm to m')
                
                # Apply rotation corrections if needed
                R = T_cam2base[:3, :3]
                flip_x = self.get_parameter('flip_x').value
                flip_y = self.get_parameter('flip_y').value
                flip_z = self.get_parameter('flip_z').value
                rotate_x = self.get_parameter('rotate_x_deg').value
                rotate_y = self.get_parameter('rotate_y_deg').value
                rotate_z = self.get_parameter('rotate_z_deg').value
                
                # Apply flips (180 degree rotations)
                if flip_x:
                    R = R @ Rotation.from_euler('x', 180, degrees=True).as_matrix()
                    self.get_logger().info('   Applied X-axis flip (180°)')
                if flip_y:
                    R = R @ Rotation.from_euler('y', 180, degrees=True).as_matrix()
                    self.get_logger().info('   Applied Y-axis flip (180°)')
                if flip_z:
                    R = R @ Rotation.from_euler('z', 180, degrees=True).as_matrix()
                    self.get_logger().info('   Applied Z-axis flip (180°)')
                
                # Apply additional rotations
                if rotate_x != 0:
                    R = R @ Rotation.from_euler('x', rotate_x, degrees=True).as_matrix()
                    self.get_logger().info(f'   Applied X rotation: {rotate_x}°')
                if rotate_y != 0:
                    R = R @ Rotation.from_euler('y', rotate_y, degrees=True).as_matrix()
                    self.get_logger().info(f'   Applied Y rotation: {rotate_y}°')
                if rotate_z != 0:
                    R = R @ Rotation.from_euler('z', rotate_z, degrees=True).as_matrix()
                    self.get_logger().info(f'   Applied Z rotation: {rotate_z}°')
                
                T_cam2base[:3, :3] = R
                
                # Apply checkerboard offset if specified
                offset_x = self.get_parameter('checkerboard_offset_x').value
                offset_y = self.get_parameter('checkerboard_offset_y').value
                offset_z = self.get_parameter('checkerboard_offset_z').value
                
                if offset_x != 0 or offset_y != 0 or offset_z != 0:
                    # The calibration assumes checkerboard at gripper, but it might be offset
                    # We need to adjust the camera position accordingly
                    offset_in_base = np.array([offset_x, offset_y, offset_z])
                    T_cam2base[:3, 3] += T_cam2base[:3, :3] @ offset_in_base
                    self.get_logger().info(f'   Applied checkerboard offset: [{offset_x:.3f}, {offset_y:.3f}, {offset_z:.3f}] m')
                
                return T_cam2base
        
        # Default transform if calibration not found
        self.get_logger().warn('⚠️  Calibration file not found, using default transform')
        self.get_logger().warn('   Camera placed 1m from base looking at robot')
        
        # Default: camera looking at robot from 1 meter away
        T_cam2base = np.eye(4)
        T_cam2base[0, 3] = 1.0  # 1m in X
        T_cam2base[2, 3] = 0.5  # 0.5m in Z (height)
        
        # Rotate to look at robot
        R = Rotation.from_euler('y', -90, degrees=True).as_matrix()
        T_cam2base[:3, :3] = R
        return T_cam2base
    
    def publish_transform(self):
        """Publish the transform connecting base to camera."""
        # Check if we should use raw calibration or inverted
        use_raw = self.get_parameter('use_raw_calibration').value
        
        if use_raw:
            # Use T_cam2base directly (camera as parent, base as child)
            self.get_logger().warn('⚠️  Using RAW calibration (camera -> base) instead of inverted!')
            T_to_publish = self.T_cam2base
            parent_frame = self.camera_frame
            child_frame = self.base_frame
        else:
            # Normal: invert to get base to camera
            T_base2cam = np.linalg.inv(self.T_cam2base)
            T_to_publish = T_base2cam
            parent_frame = self.base_frame
            child_frame = self.camera_frame
        
        # Create transform message
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        
        # Set translation
        t.transform.translation.x = T_to_publish[0, 3]
        t.transform.translation.y = T_to_publish[1, 3]
        t.transform.translation.z = T_to_publish[2, 3]
        
        # Set rotation
        r = Rotation.from_matrix(T_to_publish[:3, :3])
        q = r.as_quat()  # [x, y, z, w]
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        # Publish transform
        self.static_broadcaster.sendTransform(t)
        
        self.get_logger().info(f'✅ Transform published: {parent_frame} → {child_frame}')
        
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