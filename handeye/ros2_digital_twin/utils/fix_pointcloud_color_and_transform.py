#!/usr/bin/env python3
"""
Fix point cloud color and transformation issues
1. Ensure colored point cloud
2. Correct transformation from camera to robot
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster, Buffer, TransformListener
import numpy as np
import os
from scipy.spatial.transform import Rotation
import struct


class FixPointCloudAndTransform(Node):
    def __init__(self):
        super().__init__('fix_pointcloud_and_transform')
        
        # Parameters
        self.declare_parameter('check_pointcloud', True)
        self.declare_parameter('fix_transform', True)
        self.declare_parameter('gripper_frame', 'gripper')
        
        check_pc = self.get_parameter('check_pointcloud').value
        fix_tf = self.get_parameter('fix_transform').value
        self.gripper_frame = self.get_parameter('gripper_frame').value
        
        if check_pc:
            self.check_pointcloud_color()
        
        if fix_tf:
            self.fix_camera_transform()
    
    def check_pointcloud_color(self):
        """Check if point cloud has RGB data."""
        self.get_logger().info('🔍 Checking point cloud color data...')
        
        # Subscribe to point cloud
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.analyze_pointcloud,
            1
        )
    
    def analyze_pointcloud(self, msg):
        """Analyze point cloud message for color information."""
        self.get_logger().info('=' * 60)
        self.get_logger().info('📊 POINT CLOUD ANALYSIS')
        self.get_logger().info('=' * 60)
        
        # Check fields
        self.get_logger().info(f'Frame ID: {msg.header.frame_id}')
        self.get_logger().info(f'Width x Height: {msg.width} x {msg.height}')
        self.get_logger().info(f'Point step: {msg.point_step} bytes')
        self.get_logger().info(f'Is dense: {msg.is_dense}')
        
        # Check for RGB fields
        has_rgb = False
        has_color = False
        field_info = []
        
        for field in msg.fields:
            field_info.append(f'{field.name} (offset: {field.offset})')
            if field.name in ['rgb', 'rgba']:
                has_rgb = True
            if field.name in ['r', 'g', 'b']:
                has_color = True
        
        self.get_logger().info(f'Fields: {", ".join(field_info)}')
        
        if has_rgb:
            self.get_logger().info('✅ RGB field found - point cloud should be colored')
        elif has_color:
            self.get_logger().info('✅ R,G,B fields found - point cloud should be colored')
        else:
            self.get_logger().warn('⚠️  No color fields found!')
            self.get_logger().info('   Point cloud will appear white in RViz')
            
        # Sample some points to check color values
        if msg.data and len(msg.data) >= msg.point_step:
            # Read first point
            point_data = msg.data[:msg.point_step]
            
            # Find RGB offset
            rgb_offset = None
            for field in msg.fields:
                if field.name == 'rgb':
                    rgb_offset = field.offset
                    break
            
            if rgb_offset is not None and rgb_offset + 4 <= len(point_data):
                # Unpack RGB value
                rgb_packed = struct.unpack('f', point_data[rgb_offset:rgb_offset+4])[0]
                rgb_int = struct.unpack('I', struct.pack('f', rgb_packed))[0]
                
                r = (rgb_int >> 16) & 0xFF
                g = (rgb_int >> 8) & 0xFF
                b = rgb_int & 0xFF
                
                self.get_logger().info(f'Sample point RGB: ({r}, {g}, {b})')
                
                if r == 0 and g == 0 and b == 0:
                    self.get_logger().warn('⚠️  Sample point is black - might be no texture')
                elif r == 255 and g == 255 and b == 255:
                    self.get_logger().warn('⚠️  Sample point is white - might be default color')
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('💡 SOLUTIONS:')
        self.get_logger().info('=' * 60)
        
        if not has_rgb and not has_color:
            self.get_logger().info("""
For colored point cloud, relaunch RealSense with:

ros2 launch realsense2_camera rs_launch.py \\
    enable_rgbd:=true \\
    enable_sync:=true \\
    align_depth.enable:=true \\
    enable_color:=true \\
    enable_depth:=true \\
    pointcloud.enable:=true \\
    pointcloud.stream_filter:=2 \\
    pointcloud.allow_no_texture_points:=false
""")
        
        self.get_logger().info("""
In RViz2 PointCloud2 settings:
1. Set Color Transformer to: RGB8 (not Intensity or FlatColor)
2. Set Channel Name to: rgb (if using Intensity)
3. Try Style: Boxes or Spheres (sometimes Points don't show color)
""")
        
        # Unsubscribe after first message
        self.destroy_subscription(self.pc_sub)
    
    def fix_camera_transform(self):
        """Fix camera transformation to robot."""
        self.get_logger().info('\n🔧 Fixing camera transformation...')
        
        # Load calibration
        calibration_file = '/home/pathonai/Documents/Github/opensource_dev/handeye/output/handeye_realsense.npz'
        
        if os.path.exists(calibration_file):
            data = np.load(calibration_file)
            T = data['T_cam2gripper']
            
            self.get_logger().info('Calibration matrix:')
            self.get_logger().info(str(T))
            
            # Check if it's identity matrix (no transformation)
            if np.allclose(T, np.eye(4)):
                self.get_logger().warn('⚠️  Calibration is identity matrix!')
                self.get_logger().info('   Using default offset: camera 10cm above gripper')
                
                # Create default transform
                T = np.eye(4)
                T[2, 3] = 0.1  # 10cm in Z
        else:
            self.get_logger().warn('⚠️  No calibration file found')
            self.get_logger().info('   Using default transformation')
            
            # Default: camera above and slightly forward of gripper
            T = np.eye(4)
            T[0, 3] = 0.05  # 5cm forward (X)
            T[2, 3] = 0.1   # 10cm up (Z)
        
        # Create static transform broadcaster
        static_broadcaster = StaticTransformBroadcaster(self)
        
        # Create transform message
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.gripper_frame
        t.child_frame_id = 'camera_link'
        
        # Set translation
        t.transform.translation.x = T[0, 3]
        t.transform.translation.y = T[1, 3]
        t.transform.translation.z = T[2, 3]
        
        # Set rotation
        r = Rotation.from_matrix(T[:3, :3])
        q = r.as_quat()  # [x, y, z, w]
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        # Publish transform
        static_broadcaster.sendTransform(t)
        
        self.get_logger().info(f'✅ Published transform: {self.gripper_frame} → camera_link')
        self.get_logger().info(f'   Translation: [{T[0,3]:.3f}, {T[1,3]:.3f}, {T[2,3]:.3f}]')
        
        # Also publish camera optical frame transform (important for point cloud)
        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'camera_link'
        t2.child_frame_id = 'camera_color_optical_frame_correct'
        
        # Optical frame rotation (Z forward, Y down, X right)
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.0
        t2.transform.rotation.x = -0.5
        t2.transform.rotation.y = 0.5
        t2.transform.rotation.z = -0.5
        t2.transform.rotation.w = 0.5
        
        static_broadcaster.sendTransform([t, t2])
        
        self.get_logger().info('✅ Published optical frame transform')


def main(args=None):
    print("\n" + "=" * 60)
    print("🛠️  FIXING POINT CLOUD COLOR AND TRANSFORM")
    print("=" * 60)
    
    rclpy.init(args=args)
    
    node = FixPointCloudAndTransform()
    
    print("\n📋 CHECKLIST:")
    print("=" * 60)
    print("""
1. COLORED POINT CLOUD:
   □ Relaunch RealSense with color enabled:
     ros2 launch realsense2_camera rs_launch.py \\
         enable_rgbd:=true \\
         enable_sync:=true \\
         align_depth.enable:=true \\
         pointcloud.enable:=true \\
         pointcloud.stream_filter:=2

2. RVIZ2 SETTINGS:
   □ PointCloud2 → Color Transformer: RGB8
   □ PointCloud2 → Channel Name: rgb
   □ PointCloud2 → Size (m): 0.003
   □ PointCloud2 → Style: Boxes or Spheres

3. CORRECT TRANSFORM:
   □ Fixed Frame: base (or base_link)
   □ Point cloud should appear at gripper position
   □ Check with: ros2 run tf2_ros tf2_echo base camera_link

4. IF STILL WHITE:
   □ Try different topic:
     - /camera/camera/depth/color/points
     - /camera/depth/color/points
     - /camera/depth_registered/points
   □ Check RealSense Viewer works with color
   □ Ensure good lighting for color camera
""")
    
    print("=" * 60)
    print("\nPress Ctrl+C to exit")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()