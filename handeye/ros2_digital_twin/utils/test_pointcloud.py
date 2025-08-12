#!/usr/bin/env python3
"""
Test script to verify RealSense point cloud is publishing correctly
and can be visualized with SO-101 robot model
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import time


class PointCloudTester(Node):
    def __init__(self):
        super().__init__('pointcloud_tester')
        
        # List of possible point cloud topics
        self.topics_to_check = [
            '/camera/camera/depth/color/points',
            '/camera/depth/color/points',
            '/camera/realsense/depth/color/points',
            '/camera/aligned_depth_to_color/color/points',
            '/camera/depth/points',
            '/camera/pointcloud',
        ]
        
        self.subscribers = {}
        self.received_data = {}
        
        # Subscribe to all possible topics
        for topic in self.topics_to_check:
            self.received_data[topic] = False
            self.subscribers[topic] = self.create_subscription(
                PointCloud2,
                topic,
                lambda msg, t=topic: self.pointcloud_callback(msg, t),
                10
            )
        
        # Timer to check status
        self.timer = self.create_timer(2.0, self.check_status)
        
        self.get_logger().info('🔍 Searching for point cloud topics...')
        self.get_logger().info('This will help identify the correct topic for RViz2')
        
    def pointcloud_callback(self, msg, topic):
        """Callback when point cloud is received."""
        if not self.received_data[topic]:
            self.received_data[topic] = True
            self.get_logger().info(f'✅ Found point cloud on: {topic}')
            self.get_logger().info(f'   Frame ID: {msg.header.frame_id}')
            self.get_logger().info(f'   Width x Height: {msg.width} x {msg.height}')
            self.get_logger().info(f'   Point step: {msg.point_step} bytes')
            self.get_logger().info(f'   Is dense: {msg.is_dense}')
            
            # Print field information
            fields_info = ', '.join([f'{f.name}({f.datatype})' for f in msg.fields])
            self.get_logger().info(f'   Fields: {fields_info}')
    
    def check_status(self):
        """Periodically check and report status."""
        active_topics = [t for t, received in self.received_data.items() if received]
        
        if active_topics:
            self.get_logger().info('\n' + '=' * 60)
            self.get_logger().info('📊 ACTIVE POINT CLOUD TOPICS:')
            for topic in active_topics:
                self.get_logger().info(f'   ✅ {topic}')
            
            self.get_logger().info('\n📝 To use in RViz2:')
            self.get_logger().info('1. Add Display → PointCloud2')
            self.get_logger().info(f'2. Set Topic to: {active_topics[0]}')
            self.get_logger().info('3. Set Fixed Frame to: base_link')
            self.get_logger().info('4. Adjust Size (m) to: 0.002')
            self.get_logger().info('5. Set Color Transformer to: RGB8')
            self.get_logger().info('=' * 60)
            
            # Stop checking after finding topics
            self.timer.cancel()
            
            # Keep node alive to continue monitoring
            self.create_timer(10.0, lambda: None)
        else:
            self.get_logger().warn('⚠️  No point cloud topics found yet...')
            self.get_logger().info('Make sure RealSense is running:')
            self.get_logger().info('  ros2 launch realsense2_camera rs_launch.py \\')
            self.get_logger().info('      enable_rgbd:=true \\')
            self.get_logger().info('      enable_sync:=true \\')
            self.get_logger().info('      align_depth.enable:=true \\')
            self.get_logger().info('      enable_color:=true \\')
            self.get_logger().info('      enable_depth:=true \\')
            self.get_logger().info('      pointcloud.enable:=true')


def main(args=None):
    print("\n" + "=" * 60)
    print("🔍 RealSense Point Cloud Topic Finder")
    print("=" * 60)
    print("\nThis tool will help you find the correct point cloud topic")
    print("for visualizing in RViz2 with your SO-101 robot.\n")
    
    rclpy.init(args=args)
    
    node = PointCloudTester()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()