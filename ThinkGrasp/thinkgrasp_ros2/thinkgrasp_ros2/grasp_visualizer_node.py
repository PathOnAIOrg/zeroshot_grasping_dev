#!/usr/bin/env python3
"""
Grasp Visualizer Node
Visualizes grasp poses in RViz using markers
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from thinkgrasp_ros2.msg import GraspArray
import numpy as np


class GraspVisualizerNode(Node):
    def __init__(self):
        super().__init__('grasp_visualizer_node')
        
        # Parameters
        self.declare_parameter('update_rate', 30.0)
        self.update_rate = self.get_parameter('update_rate').value
        
        # Subscriber
        self.grasp_sub = self.create_subscription(
            GraspArray,
            '/thinkgrasp/grasps',
            self.grasp_callback,
            10
        )
        
        # Publisher
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/thinkgrasp/visualization_markers',
            10
        )
        
        # Storage
        self.latest_grasps = None
        
        # Timer for publishing markers
        self.timer = self.create_timer(
            1.0 / self.update_rate,
            self.publish_markers
        )
        
        self.get_logger().info('Grasp visualizer node started')
    
    def grasp_callback(self, msg):
        """Store latest grasp array"""
        self.latest_grasps = msg
        self.get_logger().info(f"Received {len(msg.grasp_poses.poses)} grasps")
    
    def publish_markers(self):
        """Publish visualization markers"""
        if self.latest_grasps is None:
            return
        
        marker_array = MarkerArray()
        
        # Clear previous markers
        clear_marker = Marker()
        clear_marker.header = self.latest_grasps.header
        clear_marker.header.stamp = self.get_clock().now().to_msg()
        clear_marker.ns = "grasps"
        clear_marker.id = 0
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        
        # Add grasp markers
        for i, (pose, score) in enumerate(zip(
            self.latest_grasps.grasp_poses.poses,
            self.latest_grasps.scores
        )):
            # Gripper base marker
            marker = Marker()
            marker.header = self.latest_grasps.header
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "grasps"
            marker.id = i * 3 + 1
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = pose
            
            # Scale gripper
            marker.scale.x = 0.08
            marker.scale.y = 0.02
            marker.scale.z = 0.10
            
            # Color based on score and selection
            if i == self.latest_grasps.selected_index:
                # Selected grasp - green
                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.9)
            else:
                # Other grasps - color by score
                marker.color = ColorRGBA(
                    r=1.0 - score,
                    g=score,
                    b=0.0,
                    a=0.5
                )
            
            marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()
            marker_array.markers.append(marker)
            
            # Add finger markers
            for j, offset_y in enumerate([-0.04, 0.04]):
                finger = Marker()
                finger.header = marker.header
                finger.ns = "fingers"
                finger.id = i * 3 + j + 2
                finger.type = Marker.CUBE
                finger.action = Marker.ADD
                
                # Position fingers relative to gripper
                finger.pose = pose
                finger.pose.position.y += offset_y
                
                finger.scale.x = 0.01
                finger.scale.y = 0.01
                finger.scale.z = 0.08
                
                finger.color = marker.color
                finger.lifetime = marker.lifetime
                marker_array.markers.append(finger)
            
            # Add text label with score
            if i == self.latest_grasps.selected_index:
                label = Marker()
                label.header = marker.header
                label.ns = "labels"
                label.id = i
                label.type = Marker.TEXT_VIEW_FACING
                label.action = Marker.ADD
                label.pose = pose
                label.pose.position.z += 0.15
                label.scale.z = 0.02
                label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
                label.text = f"Score: {score:.2f}"
                label.lifetime = marker.lifetime
                marker_array.markers.append(label)
        
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = GraspVisualizerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()