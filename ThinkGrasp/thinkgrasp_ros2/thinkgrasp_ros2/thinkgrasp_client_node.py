#!/usr/bin/env python3
"""
ThinkGrasp ROS2 Client Node
Example client for testing the ThinkGrasp service
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import sys

from thinkgrasp_ros2.srv import DetectGrasp


class ThinkGraspClientNode(Node):
    def __init__(self):
        super().__init__('thinkgrasp_client_node')
        
        # Create service client
        self.client = self.create_client(DetectGrasp, 'detect_grasp')
        
        # Wait for service
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for ThinkGrasp service...')
        
        self.bridge = CvBridge()
        self.get_logger().info('ThinkGrasp client ready')
        
    def send_request(self, rgb_path, depth_path, instruction="grasp the object"):
        """Send grasp detection request"""
        
        # Load images
        rgb_image = cv2.imread(rgb_path)
        if rgb_image is None:
            self.get_logger().error(f"Failed to load RGB image: {rgb_path}")
            return None
            
        depth_image = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
        if depth_image is None:
            self.get_logger().error(f"Failed to load depth image: {depth_path}")
            return None
        
        # Convert to ROS messages
        rgb_msg = self.bridge.cv2_to_imgmsg(rgb_image, "bgr8")
        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, "passthrough")
        
        # Create request
        request = DetectGrasp.Request()
        request.rgb_image = rgb_msg
        request.depth_image = depth_msg
        request.grasp_instruction = instruction
        request.save_visualization = True
        
        # Send request
        self.get_logger().info(f"Sending grasp detection request: '{instruction}'")
        future = self.client.call_async(request)
        
        return future
    
    def send_request_from_topic(self, instruction="grasp the object"):
        """Send request using images from ROS topics"""
        
        # Subscribe to image topics
        rgb_msg = None
        depth_msg = None
        
        def rgb_callback(msg):
            nonlocal rgb_msg
            rgb_msg = msg
            
        def depth_callback(msg):
            nonlocal depth_msg
            depth_msg = msg
        
        rgb_sub = self.create_subscription(
            Image, '/camera/color/image_raw', rgb_callback, 10
        )
        depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', depth_callback, 10
        )
        
        # Wait for messages
        self.get_logger().info("Waiting for camera images...")
        timeout = 5.0
        start_time = self.get_clock().now()
        
        while rgb_msg is None or depth_msg is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > timeout:
                self.get_logger().error("Timeout waiting for camera images")
                return None
        
        # Destroy subscriptions
        self.destroy_subscription(rgb_sub)
        self.destroy_subscription(depth_sub)
        
        # Create and send request
        request = DetectGrasp.Request()
        request.rgb_image = rgb_msg
        request.depth_image = depth_msg
        request.grasp_instruction = instruction
        request.save_visualization = True
        
        self.get_logger().info(f"Sending grasp detection request: '{instruction}'")
        future = self.client.call_async(request)
        
        return future


def main(args=None):
    rclpy.init(args=args)
    
    client = ThinkGraspClientNode()
    
    # Parse command line arguments
    if len(sys.argv) > 3:
        # Use provided image paths
        rgb_path = sys.argv[1]
        depth_path = sys.argv[2]
        instruction = sys.argv[3] if len(sys.argv) > 3 else "grasp the object"
        
        future = client.send_request(rgb_path, depth_path, instruction)
        
    elif len(sys.argv) > 1:
        # Use ROS topics with custom instruction
        instruction = sys.argv[1]
        future = client.send_request_from_topic(instruction)
        
    else:
        # Default: use ROS topics
        client.get_logger().info("Usage:")
        client.get_logger().info("  From files: ros2 run thinkgrasp_ros2 thinkgrasp_client_node <rgb_path> <depth_path> <instruction>")
        client.get_logger().info("  From topics: ros2 run thinkgrasp_ros2 thinkgrasp_client_node <instruction>")
        client.get_logger().info("")
        client.get_logger().info("Using default: capturing from ROS topics...")
        
        future = client.send_request_from_topic("grasp the object")
    
    if future is not None:
        # Wait for response
        rclpy.spin_until_future_complete(client, future)
        
        try:
            response = future.result()
            
            if response.success:
                client.get_logger().info("=== Grasp Detection Result ===")
                client.get_logger().info(f"Success: {response.success}")
                client.get_logger().info(f"Message: {response.message}")
                client.get_logger().info(f"Timestamp: {response.timestamp}")
                client.get_logger().info(f"Grasp Position: [{response.grasp_pose.position.x:.3f}, "
                                       f"{response.grasp_pose.position.y:.3f}, "
                                       f"{response.grasp_pose.position.z:.3f}]")
                client.get_logger().info(f"Grasp Orientation: [{response.grasp_pose.orientation.x:.3f}, "
                                       f"{response.grasp_pose.orientation.y:.3f}, "
                                       f"{response.grasp_pose.orientation.z:.3f}, "
                                       f"{response.grasp_pose.orientation.w:.3f}]")
                client.get_logger().info(f"Grasp Depth: {response.grasp_depth:.3f}")
                client.get_logger().info(f"Confidence: {response.confidence_score:.2f}")
                
                if response.visualization_url:
                    client.get_logger().info(f"Visualization: {response.visualization_url}")
            else:
                client.get_logger().error(f"Grasp detection failed: {response.message}")
                
        except Exception as e:
            client.get_logger().error(f"Service call failed: {str(e)}")
    
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()