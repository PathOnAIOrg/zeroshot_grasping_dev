#!/usr/bin/env python3
"""
Simplified ThinkGrasp ROS2 Client Node
Works without custom service definitions - uses standard ROS2 messages
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import requests
import json
import sys
import os
import tempfile
import shutil
from pathlib import Path

class SimpleThinkGraspClient(Node):
    def __init__(self):
        super().__init__('simple_thinkgrasp_client')
        
        # Parameters
        self.declare_parameter('api_host', 'localhost')
        self.declare_parameter('api_port', 5010)
        self.declare_parameter('rgb_image_path', '')
        self.declare_parameter('depth_image_path', '')
        self.declare_parameter('instruction', 'grasp the object')
        
        # Get parameters
        self.api_host = self.get_parameter('api_host').value
        self.api_port = self.get_parameter('api_port').value
        
        # Publisher for results
        self.result_pub = self.create_publisher(String, '/thinkgrasp/result', 10)
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        self.get_logger().info(f'Simple ThinkGrasp Client started')
        self.get_logger().info(f'API: http://{self.api_host}:{self.api_port}')
        
    def call_grasp_api(self, rgb_path, depth_path, instruction):
        """Call ThinkGrasp API and return results"""
        
        # Create temporary directory with proper permissions
        temp_dir = tempfile.mkdtemp(prefix='thinkgrasp_ros2_', dir='/tmp')
        os.chmod(temp_dir, 0o755)  # Ensure readable by all
        
        try:
            # Copy files to temp directory with proper permissions
            temp_rgb = os.path.join(temp_dir, 'rgb.png')
            temp_depth = os.path.join(temp_dir, 'depth.png')
            temp_text = os.path.join(temp_dir, 'instruction.txt')
            
            shutil.copy2(rgb_path, temp_rgb)
            shutil.copy2(depth_path, temp_depth)
            
            # Ensure files are readable
            os.chmod(temp_rgb, 0o644)
            os.chmod(temp_depth, 0o644)
            
            with open(temp_text, 'w') as f:
                f.write(instruction)
            os.chmod(temp_text, 0o644)
            
            # Call API
            url = f"http://{self.api_host}:{self.api_port}/grasp_pose"
            payload = {
                'image_path': temp_rgb,
                'depth_path': temp_depth,
                'text_path': temp_text
            }
            
            self.get_logger().info(f'Calling API: {url}')
            response = requests.post(url, json=payload, timeout=60)
            
            if response.status_code == 200:
                return response.json()
            else:
                self.get_logger().error(f'API error: {response.status_code} - {response.text}')
                return None
                
        except Exception as e:
            self.get_logger().error(f'Error calling API: {str(e)}')
            return None
            
        finally:
            # Clean up temp directory
            if os.path.exists(temp_dir):
                shutil.rmtree(temp_dir)
    
    def run_detection_from_files(self, rgb_path, depth_path, instruction):
        """Run detection from file paths"""
        
        # Validate files
        if not os.path.exists(rgb_path):
            self.get_logger().error(f'RGB image not found: {rgb_path}')
            return False
            
        if not os.path.exists(depth_path):
            self.get_logger().error(f'Depth image not found: {depth_path}')
            return False
        
        self.get_logger().info(f'Processing:')
        self.get_logger().info(f'  RGB: {rgb_path}')
        self.get_logger().info(f'  Depth: {depth_path}')
        self.get_logger().info(f'  Instruction: {instruction}')
        
        # Call API
        result = self.call_grasp_api(rgb_path, depth_path, instruction)
        
        if result:
            # Publish result
            result_msg = String()
            result_msg.data = json.dumps(result)
            self.result_pub.publish(result_msg)
            
            # Log results
            self.get_logger().info('✅ Grasp detection successful!')
            self.get_logger().info(f'  Timestamp: {result.get("timestamp", "N/A")}')
            
            xyz = result.get('xyz', [])
            if xyz:
                self.get_logger().info(f'  Position: X={xyz[0]:.4f}, Y={xyz[1]:.4f}, Z={xyz[2]:.4f}')
            
            self.get_logger().info(f'  Depth: {result.get("dep", "N/A")}')
            self.get_logger().info(f'  Visualization: {result.get("message", "")}')
            
            return True
        else:
            self.get_logger().error('Failed to get grasp pose')
            return False

def main(args=None):
    # Handle command line arguments
    if len(sys.argv) > 1:
        # Running with file arguments
        if len(sys.argv) < 4:
            print("Usage: ros2 run thinkgrasp_ros2 simple_client_node <rgb_image> <depth_image> <instruction>")
            print("Example: ros2 run thinkgrasp_ros2 simple_client_node rgb.png depth.png 'grasp the object'")
            sys.exit(1)
        
        rgb_path = sys.argv[1]
        depth_path = sys.argv[2]
        instruction = sys.argv[3]
        
        # Remove ROS args
        sys.argv = sys.argv[:1]
    else:
        rgb_path = None
        depth_path = None
        instruction = None
    
    rclpy.init(args=args)
    
    node = SimpleThinkGraspClient()
    
    if rgb_path and depth_path and instruction:
        # Run detection once
        success = node.run_detection_from_files(rgb_path, depth_path, instruction)
        
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()
        
        sys.exit(0 if success else 1)
    else:
        # Run as service (would need topic subscriptions)
        node.get_logger().info('Running in service mode. Waiting for requests...')
        node.get_logger().warn('Service mode not fully implemented. Use with file arguments instead.')
        
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()