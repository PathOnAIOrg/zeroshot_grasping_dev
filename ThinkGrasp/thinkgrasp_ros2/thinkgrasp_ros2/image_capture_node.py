#!/usr/bin/env python3
"""
Image Capture Node for ThinkGrasp
Captures and saves RGB-D images from RealSense or other cameras
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from datetime import datetime


class ImageCaptureNode(Node):
    def __init__(self):
        super().__init__('image_capture_node')
        
        # Parameters
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('capture_dir', '/tmp/thinkgrasp/captures')
        self.declare_parameter('auto_capture_rate', 0.0)  # Hz, 0 means manual only
        
        self.rgb_topic = self.get_parameter('rgb_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.capture_dir = self.get_parameter('capture_dir').value
        self.auto_capture_rate = self.get_parameter('auto_capture_rate').value
        
        # Create capture directory
        os.makedirs(self.capture_dir, exist_ok=True)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Storage for latest images
        self.latest_rgb = None
        self.latest_depth = None
        
        # Subscriptions
        self.rgb_sub = self.create_subscription(
            Image,
            self.rgb_topic,
            self.rgb_callback,
            10
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10
        )
        
        # Service for manual capture
        self.create_service(
            type('CaptureImages', (), {
                'Request': type('Request', (), {}),
                'Response': type('Response', (), {
                    'success': bool,
                    'rgb_path': str,
                    'depth_path': str,
                    'message': str
                })
            }),
            'capture_images',
            self.capture_service_callback
        )
        
        # Timer for auto capture
        if self.auto_capture_rate > 0:
            self.timer = self.create_timer(
                1.0 / self.auto_capture_rate,
                self.auto_capture_callback
            )
        
        self.get_logger().info(f"Image capture node started")
        self.get_logger().info(f"  RGB topic: {self.rgb_topic}")
        self.get_logger().info(f"  Depth topic: {self.depth_topic}")
        self.get_logger().info(f"  Capture directory: {self.capture_dir}")
        
    def rgb_callback(self, msg):
        """Store latest RGB image"""
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting RGB image: {e}")
    
    def depth_callback(self, msg):
        """Store latest depth image"""
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        except Exception as e:
            self.get_logger().error(f"Error converting depth image: {e}")
    
    def capture_images(self):
        """Capture and save current images"""
        if self.latest_rgb is None or self.latest_depth is None:
            self.get_logger().warn("No images available for capture")
            return None, None, "No images available"
        
        # Generate timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        capture_folder = os.path.join(self.capture_dir, timestamp)
        os.makedirs(capture_folder, exist_ok=True)
        
        # Save images
        rgb_path = os.path.join(capture_folder, "rgb.png")
        depth_path = os.path.join(capture_folder, "depth.png")
        
        try:
            cv2.imwrite(rgb_path, self.latest_rgb)
            cv2.imwrite(depth_path, self.latest_depth.astype(np.uint16))
            
            # Also save a visualization of the depth image
            depth_vis = self.visualize_depth(self.latest_depth)
            depth_vis_path = os.path.join(capture_folder, "depth_visualization.png")
            cv2.imwrite(depth_vis_path, depth_vis)
            
            self.get_logger().info(f"Images captured to {capture_folder}")
            return rgb_path, depth_path, "Capture successful"
            
        except Exception as e:
            self.get_logger().error(f"Error saving images: {e}")
            return None, None, f"Save error: {e}"
    
    def visualize_depth(self, depth_image):
        """Create a colorized visualization of depth image"""
        # Normalize depth for visualization
        depth_norm = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        # Apply colormap
        depth_colored = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
        return depth_colored
    
    def capture_service_callback(self, request, response):
        """Handle manual capture service request"""
        rgb_path, depth_path, message = self.capture_images()
        
        response.success = (rgb_path is not None)
        response.rgb_path = rgb_path or ""
        response.depth_path = depth_path or ""
        response.message = message
        
        return response
    
    def auto_capture_callback(self):
        """Auto capture timer callback"""
        self.capture_images()


def main(args=None):
    rclpy.init(args=args)
    node = ImageCaptureNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()