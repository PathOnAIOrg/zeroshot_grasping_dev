#!/usr/bin/env python3
"""
ThinkGrasp ROS2 Service Node
Provides grasp detection service using ThinkGrasp API
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray, Marker
from cv_bridge import CvBridge
import numpy as np
import cv2
import requests
import json
import tempfile
import os
from datetime import datetime
from scipy.spatial.transform import Rotation as R

# Import custom service
from thinkgrasp_ros2.srv import DetectGrasp
from thinkgrasp_ros2.msg import GraspArray


class ThinkGraspServiceNode(Node):
    def __init__(self):
        super().__init__('thinkgrasp_service_node')
        
        # Declare parameters
        self.declare_parameter('api_host', 'localhost')
        self.declare_parameter('api_port', 5010)
        self.declare_parameter('temp_dir', '/tmp/thinkgrasp')
        self.declare_parameter('publish_markers', True)
        self.declare_parameter('marker_lifetime', 10.0)
        
        # Get parameters
        self.api_host = self.get_parameter('api_host').value
        self.api_port = self.get_parameter('api_port').value
        self.temp_dir = self.get_parameter('temp_dir').value
        self.publish_markers = self.get_parameter('publish_markers').value
        self.marker_lifetime = self.get_parameter('marker_lifetime').value
        
        # Create temp directory
        os.makedirs(self.temp_dir, exist_ok=True)
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Create service
        self.service = self.create_service(
            DetectGrasp, 
            'detect_grasp', 
            self.detect_grasp_callback
        )
        
        # Create publishers
        self.grasp_array_pub = self.create_publisher(
            GraspArray, 
            'detected_grasps', 
            10
        )
        
        self.grasp_markers_pub = self.create_publisher(
            MarkerArray,
            'grasp_markers',
            10
        )
        
        self.point_cloud_pub = self.create_publisher(
            PointCloud2,
            'grasp_point_cloud',
            10
        )
        
        self.get_logger().info(f'ThinkGrasp service node started on {self.api_host}:{self.api_port}')
        
    def detect_grasp_callback(self, request, response):
        """Handle grasp detection service request"""
        try:
            # Convert ROS images to OpenCV format
            rgb_image = self.bridge.imgmsg_to_cv2(request.rgb_image, "rgb8")
            depth_image = self.bridge.imgmsg_to_cv2(request.depth_image, "passthrough")
            
            # Save images temporarily
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            temp_folder = os.path.join(self.temp_dir, timestamp)
            os.makedirs(temp_folder, exist_ok=True)
            
            rgb_path = os.path.join(temp_folder, "rgb.png")
            depth_path = os.path.join(temp_folder, "depth.png")
            text_path = os.path.join(temp_folder, "grasp_text.txt")
            
            # Save files
            cv2.imwrite(rgb_path, cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR))
            cv2.imwrite(depth_path, depth_image.astype(np.uint16))
            
            with open(text_path, 'w') as f:
                f.write(request.grasp_instruction)
            
            # Call ThinkGrasp API
            api_url = f"http://{self.api_host}:{self.api_port}/grasp_pose"
            
            payload = {
                "image_path": rgb_path,
                "depth_path": depth_path,
                "text_path": text_path
            }
            
            self.get_logger().info(f"Calling ThinkGrasp API: {api_url}")
            api_response = requests.post(api_url, json=payload)
            
            if api_response.status_code == 200:
                result = api_response.json()
                
                # Parse the result
                response.success = True
                response.message = "Grasp detection successful"
                response.timestamp = result.get('timestamp', timestamp)
                
                # Convert grasp pose
                xyz = result.get('xyz', [0, 0, 0])
                rot_matrix = np.array(result.get('rot', np.eye(3).tolist()))
                
                # Convert rotation matrix to quaternion
                rotation = R.from_matrix(rot_matrix)
                quat = rotation.as_quat()  # [x, y, z, w]
                
                # Create pose message
                response.grasp_pose = Pose()
                response.grasp_pose.position = Point(x=xyz[0], y=xyz[1], z=xyz[2])
                response.grasp_pose.orientation = Quaternion(
                    x=quat[0], y=quat[1], z=quat[2], w=quat[3]
                )
                
                response.grasp_depth = result.get('dep', 0.0)
                response.confidence_score = 1.0  # Can be enhanced with actual score
                
                # Set visualization URL
                if request.save_visualization:
                    response.visualization_url = f"http://{self.api_host}:{self.api_port}/viewer"
                
                # Publish grasp array message
                self.publish_grasp_array(result, request.rgb_image.header)
                
                # Publish visualization markers
                if self.publish_markers:
                    self.publish_grasp_markers(xyz, rot_matrix, request.rgb_image.header)
                
                self.get_logger().info(f"Grasp detected at position: {xyz}")
                
            elif api_response.status_code == 400:
                # No grasps detected
                error_data = api_response.json()
                response.success = False
                response.message = error_data.get('details', 'No grasp poses detected')
                response.timestamp = error_data.get('timestamp', timestamp)
                self.get_logger().warn(response.message)
                
            else:
                # API error
                response.success = False
                response.message = f"API error: {api_response.status_code}"
                self.get_logger().error(f"API error: {api_response.text}")
                
        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
            self.get_logger().error(f"Service error: {str(e)}")
            
        return response
    
    def publish_grasp_array(self, result, header):
        """Publish all detected grasps as GraspArray message"""
        try:
            grasp_array_msg = GraspArray()
            grasp_array_msg.header = header
            grasp_array_msg.header.stamp = self.get_clock().now().to_msg()
            
            # For now, just publish the selected grasp
            # This can be enhanced to include all grasps from the API
            pose_array = PoseArray()
            pose_array.header = grasp_array_msg.header
            
            # Add the main grasp
            xyz = result.get('xyz', [0, 0, 0])
            rot_matrix = np.array(result.get('rot', np.eye(3).tolist()))
            rotation = R.from_matrix(rot_matrix)
            quat = rotation.as_quat()
            
            pose = Pose()
            pose.position = Point(x=xyz[0], y=xyz[1], z=xyz[2])
            pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
            pose_array.poses.append(pose)
            
            grasp_array_msg.grasp_poses = pose_array
            grasp_array_msg.scores = [1.0]  # Placeholder score
            grasp_array_msg.selected_index = 0
            
            self.grasp_array_pub.publish(grasp_array_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing grasp array: {str(e)}")
    
    def publish_grasp_markers(self, position, rotation_matrix, header):
        """Publish visualization markers for the detected grasp"""
        try:
            marker_array = MarkerArray()
            
            # Create gripper marker
            marker = Marker()
            marker.header = header
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "grasp_pose"
            marker.id = 0
            marker.type = Marker.MESH_RESOURCE
            marker.action = Marker.ADD
            
            # Set position
            marker.pose.position.x = position[0]
            marker.pose.position.y = position[1]
            marker.pose.position.z = position[2]
            
            # Set orientation
            rotation = R.from_matrix(rotation_matrix)
            quat = rotation.as_quat()
            marker.pose.orientation.x = quat[0]
            marker.pose.orientation.y = quat[1]
            marker.pose.orientation.z = quat[2]
            marker.pose.orientation.w = quat[3]
            
            # Set scale
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            
            # Set color (green for success)
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            # Set lifetime
            marker.lifetime = rclpy.duration.Duration(seconds=self.marker_lifetime).to_msg()
            
            # Use a simple cube as placeholder (can be replaced with gripper mesh)
            marker.type = Marker.CUBE
            marker.scale.x = 0.08  # Gripper width
            marker.scale.y = 0.02  # Gripper thickness
            marker.scale.z = 0.10  # Gripper length
            
            marker_array.markers.append(marker)
            
            # Add approach vector marker
            approach_marker = Marker()
            approach_marker.header = marker.header
            approach_marker.ns = "grasp_approach"
            approach_marker.id = 1
            approach_marker.type = Marker.ARROW
            approach_marker.action = Marker.ADD
            
            # Arrow starts at grasp position
            approach_marker.points.append(Point(
                x=position[0], 
                y=position[1], 
                z=position[2]
            ))
            
            # Arrow points in approach direction (z-axis of rotation)
            approach_vector = rotation_matrix[:, 2]
            approach_marker.points.append(Point(
                x=position[0] - 0.1 * approach_vector[0],
                y=position[1] - 0.1 * approach_vector[1],
                z=position[2] - 0.1 * approach_vector[2]
            ))
            
            approach_marker.scale.x = 0.01  # Arrow shaft diameter
            approach_marker.scale.y = 0.02  # Arrow head diameter
            approach_marker.color.r = 1.0
            approach_marker.color.g = 0.0
            approach_marker.color.b = 0.0
            approach_marker.color.a = 0.8
            approach_marker.lifetime = marker.lifetime
            
            marker_array.markers.append(approach_marker)
            
            self.grasp_markers_pub.publish(marker_array)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing markers: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = ThinkGraspServiceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()