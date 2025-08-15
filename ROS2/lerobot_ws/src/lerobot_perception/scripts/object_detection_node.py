#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray, Marker
import cv2
import numpy as np
from cv_bridge import CvBridge
import open3d as o3d
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import message_filters
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs_py import point_cloud2

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Set up QoS profile for synchronized messages
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.rgb_sub = message_filters.Subscriber(
            self, Image, '/camera/rgb/image_raw', qos_profile=qos_profile
        )
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/camera/depth/image_raw', qos_profile=qos_profile
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/rgb/camera_info', self.camera_info_callback, 10
        )
        
        # Synchronize RGB and depth messages
        self.ts = message_filters.TimeSynchronizer([self.rgb_sub, self.depth_sub], 10)
        self.ts.registerCallback(self.image_callback)
        
        # Publishers
        self.object_pointcloud_pub = self.create_publisher(
            PointCloud2, '/detected_objects/pointcloud', 10
        )
        self.object_poses_pub = self.create_publisher(
            PoseArray, '/detected_objects/poses', 10
        )
        self.markers_pub = self.create_publisher(
            MarkerArray, '/detected_objects/markers', 10
        )
        
        # Individual object publishers
        self.red_object_pub = self.create_publisher(
            PointCloud2, '/detected_objects/red/pointcloud', 10
        )
        self.green_object_pub = self.create_publisher(
            PointCloud2, '/detected_objects/green/pointcloud', 10
        )
        self.blue_object_pub = self.create_publisher(
            PointCloud2, '/detected_objects/blue/pointcloud', 10
        )
        self.purple_object_pub = self.create_publisher(
            PointCloud2, '/detected_objects/purple/pointcloud', 10
        )
        
        # Camera parameters
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Object detection parameters
        self.min_area = 1000  # Minimum object area in pixels
        self.max_area = 50000  # Maximum object area in pixels
        
        # Color ranges for object detection (HSV)
        self.color_ranges = {
            'red': [(0, 100, 100), (10, 255, 255)],
            'green': [(40, 100, 100), (80, 255, 255)],
            'blue': [(100, 100, 100), (130, 255, 255)],
            'purple': [(130, 100, 100), (160, 255, 255)],
        }
        
        self.get_logger().info('Object Detection Node initialized')
    
    def camera_info_callback(self, msg):
        """Store camera intrinsic parameters"""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera parameters received')
    
    def image_callback(self, rgb_msg, depth_msg):
        """Process synchronized RGB and depth images"""
        if self.camera_matrix is None:
            self.get_logger().warn('Camera matrix not yet received')
            return
        
        try:
            # Convert ROS messages to OpenCV images
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
            
            self.get_logger().info(f'Processing images: RGB shape={rgb_image.shape}, Depth shape={depth_image.shape}')
            
            # Detect objects
            detected_objects = self.detect_objects(rgb_image)
            self.get_logger().info(f'Detected {len(detected_objects)} objects')
            
            if detected_objects:
                # Generate point clouds for detected objects
                object_pointclouds = self.generate_object_pointclouds(
                    rgb_image, depth_image, detected_objects
                )
                
                self.get_logger().info(f'Generated {len(object_pointclouds)} point clouds')
                
                # Publish results
                self.publish_results(object_pointclouds, detected_objects, rgb_msg.header)
            else:
                self.get_logger().info('No objects detected in this frame')
                
        except Exception as e:
            self.get_logger().error(f'Error processing images: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def detect_objects(self, rgb_image):
        """Detect objects using color-based segmentation"""
        detected_objects = []
        
        # Convert to HSV color space
        hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
        
        for color_name, (lower, upper) in self.color_ranges.items():
            # Create mask for current color
            mask = cv2.inRange(hsv_image, np.array(lower), np.array(upper))
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if self.min_area < area < self.max_area:
                    # Get bounding box
                    x, y, w, h = cv2.boundingRect(contour)
                    
                    # Calculate center
                    center_x = x + w // 2
                    center_y = y + h // 2
                    
                    detected_objects.append({
                        'color': color_name,
                        'center': (center_x, center_y),
                        'bbox': (x, y, w, h),
                        'area': area,
                        'contour': contour,
                        'object_id': f"{color_name}_{len([obj for obj in detected_objects if obj['color'] == color_name]) + 1}"
                    })
        
        return detected_objects
    
    def generate_object_pointclouds(self, rgb_image, depth_image, detected_objects):
        """Generate point clouds for detected objects"""
        object_pointclouds = []
        
        for obj in detected_objects:
            x, y, w, h = obj['bbox']
            
            # Extract region of interest
            roi_depth = depth_image[y:y+h, x:x+w]
            roi_rgb = rgb_image[y:y+h, x:x+w]
            
            # Create mask for the object contour
            mask = np.zeros((h, w), dtype=np.uint8)
            contour_shifted = obj['contour'] - np.array([x, y])
            cv2.fillPoly(mask, [contour_shifted], 255)
            
            # Filter valid depth values
            valid_mask = (roi_depth > 0) & (roi_depth < 2.0) & (mask > 0)
            
            valid_points = np.sum(valid_mask)
            self.get_logger().info(f'Object {obj["object_id"]}: {valid_points} valid depth points')
            
            if valid_points > 100:  # Minimum number of valid points
                # Get valid points
                valid_y, valid_x = np.where(valid_mask)
                valid_depths = roi_depth[valid_y, valid_x]
                valid_colors = roi_rgb[valid_y, valid_x]
                
                # Convert to 3D points
                points_3d = self.depth_to_pointcloud(
                    valid_x + x, valid_y + y, valid_depths
                )
                
                # Create Open3D point cloud
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(points_3d)
                pcd.colors = o3d.utility.Vector3dVector(valid_colors / 255.0)
                
                # Estimate object center in 3D
                center_3d = np.mean(points_3d, axis=0)
                
                object_pointclouds.append({
                    'pointcloud': pcd,
                    'center_3d': center_3d,
                    'color': obj['color'],
                    'bbox': obj['bbox'],
                    'object_id': obj['object_id']
                })
        
        return object_pointclouds
    
    def depth_to_pointcloud(self, u, v, depth):
        """Convert depth pixels to 3D points"""
        # Normalize pixel coordinates
        u_norm = (u - self.camera_matrix[0, 2]) / self.camera_matrix[0, 0]
        v_norm = (v - self.camera_matrix[1, 2]) / self.camera_matrix[1, 1]
        
        # Convert to 3D points
        x = u_norm * depth
        y = v_norm * depth
        z = depth
        
        return np.column_stack([x, y, z])
    
    def publish_results(self, object_pointclouds, detected_objects, header):
        """Publish detected objects and their point clouds"""
        # Publish point clouds
        for i, obj_data in enumerate(object_pointclouds):
            # Convert Open3D point cloud to ROS message
            points = np.asarray(obj_data['pointcloud'].points)
            colors = np.asarray(obj_data['pointcloud'].colors)
            
            # Create PointCloud2 message
            pcd_msg = PointCloud2()
            pcd_msg.header = header
            pcd_msg.header.frame_id = 'world'
            
            # Convert to ROS PointCloud2 format
            cloud_points = []
            for j in range(len(points)):
                cloud_points.append([points[j][0], points[j][1], points[j][2], 
                                   colors[j][0], colors[j][1], colors[j][2]])
            
            pcd_msg = point_cloud2.create_cloud_xyz32(header, points)
            
            # Publish to general topic
            self.object_pointcloud_pub.publish(pcd_msg)
            
            # Publish to color-specific topics
            color_publishers = {
                'red': self.red_object_pub,
                'green': self.green_object_pub,
                'blue': self.blue_object_pub,
                'purple': self.purple_object_pub
            }
            
            if obj_data['color'] in color_publishers:
                color_publishers[obj_data['color']].publish(pcd_msg)
        
        # Publish object poses
        pose_array = PoseArray()
        pose_array.header = header
        pose_array.header.frame_id = 'world'
        
        for obj_data in object_pointclouds:
            pose = Pose()
            pose.position.x = obj_data['center_3d'][0]
            pose.position.y = obj_data['center_3d'][1]
            pose.position.z = obj_data['center_3d'][2]
            pose.orientation.w = 1.0  # Default orientation
            pose_array.poses.append(pose)
            
            # Log object detection
            self.get_logger().info(f"Detected {obj_data['object_id']} at position: ({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})")
        
        self.object_poses_pub.publish(pose_array)
        
        # Publish visualization markers
        self.publish_markers(object_pointclouds, header)
    
    def publish_markers(self, object_pointclouds, header):
        """Publish visualization markers for detected objects"""
        marker_array = MarkerArray()
        
        for i, obj_data in enumerate(object_pointclouds):
            marker = Marker()
            marker.header = header
            marker.header.frame_id = 'world'
            marker.ns = 'detected_objects'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Add text marker for object ID
            text_marker = Marker()
            text_marker.header = header
            text_marker.header.frame_id = 'world'
            text_marker.ns = 'object_labels'
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            # Set position
            marker.pose.position.x = obj_data['center_3d'][0]
            marker.pose.position.y = obj_data['center_3d'][1]
            marker.pose.position.z = obj_data['center_3d'][2]
            marker.pose.orientation.w = 1.0
            
            # Set text marker position (slightly above the sphere)
            text_marker.pose.position.x = obj_data['center_3d'][0]
            text_marker.pose.position.y = obj_data['center_3d'][1]
            text_marker.pose.position.z = obj_data['center_3d'][2] + 0.1
            text_marker.pose.orientation.w = 1.0
            
            # Set scale
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            
            # Set text scale and content
            text_marker.scale.x = 0.1
            text_marker.scale.y = 0.1
            text_marker.scale.z = 0.1
            text_marker.text = obj_data['object_id']
            
            # Set color based on object color
            color_map = {
                'red': (1.0, 0.0, 0.0),
                'green': (0.0, 1.0, 0.0),
                'blue': (0.0, 0.0, 1.0),
                'purple': (1.0, 0.0, 1.0)
            }
            
            color = color_map.get(obj_data['color'], (1.0, 1.0, 1.0))
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 0.8
            
            # Set text color (white text)
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            marker_array.markers.append(marker)
            marker_array.markers.append(text_marker)
        
        self.markers_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
