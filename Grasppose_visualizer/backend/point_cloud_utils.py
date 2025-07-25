import numpy as np
import cv2
import open3d as o3d
import sys
import os

# Add ThinkGrasp to path to use its utilities
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'ThinkGrasp'))

try:
    from models.graspnet.utils.data_utils import CameraInfo, create_point_cloud_from_depth_image
    from config import CameraConfig
    THINKGRASP_AVAILABLE = True
except ImportError:
    THINKGRASP_AVAILABLE = False
    print("Warning: ThinkGrasp modules not available, using fallback implementation")

class PointCloudGenerator:
    def __init__(self):
        if THINKGRASP_AVAILABLE:
            # Use ThinkGrasp's camera configuration
            camera_info = CameraConfig.get_camera_info()
            self.width = camera_info.width
            self.height = camera_info.height
            self.fx = camera_info.fx
            self.fy = camera_info.fy
            self.cx = camera_info.cx
            self.cy = camera_info.cy
            self.depth_scale = camera_info.scale
        else:
            # Fallback camera parameters
            self.width = 640
            self.height = 480
            self.fx = 383.9592
            self.fy = 383.6245
            self.cx = 322.1625
            self.cy = 245.3161
            self.depth_scale = 1000.0
        
    def generate_point_cloud(self, rgb_image, depth_image):
        """Generate point cloud from RGB-D images using ThinkGrasp method"""
        
        # Ensure images have the same dimensions
        if rgb_image.shape[:2] != depth_image.shape[:2]:
            depth_image = cv2.resize(depth_image, (rgb_image.shape[1], rgb_image.shape[0]))
        
        if THINKGRASP_AVAILABLE:
            # Use ThinkGrasp's exact method
            camera_info = CameraInfo(
                width=self.width,
                height=self.height,
                fx=self.fx,
                fy=self.fy,
                cx=self.cx,
                cy=self.cy,
                scale=self.depth_scale
            )
            # Generate point cloud using ThinkGrasp's function
            cloud = create_point_cloud_from_depth_image(depth_image, camera_info, organized=True)
        else:
            # Fallback implementation (same as ThinkGrasp)
            cloud = self.get_pointcloud(depth_image)
        
        # Get color array
        color_array = np.array(rgb_image, dtype=np.float32) / 255.0
        
        # Filter invalid points
        height, width = depth_image.shape
        valid_mask = (depth_image > 0) & (depth_image <= 2000)
        
        # Get valid points
        if len(cloud.shape) == 3:  # Organized cloud
            valid_points = cloud[valid_mask]
            valid_colors = color_array[valid_mask]
        else:  # Already flattened
            valid_points = cloud
            valid_colors = color_array.reshape(-1, 3)
        
        # Downsample for performance
        step = 4
        if len(valid_points) > 0:
            indices = np.arange(0, len(valid_points), step)
            valid_points = valid_points[indices]
            valid_colors = valid_colors[indices]
        
        # Apply coordinate transformation for Three.js
        # Camera: X=right, Y=down, Z=forward -> Three.js: X=right, Y=up, Z=backward
        transform = np.array([
            [1, 0, 0],
            [0, -1, 0],
            [0, 0, -1]
        ])
        
        transformed_points = (transform @ valid_points.T).T
        
        # Create point cloud data
        point_cloud_with_colors = []
        for i in range(len(transformed_points)):
            point_cloud_with_colors.append({
                'position': transformed_points[i].tolist(),
                'color': valid_colors[i].tolist()
            })
        
        return point_cloud_with_colors
    
    def get_pointcloud(self, depth):
        """Get 3D pointcloud from perspective depth image (same as ThinkGrasp/utils.py)
        Args:
            depth: HxW float array of perspective depth in meters.
        Returns:
            points: HxWx3 float array of 3D points in camera coordinates.
        """
        height, width = depth.shape
        xlin = np.linspace(0, width - 1, width)
        ylin = np.linspace(0, height - 1, height)
        px, py = np.meshgrid(xlin, ylin)
        px = (px - self.cx) * (depth / self.fx)
        py = (py - self.cy) * (depth / self.fy)
        points = np.float32([px, py, depth]).transpose(1, 2, 0)
        return points
    
    def create_point_cloud_from_depth_image(self, depth_image, organized=False):
        """Generate point cloud using exact method from ThinkGrasp
        
        Args:
            depth_image: [numpy.ndarray, (H,W), numpy.float32/uint16] - depth image
            organized: bool - whether to keep the cloud in image shape (H,W,3)
        
        Returns:
            cloud: [numpy.ndarray, (H,W,3)/(H*W,3), numpy.float32] - generated cloud
        """
        if THINKGRASP_AVAILABLE:
            camera_info = CameraInfo(
                width=self.width,
                height=self.height,
                fx=self.fx,
                fy=self.fy,
                cx=self.cx,
                cy=self.cy,
                scale=self.depth_scale
            )
            return create_point_cloud_from_depth_image(depth_image, camera_info, organized=organized)
        else:
            # Fallback to local implementation
            assert(depth_image.shape[0] == self.height and depth_image.shape[1] == self.width)
            
            # Convert depth to meters
            depth_meters = depth_image.astype(np.float32) / self.depth_scale
            
            # Use get_pointcloud method (same as ThinkGrasp)
            cloud = self.get_pointcloud(depth_meters)
            
            if not organized:
                cloud = cloud.reshape([-1, 3])
            
            return cloud
    
    def downsample_point_cloud(self, points, voxel_size=0.01):
        """Downsample point cloud for better performance"""
        if len(points) == 0:
            return points
        
        # Create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array([p['position'] for p in points]))
        pcd.colors = o3d.utility.Vector3dVector(np.array([p['color'] for p in points]))
        
        # Downsample
        downsampled = pcd.voxel_down_sample(voxel_size)
        
        # Convert back to our format
        downsampled_points = []
        for i, point in enumerate(np.asarray(downsampled.points)):
            color = np.asarray(downsampled.colors)[i] if i < len(downsampled.colors) else [0.5, 0.5, 0.5]
            downsampled_points.append({
                'position': point.tolist(),
                'color': color.tolist()
            })
        
        return downsampled_points
    
    def filter_point_cloud(self, points, min_z=0.0, max_z=2.0):
        """Filter point cloud by depth range"""
        filtered_points = []
        
        for point in points:
            z = point['position'][2]
            if min_z <= z <= max_z:
                filtered_points.append(point)
        
        return filtered_points
    
    def estimate_normals(self, points, radius=0.05):
        """Estimate normals for point cloud"""
        if len(points) < 3:
            return []
        
        # Create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array([p['position'] for p in points]))
        
        # Estimate normals
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=30)
        )
        
        # Convert to list
        normals = np.asarray(pcd.normals)
        return normals.tolist()
    
