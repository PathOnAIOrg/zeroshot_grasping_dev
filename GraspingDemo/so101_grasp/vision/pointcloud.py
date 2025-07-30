"""
Point Cloud Processing Module

Handles 3D point cloud generation, processing, and filtering.
"""

import numpy as np
import open3d as o3d
from typing import Tuple, Optional, List
import cv2


class PointCloudProcessor:
    """Processes RGB-D data into 3D point clouds."""
    
    def __init__(self):
        """Initialize point cloud processor."""
        pass
    
    def rgbd_to_pointcloud(self, color_image: np.ndarray, depth_image: np.ndarray, 
                          intrinsics: object) -> Tuple[np.ndarray, np.ndarray]:
        """
        Convert RGB-D images to 3D point cloud.
        
        Args:
            color_image: RGB image array
            depth_image: Depth image array
            intrinsics: Camera intrinsics
            
        Returns:
            Tuple of (points, colors) arrays
        """
        height, width = depth_image.shape
        fx, fy = intrinsics.fx, intrinsics.fy
        cx, cy = intrinsics.ppx, intrinsics.ppy
        
        points = []
        colors = []
        
        for v in range(height):
            for u in range(width):
                depth = depth_image[v, u]
                
                # Skip invalid depth
                if depth == 0 or depth > 3000:  # 3m max range
                    continue
                
                # Convert pixel to 3D point
                depth_m = depth / 1000.0  # Convert mm to m
                x = (u - cx) * depth_m / fx
                y = (v - cy) * depth_m / fy
                z = depth_m
                
                points.append([x, y, z])
                
                # Get color (BGR to RGB)
                color = color_image[v, u]
                colors.append([color[2], color[1], color[0]])  # BGR to RGB
        
        return np.array(points), np.array(colors) / 255.0
    
    def create_open3d_pointcloud(self, points: np.ndarray, colors: np.ndarray) -> o3d.geometry.PointCloud:
        """
        Create Open3D point cloud object.
        
        Args:
            points: 3D points array
            colors: Color array
            
        Returns:
            Open3D PointCloud object
        """
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        return pcd
    
    def filter_pointcloud(self, pcd: o3d.geometry.PointCloud, 
                         voxel_size: float = 0.005,
                         nb_neighbors: int = 20,
                         std_ratio: float = 2.0) -> o3d.geometry.PointCloud:
        """
        Filter and clean point cloud.
        
        Args:
            pcd: Input point cloud
            voxel_size: Voxel size for downsampling
            nb_neighbors: Number of neighbors for outlier removal
            std_ratio: Standard deviation ratio for outlier removal
            
        Returns:
            Filtered point cloud
        """
        # Downsample
        pcd_filtered = pcd.voxel_down_sample(voxel_size)
        
        # Remove statistical outliers
        pcd_filtered, _ = pcd_filtered.remove_statistical_outlier(nb_neighbors, std_ratio)
        
        # Estimate normals
        pcd_filtered.estimate_normals()
        
        return pcd_filtered
    
    def crop_pointcloud_around_point(self, pcd: o3d.geometry.PointCloud, 
                                   center_point: np.ndarray, 
                                   radius: float = 0.1) -> o3d.geometry.PointCloud:
        """
        Crop point cloud around a center point.
        
        Args:
            pcd: Input point cloud
            center_point: Center point [x, y, z]
            radius: Cropping radius in meters
            
        Returns:
            Cropped point cloud
        """
        # Create bounding sphere
        center = center_point.astype(float)
        
        # Get points within radius
        points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors)
        
        distances = np.linalg.norm(points - center, axis=1)
        mask = distances <= radius
        
        # Create new point cloud
        cropped_pcd = o3d.geometry.PointCloud()
        cropped_pcd.points = o3d.utility.Vector3dVector(points[mask])
        cropped_pcd.colors = o3d.utility.Vector3dVector(colors[mask])
        
        return cropped_pcd
    
    def get_3d_point_from_2d(self, click_coords: Tuple[int, int], 
                           depth_image: np.ndarray, 
                           intrinsics: object) -> Optional[np.ndarray]:
        """
        Convert 2D click coordinates to 3D point.
        
        Args:
            click_coords: (u, v) pixel coordinates
            depth_image: Depth image
            intrinsics: Camera intrinsics
            
        Returns:
            3D point [x, y, z] or None if invalid
        """
        u, v = click_coords
        
        # Check bounds
        if u < 0 or u >= depth_image.shape[1] or v < 0 or v >= depth_image.shape[0]:
            return None
        
        depth = depth_image[v, u]
        if depth == 0:
            return None
        
        # Convert to 3D
        depth_m = depth / 1000.0  # mm to m
        fx, fy = intrinsics.fx, intrinsics.fy
        cx, cy = intrinsics.ppx, intrinsics.ppy
        
        x = (u - cx) * depth_m / fx
        y = (v - cy) * depth_m / fy
        z = depth_m
        
        return np.array([x, y, z])
    
    def visualize_pointcloud(self, pcd: o3d.geometry.PointCloud, 
                           window_name: str = "Point Cloud"):
        """
        Visualize point cloud using Open3D.
        
        Args:
            pcd: Point cloud to visualize
            window_name: Window title
        """
        print(f"📊 Visualizing point cloud with {len(pcd.points)} points")
        print("   Controls: Mouse to rotate, wheel to zoom, 'q' to close")
        
        o3d.visualization.draw_geometries(
            [pcd], 
            window_name=window_name,
            width=800, 
            height=600
        )
    
    def save_pointcloud(self, pcd: o3d.geometry.PointCloud, filepath: str):
        """
        Save point cloud to file.
        
        Args:
            pcd: Point cloud to save
            filepath: Output file path (.ply, .pcd, etc.)
        """
        try:
            o3d.io.write_point_cloud(filepath, pcd)
            print(f"✅ Point cloud saved to: {filepath}")
        except Exception as e:
            print(f"❌ Failed to save point cloud: {e}")
    
    def load_pointcloud(self, filepath: str) -> Optional[o3d.geometry.PointCloud]:
        """
        Load point cloud from file.
        
        Args:
            filepath: Input file path
            
        Returns:
            Loaded point cloud or None if failed
        """
        try:
            pcd = o3d.io.read_point_cloud(filepath)
            print(f"✅ Point cloud loaded: {len(pcd.points)} points")
            return pcd
        except Exception as e:
            print(f"❌ Failed to load point cloud: {e}")
            return None