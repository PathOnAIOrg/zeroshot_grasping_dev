"""
Point cloud reconstruction from depth and RGB images
"""

import numpy as np
import cv2
import open3d as o3d
from pathlib import Path
import json
from typing import Tuple, Optional, Dict, Any

class PointCloudReconstructor:
    """Point cloud reconstructor"""
    
    def __init__(self, camera_params: Optional[Dict[str, float]] = None):
        """
        Initialize reconstructor
        
        Args:
            camera_params: Camera parameters dictionary containing:
                - width: Image width
                - height: Image height
                - fx: Focal length x
                - fy: Focal length y
                - cx: Principal point x
                - cy: Principal point y
                - depth_scale: Depth scale factor (typically 1000)
        """
        if camera_params is None:
            # Default camera parameters (RealSense D435)
            self.camera_params = {
                'width': 640,
                'height': 480,
                'fx': 383.9592,
                'fy': 383.6245,
                'cx': 322.1625,
                'cy': 245.3161,
                'depth_scale': 1000.0
            }
        else:
            self.camera_params = camera_params
            
    def depth_to_pointcloud(self, depth_image: np.ndarray, rgb_image: np.ndarray) -> o3d.geometry.PointCloud:
        """
        Reconstruct point cloud from depth and RGB images
        
        Args:
            depth_image: Depth image (H, W) uint16 or float32
            rgb_image: RGB image (H, W, 3) uint8
            
        Returns:
            Open3D point cloud object
        """
        # Ensure image dimensions match
        if depth_image.shape[:2] != rgb_image.shape[:2]:
            print(f"Resizing RGB image from {rgb_image.shape[:2]} to {depth_image.shape[:2]}")
            rgb_image = cv2.resize(rgb_image, (depth_image.shape[1], depth_image.shape[0]))
        
        height, width = depth_image.shape
        
        # Camera intrinsics
        fx = self.camera_params['fx']
        fy = self.camera_params['fy']
        cx = self.camera_params['cx']
        cy = self.camera_params['cy']
        depth_scale = self.camera_params['depth_scale']
        
        # Create pixel coordinate grid
        xx, yy = np.meshgrid(np.arange(width), np.arange(height))
        
        # Convert depth values to meters
        if depth_image.dtype == np.uint16:
            z = depth_image.astype(np.float32) / depth_scale
        else:
            z = depth_image
        
        # Calculate 3D coordinates (camera coordinate system)
        x = (xx - cx) * z / fx
        y = (yy - cy) * z / fy
        
        # Create point cloud
        points = np.stack([x, y, z], axis=-1)
        colors = rgb_image.astype(np.float32) / 255.0
        
        # Filter invalid points
        valid_mask = z > 0
        points_valid = points[valid_mask]
        colors_valid = colors[valid_mask]
        
        # Create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_valid)
        pcd.colors = o3d.utility.Vector3dVector(colors_valid)
        
        return pcd
    
    def reconstruct_with_grasps(self, depth_path: str, rgb_path: str, 
                               grasp_poses: list, output_dir: str = "./output") -> Tuple[o3d.geometry.PointCloud, np.ndarray]:
        """
        Reconstruct point cloud and process grasp poses
        
        Args:
            depth_path: Depth image path
            rgb_path: RGB image path
            grasp_poses: Grasp poses list (JSON format)
            output_dir: Output directory
            
        Returns:
            Point cloud and grasp array
        """
        # Create output directory
        output_dir = Path(output_dir)
        output_dir.mkdir(exist_ok=True)
        
        # Read images
        print("Loading images...")
        depth_image = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
        rgb_image = cv2.imread(rgb_path)
        rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
        
        print(f"Depth image shape: {depth_image.shape}, dtype: {depth_image.dtype}")
        print(f"RGB image shape: {rgb_image.shape}")
        
        # Reconstruct point cloud
        print("Reconstructing point cloud...")
        pcd = self.depth_to_pointcloud(depth_image, rgb_image)
        print(f"Point cloud: {len(pcd.points)} points")
        
        # Convert grasp poses to numpy array (GraspGroup format)
        grasp_array = self.convert_grasps_to_array(grasp_poses)
        
        # Save results
        ply_path = output_dir / "reconstructed_cloud.ply"
        npy_path = output_dir / "grasp_poses.npy"
        
        o3d.io.write_point_cloud(str(ply_path), pcd)
        np.save(str(npy_path), grasp_array)
        
        print(f"\nSaved files:")
        print(f"  Point cloud: {ply_path}")
        print(f"  Grasp array: {npy_path}")
        
        # Verify alignment
        self.verify_alignment(pcd, grasp_array)
        
        return pcd, grasp_array
    
    def convert_grasps_to_array(self, grasp_poses: list) -> np.ndarray:
        """
        Convert JSON format grasp poses to GraspGroup array format
        
        Args:
            grasp_poses: List of dictionaries containing 'xyz', 'rot', 'width', 'dep', 'score'
            
        Returns:
            numpy array (N, 17)
        """
        grasp_list = []
        
        for i, grasp in enumerate(grasp_poses):
            # Create 17-dimensional array
            grasp_array = np.zeros(17)
            
            # Fill data
            grasp_array[0] = grasp.get('score', 0.5)
            grasp_array[1] = grasp.get('width', 0.08)
            grasp_array[2] = 0  # height (unused)
            grasp_array[3] = grasp.get('dep', 0.02)
            
            # Rotation matrix (3x3 -> 9 dimensions)
            rot = np.array(grasp['rot'])
            grasp_array[4:13] = rot.flatten()
            
            # Position
            grasp_array[13:16] = grasp['xyz']
            
            # object_id
            grasp_array[16] = 0
            
            grasp_list.append(grasp_array)
            
            if i < 3:  # Print first few
                print(f"\nGrasp {i+1}:")
                print(f"  Position: {grasp['xyz']}")
                print(f"  Score: {grasp.get('score', 0.5):.3f}")
        
        return np.array(grasp_list)
    
    def verify_alignment(self, pcd: o3d.geometry.PointCloud, grasp_array: np.ndarray):
        """Verify alignment of point cloud and grasps"""
        points = np.asarray(pcd.points)
        
        print("\n=== Alignment Verification ===")
        print(f"Point cloud bounds:")
        print(f"  X: [{points[:, 0].min():.3f}, {points[:, 0].max():.3f}]")
        print(f"  Y: [{points[:, 1].min():.3f}, {points[:, 1].max():.3f}]")
        print(f"  Z: [{points[:, 2].min():.3f}, {points[:, 2].max():.3f}]")
        
        # Create KD tree for nearest neighbor search
        pcd_tree = o3d.geometry.KDTreeFlann(pcd)
        
        print(f"\nGrasp positions (first 5):")
        for i in range(min(5, len(grasp_array))):
            translation = grasp_array[i][13:16]
            
            # Check if within bounds
            in_bounds = (
                points[:, 0].min() <= translation[0] <= points[:, 0].max() and
                points[:, 1].min() <= translation[1] <= points[:, 1].max() and
                points[:, 2].min() <= translation[2] <= points[:, 2].max()
            )
            
            # Find nearest point
            [k, idx, _] = pcd_tree.search_knn_vector_3d(translation, 1)
            if k > 0:
                nearest_point = points[idx[0]]
                distance = np.linalg.norm(translation - nearest_point)
            else:
                distance = float('inf')
            
            print(f"  Grasp {i+1}: {translation}")
            print(f"    Within bounds: {in_bounds}")
            print(f"    Nearest point distance: {distance:.3f}m")
    
    def visualize_reconstruction(self, pcd: o3d.geometry.PointCloud, grasp_array: np.ndarray):
        """Visualize reconstruction results"""
        vis_list = [pcd]
        
        # Add grasp visualization
        for i, grasp in enumerate(grasp_array):
            if len(grasp) >= 17:
                translation = grasp[13:16]
                rotation = grasp[4:13].reshape(3, 3)
                
                # Coordinate axes
                axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
                transform = np.eye(4)
                transform[:3, :3] = rotation
                transform[:3, 3] = translation
                axes.transform(transform)
                vis_list.append(axes)
                
                # Center sphere
                sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
                sphere.paint_uniform_color([1, 0, 0])
                sphere.translate(translation)
                vis_list.append(sphere)
        
        print("\nShowing visualization...")
        o3d.visualization.draw_geometries(vis_list, window_name="Reconstructed Point Cloud with Grasps")

def reconstruct_from_files(depth_path: str, rgb_path: str, poses_json_path: str, 
                          camera_params: Optional[Dict] = None, visualize: bool = True):
    """
    Convenience function: reconstruct point cloud from files
    
    Args:
        depth_path: Depth image path
        rgb_path: RGB image path
        poses_json_path: Grasp poses JSON file path
        camera_params: Camera parameters (optional)
        visualize: Whether to visualize results
    """
    # Read grasp poses
    with open(poses_json_path, 'r') as f:
        data = json.load(f)
        grasp_poses = data.get('grasp_poses', data)  # Support different formats
    
    # Create reconstructor
    reconstructor = PointCloudReconstructor(camera_params)
    
    # Reconstruct
    pcd, grasp_array = reconstructor.reconstruct_with_grasps(
        depth_path, rgb_path, grasp_poses
    )
    
    # Visualize
    if visualize:
        reconstructor.visualize_reconstruction(pcd, grasp_array)
    
    return pcd, grasp_array

