"""
Grasp prediction interface for custom implementation.

This file defines the interfaces you need to implement to replace the GeneralBionixClient API.
Each method shows the expected input/output format and functionality.
"""

from typing import List, Tuple, Optional
from dataclasses import dataclass
import numpy as np
import open3d as o3d
from abc import ABC, abstractmethod
import requests
import json
import tempfile
import os
from PIL import Image


@dataclass
class Grasp:
    """
    A 6D grasp pose representation.
    
    Attributes:
        rotation: 3x3 rotation matrix representing gripper orientation
        translation: 3D translation vector representing gripper position
    """
    rotation: np.ndarray  # Shape: (3, 3)
    translation: np.ndarray  # Shape: (3,)
    
    def to_list_format(self):
        """Convert to list format for compatibility."""
        return {
            "rotation": self.rotation.tolist() if hasattr(self.rotation, 'tolist') else self.rotation,
            "translation": self.translation.tolist() if hasattr(self.translation, 'tolist') else self.translation
        }
    
    @classmethod
    def from_list_format(cls, data):
        """Create from list format."""
        return cls(
            rotation=np.array(data["rotation"]),
            translation=np.array(data["translation"])
        )


class GraspPredictor(ABC):
    """
    Abstract base class for grasp prediction.
    Implement this class with your own grasp prediction algorithm.
    """
    
    @abstractmethod
    def segment_object(self, 
                      pointcloud: o3d.geometry.PointCloud, 
                      click_point: Tuple[int, int],
                      image_shape: Tuple[int, int] = (480, 640)) -> o3d.geometry.PointCloud:
        """
        Segment an object from the scene based on a 2D click point.
        
        Args:
            pointcloud: Full scene point cloud (organized as image - 480x640)
            click_point: (x, y) pixel coordinates where user clicked
            image_shape: Shape of the depth image used to create pointcloud
            
        Returns:
            Segmented point cloud containing only the selected object
            
        Example implementation approaches:
        - Use depth-based region growing from click point
        - Use color-based segmentation
        - Use geometric clustering (DBSCAN, Euclidean clustering)
        - Use pre-trained segmentation models (SAM, etc.)
        """
        pass
    
    @abstractmethod
    def predict_grasps(self, 
                      object_pointcloud: o3d.geometry.PointCloud,
                      num_grasps: int = 10) -> List[Grasp]:
        """
        Predict possible grasp poses for an object.
        
        Args:
            object_pointcloud: Segmented point cloud of the target object
            num_grasps: Maximum number of grasps to return
            
        Returns:
            List of grasp poses in CAMERA coordinate frame
            
        Example implementation approaches:
        - Geometric analysis (find antipodal points)
        - Deep learning models (GraspNet, Contact-GraspNet, etc.)
        - Heuristic methods (principal component analysis)
        - GPG (Grasp Pose Generator) algorithm
        
        Note: Returned grasps should be in the camera coordinate frame.
        They will be transformed to robot frame by the system.
        """
        pass
    
    @abstractmethod
    def filter_reachable_grasps(self, 
                               grasps: List[Grasp],
                               robot_name: str = "so101") -> Tuple[List[int], List[List[float]]]:
        """
        Filter grasps based on robot reachability and kinematics.
        
        Args:
            grasps: List of grasp poses in ROBOT coordinate frame
            robot_name: Name of the robot (for loading correct kinematics)
            
        Returns:
            Tuple of:
            - List of indices of reachable grasps
            - List of corresponding joint angles for each reachable grasp
              (each element is a list of 6 joint values in radians)
            
        Example implementation approaches:
        - Use inverse kinematics to check reachability
        - Check joint limits and singularities
        - Collision checking with robot model
        - Workspace analysis
        
        Note: Input grasps are already transformed to robot base frame.
        """
        pass


class SimpleGraspPredictor(GraspPredictor):
    """
    A simple example implementation of the GraspPredictor interface.
    Replace this with your actual implementation.
    """
    
    def segment_object(self, 
                      pointcloud: o3d.geometry.PointCloud, 
                      click_point: Tuple[int, int],
                      image_shape: Tuple[int, int] = (480, 640)) -> o3d.geometry.PointCloud:
        """
        Simple segmentation using depth-based region growing.
        """
        points = np.asarray(pointcloud.points)
        colors = np.asarray(pointcloud.colors)
        
        # Reshape to image format
        points_img = points.reshape(image_shape[0], image_shape[1], 3)
        colors_img = colors.reshape(image_shape[0], image_shape[1], 3)
        
        # Get clicked point
        click_y, click_x = click_point
        click_y = min(max(0, click_y), image_shape[0]-1)
        click_x = min(max(0, click_x), image_shape[1]-1)
        
        # Get depth at click point
        click_depth = points_img[click_y, click_x, 2]
        
        # Create mask for points within depth range
        depth_img = points_img[:, :, 2]
        depth_threshold = 0.05  # 5cm depth tolerance
        
        # Region growing from click point
        mask = np.zeros(image_shape, dtype=bool)
        visited = np.zeros(image_shape, dtype=bool)
        
        # Simple flood fill based on depth similarity
        stack = [(click_y, click_x)]
        mask[click_y, click_x] = True
        
        while stack:
            y, x = stack.pop()
            if visited[y, x]:
                continue
            visited[y, x] = True
            
            # Check 4-connected neighbors
            for dy, dx in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                ny, nx = y + dy, x + dx
                if 0 <= ny < image_shape[0] and 0 <= nx < image_shape[1]:
                    if not visited[ny, nx] and depth_img[ny, nx] > 0:
                        depth_diff = abs(depth_img[ny, nx] - click_depth)
                        if depth_diff < depth_threshold:
                            mask[ny, nx] = True
                            stack.append((ny, nx))
        
        # Apply morphological operations to clean up
        from scipy import ndimage
        mask = ndimage.binary_erosion(mask, iterations=2)
        mask = ndimage.binary_dilation(mask, iterations=3)
        
        # Create segmented pointcloud
        mask_flat = mask.flatten()
        segmented_points = points[mask_flat]
        segmented_colors = colors[mask_flat]
        
        # Remove outliers
        if len(segmented_points) > 10:
            segmented_pcd = o3d.geometry.PointCloud()
            segmented_pcd.points = o3d.utility.Vector3dVector(segmented_points)
            segmented_pcd.colors = o3d.utility.Vector3dVector(segmented_colors)
            
            # Remove statistical outliers
            segmented_pcd, _ = segmented_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        else:
            # Fallback: return a cube of points around click
            center = points_img[click_y, click_x]
            distances = np.linalg.norm(points - center, axis=1)
            mask = distances < 0.05
            segmented_pcd = o3d.geometry.PointCloud()
            segmented_pcd.points = o3d.utility.Vector3dVector(points[mask])
            segmented_pcd.colors = o3d.utility.Vector3dVector(colors[mask])
        
        return segmented_pcd
    
    def predict_grasps(self, 
                      object_pointcloud: o3d.geometry.PointCloud,
                      num_grasps: int = 10) -> List[Grasp]:
        """
        Generate multiple grasp poses using PCA and sampling strategies.
        """
        points = np.asarray(object_pointcloud.points)
        
        if len(points) < 10:
            # Not enough points, return default grasp
            return [Grasp(
                rotation=np.eye(3),
                translation=np.array([0.3, 0.0, 0.1])
            )]
        
        # Compute centroid and principal axes using PCA
        centroid = np.mean(points, axis=0)
        
        # PCA for orientation
        points_centered = points - centroid
        
        # Check if points are coplanar (2D)
        cov_matrix = np.cov(points_centered.T)
        eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)
        
        # Sort by eigenvalues
        idx = eigenvalues.argsort()[::-1]
        eigenvalues = eigenvalues[idx]
        eigenvectors = eigenvectors[:, idx]
        
        # Use bounding box extents
        extent = np.zeros(3)
        for i in range(3):
            if eigenvalues[i] > 1e-6:
                projected = points_centered @ eigenvectors[:, i]
                extent[i] = np.max(projected) - np.min(projected)
            else:
                extent[i] = 0.01  # Small default extent
        
        center = centroid
        R_obb = eigenvectors
        
        # Sort axes by extent (largest to smallest)
        axis_order = np.argsort(extent)[::-1]
        
        grasps = []
        
        # Strategy 1: Grasp along smallest dimension (precision grasp)
        for angle in [0, np.pi/2, np.pi, 3*np.pi/2]:
            # Approach along largest axis
            approach = R_obb[:, axis_order[0]]
            
            # Gripper closing direction along smallest axis
            closing = R_obb[:, axis_order[2]]
            
            # Rotate around approach axis
            rot_angle = angle
            c, s = np.cos(rot_angle), np.sin(rot_angle)
            rot_mat = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            
            # Build grasp orientation
            binormal = R_obb[:, axis_order[1]]
            
            # Apply rotation
            closing_rot = rot_mat[0, 0] * closing + rot_mat[0, 1] * binormal
            binormal_rot = rot_mat[1, 0] * closing + rot_mat[1, 1] * binormal
            
            # Grasp rotation matrix
            rotation = np.column_stack([closing_rot, binormal_rot, approach])
            
            # Grasp position (approach from outside the object)
            offset = 0.1  # 10cm approach distance
            translation = center - approach * (extent[axis_order[0]]/2 + offset)
            
            grasps.append(Grasp(rotation=rotation, translation=translation))
            
            if len(grasps) >= num_grasps:
                break
        
        # Strategy 2: Top-down grasps
        for i in range(min(4, num_grasps - len(grasps))):
            angle = i * np.pi / 2
            
            # Top-down approach
            approach = np.array([0, 0, -1])
            
            # Random orientation around Z
            c, s = np.cos(angle), np.sin(angle)
            closing = np.array([c, s, 0])
            binormal = np.array([-s, c, 0])
            
            rotation = np.column_stack([closing, binormal, approach])
            
            # Position above object
            translation = center + np.array([0, 0, extent[2]/2 + 0.1])
            
            grasps.append(Grasp(rotation=rotation, translation=translation))
            
            if len(grasps) >= num_grasps:
                break
        
        # Strategy 3: Side grasps
        for i in range(min(2, num_grasps - len(grasps))):
            # Side approach
            angle = i * np.pi
            approach = np.array([np.cos(angle), np.sin(angle), 0])
            closing = np.array([0, 0, 1])
            binormal = np.cross(approach, closing)
            
            rotation = np.column_stack([closing, binormal, approach])
            translation = center - approach * 0.15
            
            grasps.append(Grasp(rotation=rotation, translation=translation))
        
        return grasps[:num_grasps]
    
    def filter_reachable_grasps(self, 
                               grasps: List[Grasp],
                               robot_name: str = "so101") -> Tuple[List[int], List[List[float]]]:
        """
        Filter grasps based on workspace limits and provide approximate joint angles.
        """
        valid_indices = []
        valid_joint_angles = []
        
        # Define workspace limits based on robot type
        if robot_name == "so101":
            # SO-101 approximate workspace
            min_reach = 0.05
            max_reach = 0.4
            min_height = -0.1
            max_height = 0.4
        else:  # other robots
            min_reach = 0.1
            max_reach = 0.5
            min_height = 0.0
            max_height = 0.5
        
        for i, grasp in enumerate(grasps):
            pos = grasp.translation
            
            # Check if position is within workspace
            xy_distance = np.linalg.norm(pos[:2])
            
            if (min_reach < xy_distance < max_reach and 
                min_height < pos[2] < max_height):
                
                # Additional checks for grasp orientation
                # Prefer grasps where approach vector is not too horizontal
                approach_vec = grasp.rotation[:, 2]  # Z-axis of gripper
                vertical_component = abs(approach_vec[2])
                
                # Accept grasps that are not too horizontal
                if vertical_component > 0.3 or xy_distance < 0.3:
                    valid_indices.append(i)
                    
                    # Generate approximate joint angles
                    # This is a simplified inverse kinematics approximation
                    # Real implementation should use proper IK solver
                    
                    # Base rotation to point at target
                    base_angle = np.arctan2(pos[1], pos[0])
                    
                    # Approximate arm extension based on distance
                    distance = np.linalg.norm(pos)
                    shoulder_angle = -np.pi/4 + (distance - min_reach) / (max_reach - min_reach) * np.pi/3
                    elbow_angle = np.pi/3 - (distance - min_reach) / (max_reach - min_reach) * np.pi/4
                    
                    # Wrist orientation to match grasp
                    wrist_pitch = np.arctan2(-approach_vec[2], np.linalg.norm(approach_vec[:2]))
                    wrist_roll = 0.0  # Simplified
                    
                    # Generate joint configuration
                    joints = [
                        base_angle,          # Joint 0: Base rotation
                        shoulder_angle,      # Joint 1: Shoulder
                        elbow_angle,         # Joint 2: Elbow  
                        wrist_pitch,         # Joint 3: Wrist pitch
                        wrist_roll,          # Joint 4: Wrist roll
                        0.0                  # Joint 5: Gripper
                    ]
                    
                    valid_joint_angles.append(joints)
        
        # If no valid grasps found, return at least one with default pose
        if not valid_indices and grasps:
            valid_indices.append(0)
            valid_joint_angles.append([0.0, -0.5, 0.8, -0.3, 0.0, 0.0])
        
        return valid_indices, valid_joint_angles


class ThinkGraspPredictor(GraspPredictor):
    """
    ThinkGrasp implementation using the realarm310.py API.
    This predictor takes image, depth image, and goal text to predict grasps.
    """
    
    def __init__(self, api_url: str = "http://localhost:5000"):
        self.api_url = api_url
    
    def predict_grasp_from_images(self, 
                                  rgb_image: np.ndarray,
                                  depth_image: np.ndarray, 
                                  goal_text: str) -> Grasp:
        """
        Main API method that takes RGB image, depth image, and goal text to predict a grasp pose.
        
        Args:
            rgb_image: RGB image as numpy array (H, W, 3)
            depth_image: Depth image as numpy array (H, W)
            goal_text: Natural language description of the grasping goal
            
        Returns:
            Grasp: Single best grasp pose in camera coordinate frame
        """
        # Save images to temporary files
        with tempfile.TemporaryDirectory() as temp_dir:
            # Save RGB image
            rgb_path = os.path.join(temp_dir, "rgb.png")
            rgb_pil = Image.fromarray(rgb_image.astype(np.uint8))
            rgb_pil.save(rgb_path)
            
            # Save depth image
            depth_path = os.path.join(temp_dir, "depth.png")
            depth_pil = Image.fromarray(depth_image.astype(np.uint16))
            depth_pil.save(depth_path)
            
            # Save goal text
            text_path = os.path.join(temp_dir, "goal.txt")
            with open(text_path, 'w') as f:
                f.write(goal_text)
            
            # Make API request
            try:
                print("   📤 Sending request to ThinkGrasp API...")
                response = requests.post(
                    f"{self.api_url}/grasp_pose",
                    json={
                        "image_path": rgb_path,
                        "depth_path": depth_path,
                        "text_path": text_path
                    },
                    timeout=60  # Increased timeout for complex processing
                )
                
                if response.status_code == 200:
                    print("   📥 Received response from API, processing...")
                    result = response.json()
                    
                    # Extract pose from API response
                    xyz = np.array(result['xyz'])  # [x, y, z]
                    rotation_matrix = np.array(result['rot'])  # [3, 3] rotation matrix
                    
                    # Validate the response format
                    if xyz.shape != (3,):
                        raise ValueError(f"Invalid xyz shape: {xyz.shape}, expected (3,)")
                    if rotation_matrix.shape != (3, 3):
                        raise ValueError(f"Invalid rotation matrix shape: {rotation_matrix.shape}, expected (3, 3)")
                    
                    print("   ✅ Successfully parsed API response")
                    return Grasp(
                        rotation=rotation_matrix,
                        translation=xyz
                    )
                else:
                    print(f"   ❌ API request failed with status {response.status_code}")
                    if response.text:
                        print(f"   Error details: {response.text[:200]}")
                    return self._fallback_grasp()
                    
            except requests.exceptions.Timeout:
                print("   ⏰ API request timed out - your ThinkGrasp processing may be taking longer than expected")
                return self._fallback_grasp()
            except requests.exceptions.ConnectionError:
                print("   🔌 Could not connect to ThinkGrasp API - make sure realarm310.py server is running")
                return self._fallback_grasp()
            except Exception as e:
                print(f"   ❌ Error calling ThinkGrasp API: {e}")
                return self._fallback_grasp()
    
    def _fallback_grasp(self) -> Grasp:
        """Return a default grasp when API fails."""
        return Grasp(
            rotation=np.eye(3),
            translation=np.array([0.3, 0.0, 0.1])
        )
    
    def segment_object(self, 
                      pointcloud: o3d.geometry.PointCloud, 
                      click_point: Tuple[int, int],
                      image_shape: Tuple[int, int] = (480, 640)) -> o3d.geometry.PointCloud:
        """
        For ThinkGrasp, segmentation is handled internally by the API.
        This method returns the full point cloud since segmentation is done via language.
        """
        return pointcloud
    
    def predict_grasps(self, 
                      object_pointcloud: o3d.geometry.PointCloud,
                      num_grasps: int = 10) -> List[Grasp]:
        """
        For ThinkGrasp, this method is not used directly.
        Use predict_grasp_from_images instead.
        """
        # Return fallback grasp
        return [self._fallback_grasp()]
    
    def filter_reachable_grasps(self, 
                               grasps: List[Grasp],
                               robot_name: str = "so101") -> Tuple[List[int], List[List[float]]]:
        """
        Simple reachability filter based on workspace limits.
        """
        valid_indices = []
        valid_joint_angles = []
        
        # Define workspace limits based on robot type
        if robot_name == "so101":
            min_reach = 0.05
            max_reach = 0.4
            min_height = -0.1
            max_height = 0.4
        else:
            min_reach = 0.1
            max_reach = 0.5
            min_height = 0.0
            max_height = 0.5
        
        for i, grasp in enumerate(grasps):
            pos = grasp.translation
            xy_distance = np.linalg.norm(pos[:2])
            
            if (min_reach < xy_distance < max_reach and 
                min_height < pos[2] < max_height):
                
                valid_indices.append(i)
                
                # Generate approximate joint angles
                base_angle = np.arctan2(pos[1], pos[0])
                distance = np.linalg.norm(pos)
                shoulder_angle = -np.pi/4 + (distance - min_reach) / (max_reach - min_reach) * np.pi/3
                elbow_angle = np.pi/3 - (distance - min_reach) / (max_reach - min_reach) * np.pi/4
                
                joints = [base_angle, shoulder_angle, elbow_angle, 0.0, 0.0, 0.0]
                valid_joint_angles.append(joints)
        
        # If no valid grasps found, return at least one with default pose
        if not valid_indices and grasps:
            valid_indices.append(0)
            valid_joint_angles.append([0.0, -0.5, 0.8, -0.3, 0.0, 0.0])
        
        return valid_indices, valid_joint_angles


# Create a global instance that can be replaced with custom implementation
# Use ThinkGraspPredictor as the default implementation
grasp_predictor = ThinkGraspPredictor()