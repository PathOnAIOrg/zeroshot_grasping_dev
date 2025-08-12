#!/usr/bin/env python3
"""
Coordinate transformation utilities for camera to world/robot frame conversions
"""

import numpy as np
import json
from typing import Dict, List, Tuple, Optional

class CoordinateTransform:
    """
    Handles transformations between different coordinate frames:
    1. Image/Pixel coordinates (u, v) in pixels
    2. Camera coordinates (x, y, z) in meters
    3. World/Robot coordinates (X, Y, Z) in meters
    """
    
    def __init__(self, camera_intrinsics: Dict = None):
        """
        Initialize with camera intrinsics
        
        Args:
            camera_intrinsics: Dict containing fx, fy, cx, cy, width, height, depth_scale
        """
        self.camera_intrinsics = camera_intrinsics or {}
        
        # Camera to world transformation (default: identity)
        self.camera_to_world = np.eye(4)
        
        # Store transformation history
        self.transform_history = []
    
    def pixel_to_camera(self, u: float, v: float, depth: float) -> np.ndarray:
        """
        Convert pixel coordinates to camera coordinates
        
        Args:
            u: Pixel x-coordinate
            v: Pixel y-coordinate  
            depth: Depth value in millimeters
            
        Returns:
            3D point in camera frame [x, y, z] in meters
        """
        fx = self.camera_intrinsics.get('fx', 615.0)
        fy = self.camera_intrinsics.get('fy', 615.0)
        cx = self.camera_intrinsics.get('cx', 320.0)
        cy = self.camera_intrinsics.get('cy', 240.0)
        depth_scale = self.camera_intrinsics.get('depth_scale', 0.001)
        
        # Convert depth to meters
        z = depth * depth_scale
        
        # Apply pinhole camera model
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        
        return np.array([x, y, z])
    
    def camera_to_pixel(self, point_3d: np.ndarray) -> Tuple[int, int]:
        """
        Project 3D camera point back to pixel coordinates
        
        Args:
            point_3d: 3D point in camera frame [x, y, z] in meters
            
        Returns:
            Pixel coordinates (u, v)
        """
        fx = self.camera_intrinsics.get('fx', 615.0)
        fy = self.camera_intrinsics.get('fy', 615.0)
        cx = self.camera_intrinsics.get('cx', 320.0)
        cy = self.camera_intrinsics.get('cy', 240.0)
        
        x, y, z = point_3d
        
        # Project to pixel coordinates
        u = int(x * fx / z + cx)
        v = int(y * fy / z + cy)
        
        return u, v
    
    def set_camera_to_world_transform(self, rotation: np.ndarray = None, 
                                      translation: np.ndarray = None):
        """
        Set the transformation from camera to world/robot frame
        
        Args:
            rotation: 3x3 rotation matrix (default: identity)
            translation: 3D translation vector in meters (default: zero)
        """
        if rotation is None:
            rotation = np.eye(3)
        if translation is None:
            translation = np.zeros(3)
        
        self.camera_to_world = np.eye(4)
        self.camera_to_world[:3, :3] = rotation
        self.camera_to_world[:3, 3] = translation
        
        # Log transformation
        self.transform_history.append({
            'timestamp': np.datetime64('now'),
            'transform': self.camera_to_world.copy()
        })
    
    def transform_point_to_world(self, point_camera: np.ndarray) -> np.ndarray:
        """
        Transform point from camera frame to world frame
        
        Args:
            point_camera: 3D point in camera frame [x, y, z]
            
        Returns:
            3D point in world frame [X, Y, Z]
        """
        # Convert to homogeneous coordinates
        point_homo = np.append(point_camera, 1)
        
        # Apply transformation
        point_world_homo = self.camera_to_world @ point_homo
        
        # Return 3D coordinates
        return point_world_homo[:3]
    
    def get_intrinsic_matrix(self) -> np.ndarray:
        """
        Get the camera intrinsic matrix K
        
        Returns:
            3x3 intrinsic matrix
        """
        fx = self.camera_intrinsics.get('fx', 615.0)
        fy = self.camera_intrinsics.get('fy', 615.0)
        cx = self.camera_intrinsics.get('cx', 320.0)
        cy = self.camera_intrinsics.get('cy', 240.0)
        
        K = np.array([
            [fx, 0,  cx],
            [0,  fy, cy],
            [0,  0,  1]
        ])
        
        return K
    
    def get_projection_matrix(self) -> np.ndarray:
        """
        Get the full projection matrix P = K[R|t]
        
        Returns:
            3x4 projection matrix
        """
        K = self.get_intrinsic_matrix()
        RT = self.camera_to_world[:3, :]  # 3x4 [R|t]
        P = K @ RT
        
        return P
    
    def get_transform_info(self) -> Dict:
        """
        Get comprehensive transformation information for display
        
        Returns:
            Dict containing all transformation matrices and parameters
        """
        info = {
            'camera_intrinsics': {
                'fx': self.camera_intrinsics.get('fx', 615.0),
                'fy': self.camera_intrinsics.get('fy', 615.0),
                'cx': self.camera_intrinsics.get('cx', 320.0),
                'cy': self.camera_intrinsics.get('cy', 240.0),
                'width': self.camera_intrinsics.get('width', 640),
                'height': self.camera_intrinsics.get('height', 480),
                'depth_scale': self.camera_intrinsics.get('depth_scale', 0.001)
            },
            'intrinsic_matrix_K': self.get_intrinsic_matrix().tolist(),
            'camera_to_world_transform': self.camera_to_world.tolist(),
            'projection_matrix_P': self.get_projection_matrix().tolist(),
            'coordinate_frames': {
                'pixel': {
                    'description': 'Image coordinates',
                    'units': 'pixels',
                    'origin': 'Top-left corner',
                    'axes': 'u (right), v (down)'
                },
                'camera': {
                    'description': 'Camera 3D coordinates',
                    'units': 'meters',
                    'origin': 'Camera optical center',
                    'axes': 'x (right), y (down), z (forward)'
                },
                'world': {
                    'description': 'World/Robot coordinates',
                    'units': 'meters',
                    'origin': 'Robot base or calibration point',
                    'axes': 'X (robot x), Y (robot y), Z (robot z)'
                }
            },
            'transformation_chain': [
                'Pixel (u,v) + depth',
                '↓ (via intrinsic matrix K⁻¹)',
                'Camera (x,y,z)',
                '↓ (via extrinsic matrix [R|t])',
                'World (X,Y,Z)'
            ]
        }
        
        return info
    
    def visualize_coordinate_frames(self) -> Dict:
        """
        Generate visualization data for coordinate frames
        
        Returns:
            Dict with visualization data for web display
        """
        # Sample points in each coordinate frame
        sample_points = {
            'pixel_space': [
                {'name': 'Center', 'coords': [320, 240], 'color': '#FF0000'},
                {'name': 'Top-Left', 'coords': [0, 0], 'color': '#00FF00'},
                {'name': 'Bottom-Right', 'coords': [639, 479], 'color': '#0000FF'}
            ],
            'camera_space': [],
            'world_space': []
        }
        
        # Transform pixel points to camera and world space
        test_depth = 1000  # 1 meter
        
        for pixel_point in sample_points['pixel_space']:
            u, v = pixel_point['coords']
            
            # To camera space
            cam_point = self.pixel_to_camera(u, v, test_depth)
            sample_points['camera_space'].append({
                'name': pixel_point['name'],
                'coords': cam_point.tolist(),
                'color': pixel_point['color']
            })
            
            # To world space
            world_point = self.transform_point_to_world(cam_point)
            sample_points['world_space'].append({
                'name': pixel_point['name'],
                'coords': world_point.tolist(),
                'color': pixel_point['color']
            })
        
        # Generate axes for each frame
        axes_data = {
            'camera_frame': {
                'origin': [0, 0, 0],
                'x_axis': {'end': [0.1, 0, 0], 'color': 'red', 'label': 'X (right)'},
                'y_axis': {'end': [0, 0.1, 0], 'color': 'green', 'label': 'Y (down)'},
                'z_axis': {'end': [0, 0, 0.1], 'color': 'blue', 'label': 'Z (forward)'}
            },
            'world_frame': {
                'origin': self.camera_to_world[:3, 3].tolist(),
                'x_axis': {'end': (self.camera_to_world[:3, 3] + self.camera_to_world[:3, 0] * 0.1).tolist(), 
                          'color': 'red', 'label': 'X'},
                'y_axis': {'end': (self.camera_to_world[:3, 3] + self.camera_to_world[:3, 1] * 0.1).tolist(),
                          'color': 'green', 'label': 'Y'},
                'z_axis': {'end': (self.camera_to_world[:3, 3] + self.camera_to_world[:3, 2] * 0.1).tolist(),
                          'color': 'blue', 'label': 'Z'}
            }
        }
        
        return {
            'sample_points': sample_points,
            'axes': axes_data,
            'transform_matrix': self.camera_to_world.tolist()
        }


def example_robot_calibration():
    """
    Example of typical robot-camera calibration
    
    Returns:
        4x4 transformation matrix from camera to robot base
    """
    # Example: Camera mounted 30cm above and 20cm in front of robot base
    # with 45-degree downward tilt
    
    # Rotation: 45 degrees around X-axis (looking down)
    angle = np.radians(45)
    R = np.array([
        [1, 0, 0],
        [0, np.cos(angle), -np.sin(angle)],
        [0, np.sin(angle), np.cos(angle)]
    ])
    
    # Translation: Camera position relative to robot base
    t = np.array([0.0, 0.3, 0.2])  # 30cm up, 20cm forward
    
    # Build 4x4 transformation matrix
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    
    return T