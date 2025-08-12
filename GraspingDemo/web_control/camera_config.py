#!/usr/bin/env python3
"""
Camera configuration matching ThinkGrasp system
This ensures coordinate system compatibility for grasp pose generation
"""

class CameraConfig:
    """Camera configuration parameters - matching ThinkGrasp"""
    
    # Default RealSense D435 parameters (update these for your camera)
    WIDTH = 640
    HEIGHT = 480
    FX = 615.0  # Focal length x (typical for RealSense D435)
    FY = 615.0  # Focal length y
    CX = 320.0  # Principal point x (center of image)
    CY = 240.0  # Principal point y (center of image)
    SCALE = 1000.0  # Depth scale factor (depth in mm, scale to meters)
    
    # Alternative configuration for specific camera (commented out)
    # WIDTH = 640
    # HEIGHT = 480
    # FX = 382.8567  # Focal length x (from ThinkGrasp config)
    # FY = 382.4391  # Focal length y
    # CX = 331.3490  # Principal point x
    # CY = 247.1126  # Principal point y
    # SCALE = 1000.0  # Depth scale factor
    
    @classmethod
    def get_camera_info(cls):
        """Returns camera parameters in CameraInfo format compatible with ThinkGrasp"""
        return {
            'width': cls.WIDTH,
            'height': cls.HEIGHT,
            'fx': cls.FX,
            'fy': cls.FY,
            'cx': cls.CX,  # ppx in RealSense terms
            'cy': cls.CY,  # ppy in RealSense terms
            'scale': cls.SCALE
        }
    
    @classmethod
    def from_realsense_intrinsics(cls, intrinsics, depth_scale=0.001):
        """
        Update camera config from RealSense intrinsics
        
        Args:
            intrinsics: RealSense intrinsics object
            depth_scale: Depth scale from RealSense (typically 0.001 for mm to m)
        """
        cls.WIDTH = intrinsics.width
        cls.HEIGHT = intrinsics.height
        cls.FX = intrinsics.fx
        cls.FY = intrinsics.fy
        cls.CX = intrinsics.ppx
        cls.CY = intrinsics.ppy
        cls.SCALE = 1.0 / depth_scale  # Convert to ThinkGrasp scale format
    
    @classmethod
    def to_metadata(cls):
        """Convert to metadata format for saving"""
        return {
            'camera_intrinsics': {
                'width': cls.WIDTH,
                'height': cls.HEIGHT,
                'fx': cls.FX,
                'fy': cls.FY,
                'ppx': cls.CX,  # RealSense naming
                'ppy': cls.CY,  # RealSense naming
                'cx': cls.CX,   # Standard CV naming
                'cy': cls.CY,   # Standard CV naming
                'depth_scale': 1.0 / cls.SCALE  # Convert back to RealSense format
            }
        }