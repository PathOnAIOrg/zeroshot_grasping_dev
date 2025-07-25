"""
Simple coordinate transformation following ThinkGrasp conventions
"""

import numpy as np
import sys
import os
from unified_transform import UnifiedTransformer

# Add ThinkGrasp to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'ThinkGrasp'))

def process_ply_npy_simple(ply_path, npy_path):
    """
    Process PLY and NPY files with transformation consistent with ThinkGrasp
    Use unified transformer to ensure point cloud and grasp poses maintain their relative positions
    """
    transformer = UnifiedTransformer()
    return transformer.transform_point_cloud_and_grasps(ply_path, npy_path)