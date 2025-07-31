import numpy as np
from typing import Tuple
import open3d as o3d


def compute_mask_center_of_mass(mask: np.ndarray) -> Tuple[int, int]:
    """
    Compute the center of mass of a binary segmentation mask.

    Args:
        mask (np.ndarray): A binary mask of shape (H, W) where pixels belonging to 
                          the object are 1 (or True) and background pixels are 0 (or False).

    Returns:
        tuple: (x, y) coordinates of the center of mass in pixel coordinates.
    """
    # Ensure the mask is a binary mask
    if mask.dtype != np.bool_:
        mask = mask.astype(bool)

    # Get the indices of the pixels that are part of the mask
    y_indices, x_indices = np.nonzero(mask)  # y_indices and x_indices are the coordinates of the true pixels

    # Calculate the total number of pixels in the mask
    total_pixels = len(x_indices)

    # If there are no pixels in the mask, return None or appropriate value
    if total_pixels == 0:
        return None

    # Compute the center of mass
    center_x = np.sum(x_indices) / total_pixels
    center_y = np.sum(y_indices) / total_pixels

    return int(center_x), int(center_y)



def downsample_pcd(pcd: o3d.geometry.PointCloud, down_sample: int) -> o3d.geometry.PointCloud:
    """"
    Uniformly downsample the point cloud for network transfer.
    Args:
        pcd: o3d.geometry.PointCloud: Orignal point cloud
        down_sample: int
    Returns:
        o3d.geometry.PointCloud: Downsampled point cloud.
    """
    pcd_ds = pcd.uniform_down_sample(down_sample) # Downsample for faster processing
    return pcd_ds


def upsample_pcd(pcd_ds: o3d.geometry.PointCloud, pcd_full: o3d.geometry.PointCloud, up_sample: int, real_robot=False) -> o3d.geometry.PointCloud:
    """
    Upsample the downsampled point cloud based on the original pointcloud using nearest neighbor search.
    Args:
        pcd_ds: o3d.geometry.PointCloud: Downsampled point cloud.
        pcd_full: o3d.geometry.PointCloud: Orignal point cloud.
        real_robot: bool: Whether the robot is real or simulated.
        up_sample: int: Upsampling factor.
    Returns:
        pcd_us: o3d.geometry.PointCloud: Upsampled point cloud.
    """
    k = up_sample 
    voxel = pcd_full.get_max_bound() - pcd_full.get_min_bound()
    avg_spacing = (np.linalg.norm(voxel) / np.cbrt(len(pcd_full.points)))  # ~mean NN spacing
    if real_robot:
        radius = k * 0.05 * avg_spacing / 40      # anything inside this radius was probably dropped
    else:
        radius = k * 0.05 * avg_spacing       # anything inside this radius was probably dropped
    kdtree_full = o3d.geometry.KDTreeFlann(pcd_full)
    hits = set()
    for q in pcd_ds.points:                 # each “query” point
        _, idxs, _ = kdtree_full.search_radius_vector_3d(q, radius)
        hits.update(idxs)                       # collect everything nearby
    pcd_us = pcd_full.select_by_index(list(hits))
    return pcd_us


def get_3d_point_from_2d_coordinates(click_coords: Tuple[int, int], 
                                   depth_image: np.ndarray, 
                                   intrinsics: object) -> Tuple[float, float, float]:
    """
    Convert 2D click coordinates to 3D point using depth image and camera intrinsics.
    
    Args:
        click_coords: (x, y) pixel coordinates
        depth_image: Depth image array
        intrinsics: Camera intrinsics object
        
    Returns:
        Tuple[float, float, float]: 3D point [x, y, z] or None if invalid
    """
    x, y = click_coords
    
    print(f"Debug: Click coordinates: ({x}, {y})")
    print(f"Debug: Depth image shape: {depth_image.shape}")
    print(f"Debug: Depth image dtype: {depth_image.dtype}")
    
    # Check bounds
    if x < 0 or x >= depth_image.shape[1] or y < 0 or y >= depth_image.shape[0]:
        print(f"Debug: Coordinates out of bounds")
        return None
    
    depth = depth_image[y, x]
    print(f"Debug: Raw depth value at ({x}, {y}): {depth}")
    
    if depth == 0:
        print(f"Debug: Zero depth at clicked point, trying nearby pixels")
        # Try a small neighborhood around the clicked point
        for dy in range(-2, 3):
            for dx in range(-2, 3):
                ny, nx = y + dy, x + dx
                if (0 <= ny < depth_image.shape[0] and 
                    0 <= nx < depth_image.shape[1]):
                    neighbor_depth = depth_image[ny, nx]
                    if neighbor_depth > 0:
                        depth = neighbor_depth
                        print(f"Debug: Using nearby depth at ({nx}, {ny}): {depth}")
                        break
            if depth > 0:
                break
        
        if depth == 0:
            print(f"Debug: No valid depth found in neighborhood")
            return None
    
    # Convert to 3D using camera intrinsics
    depth_m = depth / 1000.0  # Convert mm to m
    fx, fy = intrinsics.fx, intrinsics.fy
    cx, cy = intrinsics.ppx, intrinsics.ppy
    
    print(f"Debug: Intrinsics fx={fx}, fy={fy}, cx={cx}, cy={cy}")
    print(f"Debug: Depth in meters: {depth_m}")
    
    x_3d = (x - cx) * depth_m / fx
    y_3d = (y - cy) * depth_m / fy
    z_3d = depth_m
    
    print(f"Debug: 3D point: ({x_3d:.4f}, {y_3d:.4f}, {z_3d:.4f})")
    
    return (x_3d, y_3d, z_3d)
