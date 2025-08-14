#!/usr/bin/env python3
"""
Utilities to ensure compatibility with ThinkGrasp coordinate system
"""

import numpy as np
import open3d as o3d
from camera_config import CameraConfig

def create_point_cloud_from_depth_image(depth, camera=None, organized=True):
    """
    Generate point cloud using depth image only.
    EXACTLY matching ThinkGrasp's implementation for coordinate compatibility.
    
    Input:
        depth: [numpy.ndarray, (H,W), numpy.float32/uint16]
            depth image in millimeters
        camera: [dict or CameraInfo]
            camera intrinsics, if None uses CameraConfig defaults
        organized: bool
            whether to keep the cloud in image shape (H,W,3)
    
    Output:
        cloud: [numpy.ndarray, (H,W,3)/(H*W,3), numpy.float32]
            generated cloud in meters, (H,W,3) for organized=True, (H*W,3) for organized=False
    """
    
    # Get camera parameters
    if camera is None:
        camera = CameraConfig.get_camera_info()
    elif isinstance(camera, dict):
        # Already in correct format
        pass
    else:
        # Assume it's a CameraInfo object
        camera = {
            'width': camera.width,
            'height': camera.height,
            'fx': camera.fx,
            'fy': camera.fy,
            'cx': camera.cx,
            'cy': camera.cy,
            'scale': camera.scale
        }
    
    assert(depth.shape[0] == camera['height'] and depth.shape[1] == camera['width'])
    
    # Create pixel coordinate grids (EXACTLY as ThinkGrasp)
    xmap = np.arange(camera['width'])
    ymap = np.arange(camera['height'])
    xmap, ymap = np.meshgrid(xmap, ymap)
    
    # Convert depth to meters (depth is in mm, scale is typically 1000)
    points_z = depth / camera['scale']
    
    # Back-project to 3D (EXACTLY as ThinkGrasp)
    points_x = (xmap - camera['cx']) * points_z / camera['fx']
    points_y = (ymap - camera['cy']) * points_z / camera['fy']
    
    # Stack into point cloud
    cloud = np.stack([points_x, points_y, points_z], axis=-1)
    
    if not organized:
        cloud = cloud.reshape([-1, 3])
    
    return cloud


def transform_to_thinkgrasp_frame(vertices, from_cm_to_m=True):
    """
    Transform point cloud to ThinkGrasp coordinate frame
    
    Args:
        vertices: numpy array of 3D points
        from_cm_to_m: if True, converts from centimeters to meters
    
    Returns:
        Transformed vertices in ThinkGrasp coordinate system (meters)
    """
    if from_cm_to_m:
        # Convert from cm to meters if needed
        vertices = vertices / 100.0
    
    # ThinkGrasp uses standard camera frame:
    # X: right, Y: down, Z: forward (into the scene)
    # This is already the standard pinhole camera model frame
    
    return vertices


def create_o3d_pointcloud(vertices, colors=None):
    """
    Create Open3D point cloud matching ThinkGrasp format
    
    Args:
        vertices: numpy array of shape (N, 3) in meters
        colors: numpy array of shape (N, 3) with values in [0, 1]
    
    Returns:
        Open3D PointCloud object
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(vertices.astype(np.float32))
    
    if colors is not None:
        # Ensure colors are in [0, 1] range
        if colors.max() > 1.0:
            colors = colors / 255.0
        pcd.colors = o3d.utility.Vector3dVector(colors.astype(np.float32))
    
    return pcd


def create_coordinate_frame(size=0.1, origin=[0, 0, 0]):
    """
    Create coordinate frame matching ThinkGrasp visualization
    
    Args:
        size: Size of the coordinate frame in meters
        origin: Origin position
    
    Returns:
        Open3D TriangleMesh coordinate frame
    """
    return o3d.geometry.TriangleMesh.create_coordinate_frame(size=size, origin=origin)


def visualize_with_grasp_pose(pcd, grasp_translation, grasp_rotation, gripper_depth=0.1):
    """
    Visualize point cloud with grasp pose overlay
    
    Args:
        pcd: Open3D PointCloud
        grasp_translation: 3D translation vector (x, y, z) in meters
        grasp_rotation: 3x3 rotation matrix
        gripper_depth: Gripper depth for visualization
    
    Returns:
        List of geometries for visualization
    """
    geometries = [pcd]
    
    # Add coordinate frame at origin
    coord_frame = create_coordinate_frame(size=0.1)
    geometries.append(coord_frame)
    
    # Create gripper visualization at grasp pose
    gripper_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=gripper_depth)
    
    # Apply grasp transformation
    transform = np.eye(4)
    transform[:3, :3] = grasp_rotation
    transform[:3, 3] = grasp_translation
    gripper_frame.transform(transform)
    
    geometries.append(gripper_frame)
    
    return geometries


def validate_coordinate_alignment(our_cloud, thinkgrasp_cloud, tolerance=0.001):
    """
    Validate that our point cloud matches ThinkGrasp's coordinate system
    
    Args:
        our_cloud: Our generated point cloud
        thinkgrasp_cloud: ThinkGrasp generated point cloud
        tolerance: Maximum allowed difference in meters
    
    Returns:
        bool: True if clouds match within tolerance
    """
    if our_cloud.shape != thinkgrasp_cloud.shape:
        print(f"Shape mismatch: {our_cloud.shape} vs {thinkgrasp_cloud.shape}")
        return False
    
    diff = np.abs(our_cloud - thinkgrasp_cloud)
    max_diff = np.max(diff)
    mean_diff = np.mean(diff)
    
    print(f"Max difference: {max_diff:.6f} m")
    print(f"Mean difference: {mean_diff:.6f} m")
    
    if max_diff > tolerance:
        print(f"WARNING: Maximum difference {max_diff:.6f} exceeds tolerance {tolerance}")
        # Find points with large differences
        large_diff_mask = np.any(diff > tolerance, axis=-1)
        num_bad_points = np.sum(large_diff_mask)
        print(f"Number of points exceeding tolerance: {num_bad_points}")
        return False
    
    return True


def create_grasp_trajectory(start_pos, grasp_pos, num_waypoints=20, approach_height=0.1):
    """
    Create a grasp trajectory from start position to grasp position
    
    Args:
        start_pos: Starting position [x, y, z] in meters
        grasp_pos: Target grasp position [x, y, z] in meters
        num_waypoints: Number of waypoints in trajectory
        approach_height: Height above grasp point for pre-grasp position
    
    Returns:
        List of waypoints forming the grasp trajectory
    """
    waypoints = []
    
    # Pre-grasp position (above the grasp point)
    pre_grasp_pos = [grasp_pos[0], grasp_pos[1], grasp_pos[2] + approach_height]
    
    # Phase 1: Move from start to pre-grasp position
    for i in range(num_waypoints // 2):
        t = i / (num_waypoints // 2 - 1) if num_waypoints > 2 else 0
        waypoint = [
            start_pos[0] + t * (pre_grasp_pos[0] - start_pos[0]),
            start_pos[1] + t * (pre_grasp_pos[1] - start_pos[1]),
            start_pos[2] + t * (pre_grasp_pos[2] - start_pos[2])
        ]
        waypoints.append(waypoint)
    
    # Phase 2: Move from pre-grasp to grasp position (vertical approach)
    for i in range(num_waypoints // 2):
        t = i / (num_waypoints // 2 - 1) if num_waypoints > 2 else 0
        waypoint = [
            pre_grasp_pos[0],
            pre_grasp_pos[1],
            pre_grasp_pos[2] + t * (grasp_pos[2] - pre_grasp_pos[2])
        ]
        waypoints.append(waypoint)
    
    return waypoints


def create_gripper_mesh(position, rotation, width=0.05, length=0.08, thickness=0.01):
    """
    Create a simple gripper mesh for visualization
    
    Args:
        position: Gripper position [x, y, z] in meters
        rotation: 3x3 rotation matrix
        width: Gripper opening width
        length: Finger length
        thickness: Finger thickness
    
    Returns:
        Open3D LineSet representing the gripper
    """
    # Define gripper geometry in local frame
    half_width = width / 2
    
    # Gripper finger points (simplified two-finger gripper)
    points = [
        # Left finger
        [-half_width, 0, 0],
        [-half_width, 0, length],
        [-half_width - thickness, 0, length],
        [-half_width - thickness, 0, 0],
        # Right finger
        [half_width, 0, 0],
        [half_width, 0, length],
        [half_width + thickness, 0, length],
        [half_width + thickness, 0, 0],
        # Base connection
        [-half_width, 0, 0],
        [half_width, 0, 0]
    ]
    
    # Define lines connecting the points
    lines = [
        # Left finger outline
        [0, 1], [1, 2], [2, 3], [3, 0],
        # Right finger outline
        [4, 5], [5, 6], [6, 7], [7, 4],
        # Base connection
        [8, 9]
    ]
    
    # Create LineSet
    gripper_lines = o3d.geometry.LineSet()
    gripper_lines.points = o3d.utility.Vector3dVector(np.array(points))
    gripper_lines.lines = o3d.utility.Vector2iVector(np.array(lines))
    
    # Apply transformation
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = position
    gripper_lines.transform(transform)
    
    # Set color (yellow for gripper)
    gripper_lines.paint_uniform_color([1.0, 1.0, 0.0])
    
    return gripper_lines


def visualize_grasp_with_trajectory(pcd, grasp_pose, trajectory=None, show=True):
    """
    Visualize point cloud with grasp pose and optional trajectory
    
    Args:
        pcd: Open3D PointCloud
        grasp_pose: Dict with 'xyz' (position), 'rot' (rotation), 'dep' (depth)
        trajectory: List of waypoints or None
        show: If True, display the visualization
    
    Returns:
        List of geometries for visualization
    """
    geometries = [pcd]
    
    # Add coordinate frame at origin
    coord_frame = create_coordinate_frame(size=0.1)
    geometries.append(coord_frame)
    
    # Extract grasp pose
    position = np.array(grasp_pose['xyz']) / 1000.0  # Convert mm to m
    rotation = np.array(grasp_pose['rot'])
    depth = grasp_pose.get('dep', 0.05)
    
    # Create grasp coordinate frame
    grasp_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=depth)
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = position
    grasp_frame.transform(transform)
    geometries.append(grasp_frame)
    
    # Create gripper visualization
    gripper = create_gripper_mesh(position, rotation, width=depth, length=depth*0.8)
    geometries.append(gripper)
    
    # Add trajectory if provided
    if trajectory is not None and len(trajectory) > 1:
        # Create trajectory line
        traj_lines = o3d.geometry.LineSet()
        traj_lines.points = o3d.utility.Vector3dVector(np.array(trajectory))
        lines = [[i, i+1] for i in range(len(trajectory)-1)]
        traj_lines.lines = o3d.utility.Vector2iVector(np.array(lines))
        traj_lines.paint_uniform_color([0.0, 1.0, 1.0])  # Cyan color
        geometries.append(traj_lines)
        
        # Add spheres at waypoints
        for i, point in enumerate(trajectory):
            sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.005)
            sphere.translate(point)
            # Color gradient from start (green) to end (red)
            t = i / (len(trajectory) - 1)
            sphere.paint_uniform_color([t, 1-t, 0])
            geometries.append(sphere)
    
    if show:
        o3d.visualization.draw_geometries(geometries, window_name="Grasp Visualization with Trajectory")
    
    return geometries