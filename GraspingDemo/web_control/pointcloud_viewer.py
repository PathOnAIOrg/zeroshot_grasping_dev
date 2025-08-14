#!/usr/bin/env python3
"""
Point Cloud Viewer using Plotly
Similar to the visualization in GenDP demo notebook
"""

import numpy as np
import json
import plotly.graph_objects as go
from pathlib import Path
from urdf_visualizer import URDFVisualizer

def load_point_cloud_from_capture(capture_path):
    """Load point cloud data from a capture folder"""
    capture_dir = Path(capture_path)
    
    # Load metadata
    metadata_path = capture_dir / 'metadata.json'
    metadata = None
    if metadata_path.exists():
        with open(metadata_path, 'r') as f:
            metadata = json.load(f)
    
    # Try to load point cloud from PLY file
    ply_path = capture_dir / 'pointcloud.ply'
    if ply_path.exists():
        try:
            return load_ply_file(str(ply_path)), metadata
        except Exception as e:
            print(f"Failed to load PLY file: {e}, falling back to depth reconstruction")
    
    # Fallback: generate point cloud from depth image
    depth_path = capture_dir / 'depth.npy'
    if depth_path.exists():
        depth = np.load(str(depth_path))
        
        # Try to load RGB image for colors
        rgb_image = None
        rgb_path = capture_dir / 'rgb.png'
        if rgb_path.exists():
            try:
                import cv2
                rgb_image = cv2.imread(str(rgb_path))
                rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
            except ImportError:
                # Try PIL if cv2 not available
                try:
                    from PIL import Image
                    rgb_image = np.array(Image.open(str(rgb_path)))
                except:
                    print("Could not load RGB image for colors")
        
        return generate_pointcloud_from_depth(depth, metadata, rgb_image), metadata
    
    return None, metadata

def load_ply_file(ply_path):
    """Load a PLY file and extract vertices and colors"""
    try:
        import open3d as o3d
        pcd = o3d.io.read_point_cloud(ply_path)
        vertices = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors)
        
        # Validate data
        if len(vertices) == 0:
            raise ValueError("No vertices found in PLY file")
        
        # If no colors, generate default ones
        if len(colors) == 0:
            colors = np.ones((len(vertices), 3)) * 0.5
        
        return {'vertices': vertices, 'colors': colors}
    except (ImportError, Exception) as e:
        print(f"Open3D not available or failed to load PLY: {e}")
        # Fallback: basic PLY reader
        try:
            vertices = []
            colors = []
            with open(ply_path, 'r') as f:
                # Parse header
                vertex_count = 0
                in_header = True
                
                for line in f:
                    if in_header:
                        if 'element vertex' in line:
                            vertex_count = int(line.split()[-1])
                        elif 'end_header' in line:
                            in_header = False
                    else:
                        # Read vertex data (assuming ASCII format)
                        parts = line.strip().split()
                        if len(parts) >= 3:
                            vertices.append([float(parts[0]), float(parts[1]), float(parts[2])])
                            if len(parts) >= 6:
                                # Has RGB values
                                colors.append([float(parts[3])/255, float(parts[4])/255, float(parts[5])/255])
                            else:
                                colors.append([0.5, 0.5, 0.5])
                        
                        if len(vertices) >= vertex_count:
                            break
            
            return {'vertices': np.array(vertices), 'colors': np.array(colors)}
        except Exception as e:
            print(f"Failed to parse PLY file: {e}")
            return {'vertices': np.array([]), 'colors': np.array([])}

def generate_pointcloud_from_depth(depth_image, metadata=None, rgb_image=None, organized=False):
    """Generate point cloud from depth image using camera intrinsics
    
    Following the standard pinhole camera model:
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    where (u,v) are pixel coordinates and (x,y,z) are 3D coordinates
    
    Args:
        depth_image: numpy array of depth values in millimeters
        metadata: dict containing camera intrinsics
        rgb_image: optional RGB image for colors
        organized: if True, returns organized point cloud (H,W,3), else returns (N,3)
    
    Returns:
        dict with 'vertices' and 'colors' arrays
    """
    h, w = depth_image.shape
    
    # Get camera intrinsics and depth scale
    if metadata and 'camera_intrinsics' in metadata:
        fx = metadata['camera_intrinsics']['fx']
        fy = metadata['camera_intrinsics']['fy']
        cx = metadata['camera_intrinsics']['ppx']  # Principal point x
        cy = metadata['camera_intrinsics']['ppy']  # Principal point y
        depth_scale = metadata['camera_intrinsics'].get('depth_scale', 0.001)  # Default 1mm
    else:
        # Default RealSense D435 intrinsics
        fx = fy = 600.0
        cx = w / 2.0
        cy = h / 2.0
        depth_scale = 0.001  # 1mm per unit
    
    # Create mesh grid for pixel coordinates
    xmap = np.arange(w)
    ymap = np.arange(h)
    xmap, ymap = np.meshgrid(xmap, ymap)
    
    if organized:
        # Keep organized structure (H, W, 3)
        # Convert depth to meters
        points_z = depth_image * depth_scale
        
        # Back-project to 3D using pinhole camera model
        points_x = (xmap - cx) * points_z / fx
        points_y = (ymap - cy) * points_z / fy
        
        # Stack to create organized point cloud
        cloud = np.stack([points_x, points_y, points_z], axis=-1)
        
        # Generate organized colors
        if rgb_image is not None and rgb_image.shape[:2] == depth_image.shape:
            colors = rgb_image / 255.0 if len(rgb_image.shape) == 3 else np.stack([rgb_image/255.0]*3, axis=-1)
        else:
            colors = np.ones((h, w, 3)) * 0.5
        
        return {'vertices': cloud, 'colors': colors, 'organized': True}
    
    else:
        # Unorganized point cloud (N, 3) - filter out invalid points
        # Valid depth mask - exclude invalid values
        max_depth = 10000  # 10 meters in mm
        valid = (depth_image > 0) & (depth_image < max_depth)
        
        # Convert depth to meters
        points_z = depth_image[valid] * depth_scale
        
        # Back-project to 3D using pinhole camera model
        points_x = (xmap[valid] - cx) * points_z / fx
        points_y = (ymap[valid] - cy) * points_z / fy
        
        # Stack to create 3D points
        vertices = np.stack([points_x, points_y, points_z], axis=-1)
        
        # Generate colors
        if rgb_image is not None and rgb_image.shape[:2] == depth_image.shape:
            # Use RGB colors if available
            if len(rgb_image.shape) == 3:
                colors = rgb_image[valid] / 255.0  # Normalize to 0-1
            else:
                # Grayscale image
                gray = rgb_image[valid] / 255.0
                colors = np.stack([gray, gray, gray], axis=-1)
        else:
            # Fallback to depth-based coloring
            colors = np.zeros((len(vertices), 3))
            z_norm = (points_z - points_z.min()) / (points_z.max() - points_z.min() + 1e-6)
            colors[:, 0] = z_norm  # Red channel
            colors[:, 2] = 1 - z_norm  # Blue channel
        
        return {'vertices': vertices, 'colors': colors}

def downsample_pointcloud(vertices, colors=None, voxel_size=0.005):
    """Downsample point cloud using voxel grid"""
    if len(vertices) == 0:
        return vertices, colors
        
    # Simple voxel grid downsampling
    voxel_indices = np.floor(vertices / voxel_size).astype(int)
    
    # Get unique voxels
    unique_voxels, inverse_indices = np.unique(
        voxel_indices, axis=0, return_inverse=True
    )
    
    # Average points in each voxel
    downsampled_vertices = []
    downsampled_colors = []
    
    for i in range(len(unique_voxels)):
        mask = inverse_indices == i
        downsampled_vertices.append(vertices[mask].mean(axis=0))
        if colors is not None:
            downsampled_colors.append(colors[mask].mean(axis=0))
    
    downsampled_vertices = np.array(downsampled_vertices)
    downsampled_colors = np.array(downsampled_colors) if colors is not None else None
    
    return downsampled_vertices, downsampled_colors


def remove_statistical_outliers(vertices, colors=None, n_neighbors=20, std_ratio=2.0):
    """Remove statistical outliers from point cloud"""
    if len(vertices) < n_neighbors:
        return vertices, colors
    
    try:
        from sklearn.neighbors import NearestNeighbors
        
        # Find k nearest neighbors
        nbrs = NearestNeighbors(n_neighbors=n_neighbors).fit(vertices)
        distances, _ = nbrs.kneighbors(vertices)
        
        # Calculate mean distance for each point
        mean_distances = distances.mean(axis=1)
        
        # Calculate threshold
        threshold = mean_distances.mean() + std_ratio * mean_distances.std()
        
        # Filter inliers
        inlier_mask = mean_distances < threshold
        
        filtered_vertices = vertices[inlier_mask]
        filtered_colors = colors[inlier_mask] if colors is not None else None
        
        return filtered_vertices, filtered_colors
    except ImportError:
        # If sklearn not available, return as-is
        return vertices, colors

def create_plotly_pointcloud(vertices, colors=None, size=1, name="Point Cloud"):
    """
    Create a Plotly 3D scatter plot for point cloud visualization
    Similar to vis_pcd_plotly in GenDP demo
    """
    if colors is None:
        colors = np.ones((len(vertices), 3)) * 0.5
    
    # Convert colors to RGB strings
    if colors.max() <= 1.0:
        colors = (colors * 255).astype(int)
    
    color_strings = ['rgb({},{},{})'.format(r, g, b) for r, g, b in colors]
    
    # Create 3D scatter plot
    trace = go.Scatter3d(
        x=vertices[:, 0],
        y=vertices[:, 1],
        z=vertices[:, 2],
        mode='markers',
        name=name,
        marker=dict(
            size=size,
            color=color_strings,
            opacity=0.8,
        )
    )
    
    return trace

def create_coordinate_frame_traces(size=10.0, origin=[0, 0, 0]):
    """
    Create Plotly traces for coordinate frame axes
    Similar to Open3D's create_coordinate_frame
    
    Args:
        size: Length of each axis in cm
        origin: Origin point of the coordinate frame
    
    Returns:
        List of Plotly traces for X, Y, Z axes
    """
    traces = []
    
    # X-axis (Red)
    traces.append(go.Scatter3d(
        x=[origin[0], origin[0] + size],
        y=[origin[1], origin[1]],
        z=[origin[2], origin[2]],
        mode='lines+text',
        name='X-axis',
        line=dict(color='red', width=5),
        text=['', 'X'],
        textposition='top center',
        textfont=dict(color='red', size=12),
        showlegend=False
    ))
    
    # Y-axis (Green)
    traces.append(go.Scatter3d(
        x=[origin[0], origin[0]],
        y=[origin[1], origin[1] + size],
        z=[origin[2], origin[2]],
        mode='lines+text',
        name='Y-axis',
        line=dict(color='green', width=5),
        text=['', 'Y'],
        textposition='top center',
        textfont=dict(color='green', size=12),
        showlegend=False
    ))
    
    # Z-axis (Blue)
    traces.append(go.Scatter3d(
        x=[origin[0], origin[0]],
        y=[origin[1], origin[1]],
        z=[origin[2], origin[2] + size],
        mode='lines+text',
        name='Z-axis',
        line=dict(color='blue', width=5),
        text=['', 'Z'],
        textposition='top center',
        textfont=dict(color='blue', size=12),
        showlegend=False
    ))
    
    return traces

def create_plotly_figure(traces, title="3D Point Cloud Viewer", adaptive_range=True, show_axes=True):
    """
    Create a Plotly figure with appropriate layout for point cloud visualization
    """
    # Add coordinate frame if requested
    if show_axes:
        # Calculate appropriate axis size based on data
        axis_size = 10.0  # Default 10cm
        if adaptive_range and len(traces) > 0:
            # Get data range to scale axes appropriately
            all_x = []
            all_y = []
            all_z = []
            for trace in traces:
                if hasattr(trace, 'x') and hasattr(trace, 'mode') and 'markers' in trace.mode:
                    all_x.extend(trace.x)
                    all_y.extend(trace.y)
                    all_z.extend(trace.z)
            
            if all_x:
                x_range = max(all_x) - min(all_x)
                y_range = max(all_y) - min(all_y)
                z_range = max(all_z) - min(all_z)
                axis_size = max(x_range, y_range, z_range) * 0.15  # 15% of max range
        
        # Add coordinate frame traces
        coord_traces = create_coordinate_frame_traces(size=axis_size)
        traces = coord_traces + traces
    
    fig = go.Figure(data=traces)
    
    # Calculate data range if adaptive
    # Use consistent range for all axes (300cm cube centered at origin)
    x_range = [-150, 150]  # Default range in cm
    y_range = [-150, 150]  # Same range for Y
    z_range = [-150, 150]  # Same range for Z to maintain uniform scale
    
    if adaptive_range and len(traces) > 0:
        # Get all vertices from traces (excluding axis lines)
        all_x = []
        all_y = []
        all_z = []
        for trace in traces:
            if hasattr(trace, 'x'):
                # Only include point cloud data, not axis lines
                if hasattr(trace, 'mode') and 'markers' in trace.mode:
                    all_x.extend(trace.x)
                    all_y.extend(trace.y)
                    all_z.extend(trace.z)
        
        if all_x:  # If we have data
            # Calculate ranges with some padding
            padding = 0.2  # 20% padding to show axes clearly
            x_min, x_max = min(all_x), max(all_x)
            y_min, y_max = min(all_y), max(all_y)
            z_min, z_max = min(all_z), max(all_z)
            
            x_range_size = x_max - x_min
            y_range_size = y_max - y_min
            z_range_size = z_max - z_min
            
            x_range = [x_min - x_range_size * padding, x_max + x_range_size * padding]
            y_range = [y_min - y_range_size * padding, y_max + y_range_size * padding]
            z_range = [z_min - z_range_size * padding, z_max + z_range_size * padding]
    
    # Update layout for better 3D visualization (using centimeters)
    fig.update_layout(
        title=title,
        scene=dict(
            xaxis=dict(
                title='X (cm)',
                gridcolor='lightgray',
                showbackground=True,
                backgroundcolor='rgb(240, 240, 240)',
                range=x_range
            ),
            yaxis=dict(
                title='Y (cm)',
                gridcolor='lightgray',
                showbackground=True,
                backgroundcolor='rgb(240, 240, 240)',
                range=y_range
            ),
            zaxis=dict(
                title='Z (cm)',
                gridcolor='lightgray',
                showbackground=True,
                backgroundcolor='rgb(240, 240, 240)',
                range=z_range
            ),
            camera=dict(
                eye=dict(x=1.5, y=1.5, z=1.0),
                center=dict(x=0, y=0, z=0),
                up=dict(x=0, y=0, z=1)
            ),
            aspectmode='cube'  # Force equal aspect ratio for all axes
        ),
        showlegend=True,
        height=600,
        margin=dict(l=0, r=0, t=30, b=0),
        paper_bgcolor='white',
        plot_bgcolor='rgb(250, 250, 250)'
    )
    
    return fig

def create_pointcloud_with_robot(vertices, colors, robot_joints=None):
    """
    Create point cloud visualization with robot pose overlay
    """
    traces = []
    
    # Add main point cloud
    trace_pcd = create_plotly_pointcloud(
        vertices, colors, size=2, name="Scene"
    )
    traces.append(trace_pcd)
    
    # Add robot joints if available
    if robot_joints is not None:
        # Create a simple robot visualization
        # This would need proper forward kinematics for accurate visualization
        joint_positions = np.array(robot_joints[:3]).reshape(1, 3)  # Simplified: just use first 3 values as position
        
        trace_robot = go.Scatter3d(
            x=joint_positions[:, 0],
            y=joint_positions[:, 1],
            z=joint_positions[:, 2],
            mode='markers',
            name='Robot',
            marker=dict(
                size=10,
                color='red',
                symbol='diamond'
            )
        )
        traces.append(trace_robot)
    
    return traces

def generate_plotly_html(fig, div_id="pointcloud-plotly"):
    """Generate HTML div with Plotly figure"""
    return fig.to_html(
        include_plotlyjs='cdn',
        div_id=div_id,
        config={
            'displayModeBar': True,
            'displaylogo': False,
            'modeBarButtonsToRemove': ['pan2d', 'select2d', 'lasso2d'],
            'toImageButtonOptions': {
                'format': 'png',
                'filename': 'pointcloud_capture',
                'height': 800,
                'width': 1200,
                'scale': 1
            }
        }
    )

def create_camera_visualization(camera_info=None):
    """
    Create camera frustum and coordinate system visualization
    
    Args:
        camera_info: Dict with camera parameters (fx, fy, cx, cy, width, height)
    
    Returns:
        List of Plotly trace objects for camera visualization
    """
    traces = []
    
    # Default camera parameters if not provided
    if camera_info is None:
        camera_info = {
            'fx': 615.0, 'fy': 615.0,
            'cx': 320.0, 'cy': 240.0,
            'width': 640, 'height': 480,
            'scale': 1000.0  # mm to m
        }
    
    # Camera position at origin
    cam_pos = np.array([0, 0, 0])
    
    # Define frustum depth (in meters)
    near_depth = 0.1  # 10 cm
    far_depth = 1.0   # 1 meter
    
    # Get image plane corners in pixel coordinates
    corners_2d = np.array([
        [0, 0],                                          # Top-left
        [camera_info['width'], 0],                      # Top-right
        [camera_info['width'], camera_info['height']],  # Bottom-right
        [0, camera_info['height']]                      # Bottom-left
    ])
    
    # Convert corners to 3D camera coordinates at near and far planes
    corners_3d_near = []
    corners_3d_far = []
    
    for u, v in corners_2d:
        # Use pinhole camera model to get 3D coordinates
        # Near plane
        x_near = (u - camera_info['cx']) * near_depth / camera_info['fx']
        y_near = (v - camera_info['cy']) * near_depth / camera_info['fy']
        corners_3d_near.append([x_near, y_near, near_depth])
        
        # Far plane
        x_far = (u - camera_info['cx']) * far_depth / camera_info['fx']
        y_far = (v - camera_info['cy']) * far_depth / camera_info['fy']
        corners_3d_far.append([x_far, y_far, far_depth])
    
    corners_3d_near = np.array(corners_3d_near)
    corners_3d_far = np.array(corners_3d_far)
    
    # Convert to centimeters for display
    corners_3d_near *= 100
    corners_3d_far *= 100
    cam_pos_cm = cam_pos * 100
    
    # Create frustum edges
    frustum_lines_x = []
    frustum_lines_y = []
    frustum_lines_z = []
    
    # Lines from camera to near plane corners
    for corner in corners_3d_near:
        frustum_lines_x.extend([cam_pos_cm[0], corner[0], None])
        frustum_lines_y.extend([cam_pos_cm[1], corner[1], None])
        frustum_lines_z.extend([cam_pos_cm[2], corner[2], None])
    
    # Near plane rectangle
    for i in range(4):
        next_i = (i + 1) % 4
        frustum_lines_x.extend([corners_3d_near[i][0], corners_3d_near[next_i][0], None])
        frustum_lines_y.extend([corners_3d_near[i][1], corners_3d_near[next_i][1], None])
        frustum_lines_z.extend([corners_3d_near[i][2], corners_3d_near[next_i][2], None])
    
    # Far plane rectangle
    for i in range(4):
        next_i = (i + 1) % 4
        frustum_lines_x.extend([corners_3d_far[i][0], corners_3d_far[next_i][0], None])
        frustum_lines_y.extend([corners_3d_far[i][1], corners_3d_far[next_i][1], None])
        frustum_lines_z.extend([corners_3d_far[i][2], corners_3d_far[next_i][2], None])
    
    # Lines from near to far plane
    for i in range(4):
        frustum_lines_x.extend([corners_3d_near[i][0], corners_3d_far[i][0], None])
        frustum_lines_y.extend([corners_3d_near[i][1], corners_3d_far[i][1], None])
        frustum_lines_z.extend([corners_3d_near[i][2], corners_3d_far[i][2], None])
    
    # Create frustum trace
    frustum_trace = go.Scatter3d(
        x=frustum_lines_x,
        y=frustum_lines_y,
        z=frustum_lines_z,
        mode='lines',
        line=dict(color='yellow', width=2),
        name='Camera Frustum',
        hoverinfo='name'
    )
    traces.append(frustum_trace)
    
    # Create coordinate axes at camera position
    axis_length = 10  # 10 cm
    
    # X-axis (red) - pointing right
    traces.append(go.Scatter3d(
        x=[cam_pos_cm[0], cam_pos_cm[0] + axis_length],
        y=[cam_pos_cm[1], cam_pos_cm[1]],
        z=[cam_pos_cm[2], cam_pos_cm[2]],
        mode='lines+text',
        line=dict(color='red', width=4),
        text=['', 'X'],
        textposition='top center',
        textfont=dict(size=12, color='red'),
        name='X-axis (Right)',
        hoverinfo='name'
    ))
    
    # Y-axis (green) - pointing down
    traces.append(go.Scatter3d(
        x=[cam_pos_cm[0], cam_pos_cm[0]],
        y=[cam_pos_cm[1], cam_pos_cm[1] + axis_length],
        z=[cam_pos_cm[2], cam_pos_cm[2]],
        mode='lines+text',
        line=dict(color='green', width=4),
        text=['', 'Y'],
        textposition='top center',
        textfont=dict(size=12, color='green'),
        name='Y-axis (Down)',
        hoverinfo='name'
    ))
    
    # Z-axis (blue) - pointing forward
    traces.append(go.Scatter3d(
        x=[cam_pos_cm[0], cam_pos_cm[0]],
        y=[cam_pos_cm[1], cam_pos_cm[1]],
        z=[cam_pos_cm[2], cam_pos_cm[2] + axis_length],
        mode='lines+text',
        line=dict(color='blue', width=4),
        text=['', 'Z'],
        textposition='top center',
        textfont=dict(size=12, color='blue'),
        name='Z-axis (Forward)',
        hoverinfo='name'
    ))
    
    # Add camera position marker
    traces.append(go.Scatter3d(
        x=[cam_pos_cm[0]],
        y=[cam_pos_cm[1]],
        z=[cam_pos_cm[2]],
        mode='markers+text',
        marker=dict(size=8, color='yellow', symbol='diamond'),
        text=['Camera'],
        textposition='bottom center',
        textfont=dict(size=10, color='yellow'),
        name='Camera Position',
        hovertemplate='Camera at origin<br>fx: %.1f, fy: %.1f<br>cx: %.1f, cy: %.1f<br>Resolution: %dx%d' % 
                      (camera_info['fx'], camera_info['fy'], 
                       camera_info['cx'], camera_info['cy'],
                       camera_info['width'], camera_info['height'])
    ))
    
    # Add image plane center indicator
    image_center_3d = np.array([0, 0, near_depth * 100])  # Center of near plane in cm
    traces.append(go.Scatter3d(
        x=[image_center_3d[0]],
        y=[image_center_3d[1]],
        z=[image_center_3d[2]],
        mode='markers',
        marker=dict(size=5, color='cyan', symbol='cross'),
        name='Image Center',
        hovertemplate='Image center projected<br>at depth: %.1f cm' % (near_depth * 100)
    ))
    
    # Add reference grid points at different depths for verification
    reference_depths = [0.3, 0.5, 0.7]  # meters
    for depth in reference_depths:
        # Create a 3x3 grid of points at this depth
        grid_points_x = []
        grid_points_y = []
        grid_points_z = []
        
        # Sample points across the image
        sample_pixels = [
            (camera_info['width'] * 0.25, camera_info['height'] * 0.25),  # Top-left
            (camera_info['width'] * 0.5, camera_info['height'] * 0.25),   # Top-center
            (camera_info['width'] * 0.75, camera_info['height'] * 0.25),  # Top-right
            (camera_info['width'] * 0.25, camera_info['height'] * 0.5),   # Middle-left
            (camera_info['width'] * 0.5, camera_info['height'] * 0.5),    # Center
            (camera_info['width'] * 0.75, camera_info['height'] * 0.5),   # Middle-right
            (camera_info['width'] * 0.25, camera_info['height'] * 0.75),  # Bottom-left
            (camera_info['width'] * 0.5, camera_info['height'] * 0.75),   # Bottom-center
            (camera_info['width'] * 0.75, camera_info['height'] * 0.75),  # Bottom-right
        ]
        
        for u, v in sample_pixels:
            x = (u - camera_info['cx']) * depth / camera_info['fx']
            y = (v - camera_info['cy']) * depth / camera_info['fy']
            grid_points_x.append(x * 100)  # Convert to cm
            grid_points_y.append(y * 100)
            grid_points_z.append(depth * 100)
        
        traces.append(go.Scatter3d(
            x=grid_points_x,
            y=grid_points_y,
            z=grid_points_z,
            mode='markers',
            marker=dict(
                size=3,
                color=depth * 100,
                colorscale='Viridis',
                showscale=False,
                symbol='circle'
            ),
            name=f'Reference Grid @ {depth*100:.0f}cm',
            hovertemplate='Grid point<br>Depth: %.0f cm<br>X: %%{x:.1f} cm<br>Y: %%{y:.1f} cm' % (depth * 100)
        ))
    
    # Add principal axis ray (center pixel ray)
    principal_ray_end = 50  # 50 cm
    traces.append(go.Scatter3d(
        x=[0, 0],
        y=[0, 0],
        z=[0, principal_ray_end],
        mode='lines',
        line=dict(color='magenta', width=3, dash='dash'),
        name='Principal Axis',
        hovertemplate='Camera principal axis<br>(center pixel ray)'
    ))
    
    return traces

def vis_pcd_plotly(pointclouds, size_ls=None, title="3D Point Cloud", 
                  show_camera=True, camera_info=None,
                  show_robot=True, robot_joint_angles=None, robot_position=None, robot_rotation=None):
    """
    Visualize point clouds using Plotly (similar to GenDP demo)
    
    Args:
        pointclouds: List of point cloud dicts with 'vertices' and 'colors'
        size_ls: List of marker sizes for each point cloud
        title: Title for the plot
        show_camera: Whether to show camera frustum and coordinate system
        camera_info: Camera parameters dict with fx, fy, cx, cy, width, height
        show_robot: Whether to show URDF robot model
        robot_joint_angles: Dictionary of joint angles in radians
        robot_position: Robot base position [x, y, z] in meters
        robot_rotation: Robot base rotation [roll, pitch, yaw] in degrees
    
    Returns:
        Plotly figure object
    """
    if size_ls is None:
        size_ls = [1] * len(pointclouds)  # Smaller default size for cm scale
    
    traces = []
    for i, pcd in enumerate(pointclouds):
        vertices = pcd.get('vertices', [])
        colors = pcd.get('colors', None)
        name = pcd.get('name', f'Point Cloud {i+1}')
        
        if len(vertices) > 0:
            trace = create_plotly_pointcloud(
                vertices, colors, size=size_ls[i], name=name
            )
            traces.append(trace)
    
    # Add camera visualization if requested
    if show_camera:
        camera_traces = create_camera_visualization(camera_info)
        traces.extend(camera_traces)
    
    # Add robot URDF model if requested
    if show_robot:
        try:
            # Try to load SO101 URDF from local folder
            urdf_path = "/home/pathonai/Documents/Github/opensource_dev/GraspingDemo/robot_description/urdf/so101_base.xacro"
            visualizer = URDFVisualizer(urdf_path)
            
            if visualizer.parse_urdf():
                # Use provided joint angles or defaults
                if robot_joint_angles is None:
                    # Default pose (slightly bent to be visible)
                    robot_joint_angles = {
                        '1': 0.0,      # Base rotation
                        '2': -0.5,     # Shoulder
                        '3': 0.8,      # Elbow
                        '4': -0.3,     # Wrist pitch
                        '5': 0.0,      # Wrist roll
                        '6': 0.0       # Gripper
                    }
                
                # Default robot position if not provided
                if robot_position is None:
                    robot_position = np.array([0, 0, 0])  # At origin
                
                robot_traces = visualizer.create_plotly_traces(
                    joint_angles=robot_joint_angles,
                    robot_position=robot_position,
                    robot_rotation=robot_rotation,
                    scale=100.0,  # Convert to cm
                    simplified=False  # Use full mesh visualization
                )
                traces.extend(robot_traces)
                print(f"Added {len(robot_traces)} robot traces to visualization")
            else:
                print("Failed to parse URDF")
        except Exception as e:
            print(f"Could not load URDF model: {e}")
            import traceback
            traceback.print_exc()
    
    fig = create_plotly_figure(traces, title=title)
    return fig