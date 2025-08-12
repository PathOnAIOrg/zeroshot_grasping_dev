#!/usr/bin/env python3
"""
Point Cloud Viewer using Plotly
Similar to the visualization in GenDP demo notebook
"""

import numpy as np
import json
import plotly.graph_objects as go
from pathlib import Path

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

def generate_pointcloud_from_depth(depth_image, metadata=None, rgb_image=None):
    """Generate point cloud from depth image using camera intrinsics"""
    h, w = depth_image.shape
    
    # Get camera intrinsics and depth scale
    if metadata and 'camera_intrinsics' in metadata:
        fx = metadata['camera_intrinsics']['fx']
        fy = metadata['camera_intrinsics']['fy']
        cx = metadata['camera_intrinsics']['ppx']
        cy = metadata['camera_intrinsics']['ppy']
        depth_scale = metadata['camera_intrinsics'].get('depth_scale', 0.001)  # Default 1mm
    else:
        # Default RealSense D435 intrinsics
        fx = fy = 600.0
        cx = w / 2.0
        cy = h / 2.0
        depth_scale = 0.001  # 1mm per unit
    
    # Create mesh grid
    xx, yy = np.meshgrid(np.arange(w), np.arange(h))
    
    # Valid depth mask - exclude saturated values (65535 is max for uint16)
    # Also exclude far values (>10 meters)
    max_depth = 10000  # 10 meters in mm
    valid = (depth_image > 0) & (depth_image < max_depth)
    
    # Back-project to 3D
    z = depth_image[valid] * depth_scale  # Convert to meters using depth scale
    x = (xx[valid] - cx) * z / fx
    y = (yy[valid] - cy) * z / fy
    
    vertices = np.stack([x, y, z], axis=-1)
    
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
        z_norm = (z - z.min()) / (z.max() - z.min() + 1e-6)
        colors[:, 0] = z_norm  # Red channel
        colors[:, 2] = 1 - z_norm  # Blue channel
    
    return {'vertices': vertices, 'colors': colors}

def downsample_pointcloud(vertices, colors=None, voxel_size=0.005):
    """Downsample point cloud using voxel grid"""
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

def create_plotly_pointcloud(vertices, colors=None, size=2, name="Point Cloud"):
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

def create_plotly_figure(traces, title="3D Point Cloud Viewer"):
    """
    Create a Plotly figure with appropriate layout for point cloud visualization
    """
    fig = go.Figure(data=traces)
    
    # Update layout for better 3D visualization
    fig.update_layout(
        title=title,
        scene=dict(
            xaxis=dict(
                title='X (m)',
                gridcolor='lightgray',
                showbackground=True,
                backgroundcolor='white'
            ),
            yaxis=dict(
                title='Y (m)',
                gridcolor='lightgray',
                showbackground=True,
                backgroundcolor='white'
            ),
            zaxis=dict(
                title='Z (m)',
                gridcolor='lightgray',
                showbackground=True,
                backgroundcolor='white'
            ),
            camera=dict(
                eye=dict(x=1.5, y=1.5, z=1.5),
                center=dict(x=0, y=0, z=0),
                up=dict(x=0, y=0, z=1)
            ),
            aspectmode='data'
        ),
        showlegend=True,
        height=600,
        margin=dict(l=0, r=0, t=30, b=0)
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

def vis_pcd_plotly(pointclouds, size_ls=None, title="3D Point Cloud"):
    """
    Visualize point clouds using Plotly (similar to GenDP demo)
    
    Args:
        pointclouds: List of point cloud dicts with 'vertices' and 'colors'
        size_ls: List of marker sizes for each point cloud
        title: Title for the plot
    
    Returns:
        Plotly figure object
    """
    if size_ls is None:
        size_ls = [2] * len(pointclouds)
    
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
    
    fig = create_plotly_figure(traces, title=title)
    return fig