#!/usr/bin/env python3
"""
Test script to verify point cloud reconstruction from captured data
"""

import numpy as np
import json
from pathlib import Path
from pointcloud_viewer import load_point_cloud_from_capture, generate_pointcloud_from_depth

def test_reconstruction():
    """Test point cloud reconstruction from captures"""
    
    # Find captures directory
    capture_base = Path.home() / "Documents/Github/opensource_dev/GraspingDemo/captures"
    
    if not capture_base.exists():
        print(f"Captures directory not found: {capture_base}")
        return
    
    # List all captures
    captures = [d for d in capture_base.iterdir() if d.is_dir()]
    
    if not captures:
        print("No captures found")
        return
    
    print(f"Found {len(captures)} captures")
    
    # Test each capture
    for capture_dir in captures[:3]:  # Test first 3
        print(f"\nTesting capture: {capture_dir.name}")
        print("-" * 50)
        
        # Load point cloud
        pcd_data, metadata = load_point_cloud_from_capture(str(capture_dir))
        
        if pcd_data is None:
            print("  ❌ Failed to load point cloud")
            continue
        
        vertices = pcd_data.get('vertices', [])
        colors = pcd_data.get('colors', [])
        
        print(f"  ✓ Loaded point cloud:")
        print(f"    - Vertices: {len(vertices)} points")
        print(f"    - Colors: {len(colors)} values")
        
        if len(vertices) > 0:
            print(f"    - Vertex range: X[{vertices[:, 0].min():.3f}, {vertices[:, 0].max():.3f}]")
            print(f"                    Y[{vertices[:, 1].min():.3f}, {vertices[:, 1].max():.3f}]")
            print(f"                    Z[{vertices[:, 2].min():.3f}, {vertices[:, 2].max():.3f}]")
        
        if len(colors) > 0:
            print(f"    - Color range: [{colors.min():.3f}, {colors.max():.3f}]")
        
        # Check files in capture
        files = list(capture_dir.iterdir())
        print(f"  Files in capture:")
        for f in files:
            size = f.stat().st_size / 1024  # KB
            print(f"    - {f.name}: {size:.1f} KB")
        
        # Check metadata
        if metadata:
            print(f"  Metadata:")
            if 'camera_intrinsics' in metadata:
                intrinsics = metadata['camera_intrinsics']
                print(f"    - Camera: {intrinsics.get('width')}x{intrinsics.get('height')}")
                print(f"    - Focal: fx={intrinsics.get('fx', 0):.1f}, fy={intrinsics.get('fy', 0):.1f}")
            if 'robot_state' in metadata:
                robot = metadata['robot_state']
                print(f"    - Robot: {robot.get('gripper_state', 'unknown')} gripper")
                if 'joint_positions' in robot:
                    joints = robot['joint_positions']
                    print(f"    - Joints: {[f'{j:.2f}' for j in joints[:6]]}")
        
        # Test depth-only reconstruction
        depth_path = capture_dir / "depth.npy"
        if depth_path.exists():
            print(f"  Testing depth reconstruction:")
            depth = np.load(str(depth_path))
            print(f"    - Depth shape: {depth.shape}")
            print(f"    - Depth range: [{depth[depth>0].min() if np.any(depth>0) else 0:.0f}, {depth.max():.0f}] mm")
            
            # Try with RGB
            rgb_path = capture_dir / "rgb.png"
            if rgb_path.exists():
                print(f"    - RGB image found, using for colors")
            else:
                print(f"    - No RGB image, using depth-based colors")

if __name__ == "__main__":
    test_reconstruction()