#!/usr/bin/env python3
"""
Simple Open3D Point Cloud Visualization Script

This script creates and visualizes point clouds using Open3D.
It handles display failures gracefully and provides fallback options.
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import open3d as o3d
import cv2
from typing import Optional, Tuple
from so101_grasp.vision.camera import CameraController
from so101_grasp.simulation.cameras import RealSenseD415

def create_point_cloud_from_rgbd(color_image: np.ndarray, depth_image: np.ndarray, 
                               intrinsics: object, use_sim_params: bool = False) -> o3d.geometry.PointCloud:
    """
    Create Open3D point cloud from RGB-D data.
    
    Args:
        color_image: RGB color image
        depth_image: Depth image in mm
        intrinsics: Camera intrinsics (real camera or simulation)
        use_sim_params: Whether to use simulation camera parameters
        
    Returns:
        Open3D PointCloud object
    """
    # Convert images to Open3D format
    color_o3d = o3d.geometry.Image(color_image.astype(np.uint8))
    
    # Convert depth from mm to meters and ensure correct data type
    depth_meters = (depth_image.astype(np.float32)) / 1000.0
    depth_o3d = o3d.geometry.Image(depth_meters)
    
    # Create RGBD image
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_o3d, depth_o3d, 
        depth_scale=1.0,  # Already converted to meters
        depth_trunc=3.0,  # 3 meter max depth
        convert_rgb_to_intensity=False
    )
    
    # Create camera intrinsics for Open3D
    height, width = color_image.shape[:2]
    
    if use_sim_params:
        # Use simulation camera parameters from cameras.py
        sim_intrinsics = RealSenseD415.intrinsics
        camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width=width,
            height=height,
            fx=sim_intrinsics[0, 0],
            fy=sim_intrinsics[1, 1], 
            cx=sim_intrinsics[0, 2],
            cy=sim_intrinsics[1, 2]
        )
        print(f"📷 Using simulation camera parameters: fx={sim_intrinsics[0,0]:.1f}, fy={sim_intrinsics[1,1]:.1f}")
    else:
        # Use real camera intrinsics
        camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width=width,
            height=height, 
            fx=intrinsics.fx,
            fy=intrinsics.fy,
            cx=intrinsics.ppx,
            cy=intrinsics.ppy
        )
        print(f"📷 Using real camera parameters: fx={intrinsics.fx:.1f}, fy={intrinsics.fy:.1f}")
    
    # Generate point cloud
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd_image, camera_intrinsic
    )
    
    return pcd

def visualize_point_cloud_safe(pcd: o3d.geometry.PointCloud, window_name: str = "Point Cloud"):
    """
    Safely visualize point cloud with fallback options.
    """
    try:
        print(f"🎨 Attempting to visualize point cloud with {len(pcd.points)} points...")
        
        # Method 1: Try standard visualization
        o3d.visualization.draw_geometries(
            [pcd], 
            window_name=window_name,
            width=1200,
            height=800,
            left=50,
            top=50
        )
        
        print("✅ Point cloud visualization completed")
        
    except Exception as e:
        print(f"⚠️  Standard visualization failed: {e}")
        
        try:
            # Method 2: Try manual visualizer
            print("🔄 Trying manual visualizer...")
            vis = o3d.visualization.Visualizer()
            
            # Check if window creation succeeds
            window_created = vis.create_window(window_name=window_name, width=1200, height=800)
            
            if window_created:
                vis.add_geometry(pcd)
                
                # Set view parameters if possible
                view_ctl = vis.get_view_control()
                if view_ctl is not None:
                    view_ctl.set_zoom(0.8)
                
                print("🖥️  Manual visualization opened - close window to continue")
                vis.run()
                vis.destroy_window()
                print("✅ Manual visualization completed")
            else:
                raise RuntimeError("Window creation failed")
                
        except Exception as e2:
            print(f"⚠️  Manual visualization also failed: {e2}")
            
            # Method 3: Save to file as fallback
            print("💾 Saving point cloud to file instead...")
            output_path = "pointcloud_visualization.ply"
            o3d.io.write_point_cloud(output_path, pcd)
            print(f"📁 Point cloud saved to: {output_path}")
            print("   You can view it later with: o3d.visualization.draw_geometries([o3d.io.read_point_cloud('pointcloud_visualization.ply')])")

def capture_and_visualize():
    """
    Capture RGB-D data from camera and visualize point cloud.
    """
    print("🔄 Initializing camera...")
    camera = CameraController()
    
    if not camera.connect():
        print("❌ Camera connection failed")
        return
    
    print("✅ Camera connected")
    print("📸 Capturing RGB-D data...")
    
    # Capture RGB-D data
    color_image, depth_image, intrinsics = camera.capture_rgbd()
    
    if color_image is None:
        print("❌ Failed to capture image")
        camera.disconnect()
        return
    
    print(f"✅ Captured images: {color_image.shape} color, {depth_image.shape} depth")
    
    # Create point clouds with both real and simulation parameters
    print("\n1️⃣ Creating point cloud with REAL camera parameters...")
    pcd_real = create_point_cloud_from_rgbd(color_image, depth_image, intrinsics, use_sim_params=False)
    print(f"   Generated {len(pcd_real.points)} points")
    
    print("\n2️⃣ Creating point cloud with SIMULATION camera parameters...")
    pcd_sim = create_point_cloud_from_rgbd(color_image, depth_image, intrinsics, use_sim_params=True)
    print(f"   Generated {len(pcd_sim.points)} points")
    
    # Visualize both point clouds
    print("\n🎨 Visualizing point cloud with REAL parameters...")
    visualize_point_cloud_safe(pcd_real, "Point Cloud - Real Camera Parameters")
    
    print("\n🎨 Visualizing point cloud with SIMULATION parameters...")
    visualize_point_cloud_safe(pcd_sim, "Point Cloud - Simulation Camera Parameters")
    
    # Compare the differences
    print(f"\n📊 Comparison:")
    print(f"   Real camera points: {len(pcd_real.points)}")
    print(f"   Simulation camera points: {len(pcd_sim.points)}")
    
    if len(pcd_real.points) > 0 and len(pcd_sim.points) > 0:
        real_bounds = pcd_real.get_axis_aligned_bounding_box()
        sim_bounds = pcd_sim.get_axis_aligned_bounding_box()
        print(f"   Real camera bounds: {np.array(real_bounds.get_extent())}")
        print(f"   Simulation camera bounds: {np.array(sim_bounds.get_extent())}")
    
    # Clean up
    camera.disconnect()
    print("📷 Camera disconnected")

def visualize_existing_pointcloud(file_path: str):
    """
    Visualize an existing point cloud file.
    """
    if not os.path.exists(file_path):
        print(f"❌ File not found: {file_path}")
        return
    
    try:
        print(f"📁 Loading point cloud from: {file_path}")
        pcd = o3d.io.read_point_cloud(file_path)
        
        if len(pcd.points) == 0:
            print("❌ Point cloud is empty")
            return
        
        print(f"✅ Loaded {len(pcd.points)} points")
        visualize_point_cloud_safe(pcd, f"Point Cloud - {os.path.basename(file_path)}")
        
    except Exception as e:
        print(f"❌ Failed to load point cloud: {e}")

def main():
    """Main function with options"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Point Cloud Visualization Tool")
    parser.add_argument("--file", type=str, help="Path to existing point cloud file (.ply)")
    parser.add_argument("--capture", action="store_true", help="Capture new RGB-D data and create point cloud")
    
    args = parser.parse_args()
    
    if args.file:
        visualize_existing_pointcloud(args.file)
    elif args.capture:
        capture_and_visualize()
    else:
        print("Point Cloud Visualization Tool")
        print("Usage:")
        print("  python visualize_pointcloud.py --capture          # Capture and visualize new data")
        print("  python visualize_pointcloud.py --file <path>      # Visualize existing .ply file")
        print("")
        print("Available point cloud files:")
        
        # Look for existing point cloud files
        for root, dirs, files in os.walk("."):
            for file in files:
                if file.endswith(".ply"):
                    print(f"  {os.path.join(root, file)}")

if __name__ == "__main__":
    main()