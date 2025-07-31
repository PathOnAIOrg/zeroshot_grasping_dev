#!/usr/bin/env python3
"""
Point Cloud Analysis Tool - Works without display/OpenGL
Shows point cloud statistics and creates 2D projections
"""

import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import os
from typing import Optional

def analyze_point_cloud(file_path: str):
    """
    Analyze and display point cloud information without requiring OpenGL.
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
        
        points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors) if pcd.has_colors() else None
        
        print(f"✅ Loaded point cloud with {len(points)} points")
        print(f"📊 Point cloud analysis:")
        print(f"   Has colors: {pcd.has_colors()}")
        print(f"   Has normals: {pcd.has_normals()}")
        
        # Basic statistics
        bounds = pcd.get_axis_aligned_bounding_box()
        min_bound = bounds.min_bound
        max_bound = bounds.max_bound
        extent = max_bound - min_bound
        center = bounds.get_center()
        
        print(f"📏 Spatial bounds:")
        print(f"   Min: [{min_bound[0]:.3f}, {min_bound[1]:.3f}, {min_bound[2]:.3f}]")
        print(f"   Max: [{max_bound[0]:.3f}, {max_bound[1]:.3f}, {max_bound[2]:.3f}]")
        print(f"   Size: [{extent[0]:.3f}, {extent[1]:.3f}, {extent[2]:.3f}]")
        print(f"   Center: [{center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f}]")
        
        # Distance from origin
        distances = np.linalg.norm(points, axis=1)
        print(f"📐 Distance from origin:")
        print(f"   Min: {distances.min():.3f}m")
        print(f"   Max: {distances.max():.3f}m") 
        print(f"   Mean: {distances.mean():.3f}m")
        print(f"   Std: {distances.std():.3f}m")
        
        # Create 2D projections using matplotlib
        create_2d_projections(points, colors, os.path.basename(file_path))
        
        # Sample some points
        print(f"\n🔍 Sample points (first 10):")
        for i in range(min(10, len(points))):
            x, y, z = points[i]
            if colors is not None:
                r, g, b = colors[i]
                print(f"   Point {i}: [{x:.3f}, {y:.3f}, {z:.3f}] RGB: [{r:.3f}, {g:.3f}, {b:.3f}]")
            else:
                print(f"   Point {i}: [{x:.3f}, {y:.3f}, {z:.3f}]")
        
        return pcd
        
    except Exception as e:
        print(f"❌ Failed to analyze point cloud: {e}")
        return None

def create_2d_projections(points: np.ndarray, colors: Optional[np.ndarray], title: str):
    """
    Create 2D projections of the 3D point cloud using matplotlib.
    """
    try:
        print("📈 Creating 2D projections...")
        
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        fig.suptitle(f'Point Cloud Projections: {title}', fontsize=14)
        
        # Use colors if available, otherwise use depth coloring
        if colors is not None:
            point_colors = colors
        else:
            # Color by Z-depth
            z_values = points[:, 2]
            point_colors = plt.cm.viridis((z_values - z_values.min()) / (z_values.max() - z_values.min()))
        
        # Sample points for faster plotting (use every 10th point if more than 10k points)
        n_points = len(points)
        if n_points > 10000:
            step = n_points // 10000
            indices = np.arange(0, n_points, step)
            plot_points = points[indices]
            plot_colors = point_colors[indices] if len(point_colors.shape) > 1 else point_colors[indices]
            print(f"   Sampling {len(plot_points)} points from {n_points} for visualization")
        else:
            plot_points = points
            plot_colors = point_colors
            print(f"   Plotting all {n_points} points")
        
        # XY projection (top view)
        axes[0, 0].scatter(plot_points[:, 0], plot_points[:, 1], c=plot_colors, s=1, alpha=0.6)
        axes[0, 0].set_title('Top View (XY)')
        axes[0, 0].set_xlabel('X (m)')
        axes[0, 0].set_ylabel('Y (m)')
        axes[0, 0].axis('equal')
        axes[0, 0].grid(True, alpha=0.3)
        
        # XZ projection (front view)
        axes[0, 1].scatter(plot_points[:, 0], plot_points[:, 2], c=plot_colors, s=1, alpha=0.6)
        axes[0, 1].set_title('Front View (XZ)')
        axes[0, 1].set_xlabel('X (m)')
        axes[0, 1].set_ylabel('Z (m)')
        axes[0, 1].axis('equal')
        axes[0, 1].grid(True, alpha=0.3)
        
        # YZ projection (side view)
        axes[1, 0].scatter(plot_points[:, 1], plot_points[:, 2], c=plot_colors, s=1, alpha=0.6)
        axes[1, 0].set_title('Side View (YZ)')
        axes[1, 0].set_xlabel('Y (m)')
        axes[1, 0].set_ylabel('Z (m)')
        axes[1, 0].axis('equal')
        axes[1, 0].grid(True, alpha=0.3)
        
        # Distance from origin histogram
        distances = np.linalg.norm(plot_points, axis=1)
        axes[1, 1].hist(distances, bins=50, alpha=0.7, color='blue', edgecolor='black')
        axes[1, 1].set_title('Distance from Origin Distribution')
        axes[1, 1].set_xlabel('Distance (m)')
        axes[1, 1].set_ylabel('Number of Points')
        axes[1, 1].grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        # Save the plot
        output_path = f"pointcloud_projections_{title.replace('.ply', '.png')}"
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"💾 2D projections saved to: {output_path}")
        
        # Try to show the plot
        try:
            plt.show(block=False)
            print("🖥️  Plot window opened (close to continue)")
            input("Press Enter to continue...")
            plt.close()
        except:
            print("⚠️  Could not display plot window, but image saved successfully")
            plt.close()
        
    except Exception as e:
        print(f"❌ Failed to create 2D projections: {e}")

def main():
    """Main function"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Point Cloud Analysis Tool (no OpenGL required)")
    parser.add_argument("file_path", help="Path to point cloud file (.ply)")
    
    args = parser.parse_args()
    
    analyze_point_cloud(args.file_path)

if __name__ == "__main__":
    main()