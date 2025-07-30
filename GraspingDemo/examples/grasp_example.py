"""


This script demonstrates the same workflow as grasp_example.py but uses
your own implementation of the grasp prediction interface.
"""

import numpy as np
from typing import List
from scipy.spatial.transform import Rotation as R
from so101_grasp.visualization.vis_grasps import vis_grasps_meshcat, launch_visualizer
from so101_grasp.utils.transform import transform_pcd_cam_to_rob, transform_cam_to_rob, transform_grasps_inv, grasp_service_to_robot_format, make_robot_config
import open3d as o3d
from so101_grasp.api.grasp_interface import Grasp, grasp_predictor, ThinkGraspPredictor
import sys
import cv2
import imageio
import pybullet as pb

from so101_grasp.tools.image_selector import ImgClick
from so101_grasp.utils.utils import get_3d_point_from_2d_coordinates
from so101_grasp.simulation.sim import (
    SimGrasp, 
    ObjectInfo, 
    CUBE_ORIENTATION, 
    CUBE_SCALING, 
    SPHERE_ORIENTATION, 
    SPHERE_SCALING, 
    TRAY_ORIENTATION, 
    TRAY_SCALING, 
    TRAY_POS, 
    CUBE_RGBA, 
    SPHERE_RGBA, 
    SPHERE_MASS,
    DUCK_ORIENTATION,
    DUCK_SCALING,
)

REAL_ROBOT = False
OS = "LINUX"  # "MAC" or "LINUX"

# Define simulation objects
SIMULATION_OBJECTS = [
    ObjectInfo(
        urdf_path="cube_small.urdf",
        position=[0.35, 0.0, 0.025],
        orientation=CUBE_ORIENTATION,
        scaling=CUBE_SCALING,
        color=CUBE_RGBA
    ),
    ObjectInfo(
        urdf_path="cube_small.urdf",
        position=[0.25, 0., 0.025],
        orientation=CUBE_ORIENTATION,
        scaling=CUBE_SCALING,
        color=CUBE_RGBA
    ),
    ObjectInfo(
        urdf_path="tray/traybox.urdf",
        position=TRAY_POS,
        orientation=TRAY_ORIENTATION,
        scaling=TRAY_SCALING
    ),
    ObjectInfo(
        urdf_path="sphere2.urdf",
        position=[0.2, 0.1, 0.025],
        orientation=SPHERE_ORIENTATION,
        scaling=SPHERE_SCALING,
        color=SPHERE_RGBA,
        mass=SPHERE_MASS
    ),
    ObjectInfo(
        urdf_path="duck_vhacd.urdf",
        position=[0.3, 0.05, 0.],
        orientation=DUCK_ORIENTATION,
        scaling=DUCK_SCALING
    )
]

FREQUENCY = 30
URDF_PATH = "../assets/urdf/so101_new_calib.urdf"





def main():
    """
    Main execution function demonstrating ThinkGrasp prediction pipeline.
    """
    
    # Step 1: Initialize Simulation Environment
    print("Initializing simulation environment...")
    env = SimGrasp(urdf_path=URDF_PATH, frequency=FREQUENCY, objects=SIMULATION_OBJECTS)
    
    # Launch 3D visualizer
    vis = launch_visualizer()

    # Step 2: Capture Scene and Generate Point Cloud
    print("Capturing camera view and generating point cloud...")
    color, depth, _ = env.render_camera()
    
    # Save RGB image, depth image, and debug text
    print("Saving images and debug information...")
    
    # Save RGB image (convert from RGB to BGR for OpenCV)
    rgb_bgr = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)
    cv2.imwrite("rgb.jpg", rgb_bgr)
    
    # Save depth image (multiple formats for debugging)
    # 1. Save raw depth as 16-bit PNG (preserves actual depth values in mm)
    depth_raw = np.clip(depth * 1000, 0, 65535).astype(np.uint16)  # Convert to mm, clip to 16-bit range
    cv2.imwrite("depth_raw.png", depth_raw)
    
    # 2. Save normalized depth for visualization (0-255 range)
    depth_valid = np.where(np.isfinite(depth) & (depth > 0), depth, 0)
    depth_max = np.max(depth_valid) if np.any(depth_valid > 0) else 10.0
    depth_normalized = np.clip(depth_valid / depth_max * 255, 0, 255).astype(np.uint8)
    cv2.imwrite("depth.png", depth_normalized)
    
    # Save debug text
    with open("text.txt", "w") as f:
        f.write("Grasp Example Debug Information\n")
        f.write("================================\n\n")
        f.write(f"Color shape: {color.shape}\n")
        f.write(f"Depth shape: {depth.shape}\n")
        f.write(f"Depth min/max: {np.min(depth):.4f} / {np.max(depth):.4f}\n")
        f.write(f"Depth finite values: {np.sum(np.isfinite(depth))} / {depth.size}\n")
        f.write(f"Depth positive values: {np.sum(depth > 0)} / {depth.size}\n")
        f.write(f"Depth range 0.1-10m: {np.sum((depth > 0.1) & (depth < 10))} / {depth.size}\n")
        f.write(f"Depth has inf values: {np.sum(np.isinf(depth))}\n")
        f.write(f"Depth has nan values: {np.sum(np.isnan(depth))}\n")
        f.write(f"Depth max valid value: {depth_max:.4f}m\n")
        f.write(f"\nSimulation Objects: {len(SIMULATION_OBJECTS)}\n")
        for i, obj in enumerate(SIMULATION_OBJECTS):
            f.write(f"  Object {i+1}: {obj.urdf_path} at {obj.position}\n")
    
    print("✅ Saved rgb.jpg, depth.png, depth_raw.png, and text.txt")
    
    # Generate heightmap
    print("Generating orthographic heightmap...")
    try:
        cmap, hmap, mask = get_true_heightmap(env)
        print("✅ Saved color_map.png, height_map.png, and mask.png")
        print(f"  Heightmap shape: {hmap.shape}")
        print(f"  Height range: {hmap.min():.4f} to {hmap.max():.4f} meters")
    except Exception as e:
        print(f"❌ Failed to generate heightmap: {e}")
        import traceback
        traceback.print_exc()
    
    # Debug depth information
    print(f"Debug info:")
    print(f"  Color shape: {color.shape}")
    print(f"  Depth shape: {depth.shape}")
    print(f"  Depth min/max: {np.min(depth):.4f} / {np.max(depth):.4f}")
    print(f"  Depth finite values: {np.sum(np.isfinite(depth))} / {depth.size}")
    print(f"  Depth positive values: {np.sum(depth > 0)} / {depth.size}")
    print(f"  Depth range 0.1-10m: {np.sum((depth > 0.1) & (depth < 10))} / {depth.size}")

    pcd = env.create_pointcloud(color, depth)
    print(f"  Point cloud points: {len(pcd.points)}")
    
    # If no points, let's try to fix the depth
    if len(pcd.points) == 0:
        print("  ⚠️  No points in point cloud, attempting to fix depth values...")
        # Clip depth to reasonable range and replace inf/nan with zeros
        depth_fixed = np.copy(depth)
        depth_fixed[~np.isfinite(depth_fixed)] = 0
        depth_fixed = np.clip(depth_fixed, 0, 10.0)  # Max 10 meters
        
        # Try creating point cloud again
        pcd = env.create_pointcloud(color, depth_fixed)
        print(f"  After fix - Point cloud points: {len(pcd.points)}")
        
        if len(pcd.points) == 0:
            print("  ❌ Still no points. Creating artificial depth map...")
            # Create a simple depth map for testing
            h, w = depth.shape
            artificial_depth = np.ones((h, w), dtype=np.float32) * 0.5  # 50cm depth
            # Add some variation
            y_coords, x_coords = np.meshgrid(np.arange(h), np.arange(w), indexing='ij')
            artificial_depth += 0.1 * np.sin(x_coords / 50) * np.cos(y_coords / 50)
            
            pcd = env.create_pointcloud(color, artificial_depth)
            print(f"  With artificial depth - Point cloud points: {len(pcd.points)}")
            depth = artificial_depth  # Use artificial depth for the rest of the pipeline

    # Step 3: Goal Specification
    print("\n" + "="*60)
    print("GOAL SPECIFICATION")
    print("="*60)
    goal_text = input("Enter your grasping goal (e.g., 'pick up the blue cube'): ")
    
    if not goal_text.strip():
        goal_text = "pick up the object"  # Default goal
    
    print(f"Goal: {goal_text}")

    # Step 4: ThinkGrasp Prediction
    print("\n" + "="*60)
    print("THINKGRASP PREDICTION")
    print("="*60)
    print("Using ThinkGrasp API to predict grasp pose...")
    print("⏳ Calling ThinkGrasp API - this may take a moment...")
    print("   Make sure your realarm310.py server is running on localhost:5000")
    
    # Use ThinkGrasp API to predict grasp directly from images and goal
    think_grasp_predictor = ThinkGraspPredictor()
    
    try:
        predicted_grasp = think_grasp_predictor.predict_grasp_from_images(
            rgb_image=color,
            depth_image=depth,
            goal_text=goal_text
        )
        predicted_grasps = [predicted_grasp]  # Single grasp prediction
        print(f"✅ Generated grasp prediction from ThinkGrasp API")
        print(f"   Position: {predicted_grasp.translation}")
        print(f"   Rotation shape: {predicted_grasp.rotation.shape}")
    except Exception as e:
        print(f"❌ Failed to get prediction from ThinkGrasp API: {e}")
        print("Using fallback grasp instead...")
        predicted_grasp = Grasp(
            rotation=np.eye(3),
            translation=np.array([0.3, 0.0, 0.1])
        )
        predicted_grasps = [predicted_grasp]

    # Step 5: Transform Grasps to Robot Frame
    print("\n" + "="*60)
    print("COORDINATE TRANSFORMATION")
    print("="*60)
    
    # Transform grasps from camera frame to robot base frame
    predicted_grasps_robot_frame = []
    for grasp in predicted_grasps:
        rot_transformed, trans_transformed = transform_cam_to_rob(
            grasp.rotation, 
            grasp.translation, 
            REAL_ROBOT
        )
        predicted_grasps_robot_frame.append(
            Grasp(rotation=rot_transformed, translation=trans_transformed)
        )
    
    print(f"✅ Transformed {len(predicted_grasps_robot_frame)} grasps to robot frame")

    # Step 6: Filter Reachable Grasps
    print("\n" + "="*60)
    print("GRASP FILTERING")
    print("="*60)
    print("Filtering grasps for robot reachability...")
    
    # Use reachability filter
    valid_indices, valid_joint_angles = think_grasp_predictor.filter_reachable_grasps(
        grasps=predicted_grasps_robot_frame,
        robot_name="so101"
    )
    
    print(f"✅ Found {len(valid_indices)} reachable grasps out of {len(predicted_grasps_robot_frame)}")

    if len(valid_indices) == 0:
        print("❌ No valid grasps found. Exiting...")
        return

    # Step 7: Visualize Valid Grasps
    print("\n" + "="*60)
    print("GRASP VISUALIZATION")
    print("="*60)
    
    # Convert to format expected by visualization
    grasps_for_vis = []
    for idx in valid_indices:
        grasp = predicted_grasps_robot_frame[idx]
        grasps_for_vis.append({
            "rotation": grasp.rotation.tolist(),
            "translation": grasp.translation.tolist()
        })
    
    # Apply visualization transformation
    grasps_vis = transform_grasps_inv([Grasp.from_list_format(g) for g in grasps_for_vis])
    
    # Transform point cloud to robot frame for visualization
    pcd_rob = transform_pcd_cam_to_rob(pcd, REAL_ROBOT)
    
    # Visualize all valid grasps
    vis_grasps_meshcat(
        vis,
        pcd_rob,
        [g.to_list_format() for g in grasps_vis],
        "all_valid_grasps"
    )
    
    print(f"✅ Visualized {len(grasps_vis)} valid grasps")
    print("Check the MeshCat viewer to see the grasps")

    # Step 8: Execute Best Grasp
    print("\n" + "="*60)
    print("GRASP EXECUTION")
    print("="*60)
    
    # Select the first valid grasp
    best_grasp_idx = valid_indices[0]
    best_grasp = predicted_grasps_robot_frame[best_grasp_idx]
    best_joint_angles = valid_joint_angles[0]
    
    print(f"Executing grasp {best_grasp_idx}:")
    print(f"  Position: {best_grasp.translation}")
    print(f"  Joint angles: {best_joint_angles}")
    
    # Visualize selected grasp
    vis_grasps_meshcat(
        vis,
        pcd_rob,
        [transform_grasps_inv([best_grasp])[0].to_list_format()],
        "selected_grasp"
    )
    
    # Apply robot-specific transformation
    robot_config = make_robot_config("so101")
    grasp_robot_format = grasp_service_to_robot_format([best_grasp], robot_config)[0]
    
    # Execute grasp in simulation
    input("Press Enter to execute the grasp...")
    success = env.execute_grasp(
        translation=grasp_robot_format.translation.tolist(),
        rotation=grasp_robot_format.rotation.tolist(),
        gripper_opening=1.0
    )
    
    if success:
        print("✅ Grasp executed successfully!")
        
        # Step 9: Place Object
        print("\nMoving to drop position...")
        drop_success = env.drop(TRAY_POS)
        if drop_success:
            print("✅ Object placed in tray successfully!")
    else:
        print("❌ Grasp execution failed")

    print("\n" + "="*60)
    print("PIPELINE COMPLETE")
    print("="*60)


if __name__ == "__main__":
    main()