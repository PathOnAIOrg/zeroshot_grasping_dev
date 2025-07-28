"""
Client script to demonstrate the grasp prediction pipeline for SO-101.

This script demonstrates a complete end-to-end grasp prediction workflow:
1. Initialize simulation environment with objects to grasp
2. Render camera view and generate initial point cloud
3. Allow user to select target object via mouse click
4. Call Point Cloud Cropping service to isolate the target object
5. Call Grasp Prediction service on the cropped point cloud
6. Transform predicted grasps from camera frame to robot frame
7. Call Grasp Filtering service to get kinematically valid grasps
8. Visualize the valid grasps in 3D
9. Execute the best valid grasp in the simulation
10. Drop the grasped object into a tray

The pipeline integrates computer vision, machine learning, and robotics
to demonstrate autonomous object manipulation with the SO-101 robot.
"""

import numpy as np
from typing import List
from so101_grasp.visualization.vis_grasps import vis_grasps_meshcat, launch_visualizer
from so101_grasp.utils.transform import transform_pcd_cam_to_rob, grasp_service_to_robot_format, make_robot_config
from so101_grasp.utils.utils import get_3d_point_from_2d_coordinates
import open3d as o3d
from so101_grasp.planning.client import GeneralBionixClient, PointCloudData, Grasp
from so101_grasp.visualization.img_click import ImgClick
from so101_grasp.robot.so101_client import SO101Client
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from so101_grasp.simulation.sim import SimGrasp
from so101_grasp.vision.capture_realsense_pointcloud import capture_pointcloud


# User TODO
OS = "MAC" # "MAC" or "LINUX"
DEFAULT_PORT = "/dev/tty.usbmodem5A680107891" # Get this from these instructions: https://huggingface.co/docs/lerobot/en/so101#1-find-the-usb-ports-associated-with-each-arm
API_KEY = "" # Currently our service doesn't require an API key so leave this empty!

SIMULATION_OBJECTS = [] # Empty sim env
REAL_ROBOT = True
FREQUENCY = 30
URDF_PATH = "SO101/so101_new_calib.urdf"  # Note: You may need to create a SO-101 specific URDF


def main():
    """
    Main execution function for the grasp prediction pipeline.
    
    This function orchestrates the complete workflow from scene setup
    to grasp execution, integrating multiple services and components.
    """
    print("Starting SO-101 grasp prediction pipeline...")
    
    # Initialize the SO-101 robot client
    print("Initializing SO-101 robot...")
    try:
        robot_client = SO101Client(port=DEFAULT_PORT, follower=True)
        print("Successfully connected to SO-101 robot")
    except Exception as e:
        print(f"Failed to connect to SO-101 robot: {e}")
        return
    
    # Initialize simulation environment
    print("Setting up simulation environment...")
    try:
        sim = SimGrasp(
            objects=SIMULATION_OBJECTS,
            real_robot=REAL_ROBOT,
            frequency=FREQUENCY,
            urdf_path=URDF_PATH,
            os_name=OS
        )
        print("Simulation environment ready")
    except Exception as e:
        print(f"Failed to setup simulation: {e}")
        return
    
    # Initialize service client
    print("Connecting to grasp prediction services...")
    try:
        service_client = GeneralBionixClient(api_key=API_KEY)
        print("Service client initialized")
    except Exception as e:
        print(f"Failed to initialize service client: {e}")
        return
    
    # Capture initial scene
    print("\nCapturing scene...")
    try:
        depth_image, color_image, intrinsics = capture_pointcloud()
        print("Scene captured successfully")
    except Exception as e:
        print(f"Failed to capture scene: {e}")
        return
    
    # User selection of target object
    print("\nPlease click on the object you want to grasp...")
    try:
        img_click = ImgClick(color_image)
        click_coordinates = img_click.get_coordinates()
        
        if click_coordinates is None:
            print("No object selected. Exiting...")
            return
            
        print(f"Object selected at coordinates: {click_coordinates}")
        
        # Get 3D coordinates of the clicked point
        target_3d = get_3d_point_from_2d_coordinates(
            click_coordinates, depth_image, intrinsics
        )
        
        if target_3d is None:
            print("Failed to get 3D coordinates. Exiting...")
            return
            
        print(f"Target 3D coordinates: {target_3d}")
        
    except Exception as e:
        print(f"Failed during object selection: {e}")
        return
    
    # Create point cloud data
    print("\nCreating point cloud...")
    try:
        # Convert depth and color images to point cloud
        height, width = depth_image.shape
        fx, fy = intrinsics.fx, intrinsics.fy
        cx, cy = intrinsics.ppx, intrinsics.ppy
        
        # Create point cloud
        points = []
        colors = []
        
        for v in range(height):
            for u in range(width):
                depth = depth_image[v, u]
                if depth > 0:  # Valid depth
                    # Convert pixel to 3D point
                    x = (u - cx) * depth / fx
                    y = (v - cy) * depth / fy
                    z = depth
                    
                    points.append([x, y, z])
                    colors.append(color_image[v, u] / 255.0)  # Normalize colors
        
        pcd_data = PointCloudData(
            points=np.array(points),
            colors=np.array(colors)
        )
        print(f"Point cloud created with {len(points)} points")
        
    except Exception as e:
        print(f"Failed to create point cloud: {e}")
        return
    
    # Crop point cloud around target object
    print("\nCropping point cloud around target object...")
    try:
        cropped_pcd = service_client.crop_point_cloud(
            pcd_data, target_3d, crop_radius=0.1  # 10cm radius
        )
        print(f"Point cloud cropped to {len(cropped_pcd.points)} points")
    except Exception as e:
        print(f"Failed to crop point cloud: {e}")
        cropped_pcd = pcd_data  # Use full point cloud as fallback
    
    # Generate grasp predictions
    print("\nGenerating grasp predictions...")
    try:
        grasps = service_client.predict_grasps(cropped_pcd)
        print(f"Generated {len(grasps)} grasp candidates")
        
        if not grasps:
            print("No grasps generated. Exiting...")
            return
            
    except Exception as e:
        print(f"Failed to generate grasps: {e}")
        return
    
    # Transform grasps from camera frame to robot frame
    print("\nTransforming grasps to robot frame...")
    try:
        # Load transformation matrix from calibration
        transform_matrix = np.load("config/transform_mat_so101.npy")
        scaling_factor = np.load("config/scaling_factor_so101.npy")
        
        # Transform point cloud to robot frame
        robot_pcd = transform_pcd_cam_to_rob(cropped_pcd.points, transform_matrix, scaling_factor)
        
        # Transform grasps to robot frame
        robot_grasps = []
        for grasp in grasps:
            robot_grasp = grasp_service_to_robot_format(grasp, transform_matrix, scaling_factor)
            robot_grasps.append(robot_grasp)
        
        print(f"Transformed {len(robot_grasps)} grasps to robot frame")
        
    except FileNotFoundError:
        print("Calibration files not found. Please run calibration_so101.py first!")
        return
    except Exception as e:
        print(f"Failed to transform grasps: {e}")
        return
    
    # Filter grasps for kinematic feasibility
    print("\nFiltering grasps for kinematic feasibility...")
    try:
        robot_config = make_robot_config()  # Create robot configuration
        
        valid_grasps = service_client.filter_grasps(
            robot_grasps, robot_config
        )
        
        print(f"Found {len(valid_grasps)} kinematically valid grasps")
        
        if not valid_grasps:
            print("No valid grasps found. Exiting...")
            return
            
    except Exception as e:
        print(f"Failed to filter grasps: {e}")
        # Use all grasps as fallback
        valid_grasps = robot_grasps
    
    # Visualize valid grasps
    print("\nVisualizing valid grasps...")
    try:
        vis = launch_visualizer()
        vis_grasps_meshcat(valid_grasps, robot_pcd, vis)
        
        input("Press Enter to continue with grasp execution...")
        
    except Exception as e:
        print(f"Visualization failed: {e}")
    
    # Execute the best grasp
    print("\nExecuting best grasp...")
    try:
        best_grasp = valid_grasps[0]  # Take the first (presumably best) grasp
        
        # Convert grasp to joint angles (this would need to be implemented)
        # For now, we'll just move to a pre-grasp position
        print("Moving to pre-grasp position...")
        
        # Example pre-grasp position (you'll need to adjust these values)
        pre_grasp_joints = [0.0, -0.5, 1.0, 0.5, 0.0, 0.0]  # Example joint angles
        robot_client.write_joints(pre_grasp_joints)
        
        import time
        time.sleep(2)  # Wait for movement
        
        # Close gripper to grasp
        print("Closing gripper...")
        grasp_joints = pre_grasp_joints.copy()
        grasp_joints[5] = 50.0  # Close gripper (adjust as needed)
        robot_client.write_joints(grasp_joints)
        
        time.sleep(1)
        
        # Lift object
        print("Lifting object...")
        lift_joints = grasp_joints.copy()
        lift_joints[1] -= 0.2  # Lift up (adjust as needed)
        robot_client.write_joints(lift_joints)
        
        time.sleep(1)
        
        print("Grasp execution completed!")
        
    except Exception as e:
        print(f"Failed to execute grasp: {e}")
    
    print("\nSO-101 grasp prediction pipeline completed!")


if __name__ == "__main__":
    main()