"""
Move-to example using object segmentation and target position calculation.

This script demonstrates:
1. Capturing scene and generating point cloud
2. User clicks on object to segment it
3. Calculate target position above the object
4. Move robot to the target position
5. Visualize the movement
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from typing import List
from scipy.spatial.transform import Rotation as R
from so101_grasp.visualization.vis_grasps import launch_visualizer
from so101_grasp.utils.transform import transform_pcd_cam_to_rob, transform_cam_to_rob
import open3d as o3d
from so101_grasp.api.grasp_interface import grasp_predictor
from scripts.tools.image_selector import ImgClick
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


def calculate_move_position(segmented_pcd: o3d.geometry.PointCloud, height_offset: float = 0.15):
    """
    Calculate a target position above the segmented object.
    
    Args:
        segmented_pcd: Point cloud of the segmented object
        height_offset: How high above the object to position the end effector
        
    Returns:
        target_position: 3D position to move to
        approach_orientation: Orientation for approaching from above
    """
    points = np.asarray(segmented_pcd.points)
    
    # Calculate centroid of the object
    centroid = np.mean(points, axis=0)
    
    # Find the highest point of the object
    max_z = np.max(points[:, 2])
    
    # Target position is above the object
    target_position = np.array([
        centroid[0],
        centroid[1],
        max_z + height_offset
    ])
    
    # Create a downward-facing orientation (approach from above)
    # This creates a rotation matrix for pointing downward
    approach_orientation = R.from_euler('xyz', [np.pi, 0, 0]).as_matrix()
    
    return target_position, approach_orientation


def main():
    """
    Main execution function demonstrating move-to functionality.
    """
    
    # Step 1: Initialize Simulation Environment
    print("Initializing simulation environment...")
    env = SimGrasp(urdf_path=URDF_PATH, frequency=FREQUENCY, objects=SIMULATION_OBJECTS)
    
    # Launch 3D visualizer
    vis = launch_visualizer()

    # Step 2: Capture Scene and Generate Point Cloud
    print("Capturing camera view and generating point cloud...")
    color, depth, _ = env.render_camera()
    
    # Convert RGBA to RGB if needed
    if color.shape[2] == 4:
        color = color[:, :, :3]
    
    pcd = env.create_pointcloud(color, depth)

    # Step 3: Interactive Object Selection
    print("\n" + "="*60)
    print("OBJECT SELECTION")
    print("="*60)
    print("Click on the object you want to move to, then close the image window")
    
    img_click = ImgClick(color, os=OS)
    img_click.get_point_from_img()
    x, y = img_click.x, img_click.y
    
    if x is None or y is None:
        print("No point selected. Exiting...")
        return
    
    print(f"Selected point: ({x}, {y})")

    # Step 4: Object Segmentation
    print("\n" + "="*60)
    print("OBJECT SEGMENTATION")
    print("="*60)
    print("Segmenting object from point cloud...")
    
    segmented_pcd = grasp_predictor.segment_object(
        pointcloud=pcd,
        click_point=(y, x),  # Note: (row, col) format
        image_shape=(480, 640)
    )
    
    num_points = len(segmented_pcd.points)
    print(f"✅ Segmented object contains {num_points} points")

    # Step 5: Calculate Target Position
    print("\n" + "="*60)
    print("TARGET CALCULATION")
    print("="*60)
    
    # Calculate position above the object
    target_position_cam, approach_orientation_cam = calculate_move_position(segmented_pcd)
    print(f"Target position in camera frame: {target_position_cam}")
    
    # Transform to robot frame
    approach_orientation_robot, target_position_robot = transform_cam_to_rob(
        approach_orientation_cam,
        target_position_cam,
        REAL_ROBOT
    )
    print(f"Target position in robot frame: {target_position_robot}")

    # Step 6: Visualize Target Position
    print("\n" + "="*60)
    print("VISUALIZATION")
    print("="*60)
    
    # Transform point cloud to robot frame for visualization
    pcd_rob = transform_pcd_cam_to_rob(pcd, REAL_ROBOT)
    segmented_pcd_rob = transform_pcd_cam_to_rob(segmented_pcd, REAL_ROBOT)
    
    # Create a coordinate frame at the target position for visualization
    target_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    target_frame.translate(target_position_robot)
    target_frame.rotate(approach_orientation_robot, center=target_position_robot)
    
    # Visualize scene with target
    vis.delete()
    vis["/scene_pointcloud"].set_object(
        g.PointCloud(
            position=np.asarray(pcd_rob.points).T,
            color=np.asarray(pcd_rob.colors).T,
            size=0.001
        )
    )
    vis["/segmented_object"].set_object(
        g.PointCloud(
            position=np.asarray(segmented_pcd_rob.points).T,
            color=np.array([[1.0], [0.0], [0.0]]),  # Red for segmented object
            size=0.003
        )
    )
    vis["/target_frame"].set_object(
        g.LineSegments(
            g.PointsGeometry(
                position=np.array([
                    [0, 0, 0], [0.1, 0, 0],  # X axis (red)
                    [0, 0, 0], [0, 0.1, 0],  # Y axis (green)
                    [0, 0, 0], [0, 0, 0.1]   # Z axis (blue)
                ]).T,
                color=np.array([
                    [1.0, 0.0, 0.0], [1.0, 0.0, 0.0],  # Red
                    [0.0, 1.0, 0.0], [0.0, 1.0, 0.0],  # Green
                    [0.0, 0.0, 1.0], [0.0, 0.0, 1.0]   # Blue
                ], dtype=np.float32).T
            ),
            g.LineBasicMaterial(vertexColors=True)
        )
    )
    
    # Apply transformation to visualize at target position
    vis["/target_frame"].set_transform(
        tf.translation_matrix(target_position_robot) @ 
        tf.quaternion_matrix(R.from_matrix(approach_orientation_robot).as_quat())
    )
    
    print(f"✅ Visualized target position")
    print("Check the MeshCat viewer to see the target position")

    # Step 7: Execute Movement
    print("\n" + "="*60)
    print("MOVEMENT EXECUTION")
    print("="*60)
    
    input("Press Enter to move to the target position...")
    
    # For a simple move-to operation, we can use the target position directly
    # In a real system, you would use inverse kinematics to find joint angles
    print(f"Moving to position: {target_position_robot}")
    print(f"With orientation: {approach_orientation_robot}")
    
    # Here you would typically:
    # 1. Use inverse kinematics to find joint angles
    # 2. Plan a trajectory to the target
    # 3. Execute the movement
    
    # For demonstration, let's just move the end effector in simulation
    success = env.move_to_position(
        position=target_position_robot,
        orientation=approach_orientation_robot
    )
    
    if success:
        print("✅ Successfully moved to target position!")
    else:
        print("❌ Failed to reach target position")

    print("\n" + "="*60)
    print("DEMO COMPLETE")
    print("="*60)
    
    # Keep visualization running
    input("Press Enter to exit...")


# Import required visualization modules
import meshcat.geometry as g
import meshcat.transformations as tf


if __name__ == "__main__":
    main()