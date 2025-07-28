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
from so101_grasp.api.grasp_interface import Grasp, grasp_predictor
import sys
sys.path.append('scripts/tools')
from image_selector import ImgClick
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
URDF_PATH = "assets/urdf/so101_new_calib.urdf"


def main():
    """
    Main execution function demonstrating custom grasp prediction pipeline.
    """
    
    # Step 1: Initialize Simulation Environment
    print("Initializing simulation environment...")
    env = SimGrasp(urdf_path=URDF_PATH, frequency=FREQUENCY, objects=SIMULATION_OBJECTS)
    
    # Launch 3D visualizer
    vis = launch_visualizer()

    # Step 2: Capture Scene and Generate Point Cloud
    print("Capturing camera view and generating point cloud...")
    color, depth, _ = env.render_camera()
    pcd = env.create_pointcloud(color, depth)

    # Step 3: Interactive Object Selection
    print("\n" + "="*60)
    print("OBJECT SELECTION")
    print("="*60)
    print("Click on the object you want to grasp, then close the image window")
    
    img_click = ImgClick(color, os=OS)
    img_click.show_and_get_click()
    x, y = img_click.x, img_click.y
    
    if x is None or y is None:
        print("No point selected. Exiting...")
        return
    
    print(f"Selected point: ({x}, {y})")

    # Step 4: Object Segmentation (Custom Implementation)
    print("\n" + "="*60)
    print("OBJECT SEGMENTATION")
    print("="*60)
    print("Segmenting object from point cloud...")
    
    # Use your custom segmentation implementation
    segmented_pcd = grasp_predictor.segment_object(
        pointcloud=pcd,
        click_point=(y, x),  # Note: (row, col) format
        image_shape=(480, 640)
    )
    
    num_points = len(segmented_pcd.points)
    print(f"✅ Segmented object contains {num_points} points")

    # Step 5: Grasp Prediction (Custom Implementation)
    print("\n" + "="*60)
    print("GRASP PREDICTION")
    print("="*60)
    print("Predicting grasps for segmented object...")
    
    # Use your custom grasp prediction implementation
    predicted_grasps = grasp_predictor.predict_grasps(
        object_pointcloud=segmented_pcd,
        num_grasps=10
    )
    
    print(f"✅ Generated {len(predicted_grasps)} grasp predictions")

    # Step 6: Transform Grasps to Robot Frame
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

    # Step 7: Filter Reachable Grasps (Custom Implementation)
    print("\n" + "="*60)
    print("GRASP FILTERING")
    print("="*60)
    print("Filtering grasps for robot reachability...")
    
    # Use your custom reachability filter
    valid_indices, valid_joint_angles = grasp_predictor.filter_reachable_grasps(
        grasps=predicted_grasps_robot_frame,
        robot_name="so101"
    )
    
    print(f"✅ Found {len(valid_indices)} reachable grasps out of {len(predicted_grasps_robot_frame)}")

    if len(valid_indices) == 0:
        print("❌ No valid grasps found. Exiting...")
        return

    # Step 8: Visualize Valid Grasps
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

    # Step 9: Execute Best Grasp
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
        
        # Step 10: Place Object
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