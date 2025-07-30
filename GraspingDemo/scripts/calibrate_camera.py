"""
This file is for calibrating your camera with respect to the SO-101 robot, i.e. computing the transformation matrix between camera and robot frame.
This is a requirement for the grasp_example_so101.py file.

Instructions:
- Ensure SO-101 is connected
- Ensure depth camera is connected and place it at desired position. We recommend an isometric view angle.
- Fill in the DEFAULT_PORT variable below with the port of your SO-101. See https://huggingface.co/docs/lerobot/en/so101#setup-motors-video
- Put 12 small markers in reachable places for the robot and viewable by the depth camera.
- Run: python3 calibration_so101.py --os LINUX or python3 calibration_so101.py --os MAC
- If you haven't yet recorded robot poses enter 'y' and move the robot jaw tip to the center of each marker and press Enter. Repeat for all 12 points.
- Then a sequence of image windows will pop up. Click the marker centers in the same order as the robot poses. Make sure to close the image window after each click.
- The script will then compute the transformation matrix and save it to config/transform_mat_so101.npy
- The script will also compute the scaling factor and save it to config/scaling_factor_so101.npy
"""

import sys
import os
# Add project root to path
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(project_root)
# Add lerobot to path
sys.path.append(os.path.join(project_root, 'lerobot', 'src'))

import pybullet as pb
import pybullet_data
import time
import numpy as np
import threading
import argparse
from typing import List, Tuple
from so101_grasp.robot.so101_client import SO101Client
from so101_grasp.utils.utils import get_3d_point_from_2d_coordinates
from so101_grasp.vision.camera import CameraController
sys.path.append('scripts/tools')
from image_selector import ImgClick
from so101_grasp.robot import calibrate

# User TODO: Fill in the default port for your SO-101 robot.
# Found ports: /dev/tty.usbmodem5A680107891 (update this to match your robot)
DEFAULT_PORT = "/dev/tty.usbmodem5A680107891"
REAL_ROBOT_ROBOT_POSES_PATH = "config/robot_poses_so101.npy"
URDF_PATH = "SO101/so101_new_calib.urdf"  # Note: You may need to create a SO-101 specific URDF
FREQUENCY = 100
DEFAULT_OS = "MAC"
NUM_POINTS_USE = 12


def setup_simulation() -> int:
    """
    Sets up the PyBullet physics simulation environment.

    Connects to the physics server (GUI), sets gravity, loads the ground plane,
    and configures the simulation environment.

    Returns:
        int: The physics client ID assigned by PyBullet.
    Raises:
        ConnectionError: If connection to the PyBullet simulation fails.
    """
    try:
        physics_client = pb.connect(pb.GUI)
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        pb.setGravity(0, 0, -9.81)
        pb.loadURDF("plane.urdf")
        pb.resetDebugVisualizerCamera(
            cameraDistance=1.5,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0, 0.5]
        )
        return physics_client
    except Exception as e:
        raise ConnectionError(f"Failed to connect to PyBullet simulation: {e}")


def load_robot_urdf(urdf_path: str) -> int:
    """
    Loads the SO-101 robot URDF model into the simulation.

    Args:
        urdf_path (str): Path to the SO-101 URDF file.

    Returns:
        int: The unique body ID assigned to the loaded robot.

    Raises:
        FileNotFoundError: If the URDF file doesn't exist.
        RuntimeError: If loading the URDF fails.
    """
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF file not found: {urdf_path}")
    
    try:
        robot_id = pb.loadURDF(urdf_path, basePosition=[0, 0, 0])
        print(f"Loaded SO-101 robot with ID: {robot_id}")
        return robot_id
    except Exception as e:
        raise RuntimeError(f"Failed to load URDF {urdf_path}: {e}")


def get_joint_info(robot_id: int) -> List[Tuple[int, str]]:
    """
    Retrieves information about all joints in the loaded robot.

    Args:
        robot_id (int): The PyBullet body ID of the robot.

    Returns:
        List[Tuple[int, str]]: List of (joint_index, joint_name) tuples.
    """
    num_joints = pb.getNumJoints(robot_id)
    joint_info = []
    
    for i in range(num_joints):
        info = pb.getJointInfo(robot_id, i)
        joint_name = info[1].decode('utf-8')
        joint_info.append((i, joint_name))
        print(f"Joint {i}: {joint_name}")
    
    return joint_info


def set_joint_positions(robot_id: int, joint_positions: List[float]) -> None:
    """
    Sets the joint positions of the robot in simulation.

    Args:
        robot_id (int): The PyBullet body ID of the robot.
        joint_positions (List[float]): List of joint angles in radians.
    """
    if len(joint_positions) != 6:
        raise ValueError(f"Expected 6 joint positions, got {len(joint_positions)}")
    
    for i in range(6):  # SO-101 has 6 joints
        pb.resetJointState(robot_id, i, joint_positions[i])


def get_end_effector_pose(robot_id: int, end_effector_link_index: int = 6) -> Tuple[List[float], List[float]]:
    """
    Gets the current pose of the end effector.

    Args:
        robot_id (int): The PyBullet body ID of the robot.
        end_effector_link_index (int): The link index of the end effector.

    Returns:
        Tuple[List[float], List[float]]: Position and orientation of the end effector.
    """
    link_state = pb.getLinkState(robot_id, end_effector_link_index)
    position = list(link_state[0])
    orientation = list(link_state[1])
    return position, orientation


def record_robot_poses(robot_client: SO101Client, num_points: int = 12) -> np.ndarray:
    """
    Records robot poses by moving the robot to marker positions.

    Args:
        robot_client (SO101Client): The SO-101 robot client.
        num_points (int): Number of calibration points to record.

    Returns:
        np.ndarray: Array of recorded robot poses.
    """
    poses = []
    print(f"\nRecording {num_points} robot poses...")
    print("Move the robot's jaw tip to the center of each marker and press Enter.")
    print("Make sure to follow the same order for both robot poses and image clicks!")
    
    for i in range(num_points):
        input(f"Position the robot at marker {i+1}/{num_points} and press Enter...")
        
        # Read current joint positions
        joint_positions = robot_client.read_joints()
        poses.append(joint_positions)
        print(f"Recorded pose {i+1}: {joint_positions}")
    
    poses_array = np.array(poses)
    np.save(REAL_ROBOT_ROBOT_POSES_PATH, poses_array)
    print(f"Saved robot poses to {REAL_ROBOT_ROBOT_POSES_PATH}")
    
    return poses_array


def simulate_robot_poses(robot_id: int, poses: np.ndarray) -> List[List[float]]:
    """
    Simulates the robot poses and gets end effector positions.

    Args:
        robot_id (int): The PyBullet body ID of the robot.
        poses (np.ndarray): Array of joint positions.

    Returns:
        List[List[float]]: List of end effector positions in 3D space.
    """
    end_effector_positions = []
    
    print("\nSimulating robot poses...")
    for i, pose in enumerate(poses):
        set_joint_positions(robot_id, pose)
        pb.stepSimulation()
        time.sleep(0.1)  # Allow simulation to update
        
        position, _ = get_end_effector_pose(robot_id)
        end_effector_positions.append(position)
        print(f"Pose {i+1} - End effector position: {position}")
    
    return end_effector_positions


def capture_and_process_images(num_points: int = 12) -> List[List[float]]:
    """
    Captures images and processes user clicks to get 2D coordinates.

    Args:
        num_points (int): Number of points to capture.

    Returns:
        List[List[float]]: List of 3D coordinates from user clicks.
    """
    coordinates_3d = []
    
    print(f"\nCapturing images and processing clicks for {num_points} points...")
    print("Click on the marker centers in the SAME ORDER as the robot poses!")
    
    for i in range(num_points):
        print(f"\nProcessing point {i+1}/{num_points}")
        
        # Capture point cloud
        depth_image, color_image, intrinsics = capture_pointcloud()
        
        # Get user click
        img_click = ImgClick(color_image)
        coordinates_2d = img_click.get_coordinates()
        
        if coordinates_2d is None:
            print(f"No click detected for point {i+1}. Skipping...")
            continue
            
        # Convert 2D coordinates to 3D
        coordinates_3d_point = get_3d_point_from_2d_coordinates(
            coordinates_2d, depth_image, intrinsics
        )
        
        if coordinates_3d_point is not None:
            coordinates_3d.append(coordinates_3d_point)
            print(f"Point {i+1} - 2D: {coordinates_2d}, 3D: {coordinates_3d_point}")
        else:
            print(f"Failed to convert 2D to 3D for point {i+1}")
    
    return coordinates_3d


def main():
    """
    Main calibration function for SO-101 robot.
    """
    parser = argparse.ArgumentParser(description="SO-101 Robot Camera Calibration")
    parser.add_argument("--os", choices=["LINUX", "MAC"], required=True,
                       help="Operating system (LINUX or MAC)")
    args = parser.parse_args()
    
    print("Starting SO-101 robot camera calibration...")
    print(f"Operating System: {args.os}")
    
    # Create config directory if it doesn't exist
    os.makedirs("config", exist_ok=True)
    
    # Initialize robot client
    try:
        print(f"Attempting to connect to SO-101 robot on port: {DEFAULT_PORT}")
        robot_client = SO101Client(port=DEFAULT_PORT, follower=True)
        print("Successfully connected to SO-101 robot")
    except Exception as e:
        print(f"Failed to connect to SO-101 robot: {e}")
        print(f"Make sure your SO-101 is connected and the port {DEFAULT_PORT} is correct.")
        print("You can find available ports by running: python -m lerobot.find_port")
        print("Look for ports like /dev/tty.usbmodem* on macOS or /dev/ttyACM* on Linux")
        return
    
    # Setup simulation
    try:
        physics_client = setup_simulation()
        robot_id = load_robot_urdf(URDF_PATH)
        joint_info = get_joint_info(robot_id)
    except Exception as e:
        print(f"Failed to setup simulation: {e}")
        return
    
    # Check if robot poses already exist
    if os.path.exists(REAL_ROBOT_ROBOT_POSES_PATH):
        response = input(f"Robot poses file {REAL_ROBOT_ROBOT_POSES_PATH} already exists. Use existing poses? (y/n): ")
        if response.lower() == 'y':
            poses = np.load(REAL_ROBOT_ROBOT_POSES_PATH)
            print(f"Loaded existing robot poses: {poses.shape}")
        else:
            poses = record_robot_poses(robot_client, NUM_POINTS_USE)
    else:
        poses = record_robot_poses(robot_client, NUM_POINTS_USE)
    
    # Simulate robot poses to get end effector positions
    end_effector_positions = simulate_robot_poses(robot_id, poses)
    
    # Capture images and get 3D coordinates from user clicks
    coordinates_3d = capture_and_process_images(NUM_POINTS_USE)
    
    # Ensure we have the same number of points
    min_points = min(len(end_effector_positions), len(coordinates_3d))
    if min_points < 4:
        print(f"Error: Need at least 4 matching points for calibration, got {min_points}")
        return
    
    print(f"\nUsing {min_points} points for calibration")
    
    # Perform calibration
    try:
        transform_matrix, scaling_factor = calibrate(
            np.array(end_effector_positions[:min_points]),
            np.array(coordinates_3d[:min_points])
        )
        
        # Save results
        np.save("config/transform_mat_so101.npy", transform_matrix)
        np.save("config/scaling_factor_so101.npy", scaling_factor)
        
        print(f"\nCalibration completed successfully!")
        print(f"Transform matrix saved to: config/transform_mat_so101.npy")
        print(f"Scaling factor saved to: config/scaling_factor_so101.npy")
        print(f"Transform matrix:\n{transform_matrix}")
        print(f"Scaling factor: {scaling_factor}")
        
    except Exception as e:
        print(f"Calibration failed: {e}")
    
    finally:
        # Cleanup
        pb.disconnect()
        print("Calibration process completed.")


if __name__ == "__main__":
    main()