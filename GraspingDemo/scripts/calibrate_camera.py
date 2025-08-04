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

from so101_grasp.tools.image_selector import ImgClick
from so101_grasp.robot.calibration import RobotCalibrator
from scipy.optimize import least_squares

# User TODO: Fill in the default port for your SO-101 robot.
# Found ports: /dev/tty.usbmodem5A680107891 (update this to match your robot)
DEFAULT_PORT = "/dev/ttyACM0"
REAL_ROBOT_ROBOT_POSES_PATH = "config/robot_poses_so101.npy"
URDF_PATH = "../assets/urdf/so101_new_calib.urdf"  # Note: You may need to create a SO-101 specific URDF
FREQUENCY = 100
DEFAULT_OS = "LINUX"
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
    
    print(f"\nRobot has {num_joints} joints:")
    for i in range(num_joints):
        info = pb.getJointInfo(robot_id, i)
        joint_name = info[1].decode('utf-8')
        link_name = info[12].decode('utf-8')
        joint_info.append((i, joint_name))
        print(f"Joint {i}: {joint_name} -> Link: {link_name}")
    
    # Test end effector pose detection
    print(f"\nTesting end effector links:")
    for i in range(num_joints):
        try:
            link_state = pb.getLinkState(robot_id, i)
            if link_state is not None:
                pos = link_state[0]
                print(f"Link {i}: Valid pose at {pos}")
            else:
                print(f"Link {i}: getLinkState returned None")
        except Exception as e:
            print(f"Link {i}: Error - {e}")
    
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


def get_end_effector_pose(robot_id: int, end_effector_link_index: int = None) -> Tuple[List[float], List[float]]:
    """
    Gets the current pose of the end effector.

    Args:
        robot_id (int): The PyBullet body ID of the robot.
        end_effector_link_index (int): The link index of the end effector. If None, uses the last link.

    Returns:
        Tuple[List[float], List[float]]: Position and orientation of the end effector.
    """
    # If no link index specified, use the last link (gripper/jaw)
    if end_effector_link_index is None:
        num_joints = pb.getNumJoints(robot_id)
        # Try different end effector indices, starting from the last one
        for link_idx in range(num_joints - 1, -1, -1):
            try:
                link_state = pb.getLinkState(robot_id, link_idx)
                if link_state is not None:
                    end_effector_link_index = link_idx
                    break
            except:
                continue
        
        if end_effector_link_index is None:
            raise ValueError(f"Could not find valid end effector link for robot {robot_id}")
    
    try:
        link_state = pb.getLinkState(robot_id, end_effector_link_index)
        if link_state is None:
            raise ValueError(f"getLinkState returned None for robot {robot_id}, link {end_effector_link_index}")
        
        position = list(link_state[0])
        orientation = list(link_state[1])
        return position, orientation
        
    except Exception as e:
        print(f"Error getting end effector pose for link {end_effector_link_index}: {e}")
        # Fallback: try using the base link position if available
        try:
            base_pos, base_orn = pb.getBasePositionAndOrientation(robot_id)
            print(f"Using base position as fallback: {base_pos}")
            return list(base_pos), list(base_orn)
        except Exception as e2:
            raise RuntimeError(f"Failed to get end effector pose and base pose: {e}, {e2}")


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
    print("Disabling motor torque for manual positioning...")
    
    # Disable torque on all motors for manual movement
    try:
        robot_client.robot.bus.disable_torque()
        print("✅ Motor torque disabled - you can now move the robot manually")
    except Exception as e:
        print(f"⚠️ Warning: Could not disable torque: {e}")
        print("You may need to manually disable torque or use the robot's physical button")
    
    print("\nMove the robot's jaw tip to the center of each marker and press Enter.")
    print("Make sure to follow the same order for both robot poses and image clicks!")
    
    for i in range(num_points):
        input(f"Position the robot at marker {i+1}/{num_points} and press Enter...")
        
        # Temporarily enable torque to read position
        try:
            robot_client.robot.bus.enable_torque()
            time.sleep(0.5)  # Allow motors to stabilize
            
            # Read current joint positions
            joint_positions = robot_client.read_joints()
            poses.append(joint_positions)
            print(f"Recorded pose {i+1}: {[f'{p:.3f}' for p in joint_positions]}")
            
            # Disable torque again for next positioning (except for last point)
            if i < num_points - 1:
                robot_client.robot.bus.disable_torque()
                    
        except Exception as e:
            print(f"⚠️ Error reading position: {e}")
            # Try to read anyway
            joint_positions = robot_client.read_joints()
            poses.append(joint_positions)
    
    # Re-enable torque after all poses are recorded
    try:
        robot_client.robot.bus.enable_torque()
        time.sleep(1.0)  # Allow motors to stabilize
        print("✅ Motor torque re-enabled")
    except Exception as e:
        print(f"⚠️ Warning: Could not re-enable torque: {e}")
    
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


def compute_camera_robot_transform(robot_points: np.ndarray, camera_points: np.ndarray) -> tuple:
    """
    Compute the transformation matrix from camera to robot coordinate system.
    
    Args:
        robot_points: Nx3 array of 3D points in robot coordinate system
        camera_points: Nx3 array of corresponding 3D points in camera coordinate system
        
    Returns:
        tuple: (4x4 transformation matrix, scaling factor)
    """
    def residual_function(params, robot_pts, camera_pts):
        """Residual function for optimization."""
        # Extract parameters: 3 rotation angles + 3 translation + 1 scale
        rx, ry, rz, tx, ty, tz, scale = params
        
        # Create rotation matrix from Euler angles
        from scipy.spatial.transform import Rotation as R
        rotation_matrix = R.from_euler('xyz', [rx, ry, rz]).as_matrix()
        
        # Create homogeneous transformation matrix
        transform = np.eye(4)
        transform[:3, :3] = rotation_matrix
        transform[:3, 3] = [tx, ty, tz]
        
        # Apply transformation to camera points
        camera_homogeneous = np.hstack([camera_pts * scale, np.ones((camera_pts.shape[0], 1))])
        transformed_points = (transform @ camera_homogeneous.T).T[:, :3]
        
        # Calculate residuals
        residuals = (transformed_points - robot_pts).flatten()
        return residuals
    
    # Initial guess: no rotation, no translation, scale = 1
    initial_params = [0, 0, 0, 0, 0, 0, 1.0]
    
    # Perform optimization
    result = least_squares(
        residual_function, 
        initial_params, 
        args=(robot_points, camera_points),
        method='lm'
    )
    
    if not result.success:
        raise ValueError(f"Calibration optimization failed: {result.message}")
    
    # Extract optimized parameters
    rx, ry, rz, tx, ty, tz, scale = result.x
    
    # Create final transformation matrix
    from scipy.spatial.transform import Rotation as R
    rotation_matrix = R.from_euler('xyz', [rx, ry, rz]).as_matrix()
    
    transform_matrix = np.eye(4)
    transform_matrix[:3, :3] = rotation_matrix
    transform_matrix[:3, 3] = [tx, ty, tz]
    
    # Calculate RMS error
    final_residuals = residual_function(result.x, robot_points, camera_points)
    rms_error = np.sqrt(np.mean(final_residuals**2))
    
    print(f"Calibration RMS error: {rms_error:.4f} meters")
    print(f"Scaling factor: {scale:.4f}")
    
    return transform_matrix, scale


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
        
        # Create and connect camera controller with lower resolution for calibration
        camera_controller = CameraController(width=640, height=480, fps=30)
        if not camera_controller.connect():
            print(f"❌ Failed to connect camera for point {i+1}")
            continue
        
        # Capture point cloud
        color_image, depth_image, intrinsics = camera_controller.capture_pointcloud()
        
        if color_image is None or depth_image is None:
            print(f"❌ Failed to capture image for point {i+1}")
            camera_controller.disconnect()
            continue
        
        # Get user click with correct OS setting
        img_click = ImgClick(color_image, os=DEFAULT_OS)
        coordinates_2d = img_click.run()  # Use run() method instead of get_coordinates()
        
        # Disconnect camera after use
        camera_controller.disconnect()
        
        if coordinates_2d is None or coordinates_2d[0] is None:
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
        transform_matrix, scaling_factor = compute_camera_robot_transform(
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