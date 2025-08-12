#!/usr/bin/env python3
"""
Hand-Eye Calibration for SO-101 Robot

This script performs hand-eye calibration by:
1. Moving the robot to different poses
2. Capturing checkerboard images at each pose
3. Computing the transformation between robot end-effector and camera
"""

import cv2 as cv
import numpy as np
import sys
import os
import time
import json
from datetime import datetime

# Add parent directory to path for robot imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'GraspingDemo'))

from so101_grasp.robot.so101_client import SO101Client
from so101_grasp.utils.config import ConfigManager


def test_camera_only(calibrator, camera_id=0):
    """Test camera and checkerboard detection without robot."""
    cap = cv.VideoCapture(camera_id)
    if not cap.isOpened():
        print(f"❌ Cannot open camera {camera_id}")
        return
    
    print("\n📸 Camera test mode")
    print("Press SPACE to test checkerboard detection")
    print("Press Q to quit")
    
    detected_count = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        
        # Try to detect checkerboard
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        ret_corners, corners = cv.findChessboardCorners(
            gray, calibrator.checkerboard_size, None
        )
        
        if ret_corners:
            # Draw detected corners
            cv.drawChessboardCorners(frame, calibrator.checkerboard_size, corners, ret_corners)
            cv.putText(frame, "Checkerboard DETECTED", (10, 30), 
                      cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Refine and solve PnP
            corners = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), calibrator.criteria)
            ret_pnp, rvec, tvec = cv.solvePnP(
                calibrator.objp, corners,
                calibrator.camera_matrix, calibrator.dist_coeffs
            )
            
            if ret_pnp:
                cv.putText(frame, f"Distance: {tvec[2, 0]:.1f}mm", (10, 60),
                          cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv.putText(frame, "No checkerboard detected", (10, 30),
                      cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        cv.putText(frame, f"Detections: {detected_count}", (10, 90),
                  cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        cv.imshow('Camera Test', frame)
        
        key = cv.waitKey(1)
        if key == ord('q') or key == ord('Q'):
            break
        elif key == 32:  # SPACE
            if ret_corners:
                detected_count += 1
                print(f"✅ Detection {detected_count} successful")
            else:
                print("❌ No checkerboard detected")
    
    cap.release()
    cv.destroyAllWindows()
    print(f"\n📊 Test complete: {detected_count} successful detections")


class HandEyeCalibrator:
    def __init__(self, checkerboard_size=(7, 4), square_size=25.0):
        """
        Initialize hand-eye calibrator.
        
        Args:
            checkerboard_size: (columns, rows) of internal corners
            square_size: Size of checkerboard square in mm
        """
        self.checkerboard_size = checkerboard_size
        self.square_size = square_size
        
        # Calibration data storage
        self.robot_poses = []  # Robot end-effector poses
        self.rvecs = []  # Rotation vectors from camera calibration
        self.tvecs = []  # Translation vectors from camera calibration
        
        # Camera calibration parameters (to be loaded)
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Setup checkerboard points
        self.objp = np.zeros((checkerboard_size[1] * checkerboard_size[0], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)
        self.objp *= square_size
        
        # OpenCV detection criteria
        self.criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        
    def load_camera_calibration(self, calibration_file='calibration_data.npz'):
        """Load camera intrinsic calibration."""
        if not os.path.exists(calibration_file):
            raise FileNotFoundError(f"Camera calibration file '{calibration_file}' not found. "
                                  "Please run calibrate_from_capture.py first.")
        
        data = np.load(calibration_file)
        self.camera_matrix = data['mtx']
        self.dist_coeffs = data['dist']
        print(f"✅ Loaded camera calibration from '{calibration_file}'")
        print(f"   Camera matrix:\n{self.camera_matrix}")
        
    def joints_to_pose_matrix(self, joints):
        """
        Convert joint angles to 4x4 pose matrix.
        This is a simplified version - you may need to adjust based on your robot's kinematics.
        """
        # For SO-101, we need proper forward kinematics
        # This is a placeholder that assumes the robot provides end-effector position
        # In practice, you'd use the robot's forward kinematics or get pose from robot API
        
        # Simplified approach: use joint values to create a transformation
        # You should replace this with actual forward kinematics for SO-101
        x, y, z = joints[0] * 100, joints[1] * 100, (joints[2] + 0.5) * 100  # Convert to mm
        
        # Create rotation from joint angles (simplified)
        rx, ry, rz = joints[3], joints[4], joints[0]  # Use some joints for rotation
        
        # Create rotation matrix using Euler angles
        Rx = np.array([[1, 0, 0],
                      [0, np.cos(rx), -np.sin(rx)],
                      [0, np.sin(rx), np.cos(rx)]])
        
        Ry = np.array([[np.cos(ry), 0, np.sin(ry)],
                      [0, 1, 0],
                      [-np.sin(ry), 0, np.cos(ry)]])
        
        Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                      [np.sin(rz), np.cos(rz), 0],
                      [0, 0, 1]])
        
        R = Rz @ Ry @ Rx
        
        # Create 4x4 transformation matrix
        pose = np.eye(4)
        pose[:3, :3] = R
        pose[:3, 3] = [x, y, z]
        
        return pose
    
    def capture_calibration_data(self, robot_client, camera_id=0, num_positions=15):
        """
        Capture calibration data by moving robot and detecting checkerboard.
        
        Args:
            robot_client: Connected SO101Client instance
            camera_id: Camera device ID
            num_positions: Number of calibration positions to capture
        """
        # Open camera
        cap = cv.VideoCapture(camera_id)
        if not cap.isOpened():
            raise RuntimeError(f"Cannot open camera {camera_id}")
        
        print(f"\n📸 Starting calibration capture ({num_positions} positions)")
        print("Press SPACE to capture at current position, ESC to finish early")
        
        # Define robot positions for calibration (vary position and orientation)
        calibration_positions = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.3, -0.2, 0.5, 0.0, 0.0, 0.0],
            [0.3, -0.2, 0.5, 0.2, 0.0, 0.0],
            [0.3, -0.2, 0.5, -0.2, 0.0, 0.0],
            [0.3, -0.2, 0.5, 0.0, 0.2, 0.0],
            [0.3, -0.2, 0.5, 0.0, -0.2, 0.0],
            [-0.3, -0.2, 0.5, 0.0, 0.0, 0.0],
            [-0.3, -0.2, 0.5, 0.2, 0.0, 0.0],
            [-0.3, -0.2, 0.5, -0.2, 0.0, 0.0],
            [0.0, -0.5, 1.0, 0.0, 0.0, 0.0],
            [0.0, -0.5, 1.0, 0.3, 0.0, 0.0],
            [0.0, -0.5, 1.0, -0.3, 0.0, 0.0],
            [0.2, -0.3, 0.7, 0.1, 0.1, 0.0],
            [-0.2, -0.3, 0.7, -0.1, 0.1, 0.0],
            [0.0, -0.4, 0.8, 0.0, 0.0, 0.0],
        ]
        
        captured = 0
        for i, target_pos in enumerate(calibration_positions):
            if captured >= num_positions:
                break
                
            print(f"\n🎯 Moving to position {i+1}/{num_positions}")
            print(f"   Target: {[f'{p:.3f}' for p in target_pos]}")
            
            # Move robot to position
            current_pos = robot_client.read_joints()
            robot_client.interpolate_waypoint(
                current_pos,
                target_pos,
                steps=30,
                timestep=0.05
            )
            
            # Wait for robot to stabilize
            time.sleep(2)
            
            # Read actual robot position
            robot_joints = robot_client.read_joints()
            print(f"   Actual: {[f'{p:.3f}' for p in robot_joints]}")
            
            # Prompt user to show checkerboard
            print("\n   📋 Please show the checkerboard to the camera")
            print("      Press SPACE when checkerboard is in position")
            print("      Press S to skip this position")
            
            # Wait for user to position checkerboard
            while True:
                ret, frame = cap.read()
                if ret:
                    # Show current camera view
                    cv.putText(frame, "Position checkerboard and press SPACE", 
                             (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv.putText(frame, f"Position {i+1}/{num_positions}", 
                             (10, 60), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    cv.imshow('Hand-Eye Calibration', frame)
                
                key = cv.waitKey(30)
                if key == 32:  # SPACE - ready to capture
                    break
                elif key == ord('s') or key == ord('S'):  # Skip
                    print("   ⏭️  Skipping this position")
                    break
                elif key == 27:  # ESC
                    print("\n⏹️  Calibration stopped by user")
                    cap.release()
                    cv.destroyAllWindows()
                    return captured
            
            if key == ord('s') or key == ord('S'):
                continue
            
            # Capture and process image
            success = False
            attempts = 0
            max_attempts = 30  # Try for up to 3 seconds
            
            while not success and attempts < max_attempts:
                ret, frame = cap.read()
                if not ret:
                    print("Failed to capture frame")
                    attempts += 1
                    time.sleep(0.1)
                    continue
                
                # Convert to grayscale and detect checkerboard
                gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
                ret_corners, corners = cv.findChessboardCorners(
                    gray, self.checkerboard_size, None
                )
                
                if ret_corners:
                    # Refine corners
                    corners = cv.cornerSubPix(
                        gray, corners, (11, 11), (-1, -1), self.criteria
                    )
                    
                    # Solve PnP to get camera pose relative to checkerboard
                    ret_pnp, rvec, tvec = cv.solvePnP(
                        self.objp, corners, 
                        self.camera_matrix, self.dist_coeffs
                    )
                    
                    if ret_pnp:
                        # Store calibration data
                        robot_pose = self.joints_to_pose_matrix(robot_joints)
                        self.robot_poses.append(robot_pose)
                        self.rvecs.append(rvec)
                        self.tvecs.append(tvec)
                        
                        captured += 1
                        success = True
                        
                        # Draw and display
                        cv.drawChessboardCorners(frame, self.checkerboard_size, corners, ret_corners)
                        cv.putText(frame, f"Captured {captured}/{num_positions}", 
                                 (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        cv.imshow('Hand-Eye Calibration', frame)
                        cv.waitKey(500)
                        
                        print(f"   ✅ Captured position {captured}")
                    else:
                        print("   ⚠️  Failed to solve PnP")
                else:
                    # Show current view
                    cv.putText(frame, f"No checkerboard detected (attempt {attempts+1}/{max_attempts})", 
                             (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    cv.imshow('Hand-Eye Calibration', frame)
                    
                attempts += 1
                
                key = cv.waitKey(100)
                if key == 27:  # ESC
                    print("\n⏹️  Calibration capture stopped by user")
                    break
            
            if not success:
                print(f"   ❌ Failed to detect checkerboard at this position")
        
        cap.release()
        cv.destroyAllWindows()
        
        print(f"\n📊 Captured {captured} calibration positions")
        return captured
    
    def compute_hand_eye_calibration(self):
        """
        Compute hand-eye calibration using collected data.
        Returns the transformation matrix from end-effector to camera.
        """
        if len(self.robot_poses) < 3:
            raise ValueError(f"Need at least 3 calibration positions, got {len(self.robot_poses)}")
        
        print(f"\n🔧 Computing hand-eye calibration with {len(self.robot_poses)} positions...")
        
        # Convert robot poses to rotation and translation
        R_gripper2base = []
        t_gripper2base = []
        
        for pose in self.robot_poses:
            R_gripper2base.append(pose[:3, :3])
            t_gripper2base.append(pose[:3, 3].reshape(-1, 1))
        
        # Convert camera poses to rotation matrices
        R_target2cam = []
        t_target2cam = []
        
        for rvec, tvec in zip(self.rvecs, self.tvecs):
            R, _ = cv.Rodrigues(rvec)
            R_target2cam.append(R)
            t_target2cam.append(tvec)
        
        # Compute hand-eye calibration
        # This computes X where AX = XB
        # A = gripper motion, B = camera motion, X = camera to gripper transform
        R_cam2gripper, t_cam2gripper = cv.calibrateHandEye(
            R_gripper2base, t_gripper2base,
            R_target2cam, t_target2cam,
            method=cv.CALIB_HAND_EYE_TSAI
        )
        
        # Create 4x4 transformation matrix
        T_cam2gripper = np.eye(4)
        T_cam2gripper[:3, :3] = R_cam2gripper
        T_cam2gripper[:3, 3] = t_cam2gripper.flatten()
        
        print("✅ Hand-eye calibration complete!")
        print(f"\nCamera to gripper transformation:")
        print(T_cam2gripper)
        
        # Calculate reprojection error for validation
        errors = []
        for i in range(len(self.robot_poses)):
            # Transform checkerboard to base frame through: base <- gripper <- camera <- checkerboard
            T_target2cam = np.eye(4)
            R, _ = cv.Rodrigues(self.rvecs[i])
            T_target2cam[:3, :3] = R
            T_target2cam[:3, 3] = self.tvecs[i].flatten()
            
            T_target2base = self.robot_poses[i] @ T_cam2gripper @ T_target2cam
            
            # For validation, we'd compare consecutive transformations
            # This is simplified - proper validation would reproject points
            if i > 0:
                diff = np.linalg.norm(T_target2base[:3, 3] - prev_T[:3, 3])
                errors.append(diff)
            prev_T = T_target2base
        
        if errors:
            mean_error = np.mean(errors)
            print(f"\nValidation: Mean position consistency: {mean_error:.2f} mm")
        
        return T_cam2gripper
    
    def save_calibration(self, T_cam2gripper, filename='handeye_calibration.npz'):
        """Save hand-eye calibration results."""
        # Also save as readable JSON
        json_data = {
            'transformation_matrix': T_cam2gripper.tolist(),
            'rotation_matrix': T_cam2gripper[:3, :3].tolist(),
            'translation_vector': T_cam2gripper[:3, 3].tolist(),
            'timestamp': datetime.now().isoformat(),
            'num_calibration_poses': len(self.robot_poses),
            'checkerboard_size': list(self.checkerboard_size),
            'square_size_mm': self.square_size
        }
        
        json_filename = filename.replace('.npz', '.json')
        with open(json_filename, 'w') as f:
            json.dump(json_data, f, indent=2)
        
        # Save numpy data
        np.savez(filename,
                T_cam2gripper=T_cam2gripper,
                robot_poses=self.robot_poses,
                rvecs=self.rvecs,
                tvecs=self.tvecs,
                camera_matrix=self.camera_matrix,
                dist_coeffs=self.dist_coeffs)
        
        print(f"\n💾 Calibration saved to:")
        print(f"   - {filename} (numpy format)")
        print(f"   - {json_filename} (human-readable)")


def main():
    """Main hand-eye calibration routine."""
    print("=" * 60)
    print("SO-101 HAND-EYE CALIBRATION")
    print("=" * 60)
    
    # Parse command line arguments
    import argparse
    parser = argparse.ArgumentParser(description='SO-101 Hand-Eye Calibration')
    parser.add_argument('--port', type=str, default=None, 
                       help='Robot serial port (e.g., /dev/ttyUSB0, /dev/ttyACM0)')
    parser.add_argument('--camera', type=int, default=0,
                       help='Camera device ID (default: 0)')
    parser.add_argument('--skip-robot', action='store_true',
                       help='Skip robot connection for testing camera only')
    parser.add_argument('--positions', type=int, default=12,
                       help='Number of calibration positions (default: 12)')
    args = parser.parse_args()
    
    # Configuration
    CHECKERBOARD_SIZE = (7, 4)  # Adjust to match your checkerboard
    SQUARE_SIZE = 25.0  # Size of checkerboard square in mm
    NUM_POSITIONS = args.positions  # Number of calibration positions
    CAMERA_ID = args.camera  # Camera device ID
    
    # Create calibrator
    calibrator = HandEyeCalibrator(
        checkerboard_size=CHECKERBOARD_SIZE,
        square_size=SQUARE_SIZE
    )
    
    try:
        # Load camera calibration
        calibrator.load_camera_calibration('calibration_data.npz')
        
        # Skip robot if requested (for testing camera only)
        if args.skip_robot:
            print("\n⚠️  Robot connection skipped (test mode)")
            print("This mode only tests camera and checkerboard detection")
            test_camera_only(calibrator, CAMERA_ID)
            return
        
        # Connect to robot
        print("\n🤖 Connecting to robot...")
        
        # Determine port to use
        if args.port:
            port = args.port
            print(f"Using specified port: {port}")
        else:
            # Try to load from config
            try:
                config_manager = ConfigManager()
                robot_config = config_manager.get_robot_config()
                port = robot_config.get('port', '/dev/ttyACM0')
                print(f"Using port from config: {port}")
            except:
                # Try common ports
                import os
                possible_ports = ['/dev/ttyACM0', '/dev/ttyUSB0', '/dev/ttyACM1', '/dev/ttyUSB1']
                port = None
                for p in possible_ports:
                    if os.path.exists(p):
                        port = p
                        print(f"Found port: {port}")
                        break
                
                if not port:
                    print("\n❌ No robot port found!")
                    print("Please specify port with --port argument")
                    print("Common ports: /dev/ttyACM0, /dev/ttyUSB0")
                    print("\nTo find your port, run:")
                    print("  ls /dev/tty* | grep -E 'ACM|USB'")
                    return
        
        print(f"Attempting connection on port: {port}")
        
        try:
            client = SO101Client(
                port=port,
                follower=True,
                force_calibration=False
            )
            print("✅ Robot connected!")
        except Exception as e:
            print(f"\n❌ Failed to connect to robot on {port}")
            print(f"Error: {e}")
            print("\nTroubleshooting:")
            print("1. Check if robot is powered on")
            print("2. Check USB connection")
            print("3. Try finding the correct port:")
            print("   ls /dev/tty* | grep -E 'ACM|USB'")
            print("4. Check permissions:")
            print(f"   sudo chmod 666 {port}")
            print("5. Add user to dialout group:")
            print("   sudo usermod -a -G dialout $USER")
            print("   (then logout and login again)")
            print("\nYou can also test camera only with:")
            print("  python handeye_calibration_so101.py --skip-robot")
            return
        
        # Move to initial position
        print("\n🏠 Moving to home position...")
        home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        client.interpolate_waypoint(
            client.read_joints(),
            home_position,
            steps=30,
            timestep=0.05
        )
        time.sleep(2)
        
        # Capture calibration data
        num_captured = calibrator.capture_calibration_data(
            client, 
            camera_id=CAMERA_ID,
            num_positions=NUM_POSITIONS
        )
        
        if num_captured >= 3:
            # Compute calibration
            T_cam2gripper = calibrator.compute_hand_eye_calibration()
            
            # Save results
            calibrator.save_calibration(T_cam2gripper)
            
            print("\n" + "=" * 60)
            print("✅ HAND-EYE CALIBRATION COMPLETE!")
            print("=" * 60)
            print("\nTo use the calibration in your code:")
            print("  data = np.load('handeye_calibration.npz')")
            print("  T_cam2gripper = data['T_cam2gripper']")
            
        else:
            print("\n❌ Insufficient calibration data captured")
            
        # Return to home
        print("\n🏠 Returning to home position...")
        client.interpolate_waypoint(
            client.read_joints(),
            home_position,
            steps=30,
            timestep=0.05
        )
        
    except KeyboardInterrupt:
        print("\n⏹️  Calibration interrupted by user")
    except Exception as e:
        print(f"\n❌ Error during calibration: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            client.disconnect()
            print("👋 Disconnected from robot")
        except:
            pass


if __name__ == "__main__":
    main()