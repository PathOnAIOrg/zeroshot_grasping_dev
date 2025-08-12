#!/usr/bin/env python3
"""
Manual Hand-Eye Calibration for SO-101 Robot

This script allows manual robot control during calibration:
1. You move the robot manually to desired positions
2. Press SPACE to capture calibration data at current position
3. The script computes hand-eye transformation after collecting enough data
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


class ManualHandEyeCalibrator:
    def __init__(self, checkerboard_size=(7, 4), square_size=25.0):
        """
        Initialize manual hand-eye calibrator.
        
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
        self.robot_joints_history = []  # Store joint positions for reference
        
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
        
    def joints_to_pose_matrix(self, joints):
        """
        Convert joint angles to 4x4 pose matrix.
        This is a simplified version - adjust based on your robot's kinematics.
        """
        # Simplified approach - should be replaced with actual forward kinematics
        x, y, z = joints[0] * 100, joints[1] * 100, (joints[2] + 0.5) * 100  # Convert to mm
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
    
    def manual_calibration_capture(self, robot_client, camera_id=0, min_positions=3, max_positions=30):
        """
        Capture calibration data with manual robot control.
        
        Args:
            robot_client: Connected SO101Client instance
            camera_id: Camera device ID
            min_positions: Minimum number of positions required
            max_positions: Maximum number of positions to capture
        """
        # Open camera
        cap = cv.VideoCapture(camera_id)
        if not cap.isOpened():
            raise RuntimeError(f"Cannot open camera {camera_id}")
        
        print("\n" + "=" * 60)
        print("📸 MANUAL CALIBRATION MODE")
        print("=" * 60)
        print("\nInstructions:")
        print("1. Move the robot manually to desired positions")
        print("2. Position the checkerboard in camera view")
        print("3. Press SPACE to capture calibration data")
        print("4. Press D to delete last capture")
        print("5. Press F to finish (need at least 3 positions)")
        print("6. Press ESC to cancel")
        print("\nTips:")
        print("- Vary robot positions and orientations")
        print("- Keep checkerboard clearly visible")
        print("- Avoid similar positions")
        print("=" * 60)
        
        captured = 0
        
        while captured < max_positions:
            ret, frame = cap.read()
            if not ret:
                continue
            
            # Read current robot position
            try:
                robot_joints = robot_client.read_joints()
            except:
                robot_joints = [0, 0, 0, 0, 0, 0]  # Fallback if read fails
            
            # Display current joint positions
            info_frame = frame.copy()
            
            # Try to detect checkerboard
            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            ret_corners, corners = cv.findChessboardCorners(
                gray, self.checkerboard_size, None
            )
            
            checkerboard_ready = False
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
                    checkerboard_ready = True
                    # Draw checkerboard
                    cv.drawChessboardCorners(info_frame, self.checkerboard_size, corners, ret_corners)
                    
                    # Display status
                    cv.putText(info_frame, "CHECKERBOARD READY - Press SPACE to capture", 
                             (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv.putText(info_frame, f"Distance: {tvec[2, 0]:.1f}mm", 
                             (10, 60), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                cv.putText(info_frame, "Position checkerboard in view", 
                         (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Display capture count and controls
            cv.putText(info_frame, f"Captured: {captured}/{min_positions} min, {max_positions} max", 
                     (10, 90), cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Display joint positions
            joint_text = f"Joints: [{', '.join([f'{j:.2f}' for j in robot_joints[:6]])}]"
            cv.putText(info_frame, joint_text, 
                     (10, 120), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Display controls
            cv.putText(info_frame, "SPACE: Capture | D: Delete last | F: Finish | ESC: Cancel", 
                     (10, info_frame.shape[0] - 20), cv.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            
            cv.imshow('Manual Hand-Eye Calibration', info_frame)
            
            key = cv.waitKey(30)
            
            if key == 32 and checkerboard_ready:  # SPACE - capture
                # Store calibration data
                robot_pose = self.joints_to_pose_matrix(robot_joints)
                self.robot_poses.append(robot_pose)
                self.rvecs.append(rvec)
                self.tvecs.append(tvec)
                self.robot_joints_history.append(robot_joints.copy())
                
                captured += 1
                print(f"\n✅ Captured position {captured}")
                print(f"   Joints: {[f'{j:.3f}' for j in robot_joints]}")
                print(f"   Checkerboard distance: {tvec[2, 0]:.1f}mm")
                
                # Flash green to indicate capture
                flash_frame = info_frame.copy()
                cv.rectangle(flash_frame, (0, 0), (flash_frame.shape[1], flash_frame.shape[0]), 
                           (0, 255, 0), 10)
                cv.imshow('Manual Hand-Eye Calibration', flash_frame)
                cv.waitKey(200)
                
            elif key == ord('d') or key == ord('D'):  # Delete last capture
                if captured > 0:
                    self.robot_poses.pop()
                    self.rvecs.pop()
                    self.tvecs.pop()
                    self.robot_joints_history.pop()
                    captured -= 1
                    print(f"\n🗑️  Deleted last capture. Now have {captured} positions")
                    
            elif key == ord('f') or key == ord('F'):  # Finish
                if captured >= min_positions:
                    print(f"\n✅ Finishing with {captured} positions")
                    break
                else:
                    print(f"\n⚠️  Need at least {min_positions} positions (currently have {captured})")
                    
            elif key == 27:  # ESC - cancel
                print("\n⏹️  Calibration cancelled by user")
                cap.release()
                cv.destroyAllWindows()
                return 0
        
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
        
        return T_cam2gripper
    
    def save_calibration(self, T_cam2gripper, filename='handeye_calibration_manual.npz'):
        """Save hand-eye calibration results."""
        # Save joint positions for reference
        joint_positions = {
            f'position_{i+1}': joints.tolist() 
            for i, joints in enumerate(self.robot_joints_history)
        }
        
        # Create JSON data
        json_data = {
            'transformation_matrix': T_cam2gripper.tolist(),
            'rotation_matrix': T_cam2gripper[:3, :3].tolist(),
            'translation_vector': T_cam2gripper[:3, 3].tolist(),
            'timestamp': datetime.now().isoformat(),
            'num_calibration_poses': len(self.robot_poses),
            'checkerboard_size': list(self.checkerboard_size),
            'square_size_mm': self.square_size,
            'calibration_positions': joint_positions
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
                dist_coeffs=self.dist_coeffs,
                robot_joints=self.robot_joints_history)
        
        print(f"\n💾 Calibration saved to:")
        print(f"   - {filename} (numpy format)")
        print(f"   - {json_filename} (human-readable)")


def main():
    """Main manual hand-eye calibration routine."""
    print("=" * 60)
    print("SO-101 MANUAL HAND-EYE CALIBRATION")
    print("=" * 60)
    
    # Parse command line arguments
    import argparse
    parser = argparse.ArgumentParser(description='Manual SO-101 Hand-Eye Calibration')
    parser.add_argument('--port', type=str, default=None, 
                       help='Robot serial port (e.g., /dev/ttyUSB0, /dev/ttyACM0)')
    parser.add_argument('--camera', type=int, default=0,
                       help='Camera device ID (default: 0)')
    parser.add_argument('--min-positions', type=int, default=3,
                       help='Minimum calibration positions (default: 3)')
    parser.add_argument('--max-positions', type=int, default=30,
                       help='Maximum calibration positions (default: 30)')
    parser.add_argument('--no-robot', action='store_true',
                       help='Run without robot connection (simulation mode)')
    args = parser.parse_args()
    
    # Configuration
    CHECKERBOARD_SIZE = (7, 4)  # Adjust to match your checkerboard
    SQUARE_SIZE = 25.0  # Size of checkerboard square in mm
    
    # Create calibrator
    calibrator = ManualHandEyeCalibrator(
        checkerboard_size=CHECKERBOARD_SIZE,
        square_size=SQUARE_SIZE
    )
    
    try:
        # Load camera calibration
        calibrator.load_camera_calibration('calibration_data.npz')
        
        # Connect to robot (unless in no-robot mode)
        client = None
        if not args.no_robot:
            print("\n🤖 Connecting to robot...")
            
            # Determine port
            if args.port:
                port = args.port
            else:
                # Try to find port automatically
                import os
                possible_ports = ['/dev/ttyACM0', '/dev/ttyUSB0', '/dev/ttyACM1', '/dev/ttyUSB1']
                port = None
                for p in possible_ports:
                    if os.path.exists(p):
                        port = p
                        break
                
                if not port:
                    print("\n⚠️  No robot port found, running in simulation mode")
                    print("To connect robot, specify port with --port argument")
                    args.no_robot = True
            
            if not args.no_robot:
                try:
                    print(f"Attempting connection on port: {port}")
                    client = SO101Client(
                        port=port,
                        follower=True,
                        force_calibration=False
                    )
                    print("✅ Robot connected!")
                    print("\n⚠️  YOU ARE IN MANUAL CONTROL MODE")
                    print("Move the robot carefully by hand to desired positions")
                    
                except Exception as e:
                    print(f"\n⚠️  Failed to connect to robot: {e}")
                    print("Running in simulation mode (robot joint reading disabled)")
                    client = None
        
        # Capture calibration data
        num_captured = calibrator.manual_calibration_capture(
            client, 
            camera_id=args.camera,
            min_positions=args.min_positions,
            max_positions=args.max_positions
        )
        
        if num_captured >= args.min_positions:
            # Compute calibration
            T_cam2gripper = calibrator.compute_hand_eye_calibration()
            
            # Save results
            calibrator.save_calibration(T_cam2gripper)
            
            print("\n" + "=" * 60)
            print("✅ MANUAL HAND-EYE CALIBRATION COMPLETE!")
            print("=" * 60)
            print("\nTo use the calibration in your code:")
            print("  data = np.load('handeye_calibration_manual.npz')")
            print("  T_cam2gripper = data['T_cam2gripper']")
            print("\nCalibration positions are saved in the JSON file for reference")
            
        else:
            print(f"\n❌ Insufficient calibration data captured ({num_captured} < {args.min_positions})")
            
    except KeyboardInterrupt:
        print("\n⏹️  Calibration interrupted by user")
    except Exception as e:
        print(f"\n❌ Error during calibration: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if client:
            try:
                client.disconnect()
                print("👋 Disconnected from robot")
            except:
                pass


if __name__ == "__main__":
    main()