#!/usr/bin/env python3
"""
Manual Hand-Eye Calibration with STATIONARY RealSense Camera (Eye-to-Hand)

This version is for when the camera is fixed in the workspace, not on the robot.
The calibration finds the transformation from camera to robot base.
"""

import cv2 as cv
import numpy as np
import sys
import os
import time
import json
from datetime import datetime

try:
    import pyrealsense2 as rs
except ImportError:
    print("Error: pyrealsense2 not installed")
    print("Install with: pip install pyrealsense2")
    sys.exit(1)

# Add parent directory to path for robot imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'GraspingDemo'))

from so101_grasp.robot.so101_client import SO101Client


class StationaryRealSenseCalibrator:
    def __init__(self, checkerboard_size=None, square_size=25.0):
        """
        Initialize RealSense hand-eye calibrator for STATIONARY camera.
        
        Args:
            checkerboard_size: (columns, rows) of internal corners, None for auto-detect
            square_size: Size of checkerboard square in mm
        """
        self.checkerboard_size = checkerboard_size
        self.square_size = square_size
        self.auto_detect_pattern = checkerboard_size is None
        
        # Calibration data storage
        self.robot_poses = []  # Gripper poses in base frame
        self.rvecs = []  # Checkerboard poses in camera frame
        self.tvecs = []
        self.robot_joints_history = []
        
        # Camera calibration parameters
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # RealSense pipeline
        self.pipeline = None
        self.config = None
        
        # Setup checkerboard points (only if pattern is known)
        if checkerboard_size is not None:
            self.objp = np.zeros((checkerboard_size[1] * checkerboard_size[0], 3), np.float32)
            self.objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)
            self.objp *= square_size
        else:
            self.objp = None  # Will be set later when pattern is detected
        
        # OpenCV detection criteria
        self.criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    def load_camera_calibration(self, calibration_file='calibration_data.npz'):
        """Load camera intrinsic calibration."""
        if not os.path.exists(calibration_file):
            print(f"⚠️  Camera calibration file '{calibration_file}' not found.")
            print("   Using RealSense intrinsics instead.")
            return False
        
        data = np.load(calibration_file)
        self.camera_matrix = data['mtx']
        self.dist_coeffs = data['dist']
        print(f"✅ Loaded camera calibration from '{calibration_file}'")
        return True
    
    def init_realsense(self):
        """Initialize RealSense camera."""
        try:
            # Create pipeline
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            
            # Configure streams
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            
            # Start pipeline
            profile = self.pipeline.start(self.config)
            
            # Get camera intrinsics if not loaded from file
            if self.camera_matrix is None:
                color_stream = profile.get_stream(rs.stream.color)
                intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
                
                # Convert to OpenCV format
                self.camera_matrix = np.array([
                    [intrinsics.fx, 0, intrinsics.ppx],
                    [0, intrinsics.fy, intrinsics.ppy],
                    [0, 0, 1]
                ])
                
                # RealSense distortion model
                self.dist_coeffs = np.array(intrinsics.coeffs)
                
                print("✅ Using RealSense camera intrinsics")
                print(f"   fx={intrinsics.fx:.1f}, fy={intrinsics.fy:.1f}")
                print(f"   cx={intrinsics.ppx:.1f}, cy={intrinsics.ppy:.1f}")
            
            # Allow auto-exposure to settle
            for _ in range(30):
                self.pipeline.wait_for_frames()
            
            print("✅ RealSense camera initialized (STATIONARY mode)")
            return True
            
        except Exception as e:
            print(f"❌ Failed to initialize RealSense: {e}")
            return False
    
    def get_frame(self):
        """Get color frame from RealSense."""
        try:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            
            if not color_frame:
                return None
            
            # Convert to numpy array
            color_image = np.asanyarray(color_frame.get_data())
            return color_image
            
        except:
            return None
    
    def joints_to_pose_matrix(self, joints):
        """Convert joint angles to 4x4 pose matrix (simplified)."""
        # This represents the gripper pose in the base frame
        x, y, z = joints[0] * 100, joints[1] * 100, (joints[2] + 0.5) * 100
        rx, ry, rz = joints[3], joints[4], joints[0]
        
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
        
        pose = np.eye(4)
        pose[:3, :3] = R
        pose[:3, 3] = [x, y, z]
        
        return pose
    
    def detect_checkerboard_pattern(self, frame):
        """
        Auto-detect checkerboard pattern size.
        Returns (pattern_size, corners) or (None, None) if not found.
        """
        patterns_to_try = [
            (7, 4), (4, 7),  # Common pattern from your setup
            (9, 6), (6, 9),  # Standard OpenCV calibration pattern
            (8, 6), (6, 8),  # Another common pattern
            (7, 5), (5, 7),  
            (7, 6), (6, 7),  
            (6, 4), (4, 6),  
            (5, 4), (4, 5),  
            (8, 5), (5, 8),  
            (10, 7), (7, 10),  # Larger patterns
            (11, 8), (8, 11),
        ]
        
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        
        for pattern in patterns_to_try:
            ret, corners = cv.findChessboardCorners(gray, pattern, None)
            if ret:
                print(f"✅ Detected checkerboard pattern: {pattern[0]}×{pattern[1]}")
                return pattern, corners
        
        return None, None
    
    def manual_calibration_capture(self, robot_client, min_positions=3, max_positions=30):
        """
        Capture calibration data with manual robot control for STATIONARY camera.
        Robot holds checkerboard, camera is fixed.
        """
        if not self.init_realsense():
            raise RuntimeError("Failed to initialize RealSense camera")
        
        print("\n" + "=" * 60)
        print("📸 STATIONARY CAMERA CALIBRATION MODE (Eye-to-Hand)")
        print("=" * 60)
        print("⚠️  IMPORTANT: Camera is STATIONARY, robot holds checkerboard")
        print("=" * 60)
        
        # Auto-detect pattern if needed
        if self.auto_detect_pattern:
            print("\n🔍 Auto-detecting checkerboard pattern...")
            print("Please attach checkerboard to robot gripper and show to camera...")
            
            detected = False
            for _ in range(100):  # Try for ~10 seconds
                frame = self.get_frame()
                if frame is not None:
                    pattern, _ = self.detect_checkerboard_pattern(frame)
                    if pattern:
                        self.checkerboard_size = pattern
                        detected = True
                        
                        # Regenerate object points for the detected pattern
                        self.objp = np.zeros((pattern[1] * pattern[0], 3), np.float32)
                        self.objp[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2)
                        self.objp *= self.square_size
                        
                        print(f"✅ Using detected pattern: {pattern[0]}×{pattern[1]} internal corners")
                        break
                    
                    # Show frame with status
                    cv.putText(frame, "Detecting checkerboard pattern...", 
                             (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                    cv.imshow('Pattern Detection', frame)
                    if cv.waitKey(100) == 27:  # ESC
                        break
            
            cv.destroyWindow('Pattern Detection')
            
            if not detected:
                print("❌ Could not detect pattern. Using default 7×4")
                self.checkerboard_size = (7, 4)
                self.objp = np.zeros((4 * 7, 3), np.float32)
                self.objp[:, :2] = np.mgrid[0:7, 0:4].T.reshape(-1, 2)
                self.objp *= self.square_size
                
        print("\nInstructions for STATIONARY CAMERA:")
        print("1. Attach checkerboard to robot gripper")
        print("2. Move robot to different positions")
        print("3. Ensure checkerboard is visible to stationary camera")
        print("4. Press SPACE to capture calibration data")
        print("5. Press D to delete last capture")
        print("6. Press F to finish (need at least 3 positions)")
        print("7. Press ESC to cancel")
        print("\nTips for Eye-to-Hand calibration:")
        print("- Camera should NOT move during calibration")
        print("- Vary robot positions widely in camera view")
        print("- Keep checkerboard rigidly attached to gripper")
        print("=" * 60)
        
        captured = 0
        
        try:
            while captured < max_positions:
                # Get frame from RealSense
                frame = self.get_frame()
                if frame is None:
                    continue
                
                # Read current robot position
                try:
                    robot_joints = robot_client.read_joints() if robot_client else [0, 0, 0, 0, 0, 0]
                except:
                    robot_joints = [0, 0, 0, 0, 0, 0]
                
                # Display frame
                info_frame = frame.copy()
                
                # Add text to indicate stationary camera mode
                cv.putText(info_frame, "STATIONARY CAMERA MODE", 
                         (10, info_frame.shape[0] - 40), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                
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
                    
                    # Solve PnP
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
                    cv.putText(info_frame, "Move robot with checkerboard into view", 
                             (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                # Display capture count
                cv.putText(info_frame, f"Captured: {captured}/{min_positions} min, {max_positions} max", 
                         (10, 90), cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                # Display joint positions
                joint_text = f"Joints: [{', '.join([f'{j:.2f}' for j in robot_joints[:6]])}]"
                cv.putText(info_frame, joint_text, 
                         (10, 120), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # Display controls
                cv.putText(info_frame, "SPACE: Capture | D: Delete | F: Finish | ESC: Cancel", 
                         (10, info_frame.shape[0] - 20), cv.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
                
                cv.imshow('Stationary Camera Calibration', info_frame)
                
                key = cv.waitKey(30) & 0xFF
                
                if key == 32:  # SPACE pressed
                    if checkerboard_ready:
                        # Store calibration data
                        robot_pose = self.joints_to_pose_matrix(robot_joints)
                        self.robot_poses.append(robot_pose)
                        self.rvecs.append(rvec)
                        self.tvecs.append(tvec)
                        self.robot_joints_history.append(list(robot_joints))
                        
                        captured += 1
                        print(f"\n✅ Captured position {captured}")
                        print(f"   Joints: {[f'{j:.3f}' for j in robot_joints]}")
                        print(f"   Checkerboard distance: {tvec[2, 0]:.1f}mm")
                        
                        # Flash green
                        flash_frame = info_frame.copy()
                        cv.rectangle(flash_frame, (0, 0), (flash_frame.shape[1], flash_frame.shape[0]), 
                                   (0, 255, 0), 10)
                        cv.imshow('Stationary Camera Calibration', flash_frame)
                        cv.waitKey(200)
                    else:
                        print("\n⚠️  Cannot capture - checkerboard not detected!")
                        print("   Move robot with checkerboard into camera view")
                    
                elif key == ord('d') or key == ord('D'):  # Delete last
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
                        
                elif key == 27:  # ESC
                    print("\n⏹️  Calibration cancelled by user")
                    return 0
                    
        finally:
            # Clean up
            cv.destroyAllWindows()
            if self.pipeline:
                self.pipeline.stop()
        
        print(f"\n📊 Captured {captured} calibration positions")
        return captured
    
    def compute_hand_eye_calibration(self):
        """Compute hand-eye calibration for STATIONARY camera (eye-to-hand)."""
        if len(self.robot_poses) < 3:
            raise ValueError(f"Need at least 3 calibration positions, got {len(self.robot_poses)}")
        
        print(f"\n🔧 Computing eye-to-hand calibration with {len(self.robot_poses)} positions...")
        print("   (Camera is stationary, robot holds checkerboard)")
        
        # Convert robot poses to rotation and translation
        # These are gripper poses in base frame
        R_gripper2base = []
        t_gripper2base = []
        
        for pose in self.robot_poses:
            R_gripper2base.append(pose[:3, :3])
            t_gripper2base.append(pose[:3, 3].reshape(-1, 1))
        
        # Convert camera poses to rotation matrices
        # These are checkerboard poses in camera frame
        R_target2cam = []
        t_target2cam = []
        
        for rvec, tvec in zip(self.rvecs, self.tvecs):
            R, _ = cv.Rodrigues(rvec)
            R_target2cam.append(R)
            t_target2cam.append(tvec)
        
        # For eye-to-hand: compute transformation from camera to base
        # We need to use calibrateHandEye with different interpretation
        # The checkerboard is attached to gripper, camera sees it
        
        # In eye-to-hand setup:
        # A = gripper motion in base frame
        # B = checkerboard motion in camera frame (same as gripper motion)
        # X = camera to base transformation (what we want)
        # AX = XB
        
        R_cam2base, t_cam2base = cv.calibrateHandEye(
            R_gripper2base, t_gripper2base,
            R_target2cam, t_target2cam,
            method=cv.CALIB_HAND_EYE_TSAI
        )
        
        # Create 4x4 transformation matrix (camera to base)
        T_cam2base = np.eye(4)
        T_cam2base[:3, :3] = R_cam2base
        T_cam2base[:3, 3] = t_cam2base.flatten()
        
        print("✅ Eye-to-hand calibration complete!")
        print(f"\nCamera to base transformation:")
        print(T_cam2base)
        
        # Also compute base to camera for convenience
        T_base2cam = np.linalg.inv(T_cam2base)
        print(f"\nBase to camera transformation (inverse):")
        print(T_base2cam)
        
        return T_cam2base, T_base2cam
    
    def save_calibration(self, T_cam2base, T_base2cam, filename='handeye_realsense_stationary.npz'):
        """Save eye-to-hand calibration results."""
        # Save joint positions for reference
        joint_positions = {
            f'position_{i+1}': joints 
            for i, joints in enumerate(self.robot_joints_history)
        }
        
        # Create JSON data
        json_data = {
            'calibration_type': 'eye-to-hand (stationary camera)',
            'camera_to_base_matrix': T_cam2base.tolist(),
            'base_to_camera_matrix': T_base2cam.tolist(),
            'rotation_matrix_cam2base': T_cam2base[:3, :3].tolist(),
            'translation_vector_cam2base': T_cam2base[:3, 3].tolist(),
            'timestamp': datetime.now().isoformat(),
            'num_calibration_poses': len(self.robot_poses),
            'checkerboard_size': list(self.checkerboard_size),
            'square_size_mm': self.square_size,
            'calibration_positions': joint_positions,
            'camera_type': 'RealSense (Stationary)'
        }
        
        json_filename = filename.replace('.npz', '.json')
        with open(json_filename, 'w') as f:
            json.dump(json_data, f, indent=2)
        
        # Save numpy data
        np.savez(filename,
                T_cam2base=T_cam2base,
                T_base2cam=T_base2cam,
                T_cam2gripper=None,  # Not applicable for stationary camera
                robot_poses=self.robot_poses,
                rvecs=self.rvecs,
                tvecs=self.tvecs,
                camera_matrix=self.camera_matrix,
                dist_coeffs=self.dist_coeffs,
                robot_joints=self.robot_joints_history,
                calibration_type='eye-to-hand')
        
        print(f"\n💾 Stationary camera calibration saved to:")
        print(f"   - {filename} (numpy format)")
        print(f"   - {json_filename} (human-readable)")


def main():
    """Main RealSense eye-to-hand calibration routine."""
    print("=" * 60)
    print("STATIONARY REALSENSE CAMERA CALIBRATION (Eye-to-Hand)")
    print("=" * 60)
    
    # Parse command line arguments
    import argparse
    parser = argparse.ArgumentParser(description='Stationary RealSense Eye-to-Hand Calibration')
    parser.add_argument('--port', type=str, default='/dev/ttyACM0',
                       help='Robot serial port (default: /dev/ttyACM0)')
    parser.add_argument('--min-positions', type=int, default=3,
                       help='Minimum calibration positions (default: 3)')
    parser.add_argument('--max-positions', type=int, default=30,
                       help='Maximum calibration positions (default: 30)')
    parser.add_argument('--no-robot', action='store_true',
                       help='Run without robot connection')
    parser.add_argument('--pattern', type=str, default=None,
                       help='Checkerboard pattern size, e.g., "9x6" for 9 columns, 6 rows')
    parser.add_argument('--output', type=str, default='../output/handeye_realsense_stationary.npz',
                       help='Output file path')
    args = parser.parse_args()
    
    # Configuration
    SQUARE_SIZE = 25.0  # Size in mm
    
    # Parse pattern if provided
    checkerboard_size = None
    if args.pattern:
        try:
            cols, rows = map(int, args.pattern.lower().split('x'))
            checkerboard_size = (cols, rows)
            print(f"Using specified pattern: {cols}×{rows}")
        except:
            print(f"Invalid pattern format: {args.pattern}")
            print("Use format like '9x6' for 9 columns, 6 rows")
            checkerboard_size = None
    
    # Create calibrator
    calibrator = StationaryRealSenseCalibrator(
        checkerboard_size=checkerboard_size,  # None = auto-detect
        square_size=SQUARE_SIZE
    )
    
    try:
        # Try to load camera calibration (optional for RealSense)
        calibrator.load_camera_calibration('../output/calibration_data.npz')
        
        # Connect to robot
        client = None
        if not args.no_robot:
            print(f"\n🤖 Connecting to robot on {args.port}...")
            try:
                from so101_grasp.robot.so101_client import SO101Client
                # First check if robot needs calibration
                print("Checking robot calibration...")
                
                # Try to connect with calibration check
                try:
                    client = SO101Client(
                        port=args.port,
                        follower=True,
                        force_calibration=False
                    )
                    print("✅ Robot connected with existing calibration!")
                except Exception as e:
                    if "calibration" in str(e).lower():
                        print("\n⚠️  Robot needs calibration")
                        print("Press ENTER to use existing calibration or 'c' to calibrate")
                        
                        user_input = input().strip().lower()
                        if user_input == 'c':
                            # Run calibration
                            import subprocess
                            result = subprocess.run([
                                sys.executable, '-m', 'lerobot.calibrate',
                                '--robot.type=so101_follower',
                                f'--robot.port={args.port}'
                            ], capture_output=False, text=True)
                            
                            if result.returncode == 0:
                                print("✅ Robot calibration completed!")
                                # Try connecting again
                                client = SO101Client(
                                    port=args.port,
                                    follower=True,
                                    force_calibration=False
                                )
                            else:
                                raise RuntimeError("Robot calibration failed")
                        else:
                            # Try with force_calibration=True to use default
                            client = SO101Client(
                                port=args.port,
                                follower=True,
                                force_calibration=True
                            )
                    else:
                        raise e
                
                print("✅ Robot connected!")
                
                # Disable torque to allow manual movement
                try:
                    if hasattr(client, 'robot') and hasattr(client.robot, 'bus'):
                        client.robot.bus.disable_torque()
                        print("✅ Torque disabled - robot can be moved by hand")
                    elif hasattr(client, 'disable_torque'):
                        client.disable_torque()
                        print("✅ Torque disabled - robot can be moved by hand")
                    else:
                        print("⚠️  Could not disable torque - robot may resist movement")
                except Exception as e:
                    print(f"⚠️  Could not disable torque: {e}")
                    print("   Robot may resist manual movement")
                
                print("\n⚠️  MANUAL CONTROL MODE")
                print("The robot should now be moveable by hand")
                print("Remember to attach checkerboard to gripper!")
            except Exception as e:
                print(f"\n⚠️  Failed to connect to robot: {e}")
                print("Running without robot connection")
                client = None
        
        # Capture calibration data
        num_captured = calibrator.manual_calibration_capture(
            client,
            min_positions=args.min_positions,
            max_positions=args.max_positions
        )
        
        if num_captured >= args.min_positions:
            # Compute calibration
            T_cam2base, T_base2cam = calibrator.compute_hand_eye_calibration()
            
            # Save results
            calibrator.save_calibration(T_cam2base, T_base2cam, args.output)
            
            print("\n" + "=" * 60)
            print("✅ STATIONARY CAMERA CALIBRATION COMPLETE!")
            print("=" * 60)
            print("\nTo use the calibration:")
            print(f"  data = np.load('{args.output}')")
            print("  T_cam2base = data['T_cam2base']  # Camera to base transform")
            print("  T_base2cam = data['T_base2cam']  # Base to camera transform")
            print("\nCalibration type: Eye-to-Hand (Stationary Camera)")
            
        else:
            print(f"\n❌ Insufficient data ({num_captured} < {args.min_positions})")
            
    except KeyboardInterrupt:
        print("\n⏹️  Calibration interrupted")
    except Exception as e:
        print(f"\n❌ Error: {e}")
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