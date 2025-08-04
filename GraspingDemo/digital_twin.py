#!/usr/bin/env python3
"""
Digital Twin Calibration Test System

Features:
1. Click on objects to get 3D coordinates
2. Preview robot movements in simulation
3. Optional execution on real robot after user confirmation
4. Complete digital twin workflow
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from typing import Optional, Tuple, List
from so101_grasp.vision.camera import CameraController
from so101_grasp.tools.image_selector import ImgClick
from so101_grasp.utils.utils import get_3d_point_from_2d_coordinates
from so101_grasp.utils.transform import transform_cam_to_rob
from so101_grasp.simulation.sim import SimGrasp, ObjectInfo
import open3d as o3d
from so101_grasp.robot.so101_client import SO101Client
import time
import cv2
import os
from datetime import datetime
try:
    from lerobot.model.kinematics import RobotKinematics
    KINEMATICS_AVAILABLE = True
except ImportError:
    KINEMATICS_AVAILABLE = False
    print("⚠️  Warning: lerobot kinematics module not available. Real robot execution will be limited.")

# Simulation environment configuration
SIMULATION_OBJECTS = [
    ObjectInfo(
        urdf_path="cube_small.urdf",
        position=[0.3, 0.0, 0.025],
        orientation=[0, 0, 0],  # Euler angles [roll, pitch, yaw]
        scaling=0.03,  # Single float value for uniform scaling
        color=[1, 0, 0, 1]  # Red cube as reference
    ),
    ObjectInfo(
        urdf_path="sphere2.urdf", 
        position=[0.2, 0.15, 0.025],
        orientation=[0, 0, 0],  # Euler angles [roll, pitch, yaw]
        scaling=0.02,  # Single float value for uniform scaling
        color=[0, 1, 0, 1]  # Green sphere as reference
    )
]

class DigitalTwinTester:
    def __init__(self, robot_port: Optional[str] = None, simulation_only: bool = False):
        """Initialize digital twin test system"""
        self.camera = None
        self.sim_grasp = None
        self.real_robot = None
        self.robot_port = robot_port
        self.simulation_only = simulation_only
        self.transform_matrix = None
        self.scaling_factor = None
        self.kinematics = None
        self.current_point_cloud = None  # Store current point cloud for simulation
        
        # Load calibration data
        self.load_calibration()
        
        # Initialize kinematics if available
        self.initialize_kinematics()
        
    def load_calibration(self) -> bool:
        """Load calibration data"""
        try:
            self.transform_matrix = np.load('scripts/config/transform_mat_so101.npy')
            self.scaling_factor = np.load('scripts/config/scaling_factor_so101.npy')
            print("✅ Calibration data loaded successfully")
            print(f"   RMS error: 0.0257m")
            print(f"   Scaling factor: {self.scaling_factor:.4f}")
            return True
        except FileNotFoundError as e:
            print(f"❌ Calibration file not found: {e}")
            print("Please run first: python scripts/calibrate_camera.py")
            return False
    
    def initialize_kinematics(self) -> bool:
        """Initialize inverse kinematics solver"""
        if not KINEMATICS_AVAILABLE:
            print("⚠️  Kinematics module not available - real robot execution disabled")
            return False
        
        try:
            # SO-101 URDF path
            urdf_path = "assets/urdf/so101_new_calib.urdf"
            
            # SO-101 joint names (excluding gripper for now)
            joint_names = [
                "shoulder_pan_joint",
                "shoulder_lift_joint", 
                "elbow_flex_joint",
                "wrist_flex_joint",
                "wrist_roll_joint"
            ]
            
            self.kinematics = RobotKinematics(
                urdf_path=urdf_path,
                target_frame_name="gripper_frame_link",  # End effector frame name
                joint_names=joint_names
            )
            
            print("✅ Inverse kinematics solver initialized")
            return True
            
        except Exception as e:
            print(f"❌ Failed to initialize kinematics: {e}")
            print("Real robot execution will be disabled")
            return False
    
    def transform_point_cam_to_rob(self, point_cam: np.ndarray) -> np.ndarray:
        """
        Transform a 3D point from camera coordinate system to robot coordinate system.
        
        Args:
            point_cam: 3D point in camera coordinates [x, y, z]
            
        Returns:
            3D point in robot coordinates [x, y, z]
        """
        # Use the loaded transformation matrix
        if self.transform_matrix is None:
            print("❌ No transformation matrix available")
            return point_cam
        
        # Convert point to homogeneous coordinates
        point_cam_homo = np.append(point_cam, 1.0)
        
        # Apply transformation
        point_rob_homo = self.transform_matrix @ point_cam_homo
        
        # Convert back to 3D coordinates
        return point_rob_homo[:3]
    
    def save_capture_images(self, color_image: np.ndarray, depth_image: np.ndarray, 
                          click_coords: Optional[Tuple[int, int]] = None, 
                          selected_point: Optional[Tuple[float, float, float]] = None) -> str:
        """
        Save RGB and depth images with timestamp and optional click information.
        
        Args:
            color_image: RGB color image
            depth_image: Depth image 
            click_coords: Optional click coordinates [x, y]
            selected_point: Optional selected 3D point [x, y, z]
            
        Returns:
            Base filename (without extension) used for saving
        """
        try:
            # Create output directory if it doesn't exist
            output_dir = "captures"
            os.makedirs(output_dir, exist_ok=True)
            
            # Generate timestamp-based filename
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            base_filename = f"capture_{timestamp}"
            
            # Save RGB image
            rgb_path = os.path.join(output_dir, f"{base_filename}_rgb.png")
            # Convert BGR to RGB for saving (OpenCV uses BGR)
            color_bgr = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
            
            # Add click marker if provided
            if click_coords is not None:
                x, y = click_coords
                # Draw a red circle at click location
                cv2.circle(color_bgr, (x, y), 10, (0, 0, 255), 2)  # Red circle
                cv2.circle(color_bgr, (x, y), 3, (0, 0, 255), -1)   # Red filled center
                
                # Add text with coordinates
                text = f"Click: ({x}, {y})"
                cv2.putText(color_bgr, text, (x + 15, y - 15), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                
                # Add 3D coordinates if available
                if selected_point is not None:
                    text_3d = f"3D: ({selected_point[0]:.3f}, {selected_point[1]:.3f}, {selected_point[2]:.3f})"
                    cv2.putText(color_bgr, text_3d, (x + 15, y + 20), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            
            cv2.imwrite(rgb_path, color_bgr)
            
            # Save depth image (both raw and colorized versions)
            depth_path = os.path.join(output_dir, f"{base_filename}_depth_raw.png")
            cv2.imwrite(depth_path, depth_image.astype(np.uint16))
            
            # Create colorized depth image for visualization
            depth_colorized = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.08), 
                cv2.COLORMAP_JET
            )
            
            # Add click marker to depth image too
            if click_coords is not None:
                x, y = click_coords
                cv2.circle(depth_colorized, (x, y), 10, (0, 0, 255), 2)
                cv2.circle(depth_colorized, (x, y), 3, (0, 0, 255), -1)
                
                # Show depth value at clicked location
                depth_value = depth_image[y, x] if 0 <= y < depth_image.shape[0] and 0 <= x < depth_image.shape[1] else 0
                depth_text = f"Depth: {depth_value}mm"
                cv2.putText(depth_colorized, depth_text, (x + 15, y - 15),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            depth_color_path = os.path.join(output_dir, f"{base_filename}_depth_color.png")
            cv2.imwrite(depth_color_path, depth_colorized)
            
            print(f"💾 Images saved:")
            print(f"   📸 RGB: {rgb_path}")
            print(f"   📏 Depth (raw): {depth_path}")
            print(f"   🎨 Depth (colorized): {depth_color_path}")
            
            return base_filename
            
        except Exception as e:
            print(f"❌ Failed to save images: {e}")
            return ""
    
    def cartesian_to_joint_angles(self, target_pos: Tuple[float, float, float], 
                                 current_joints: Optional[List[float]] = None) -> Optional[List[float]]:
        """
        Convert Cartesian target position to joint angles using inverse kinematics.
        
        Args:
            target_pos: Target position [x, y, z] in robot base frame
            current_joints: Current joint positions (optional, will read from robot if not provided)
            
        Returns:
            Joint angles in degrees, or None if IK failed
        """
        if not self.kinematics:
            print("❌ Kinematics solver not available")
            return None
        
        try:
            # Get current joint positions if not provided
            if current_joints is None:
                if self.real_robot:
                    current_joints = self.real_robot.read_joints()
                    # Convert to degrees (SO101Client uses radians internally)
                    current_joints = [np.degrees(angle) for angle in current_joints]
                else:
                    # Default neutral position if no robot available
                    current_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            
            # Create desired pose matrix (position + orientation)
            desired_pose = np.eye(4)
            desired_pose[:3, 3] = target_pos
            
            # Use downward-facing orientation (similar to move_to_example.py)
            from scipy.spatial.transform import Rotation as R
            desired_pose[:3, :3] = R.from_euler('xyz', [np.pi, 0, 0]).as_matrix()
            
            # Solve inverse kinematics
            target_joint_angles = self.kinematics.inverse_kinematics(
                current_joint_pos=current_joints,
                desired_ee_pose=desired_pose,
                position_weight=1.0,
                orientation_weight=0.1  # Lower weight for orientation
            )
            
            print(f"🔧 IK Solution: {[f'{angle:.2f}°' for angle in target_joint_angles[:5]]}")
            return target_joint_angles
            
        except Exception as e:
            print(f"❌ Inverse kinematics failed: {e}")
            return None
    
    def create_point_cloud_from_rgbd(self, color_image: np.ndarray, depth_image: np.ndarray, 
                                   intrinsics: object) -> o3d.geometry.PointCloud:
        """
        Create Open3D point cloud from RGB-D data.
        
        Args:
            color_image: RGB color image
            depth_image: Depth image in mm
            intrinsics: Camera intrinsics
            
        Returns:
            Open3D PointCloud object
        """
        # Convert images to Open3D format
        color_o3d = o3d.geometry.Image(color_image.astype(np.uint8))
        
        # Convert depth from mm to meters and filter invalid/far values
        depth_meters = (depth_image.astype(np.float32)) / 1000.0
        
        # Filter depth: keep only reasonable tabletop distances (30cm to 1.2m)
        depth_filtered = depth_meters.copy()
        depth_filtered[(depth_meters < 0.3) | (depth_meters > 1.2)] = 0.0
        
        depth_o3d = o3d.geometry.Image(depth_filtered)
        
        # Create RGBD image with stricter depth limits for tabletop scenes
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_o3d, depth_o3d, 
            depth_scale=1.0,  # Already converted to meters
            depth_trunc=1.2,  # 1.2 meter max depth for tabletop
            convert_rgb_to_intensity=False
        )
        
        # Create camera intrinsics for Open3D
        height, width = color_image.shape[:2]
        camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width=width,
            height=height, 
            fx=intrinsics.fx,
            fy=intrinsics.fy,
            cx=intrinsics.ppx,
            cy=intrinsics.ppy
        )
        
        # Generate point cloud
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image, camera_intrinsic
        )
        
        return pcd
    
    def get_target_position_from_pointcloud(self) -> Optional[Tuple[float, float, float]]:
        """
        Get target position using point cloud selection instead of depth image lookup.
        
        Returns:
            3D position in camera coordinates, or None if failed
        """
        if not self.camera:
            return None
            
        # Capture RGB-D data
        color_image, depth_image, intrinsics = self.camera.capture_rgbd()
        if color_image is None:
            print("❌ Unable to capture image")
            return None
        
        print("📊 Creating point cloud from RGB-D data...")
        
        # Create point cloud
        try:
            pcd = self.create_point_cloud_from_rgbd(color_image, depth_image, intrinsics)
            num_points = len(pcd.points)
            print(f"✅ Point cloud created with {num_points} points")
            
            if num_points == 0:
                print("❌ Point cloud is empty - no valid depth data")
                return None
            
            # Store point cloud for simulation use
            self.current_point_cloud = pcd
                
        except Exception as e:
            print(f"❌ Failed to create point cloud: {e}")
            return None
        
        # Get click coordinates on image
        print("🖱️  Click on target object in the image...")
        img_click = ImgClick(color_image, os="LINUX")
        click_coords = img_click.run()
        
        if click_coords is None:
            return None
        
        x_click, y_click = click_coords
        print(f"🖱️  Click coordinates: ({x_click}, {y_click})")
        
        # Find nearest point in point cloud to clicked pixel
        try:
            # Get all points and colors
            points = np.asarray(pcd.points)
            colors = np.asarray(pcd.colors)
            
            if len(points) == 0:
                print("❌ No points in point cloud")
                return None
            
            # Project 3D points back to image coordinates to find closest to click
            height, width = color_image.shape[:2]
            
            # Convert 3D points to image coordinates
            fx, fy = intrinsics.fx, intrinsics.fy
            cx, cy = intrinsics.ppx, intrinsics.ppy
            
            # Project points to image plane
            image_coords = []
            valid_indices = []
            
            for i, point in enumerate(points):
                x, y, z = point
                if z > 0:  # Valid depth
                    u = int(fx * x / z + cx)
                    v = int(fy * y / z + cy)
                    if 0 <= u < width and 0 <= v < height:
                        image_coords.append([u, v])
                        valid_indices.append(i)
            
            if len(image_coords) == 0:
                print("❌ No valid points found in point cloud")
                return None
            
            image_coords = np.array(image_coords)
            valid_indices = np.array(valid_indices)
            
            # Find point closest to click
            distances = np.sqrt((image_coords[:, 0] - x_click)**2 + (image_coords[:, 1] - y_click)**2)
            closest_idx = valid_indices[np.argmin(distances)]
            selected_point = points[closest_idx]
            
            min_distance = np.min(distances)
            print(f"📍 Selected point: [{selected_point[0]:.3f}, {selected_point[1]:.3f}, {selected_point[2]:.3f}]")
            print(f"📏 Distance from click: {min_distance:.1f} pixels")
            
            # Save images with click information
            self.save_capture_images(
                color_image, depth_image, 
                click_coords=(x_click, y_click), 
                selected_point=tuple(selected_point)
            )
            
            # Add point cloud to simulation if available
            if self.sim_grasp is not None:
                print("🎬 Adding point cloud to simulation environment...")
                self.add_pointcloud_to_simulation()
            
            # Visualize the selection (optional)
            self.visualize_point_selection(pcd, closest_idx, click_coords)
            
            return tuple(selected_point)
            
        except Exception as e:
            print(f"❌ Failed to find point in point cloud: {e}")
            return None
    
    def visualize_point_selection(self, pcd: o3d.geometry.PointCloud, selected_idx: int, 
                                click_coords: Tuple[int, int]):
        """
        Visualize the selected point in the point cloud. Falls back to headless mode if display fails.
        """
        try:
            print("🎨 Creating point cloud visualization...")
            
            # Create a copy for visualization
            vis_pcd = o3d.geometry.PointCloud(pcd)
            points = np.asarray(vis_pcd.points)
            colors = np.asarray(vis_pcd.colors)
            
            # Highlight selected point in bright red
            colors[selected_idx] = [1.0, 0.0, 0.0]
            vis_pcd.colors = o3d.utility.Vector3dVector(colors)
            
            # Create a sphere at the selected point for better visibility
            selected_point = points[selected_idx]
            sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
            sphere.translate(selected_point)
            sphere.paint_uniform_color([1.0, 0.0, 0.0])  # Red sphere
            
            # Add coordinate frame at selected point
            coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
                size=0.05, origin=selected_point
            )
            
            # Try to create visualization window - fall back gracefully if it fails
            try:
                # Create visualization window
                vis = o3d.visualization.Visualizer()
                window_created = vis.create_window(
                    window_name=f"Point Cloud - Click at pixel ({click_coords[0]}, {click_coords[1]})",
                    width=1200, 
                    height=800
                )
                
                if not window_created:
                    raise RuntimeError("Failed to create visualization window")
                
                # Add geometries
                vis.add_geometry(vis_pcd)
                vis.add_geometry(sphere)
                vis.add_geometry(coord_frame)
                
                # Set nice viewing parameters - check if view control is available
                view_ctl = vis.get_view_control()
                if view_ctl is not None:
                    view_ctl.set_front([0, 0, -1])
                    view_ctl.set_up([0, -1, 0])
                    view_ctl.set_lookat(selected_point)
                    view_ctl.set_zoom(0.8)
                
                print("🖥️  Point cloud visualization opened!")
                print(f"📍 Red sphere shows selected point: [{selected_point[0]:.3f}, {selected_point[1]:.3f}, {selected_point[2]:.3f}]")
                print("   - Rotate: Left mouse + drag")
                print("   - Zoom: Scroll wheel")
                print("   - Pan: Ctrl + left mouse + drag")
                print("   - Close window to continue...")
                
                # Run visualization (blocks until window is closed)
                vis.run()
                vis.destroy_window()
                
                print("✅ Visualization closed")
                
            except Exception as display_error:
                print(f"⚠️  Display visualization failed: {display_error}")
                print("💾 Saving point cloud to file instead...")
                
                # Save point cloud for offline viewing
                output_dir = "captures"
                os.makedirs(output_dir, exist_ok=True)
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                pcd_path = os.path.join(output_dir, f"pointcloud_{timestamp}.ply")
                
                # Combine all geometries into one point cloud for saving
                combined_pcd = vis_pcd
                o3d.io.write_point_cloud(pcd_path, combined_pcd)
                
                print(f"📁 Point cloud saved to: {pcd_path}")
                print("   You can view it later with: o3d.visualization.draw_geometries([o3d.io.read_point_cloud('path')])")
            
        except Exception as e:
            print(f"⚠️  Point cloud processing failed: {e}")
            print("Continuing without visualization...")
    
    def add_pointcloud_to_simulation(self) -> bool:
        """
        Add the current point cloud to the simulation environment for visualization.
        
        Returns:
            True if successful, False otherwise
        """
        if not self.sim_grasp:
            print("❌ Simulation environment not initialized")
            return False
        
        if self.current_point_cloud is None:
            print("❌ No point cloud available - capture one first")
            return False
        
        try:
            print("🎬 Adding point cloud to simulation...")
            
            # Transform point cloud to robot coordinate system for simulation
            pcd_robot = o3d.geometry.PointCloud(self.current_point_cloud)
            
            # Apply transformation if available
            if self.transform_matrix is not None:
                pcd_robot.transform(self.transform_matrix)
                print("✅ Point cloud transformed to robot coordinate system")
            
            # Add point cloud to simulation using existing method
            debug_item_id = self.sim_grasp.add_pointcloud(
                pcd_robot,
                default_color=(0.6, 0.8, 1.0),  # Light blue
                point_size=3.0,
                life_time=0.0  # Permanent
            )
            
            if debug_item_id is not None:
                print(f"✅ Point cloud added to simulation (ID: {debug_item_id})")
                print(f"   Points: {len(pcd_robot.points)}")
                return True
            else:
                print("❌ Failed to add point cloud to simulation")
                return False
                
        except Exception as e:
            print(f"❌ Failed to add point cloud to simulation: {e}")
            return False
    
    def get_target_from_simulation_pointcloud(self, click_coords: Tuple[int, int]) -> Optional[Tuple[float, float, float]]:
        """
        Get target position directly from simulation point cloud without needing camera.
        Useful when you want to select points from the simulation environment.
        
        Args:
            click_coords: Click coordinates from image
            
        Returns:
            3D position in robot coordinate system
        """
        if self.current_point_cloud is None:
            print("❌ No point cloud available")
            return None
        
        try:
            # Transform point cloud to robot frame
            pcd_robot = o3d.geometry.PointCloud(self.current_point_cloud)
            if self.transform_matrix is not None:
                pcd_robot.transform(self.transform_matrix)
            
            points = np.asarray(pcd_robot.points)
            if len(points) == 0:
                return None
            
            # For simulation, we can use the click coordinates to select from the transformed point cloud
            # This is a simplified version - in practice you might want more sophisticated selection
            
            # Find center point as a simple selection method
            center_point = np.mean(points, axis=0)
            print(f"📍 Selected center point from point cloud: [{center_point[0]:.3f}, {center_point[1]:.3f}, {center_point[2]:.3f}]")
            
            return tuple(center_point)
            
        except Exception as e:
            print(f"❌ Failed to get target from simulation point cloud: {e}")
            return None
    
    def initialize_camera(self) -> bool:
        """Initialize camera"""
        self.camera = CameraController(width=640, height=480, fps=30)
        if self.camera.connect():
            print("✅ Camera connected successfully")
            return True
        else:
            print("❌ Camera connection failed")
            return False
    
    def initialize_simulation(self) -> bool:
        """Initialize simulation environment"""
        try:
            # Set environment variable to force headless mode
            import os
            os.environ['DISPLAY'] = ''  # Prevent X11 display access
            
            # Try the simplest initialization first
            self.sim_grasp = SimGrasp(
                objects=SIMULATION_OBJECTS,
                urdf_path="assets/urdf/so101_new_calib.urdf",
                frequency=30
            )
            print("✅ Simulation environment initialized successfully")
            
            # Add point cloud to simulation if available
            if self.current_point_cloud is not None:
                self.add_pointcloud_to_simulation()
            
            return True
        except Exception as e:
            print(f"❌ Simulation environment initialization failed: {e}")
            print(f"Error details: {str(e)}")
            # Try version without URDF
            try:
                print("🔄 Trying initialization without URDF...")
                self.sim_grasp = SimGrasp(
                    objects=SIMULATION_OBJECTS,
                    frequency=30
                )
                print("✅ Simulation environment without robot initialized successfully")
                
                # Add point cloud to simulation if available
                if self.current_point_cloud is not None:
                    self.add_pointcloud_to_simulation()
                
                return True
            except Exception as e2:
                print(f"❌ Initialization without robot also failed: {e2}")
                return False
    
    def connect_real_robot(self) -> bool:
        """Connect to real robot"""
        if not self.robot_port:
            print("⚠️  No robot port specified, skipping real robot connection")
            return False
            
        try:
            # SO101Client connects automatically during initialization
            self.real_robot = SO101Client(port=self.robot_port)
            print(f"✅ Real robot connected successfully: {self.robot_port}")
            
            if self.kinematics:
                print("🔧 Inverse kinematics available - full Cartesian control enabled")
            else:
                print("⚠️  Inverse kinematics not available - limited to joint space control")
            
            return True
        except Exception as e:
            print(f"❌ Real robot connection exception: {e}")
            return False
    
    def get_target_position(self) -> Optional[Tuple[float, float, float]]:
        """Get target position using point cloud method"""
        if not self.camera:
            return None
        
        # Use point cloud-based selection instead of depth image lookup
        print("🔍 Using point cloud-based target selection...")
        point_3d_cam = self.get_target_position_from_pointcloud()
        
        if point_3d_cam is None:
            print("❌ Failed to get target position from point cloud")
            # Fallback to original depth image method
            print("🔄 Falling back to depth image method...")
            return self.get_target_position_fallback()
        
        # Convert to robot coordinate system  
        point_3d_robot = self.transform_point_cam_to_rob(np.array(point_3d_cam))
        
        print(f"📍 Camera coordinate system: [{point_3d_cam[0]:.3f}, {point_3d_cam[1]:.3f}, {point_3d_cam[2]:.3f}]")
        print(f"🤖 Robot coordinate system: [{point_3d_robot[0]:.3f}, {point_3d_robot[1]:.3f}, {point_3d_robot[2]:.3f}]")
        
        return tuple(point_3d_robot)
    
    def get_target_position_fallback(self) -> Optional[Tuple[float, float, float]]:
        """Fallback method using original depth image approach"""
        # Capture image
        color_image, depth_image, intrinsics = self.camera.capture_rgbd()
        if color_image is None:
            print("❌ Unable to capture image")
            return None
        
        # Click to get coordinates
        img_click = ImgClick(color_image, os="LINUX")
        click_coords = img_click.run()
        
        if click_coords is None:
            return None
        
        print(f"🖱️  Click coordinates: ({click_coords[0]}, {click_coords[1]})")
        
        # Calculate 3D coordinates
        point_3d_cam = get_3d_point_from_2d_coordinates(
            click_coords, depth_image, intrinsics
        )
        
        if point_3d_cam is None:
            print("❌ Unable to calculate 3D coordinates")
            # Still save images even if 3D calculation failed
            self.save_capture_images(color_image, depth_image, click_coords=click_coords)
            return None
        
        # Save images with successful 3D point
        self.save_capture_images(
            color_image, depth_image, 
            click_coords=click_coords, 
            selected_point=point_3d_cam
        )
        
        # Convert to robot coordinate system  
        point_3d_robot = self.transform_point_cam_to_rob(np.array(point_3d_cam))
        
        print(f"📍 Camera coordinate system: [{point_3d_cam[0]:.3f}, {point_3d_cam[1]:.3f}, {point_3d_cam[2]:.3f}]")
        print(f"🤖 Robot coordinate system: [{point_3d_robot[0]:.3f}, {point_3d_robot[1]:.3f}, {point_3d_robot[2]:.3f}]")
        
        return tuple(point_3d_robot)
    
    def preview_in_simulation(self, target_pos: Tuple[float, float, float]) -> bool:
        """Preview movement in simulation"""
        if not self.sim_grasp:
            print("❌ Simulation environment not initialized")
            return False
        
        try:
            print("🎬 Previewing movement in simulation...")
            
            # Move to above target position
            hover_pos = [target_pos[0], target_pos[1], target_pos[2] + 0.1]
            
            print(f"   1. Move to hover position: [{hover_pos[0]:.3f}, {hover_pos[1]:.3f}, {hover_pos[2]:.3f}]")
            success1 = self.sim_grasp.move_to_position(hover_pos)
            
            if success1:
                time.sleep(1.0)  # Brief pause for user observation
                
                print(f"   2. Descend to target position: [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}]")
                success2 = self.sim_grasp.move_to_position(list(target_pos))
                
                if success2:
                    time.sleep(1.0)
                    print("✅ Simulation preview completed")
                    return True
                else:
                    print("❌ Cannot reach target position in simulation")
                    return False
            else:
                print("❌ Cannot reach hover position in simulation")
                return False
                
        except Exception as e:
            print(f"❌ Simulation preview failed: {e}")
            return False
    
    def execute_on_real_robot(self, target_pos: Tuple[float, float, float]) -> bool:
        """Execute on real robot"""
        if not self.real_robot:
            print("❌ Real robot not connected")
            return False
        
        try:
            print("🤖 Executing on real robot...")
            
            # Safety check
            x, y, z = target_pos
            if not (-0.5 < x < 0.5 and -0.5 < y < 0.5 and 0 < z < 0.4):
                print("❌ Target position outside safety range, canceling execution")
                return False
            
            print(f"   Target position: [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}]")
            
            # Convert Cartesian coordinates to joint angles
            joint_angles = self.cartesian_to_joint_angles(target_pos)
            
            if joint_angles is None:
                print("❌ Failed to compute inverse kinematics")
                return False
            
            # Execute movement in steps: hover -> target -> hover
            current_joints = self.real_robot.read_joints()
            current_joints_deg = [np.degrees(angle) for angle in current_joints]
            
            # 1. Move to hover position (10cm above target)
            hover_pos = [target_pos[0], target_pos[1], target_pos[2] + 0.1]
            hover_joint_angles = self.cartesian_to_joint_angles(hover_pos, current_joints_deg)
            
            if hover_joint_angles is not None:
                print(f"   1. Moving to hover position...")
                # Convert back to radians for SO101Client
                hover_joints_rad = [np.radians(angle) for angle in hover_joint_angles[:6]]
                self.real_robot.interpolate_waypoint(
                    current_joints, hover_joints_rad, steps=30, timestep=0.1
                )
                time.sleep(2.0)
                
                # 2. Move to target position
                print(f"   2. Descending to target position...")
                target_joints_rad = [np.radians(angle) for angle in joint_angles[:6]]
                self.real_robot.interpolate_waypoint(
                    hover_joints_rad, target_joints_rad, steps=20, timestep=0.1
                )
                time.sleep(1.0)
                
                # 3. Return to hover position
                print(f"   3. Returning to hover position...")
                self.real_robot.interpolate_waypoint(
                    target_joints_rad, hover_joints_rad, steps=20, timestep=0.1
                )
                
                print("✅ Real robot execution completed successfully!")
                return True
            else:
                print("❌ Failed to compute hover position kinematics")
                return False
                
        except Exception as e:
            print(f"❌ Real robot execution failed: {e}")
            return False
    
    def run_test(self):
        """Run digital twin test"""
        print("🔄 Digital twin calibration test system starting")
        print("=" * 60)
        
        # Initialize all components
        if not self.initialize_camera():
            return
        
        if not self.initialize_simulation():
            return
        
        has_real_robot = self.connect_real_robot()
        
        print("\n📋 Test instructions:")
        print("1. Click on target position in camera image")
        print("2. System calculates 3D coordinates and previews in simulation")
        print("3. Choose whether to execute on real robot")
        print("4. Press ESC or close window to exit")
        print("-" * 60)
        
        test_count = 0
        
        while True:
            print(f"\n🎯 Test #{test_count + 1}")
            print("Please click on target position in camera image...")
            
            # Get target position
            target_pos = self.get_target_position()
            if target_pos is None:
                print("👋 Test ended")
                break
            
            test_count += 1
            
            # Simulation preview
            preview_success = self.preview_in_simulation(target_pos)
            
            if preview_success:
                # Ask whether to execute on real robot
                if has_real_robot:
                    response = input("\n❓ Simulation preview completed! Execute on real robot? (y/N): ").strip().lower()
                    
                    if response in ['y', 'yes', 'Y']:
                        print("⚠️  Preparing to execute on real robot...")
                        confirm = input("Confirm execution again? (y/N): ").strip().lower()
                        
                        if confirm in ['y', 'yes', 'Y']:
                            self.execute_on_real_robot(target_pos)
                        else:
                            print("✋ User canceled real robot execution")
                    else:
                        print("📝 Simulation preview only completed")
                else:
                    print("💡 To test real robot, please specify robot port at startup")
            
            print("-" * 60)
        
        # Clean up resources
        self.cleanup()
        print(f"🎉 Test completed! Total tested {test_count} points")
    
    def cleanup(self):
        """Clean up resources"""
        if self.camera:
            self.camera.disconnect()
            print("📷 Camera disconnected")
        
        if self.sim_grasp:
            try:
                import pybullet as pb
                pb.disconnect()
                print("🎬 Simulation environment closed")
            except:
                pass
        
        if self.real_robot:
            self.real_robot.disconnect()
            print("🤖 Real robot disconnected")

def main():
    """Main function"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Digital twin calibration test system")
    parser.add_argument("--robot-port", type=str, default=None,
                       help="Real robot port (e.g.: /dev/ttyACM0)")
    
    args = parser.parse_args()
    
    # Create and run test system
    tester = DigitalTwinTester(robot_port=args.robot_port)
    tester.run_test()

if __name__ == "__main__":
    main()