import numpy as np
from record3d import Record3DStream

from threading import Event

import numpy as np
import open3d as o3d

from PIL import Image
from quaternion import as_rotation_matrix, quaternion

import plotly.graph_objs as go
import plotly.io as pio
from scipy.spatial.transform import Rotation

import time
import sys
import os
import logging
from dataclasses import asdict, dataclass
from pprint import pformat

# Add LeRobot to Python path
sys.path.insert(0, '/home/freax/Documents/GitHub/opensource_dev/lerobot/src')

# Import LeRobot components for SO-101 control
import draccus
from lerobot.robots import RobotConfig
from lerobot.robots.so101_follower import SO101Follower
from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
from lerobot.utils.utils import init_logging
import numpy as np

# ----------------------------------------------------------------------------
# -                        Open3D: www.open3d.org                            -
# ----------------------------------------------------------------------------
# Copyright (c) 2018-2023 www.open3d.org
# SPDX-License-Identifier: MIT
# ----------------------------------------------------------------------------

# examples/python/reconstruction_system/sensors/realsense_pcd_visualizer.py

# pyrealsense2 is required.
# Please see instructions in https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python

class SO101RobotController:
    """
    SO-101 Robot arm controller for iPhone camera pose-based control
    """
    def __init__(self, port="/dev/ttyUSB0", robot_id="so101_iphone"):
        init_logging()
        
        # Configure SO-101 robot
        self.robot_config = SO101FollowerConfig(
            port=port,
            id=robot_id,
            use_degrees=True,  # Use degrees for easier debugging
            cameras={}  # No cameras on follower arm
        )
        
        # Initialize robot
        self.robot = SO101Follower(self.robot_config)
        self.is_connected = False
        
        # Transform parameters for iPhone pose to robot control
        self.pose_scale = 100.0  # Scale factor for pose translation
        self.pose_offset = np.array([0.0, 0.0, 0.0])  # Offset for pose centering
        
        # Joint limits and safety
        self.joint_limits = {
            "shoulder_pan": [-90, 90],
            "shoulder_lift": [-90, 90], 
            "elbow_flex": [-90, 90],
            "wrist_flex": [-90, 90],
            "wrist_roll": [-180, 180],
            "gripper": [0, 100]
        }
        
        # Initial/home position
        self.home_position = {
            "shoulder_pan.pos": 0.0,
            "shoulder_lift.pos": 0.0,
            "elbow_flex.pos": 0.0,
            "wrist_flex.pos": 0.0,
            "wrist_roll.pos": 0.0,
            "gripper.pos": 0.0
        }
        
        self.current_action = self.home_position.copy()
        
    def connect(self):
        """Connect to SO-101 robot"""
        try:
            self.robot.connect(calibrate=True)
            self.is_connected = True
            print("✅ SO-101 robot connected successfully")
            
            # Move to home position
            self.move_to_home()
            
        except Exception as e:
            print(f"❌ Failed to connect to SO-101 robot: {e}")
            self.is_connected = False
            
    def disconnect(self):
        """Disconnect from SO-101 robot"""
        if self.is_connected:
            try:
                self.robot.disconnect()
                self.is_connected = False
                print("👋 SO-101 robot disconnected")
            except Exception as e:
                print(f"⚠️ Error during disconnect: {e}")
                
    def move_to_home(self):
        """Move robot to home position"""
        if not self.is_connected:
            return
            
        try:
            self.robot.send_action(self.home_position)
            print("🏠 Robot moved to home position")
        except Exception as e:
            print(f"⚠️ Error moving to home: {e}")
            
    def pose_to_joint_angles(self, camera_pose):
        """
        Convert iPhone camera pose to SO-101 joint angles
        This is a simple mapping - you'll need to tune this based on your setup
        """
        # Extract pose components
        qx, qy, qz, qw = camera_pose.qx, camera_pose.qy, camera_pose.qz, camera_pose.qw
        px, py, pz = camera_pose.tx, camera_pose.ty, camera_pose.tz
        
        # Convert quaternion to euler angles
        rotation = Rotation.from_quat([qx, qy, qz, qw])
        euler_angles = rotation.as_euler('xyz', degrees=True)
        
        # Scale and offset position
        scaled_pos = np.array([px, py, pz]) * self.pose_scale + self.pose_offset
        
        # Map pose to joint angles (this is a simple example - adjust as needed)
        joint_angles = {
            "shoulder_pan.pos": np.clip(scaled_pos[0], *self.joint_limits["shoulder_pan"]),
            "shoulder_lift.pos": np.clip(scaled_pos[1], *self.joint_limits["shoulder_lift"]),
            "elbow_flex.pos": np.clip(scaled_pos[2], *self.joint_limits["elbow_flex"]),
            "wrist_flex.pos": np.clip(euler_angles[0], *self.joint_limits["wrist_flex"]),
            "wrist_roll.pos": np.clip(euler_angles[2], *self.joint_limits["wrist_roll"]),
            "gripper.pos": 0.0  # Keep gripper closed for now
        }
        
        return joint_angles
        
    def send_pose_command(self, camera_pose):
        """Send camera pose as robot command"""
        if not self.is_connected:
            return
            
        try:
            # Convert pose to joint angles
            joint_angles = self.pose_to_joint_angles(camera_pose)
            
            # Send action to robot
            self.robot.send_action(joint_angles)
            self.current_action = joint_angles
            
        except Exception as e:
            print(f"⚠️ Error sending pose command: {e}")
            
    def get_robot_state(self):
        """Get current robot state"""
        if not self.is_connected:
            return None
            
        try:
            return self.robot.get_observation()
        except Exception as e:
            print(f"⚠️ Error getting robot state: {e}")
            return None








class DemoApp:
    def __init__(self, enable_robot=True, robot_port="/dev/ttyUSB0"):
        self.event = Event()
        self.session = None
        self.DEVICE_TYPE__TRUEDEPTH = 0
        self.DEVICE_TYPE__LIDAR = 1
        self.rgb_width = 720
        self.rgb_height = 960

        self.init_camera_pose = None
        # Create a Visualizer object
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window("iPhone Point Cloud Streaming + SO-101 Control", width=self.rgb_width, height=self.rgb_height)
        self.vis.get_view_control()

        # Create a PointCloud object
        self.pcd = o3d.geometry.PointCloud()

        # Initialize SO-101 robot controller
        self.enable_robot = enable_robot
        self.robot_controller = None
        if self.enable_robot:
            try:
                self.robot_controller = SO101RobotController(port=robot_port)
                self.robot_controller.connect()
                print("🤖 SO-101 robot integration enabled")
            except Exception as e:
                print(f"⚠️ Robot initialization failed: {e}")
                self.enable_robot = False

        # Control parameters
        self.pose_control_enabled = True
        self.frame_skip = 5  # Only send robot commands every N frames
        self.frame_count = 0


    def on_new_frame(self):
        """
        This method is called from non-main thread, therefore cannot be used for presenting UI.
        """
        self.event.set()  # Notify the main thread to stop waiting and process new frame.

    def on_stream_stopped(self):
        print('Stream stopped')

    def connect_to_device(self, dev_idx):
        print('Searching for devices')
        devs = Record3DStream.get_connected_devices()
        print('{} device(s) found'.format(len(devs)))
        for dev in devs:
            print('\tID: {}\n\tUDID: {}\n'.format(dev.product_id, dev.udid))

        if len(devs) <= dev_idx:
            raise RuntimeError('Cannot connect to device #{}, try different index.'
                               .format(dev_idx))

        dev = devs[dev_idx]
        self.session = Record3DStream()
        self.session.on_new_frame = self.on_new_frame
        self.session.on_stream_stopped = self.on_stream_stopped
        self.session.connect(dev)  # Initiate connection and start capturing

    def get_intrinsic_mat_from_coeffs(self, coeffs):
        return np.array([[coeffs.fx,         0, coeffs.tx],
                         [        0, coeffs.fy, coeffs.ty],
                         [        0,         0,         1]])


    def reshape_depth_and_conf(self, depth_image, confidence, rgb_image):


        pil_depth = Image.fromarray(depth_image)
        reshaped_depth = pil_depth.resize((self.rgb_width, self.rgb_height))
        reshaped_depth = np.asarray(reshaped_depth)


        conf_img = Image.fromarray(confidence)
        reshaped_conf = conf_img.resize((self.rgb_width, self.rgb_height))
        reshaped_conf = np.asarray(reshaped_conf)
      
      
        rgb = Image.fromarray(rgb_image)
        reshaped_rgb = rgb.resize((self.rgb_width, self.rgb_height))
        reshaped_rgb = np.asarray(reshaped_rgb)
        

        return reshaped_depth, reshaped_conf, reshaped_rgb


    





    def create_wireframe_cube(self, size=1.0):
        points = [[-size, -size, -size],
                [-size, -size, size],
                [-size, size, -size],
                [-size, size, size],
                [size, -size, -size],
                [size, -size, size],
                [size, size, -size],
                [size, size, size]]

        lines = [[0, 1], [1, 3], [3, 2], [2, 0],
                [4, 5], [5, 7], [7, 6], [6, 4],
                [0, 4], [1, 5], [2, 6], [3, 7]]

        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(points)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        return line_set



    def pose_to_extrinsic_matrix(self, camera_pose):
        """
        Convert a pose to an extrinsic matrix.
        """

        extrinsic_matrix = np.eye(4)
        qx, qy, qz, qw, px, py, pz = camera_pose.qx, camera_pose.qy, camera_pose.qz, camera_pose.qw, camera_pose.tx, camera_pose.ty, camera_pose.tz
        extrinsic_matrix[:3, :3] = as_rotation_matrix(quaternion(qw, qx, qy, qz))
        extrinsic_matrix[:3, -1] = [px, py, pz]



        return extrinsic_matrix




       
    def start_processing_stream(self):

        frame_count = 0
        frames = []
        prev_pose_matrix = None
        
        while True:                  
                self.event.wait()  # Wait for new frame to arrive
                # Copy the newly arrived RGBD frame
                depth = self.session.get_depth_frame()
                rgb = self.session.get_rgb_frame()
                # print(rgb.shape)    (960, 720, 3)
                # print(depth.shape)   (256, 192)
                confidence = self.session.get_confidence_frame()

                depth, confidence, rgb = self.reshape_depth_and_conf(depth, confidence, rgb)
                depth = depth.copy()
                rgb = rgb.copy()


                # this is the camera pose reading from iphone with respect to the world frame,
                # but the world frame is not the same as the inial frame camera frame when this script starts
                camera_pose = self.session.get_camera_pose() 
                extrinsic = self.pose_to_extrinsic_matrix(camera_pose)



            


                if self.init_camera_pose is None:




                    self.init_camera_pose = extrinsic

                    # define world frame as the initial camera pose
                    self.init_camera_frame_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
                    self.vis.add_geometry(self.init_camera_frame_mesh.transform(self.init_camera_pose))
                    # add a cube as the work space
                    cube = self.create_wireframe_cube(size=1)
                    self.vis.add_geometry(cube.transform(self.init_camera_pose))


                    self.camera_frame_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
                    self.vis.add_geometry(self.camera_frame_mesh.transform(self.init_camera_pose))


                else:
                                
             
             
                    self.vis.update_geometry(self.camera_frame_mesh.transform(np.linalg.inv(prev_pose_matrix)))

                    self.vis.update_geometry(self.camera_frame_mesh.transform(extrinsic))


                    
                    # Update visualization
                    self.vis.poll_events()
                    self.vis.update_renderer()



                frame_count += 1
                prev_pose_matrix = extrinsic

                self.event.clear()

   

if __name__ == '__main__':
    app = DemoApp()
    app.connect_to_device(dev_idx=0)
    app.start_processing_stream()
