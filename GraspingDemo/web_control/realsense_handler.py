#!/usr/bin/env python3
"""
RealSense Camera Handler for Web Control
Handles RealSense D435/D415 camera operations including RGB, depth, and point cloud
"""

import numpy as np
import base64
import json
import time
from datetime import datetime
from pathlib import Path
import threading

# Try to import optional dependencies
try:
    import cv2
except ImportError:
    print("Warning: OpenCV not installed. Install with: pip install opencv-python")
    cv2 = None

try:
    import pyrealsense2 as rs
except ImportError:
    print("Warning: pyrealsense2 not installed. Install with: pip install pyrealsense2")
    rs = None

class RealSenseHandler:
    def __init__(self):
        if rs is None:
            raise ImportError("pyrealsense2 is not installed. Please install it with: pip install pyrealsense2")
        if cv2 is None:
            raise ImportError("OpenCV is not installed. Please install it with: pip install opencv-python")
            
        self.pipeline = None
        self.config = None
        self.align = None
        self.is_streaming = False
        self.current_frames = {}
        self.capture_dir = Path.home() / "Documents/Github/opensource_dev/GraspingDemo/captures"
        self.capture_dir.mkdir(exist_ok=True)
        self.colorizer = rs.colorizer()
        self.pc = rs.pointcloud()
        self.intrinsics = None
        
    def connect(self):
        """Connect to RealSense camera"""
        try:
            # Create a pipeline
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            
            # Configure streams
            self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)  # Use RGB format
            
            # Start streaming
            profile = self.pipeline.start(self.config)
            
            # Create align object
            align_to = rs.stream.color
            self.align = rs.align(align_to)
            
            # Get camera intrinsics
            depth_profile = profile.get_stream(rs.stream.depth)
            self.intrinsics = depth_profile.as_video_stream_profile().get_intrinsics()
            
            # Get depth scale (units per meter)
            depth_sensor = profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()
            
            # Update CameraConfig to match RealSense for ThinkGrasp compatibility
            from camera_config import CameraConfig
            CameraConfig.from_realsense_intrinsics(self.intrinsics, self.depth_scale)
            
            self.is_streaming = True
            
            # Start background thread to continuously update frames
            self.update_thread = threading.Thread(target=self._update_frames)
            self.update_thread.daemon = True
            self.update_thread.start()
            
            return True, "RealSense camera connected"
            
        except Exception as e:
            return False, f"Failed to connect: {str(e)}"
    
    def disconnect(self):
        """Disconnect from RealSense camera"""
        try:
            self.is_streaming = False
            if self.pipeline:
                self.pipeline.stop()
                self.pipeline = None
            return True, "Camera disconnected"
        except Exception as e:
            return False, f"Failed to disconnect: {str(e)}"
    
    def _update_frames(self):
        """Background thread to continuously update frames"""
        while self.is_streaming:
            try:
                # Wait for frames
                frames = self.pipeline.wait_for_frames(timeout_ms=1000)
                
                # Align frames
                aligned_frames = self.align.process(frames)
                
                # Get aligned frames
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                
                if depth_frame and color_frame:
                    # Convert to numpy arrays
                    depth_image = np.asanyarray(depth_frame.get_data())
                    color_image = np.asanyarray(color_frame.get_data())
                    
                    # Colorize depth for visualization
                    depth_colormap = np.asanyarray(
                        self.colorizer.colorize(depth_frame).get_data()
                    )
                    
                    # Generate point cloud
                    points = self.pc.calculate(depth_frame)
                    self.pc.map_to(color_frame)
                    
                    # Store current frames (keep original frame objects for export)
                    self.current_frames = {
                        'color': color_image,
                        'depth': depth_image,
                        'depth_colormap': depth_colormap,
                        'points': points,
                        'color_frame': color_frame,  # Keep original frame for export
                        'depth_frame': depth_frame,  # Keep original frame
                        'timestamp': time.time()
                    }
                    
            except Exception as e:
                print(f"Frame update error: {e}")
                time.sleep(0.1)
    
    def get_rgb_frame(self):
        """Get current RGB frame as base64 encoded JPEG"""
        if not self.current_frames or 'color' not in self.current_frames:
            return None
            
        try:
            color_image = self.current_frames['color']
            
            # Convert RGB to BGR for cv2.imencode
            bgr_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
            
            # Convert to JPEG
            _, buffer = cv2.imencode('.jpg', bgr_image)
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')
            
            return jpg_as_text
            
        except Exception as e:
            print(f"Error getting RGB frame: {e}")
            return None
    
    def get_depth_frame(self):
        """Get current depth frame as base64 encoded image"""
        if not self.current_frames or 'depth_colormap' not in self.current_frames:
            return None
            
        try:
            depth_colormap = self.current_frames['depth_colormap']
            
            # Convert to JPEG
            _, buffer = cv2.imencode('.jpg', depth_colormap)
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')
            
            return jpg_as_text
            
        except Exception as e:
            print(f"Error getting depth frame: {e}")
            return None
    
    def get_point_cloud(self, downsample=4, max_points=20000):
        """Get point cloud data for visualization"""
        if not self.current_frames or 'points' not in self.current_frames:
            return None
            
        try:
            points = self.current_frames['points']
            color_image = self.current_frames['color']
            
            # Get vertices
            vertices = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)
            
            # Get texture coordinates
            tex_coords = np.asanyarray(points.get_texture_coordinates()).view(np.float32).reshape(-1, 2)
            
            # Filter valid points (exclude saturated and far values)
            valid_mask = (vertices[:, 2] > 0) & (vertices[:, 2] < 10.0)  # Keep points within 10 meters
            vertices = vertices[valid_mask]
            tex_coords = tex_coords[valid_mask]
            
            # Downsample for web visualization
            vertices = vertices[::downsample]
            tex_coords = tex_coords[::downsample]
            
            # Further limit if exceeding max_points
            if len(vertices) > max_points:
                indices = np.random.choice(len(vertices), max_points, replace=False)
                vertices = vertices[indices]
                tex_coords = tex_coords[indices]
            
            # Get colors from texture coordinates
            h, w = color_image.shape[:2]
            colors = []
            for tc in tex_coords:
                x = int(tc[0] * w)
                y = int(tc[1] * h)
                if 0 <= x < w and 0 <= y < h:
                    # RealSense gives RGB directly (not BGR)
                    color = color_image[y, x]
                    colors.append(color)
                else:
                    colors.append([128, 128, 128])
            
            colors = np.array(colors)
            
            # Convert to lists for JSON serialization
            point_cloud_data = {
                'vertices': vertices.tolist(),
                'colors': colors.tolist(),
                'num_points': len(vertices)
            }
            
            return point_cloud_data
            
        except Exception as e:
            print(f"Error getting point cloud: {e}")
            return None
    
    def capture(self, include_pointcloud=True, robot_state=None):
        """Capture current frames and save to disk with metadata"""
        if not self.current_frames:
            return False, "No frames available"
            
        try:
            timestamp = datetime.now()
            timestamp_str = timestamp.strftime("%Y%m%d_%H%M%S")
            capture_folder = self.capture_dir / timestamp_str
            capture_folder.mkdir(exist_ok=True)
            
            # Create comprehensive metadata
            metadata = {
                'timestamp': timestamp.isoformat(),
                'timestamp_unix': time.time(),
                'camera_type': 'realsense',
                'files': {}
            }
            
            # Save RGB image
            if 'color' in self.current_frames:
                rgb_path = capture_folder / "rgb.png"
                # Now RealSense provides RGB format, but cv2.imwrite expects BGR
                bgr_image = cv2.cvtColor(self.current_frames['color'], cv2.COLOR_RGB2BGR)
                cv2.imwrite(str(rgb_path), bgr_image)
                metadata['files']['rgb'] = "rgb.png"
                metadata['rgb_shape'] = self.current_frames['color'].shape
            
            # Save depth image (raw)
            if 'depth' in self.current_frames:
                depth_path = capture_folder / "depth.npy"
                np.save(str(depth_path), self.current_frames['depth'])
                metadata['files']['depth'] = "depth.npy"
                metadata['depth_shape'] = self.current_frames['depth'].shape
                
                # Also save depth as PNG for ThinkGrasp compatibility
                depth_png_path = capture_folder / "depth.png"
                depth_data = self.current_frames['depth']
                # Convert depth to 16-bit PNG (maintaining millimeter precision)
                depth_uint16 = np.clip(depth_data, 0, 65535).astype(np.uint16)
                cv2.imwrite(str(depth_png_path), depth_uint16)
                metadata['files']['depth_png'] = "depth.png"
                
                # Get depth statistics
                metadata['depth_stats'] = {
                    'min': float(np.min(depth_data[depth_data > 0])) if np.any(depth_data > 0) else 0,
                    'max': float(np.max(depth_data)),
                    'mean': float(np.mean(depth_data[depth_data > 0])) if np.any(depth_data > 0) else 0,
                    'units': 'millimeters'
                }
                
                # Also save colorized depth for visualization
                depth_vis_path = capture_folder / "depth_colorized.png"
                cv2.imwrite(str(depth_vis_path), self.current_frames['depth_colormap'])
                metadata['files']['depth_colorized'] = "depth_colorized.png"
            
            # Save point cloud if requested
            if include_pointcloud and 'points' in self.current_frames and 'color_frame' in self.current_frames:
                try:
                    points = self.current_frames['points']
                    color_frame = self.current_frames['color_frame']
                    ply_path = capture_folder / "pointcloud.ply"
                    points.export_to_ply(str(ply_path), color_frame)
                    metadata['files']['pointcloud'] = "pointcloud.ply"
                except Exception as e:
                    print(f"Warning: Could not save point cloud: {e}")
            
            # Save camera intrinsics in both RealSense and ThinkGrasp formats
            if self.intrinsics:
                intrinsics_data = {
                    'width': self.intrinsics.width,
                    'height': self.intrinsics.height,
                    'fx': self.intrinsics.fx,
                    'fy': self.intrinsics.fy,
                    'ppx': self.intrinsics.ppx,  # RealSense naming
                    'ppy': self.intrinsics.ppy,  # RealSense naming
                    'cx': self.intrinsics.ppx,   # ThinkGrasp naming
                    'cy': self.intrinsics.ppy,   # ThinkGrasp naming
                    'model': str(self.intrinsics.model),
                    'coeffs': list(self.intrinsics.coeffs),
                    'depth_scale': getattr(self, 'depth_scale', 0.001),  # RealSense format (m per unit)
                    'scale': 1.0 / getattr(self, 'depth_scale', 0.001)   # ThinkGrasp format (units per m)
                }
                metadata['camera_intrinsics'] = intrinsics_data
            
            # Add robot state if provided
            if robot_state:
                metadata['robot_state'] = robot_state
            
            # Save metadata
            metadata_path = capture_folder / "metadata.json"
            with open(metadata_path, 'w') as f:
                json.dump(metadata, f, indent=2)
            
            return True, f"Captured to {capture_folder.name}"
            
        except Exception as e:
            return False, f"Capture failed: {str(e)}"
    
    def get_camera_info(self):
        """Get camera information"""
        if not self.pipeline:
            return None
            
        try:
            # Get device info
            pipeline_profile = self.pipeline.get_active_profile()
            device = pipeline_profile.get_device()
            
            info = {
                'name': device.get_info(rs.camera_info.name),
                'serial': device.get_info(rs.camera_info.serial_number),
                'firmware': device.get_info(rs.camera_info.firmware_version),
                'is_streaming': self.is_streaming
            }
            
            if self.intrinsics:
                info['intrinsics'] = {
                    'width': self.intrinsics.width,
                    'height': self.intrinsics.height,
                    'fx': self.intrinsics.fx,
                    'fy': self.intrinsics.fy,
                    'ppx': self.intrinsics.ppx,
                    'ppy': self.intrinsics.ppy
                }
            
            return info
            
        except Exception as e:
            return None


class SimpleWebcamHandler:
    """Fallback handler for regular webcam if RealSense is not available"""
    
    def __init__(self):
        if cv2 is None:
            raise ImportError("OpenCV is not installed. Please install it with: pip install opencv-python")
            
        self.cap = None
        self.is_streaming = False
        self.capture_dir = Path.home() / "Documents/Github/opensource_dev/GraspingDemo/captures"
        self.capture_dir.mkdir(exist_ok=True)
        
    def connect(self, camera_index=0):
        """Connect to webcam"""
        try:
            self.cap = cv2.VideoCapture(camera_index)
            if self.cap.isOpened():
                self.is_streaming = True
                return True, "Webcam connected"
            else:
                return False, "Failed to open webcam"
        except Exception as e:
            return False, f"Failed to connect: {str(e)}"
    
    def disconnect(self):
        """Disconnect webcam"""
        try:
            if self.cap:
                self.cap.release()
                self.cap = None
            self.is_streaming = False
            return True, "Webcam disconnected"
        except Exception as e:
            return False, f"Failed to disconnect: {str(e)}"
    
    def get_rgb_frame(self):
        """Get current frame as base64 encoded JPEG"""
        if not self.cap or not self.is_streaming:
            return None
            
        try:
            ret, frame = self.cap.read()
            if ret:
                # Convert to JPEG
                _, buffer = cv2.imencode('.jpg', frame)
                jpg_as_text = base64.b64encode(buffer).decode('utf-8')
                return jpg_as_text
            return None
        except Exception as e:
            print(f"Error getting frame: {e}")
            return None
    
    def capture(self, robot_state=None):
        """Capture current frame with metadata"""
        if not self.cap or not self.is_streaming:
            return False, "Camera not streaming"
            
        try:
            ret, frame = self.cap.read()
            if ret:
                timestamp = datetime.now()
                timestamp_str = timestamp.strftime("%Y%m%d_%H%M%S")
                capture_folder = self.capture_dir / timestamp_str
                capture_folder.mkdir(exist_ok=True)
                
                # Save RGB image
                rgb_path = capture_folder / "rgb.png"
                cv2.imwrite(str(rgb_path), frame)
                
                # Create metadata
                metadata = {
                    'timestamp': timestamp.isoformat(),
                    'timestamp_unix': time.time(),
                    'camera_type': 'webcam',
                    'files': {
                        'rgb': "rgb.png"
                    },
                    'rgb_shape': frame.shape
                }
                
                # Add robot state if provided
                if robot_state:
                    metadata['robot_state'] = robot_state
                
                # Save metadata
                metadata_path = capture_folder / "metadata.json"
                with open(metadata_path, 'w') as f:
                    json.dump(metadata, f, indent=2)
                
                return True, f"Captured to {capture_folder.name}"
            return False, "Failed to capture frame"
        except Exception as e:
            return False, f"Capture failed: {str(e)}"