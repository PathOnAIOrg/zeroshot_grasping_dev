"""
Camera Controller Module

Handles Intel RealSense camera operations and point cloud capture.
"""

import numpy as np
import pyrealsense2 as rs
from typing import Tuple, Optional
import cv2


class CameraController:
    """Controls Intel RealSense camera for RGB-D capture."""
    
    def __init__(self, width: int = 1280, height: int = 720, fps: int = 30):
        """
        Initialize camera controller.
        
        Args:
            width: Image width
            height: Image height  
            fps: Frames per second
        """
        self.width = width
        self.height = height
        self.fps = fps
        self.pipeline = None
        self.config = None
        self.intrinsics = None
        
    def connect(self) -> bool:
        """
        Connect to RealSense camera.
        
        Returns:
            True if connection successful
        """
        try:
            # Create pipeline and config
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            
            # Configure streams
            self.config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
            self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
            
            # Start streaming
            profile = self.pipeline.start(self.config)
            
            # Get camera intrinsics
            color_profile = profile.get_stream(rs.stream.color)
            self.intrinsics = color_profile.as_video_stream_profile().get_intrinsics()
            
            print(f"✅ Camera connected: {self.width}x{self.height} @ {self.fps}fps")
            print(f"   Intrinsics: fx={self.intrinsics.fx:.1f}, fy={self.intrinsics.fy:.1f}")
            print(f"   Principal point: ({self.intrinsics.ppx:.1f}, {self.intrinsics.ppy:.1f})")
            
            return True
            
        except Exception as e:
            print(f"❌ Camera connection failed: {e}")
            return False
    
    def capture_rgbd(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[object]]:
        """
        Capture RGB-D frame.
        
        Returns:
            Tuple of (color_image, depth_image, intrinsics)
        """
        if not self.pipeline:
            print("❌ Camera not connected")
            return None, None, None
            
        try:
            # Wait for frames
            frames = self.pipeline.wait_for_frames()
            
            # Align depth to color
            align = rs.align(rs.stream.color)
            aligned_frames = align.process(frames)
            
            # Get aligned frames
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                return None, None, None
            
            # Convert to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            
            return color_image, depth_image, self.intrinsics
            
        except Exception as e:
            print(f"❌ Frame capture failed: {e}")
            return None, None, None
    
    def capture_pointcloud(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[object]]:
        """
        Capture RGB-D data and return as point cloud format.
        
        Returns:
            Tuple of (color_image, depth_image, intrinsics) for point cloud generation
        """
        return self.capture_rgbd()
    
    def preview(self, duration: int = 10):
        """
        Show live camera preview.
        
        Args:
            duration: Preview duration in seconds
        """
        if not self.pipeline:
            print("❌ Camera not connected")
            return
            
        print(f"📹 Camera preview for {duration} seconds. Press 'q' to quit early.")
        
        import time
        start_time = time.time()
        
        try:
            while time.time() - start_time < duration:
                color_image, depth_image, _ = self.capture_rgbd()
                
                if color_image is not None:
                    # Create depth colormap for visualization
                    depth_colormap = cv2.applyColorMap(
                        cv2.convertScaleAbs(depth_image, alpha=0.08), 
                        cv2.COLORMAP_JET
                    )
                    
                    # Stack images horizontally
                    images = np.hstack((color_image, depth_colormap))
                    
                    # Resize for display
                    display_image = cv2.resize(images, (1280, 360))
                    
                    cv2.imshow('RealSense Preview (Color | Depth)', display_image)
                    
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                        
        except KeyboardInterrupt:
            pass
        finally:
            cv2.destroyAllWindows()
    
    def disconnect(self):
        """Disconnect from camera."""
        if self.pipeline:
            self.pipeline.stop()
            self.pipeline = None
            print("📷 Camera disconnected")
    
    def __del__(self):
        """Cleanup on object destruction."""
        self.disconnect()