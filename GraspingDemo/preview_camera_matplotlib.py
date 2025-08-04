#!/usr/bin/env python3
"""
Camera preview script using matplotlib
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import matplotlib
matplotlib.use('TkAgg')  # Use TkAgg backend for Linux
import matplotlib.pyplot as plt
import numpy as np

from so101_grasp.vision.camera import CameraController

def main():
    """Preview camera feed using matplotlib."""
    print("Camera Preview (Matplotlib)")
    print("=" * 40)
    
    # Create camera controller
    camera = CameraController(width=640, height=480, fps=30)
    
    try:
        # Connect to camera
        if not camera.connect():
            print("❌ Failed to connect to camera")
            return
        
        print("📹 Capturing single frame...")
        
        # Capture single frame
        color_image, depth_image, intrinsics = camera.capture_rgbd()
        
        if color_image is None or depth_image is None:
            print("❌ Failed to capture frame")
            return
        
        print(f"✅ Captured frame: {color_image.shape} color, {depth_image.shape} depth")
        
        # Create depth colormap for visualization
        depth_normalized = (depth_image.astype(np.float32) / depth_image.max() * 255).astype(np.uint8)
        
        # Display images
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        
        # Show color image (convert BGR to RGB for matplotlib)
        ax1.imshow(color_image[:,:,::-1])  # BGR to RGB
        ax1.set_title('Color Image')
        ax1.axis('off')
        
        # Show depth image
        ax2.imshow(depth_normalized, cmap='jet')
        ax2.set_title('Depth Image')
        ax2.axis('off')
        
        plt.tight_layout()
        plt.suptitle('RealSense D415 Camera View')
        print("📷 Close the window to continue...")
        plt.show()
        
    except KeyboardInterrupt:
        print("\n⏹️  Preview interrupted by user")
    except Exception as e:
        print(f"❌ Error during preview: {e}")
        import traceback
        traceback.print_exc()
    finally:
        camera.disconnect()
        print("👋 Camera preview ended")

if __name__ == "__main__":
    main()