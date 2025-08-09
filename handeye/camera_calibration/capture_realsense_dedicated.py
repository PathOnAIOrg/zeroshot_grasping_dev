#!/usr/bin/env python3
"""
RealSense camera calibration image capture using pyrealsense2
"""

import os
import datetime
import cv2 as cv
import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:
    print("ERROR: pyrealsense2 not installed!")
    print("Install it with: pip install pyrealsense2")
    exit(1)

# Configuration
OUTPUT_FOLDER = "calibration_images"
IMAGE_PREFIX = "calibration"

# Create output folder if it doesn't exist
if not os.path.exists(OUTPUT_FOLDER):
    os.makedirs(OUTPUT_FOLDER)
    print(f"Created folder: {OUTPUT_FOLDER}")

# Configure RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

print(f"Found RealSense device: {device_product_line}")

# Configure streams
# Use 640x480 @ 30fps for better performance during calibration
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
try:
    pipeline.start(config)
    print("RealSense camera started successfully!")
except Exception as e:
    print(f"Failed to start RealSense camera: {e}")
    print("\nMake sure:")
    print("1. RealSense camera is connected")
    print("2. No other program is using the camera")
    print("3. You have proper USB 3.0 connection")
    exit(1)

# Checkerboard detection parameters
CHECKERBOARD_COLS = 7
CHECKERBOARD_ROWS = 4
show_checkerboard = True

print("\n=== REALSENSE CALIBRATION IMAGE CAPTURE ===")
print("Controls:")
print("  SPACE or ENTER - Capture and save image")
print("  'c' - Toggle checkerboard detection overlay")
print("  'q' or ESC - Quit")
print(f"\nImages will be saved to: {OUTPUT_FOLDER}/")
print("\nPress SPACE/ENTER to capture images of the checkerboard from different angles.")
print("Try to capture 10-20 images for good calibration.\n")

image_count = 0

try:
    while True:
        # Wait for a coherent pair of frames
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        
        if not color_frame:
            continue
        
        # Convert to numpy array
        color_image = np.asanyarray(color_frame.get_data())
        
        # Create display frame
        display_frame = color_image.copy()
        height, width = display_frame.shape[:2]
        
        # Try to detect checkerboard for preview
        if show_checkerboard:
            gray = cv.cvtColor(color_image, cv.COLOR_BGR2GRAY)
            ret_chess, corners = cv.findChessboardCorners(
                gray, (CHECKERBOARD_COLS, CHECKERBOARD_ROWS), None
            )
            
            if ret_chess:
                # Draw checkerboard corners for preview
                cv.drawChessboardCorners(
                    display_frame, (CHECKERBOARD_COLS, CHECKERBOARD_ROWS), 
                    corners, ret_chess
                )
                cv.putText(display_frame, "Checkerboard detected!", (10, 30), 
                          cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            else:
                cv.putText(display_frame, "No checkerboard detected", (10, 30), 
                          cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        # Display image count and camera info
        cv.putText(display_frame, f"Images captured: {image_count}", (10, height - 40), 
                  cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv.putText(display_frame, f"RealSense {device_product_line}", (10, height - 15), 
                  cv.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        # Show the frame
        cv.imshow('RealSense Calibration Capture', display_frame)
        
        # Handle keyboard input
        key = cv.waitKey(1) & 0xFF
        
        if key == ord(' ') or key == 13:  # SPACE or ENTER key
            # Generate filename with timestamp
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"{IMAGE_PREFIX}_{image_count:03d}_{timestamp}.png"
            filepath = os.path.join(OUTPUT_FOLDER, filename)
            
            # Save the original frame (not the one with drawings)
            cv.imwrite(filepath, color_image)
            image_count += 1
            print(f"Saved: {filepath} (Image #{image_count})")
            
            # Flash effect to indicate capture
            cv.imshow('RealSense Calibration Capture', cv.bitwise_not(display_frame))
            cv.waitKey(100)
            
        elif key == ord('c'):
            # Toggle checkerboard detection
            show_checkerboard = not show_checkerboard
            status = "ON" if show_checkerboard else "OFF"
            print(f"Checkerboard detection: {status}")
            
        elif key == ord('q') or key == 27:  # 'q' or ESC key
            print("\nExiting...")
            break

finally:
    # Stop streaming
    pipeline.stop()
    cv.destroyAllWindows()

print(f"\n=== CAPTURE COMPLETE ===")
print(f"Total images captured: {image_count}")
if image_count > 0:
    print(f"Images saved in: {os.path.abspath(OUTPUT_FOLDER)}/")
    print("\nYou can now run the calibration script with these images:")
    print("  python calibrate_from_capture.py")
else:
    print("No images were captured.")