import cv2 as cv
import os
import datetime
import sys

# Configuration
CAMERA_INDEX = 8  # Change this to your camera index (0, 2, 4, 6, 8, etc.)
OUTPUT_FOLDER = "calibration_images"
IMAGE_PREFIX = "calibration"

# Create output folder if it doesn't exist
if not os.path.exists(OUTPUT_FOLDER):
    os.makedirs(OUTPUT_FOLDER)
    print(f"Created folder: {OUTPUT_FOLDER}")

print(f"Opening camera at index {CAMERA_INDEX}...")

# Open the specified camera
cap = cv.VideoCapture(CAMERA_INDEX)

if not cap.isOpened():
    print(f"\n=== CAMERA NOT FOUND ===")
    print(f"Could not open camera at index {CAMERA_INDEX}")
    print("\nTroubleshooting:")
    print("1. Check available cameras with: ls /dev/video*")
    print("2. Try different CAMERA_INDEX values (0, 2, 4, 6, 8, etc.)")
    print("3. For RealSense cameras, use: capture_realsense_dedicated.py")
    exit(1)

# Test if we can read a frame
ret, frame = cap.read()
if not ret or frame is None:
    print(f"\n=== CAMERA ERROR ===")
    print(f"Camera {CAMERA_INDEX} opened but cannot read frames")
    print("Try a different camera index")
    cap.release()
    exit(1)

print(f"✓ Successfully opened camera at index {CAMERA_INDEX}")

# Get actual camera resolution
width = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))
print(f"\nCamera resolution: {width}x{height}")
print(f"Using camera index: {CAMERA_INDEX}")

# Checkerboard detection parameters (for preview)
# Try multiple patterns to find the right one
PATTERNS_TO_TRY = [
    (9, 6), (6, 9),  # OpenCV standard - try this first
    (8, 6), (6, 8),  # Common pattern
    (7, 6), (6, 7),  # Another common pattern
    (7, 5), (5, 7),  # Medium pattern
    (7, 4), (4, 7),  # Your original pattern
]

# Will be set after detection (default to most common)
CHECKERBOARD_COLS = 9
CHECKERBOARD_ROWS = 6
pattern_detected = False
show_checkerboard = True

# Try to detect pattern from a test frame
print("\nDetecting checkerboard pattern size...")
test_ret, test_frame = cap.read()
if test_ret and test_frame is not None:
    test_gray = cv.cvtColor(test_frame, cv.COLOR_BGR2GRAY)
    for cols, rows in PATTERNS_TO_TRY:
        ret_test, _ = cv.findChessboardCorners(test_gray, (cols, rows), None)
        if ret_test:
            CHECKERBOARD_COLS = cols
            CHECKERBOARD_ROWS = rows
            pattern_detected = True
            print(f"✓ Detected pattern: {cols}×{rows} internal corners")
            break
    if not pattern_detected:
        print("Could not auto-detect pattern. Using default 7×4")
        print("You can still capture images and detection will work during calibration.")

print("\n=== CAMERA CALIBRATION IMAGE CAPTURE ===")
print("Controls:")
print("  SPACE or ENTER - Capture and save image")
print("  'c' - Toggle checkerboard detection overlay")
print("  'q' or ESC - Quit")
print(f"\nImages will be saved to: {OUTPUT_FOLDER}/")
print("\nPress SPACE/ENTER to capture images of the checkerboard from different angles and distances.")
print("Try to capture 10-20 images for good calibration.\n")

image_count = 0

while True:
    ret, frame = cap.read()
    
    if not ret or frame is None:
        print("Warning: Failed to capture frame, retrying...")
        continue
    
    display_frame = frame.copy()
    
    # Try to detect checkerboard for preview (optional)
    if show_checkerboard:
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        ret_chess, corners = cv.findChessboardCorners(gray, (CHECKERBOARD_COLS, CHECKERBOARD_ROWS), None)
        
        if ret_chess:
            # Draw checkerboard corners for preview
            cv.drawChessboardCorners(display_frame, (CHECKERBOARD_COLS, CHECKERBOARD_ROWS), corners, ret_chess)
            cv.putText(display_frame, "Checkerboard detected!", (10, 30), 
                      cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            cv.putText(display_frame, "No checkerboard detected", (10, 30), 
                      cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    
    # Display image count
    cv.putText(display_frame, f"Images captured: {image_count}", (10, height - 20), 
              cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    # Show the frame
    cv.imshow('Camera Calibration Capture', display_frame)
    
    # Handle keyboard input
    key = cv.waitKey(1) & 0xFF
    
    if key == ord(' ') or key == 13:  # SPACE or ENTER key
        # Generate filename with timestamp
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{IMAGE_PREFIX}_{image_count:03d}_{timestamp}.png"
        filepath = os.path.join(OUTPUT_FOLDER, filename)
        
        # Save the original frame (not the one with drawings)
        cv.imwrite(filepath, frame)
        image_count += 1
        print(f"Saved: {filepath} (Image #{image_count})")
        
        # Flash effect to indicate capture
        cv.imshow('Camera Calibration Capture', cv.bitwise_not(display_frame))
        cv.waitKey(100)
        
    elif key == ord('c'):
        # Toggle checkerboard detection
        show_checkerboard = not show_checkerboard
        status = "ON" if show_checkerboard else "OFF"
        print(f"Checkerboard detection: {status}")
        
    elif key == ord('q') or key == 27:  # 'q' or ESC key
        print("\nExiting...")
        break

# Release resources
cap.release()
cv.destroyAllWindows()

print(f"\n=== CAPTURE COMPLETE ===")
print(f"Total images captured: {image_count}")
if image_count > 0:
    print(f"Images saved in: {os.path.abspath(OUTPUT_FOLDER)}/")
    print("\nYou can now run the calibration script with these images:")
    print("  python calibrate_from_capture.py")
else:
    print("No images were captured.")