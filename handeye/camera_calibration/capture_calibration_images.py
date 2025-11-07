import cv2 as cv
import os
import datetime

# Configuration
CAMERA_INDEX = 8  # Change this if you have multiple cameras (0 is usually the default webcam)
OUTPUT_FOLDER = "calibration_images"
IMAGE_PREFIX = "calibration"

# Create output folder if it doesn't exist
if not os.path.exists(OUTPUT_FOLDER):
    os.makedirs(OUTPUT_FOLDER)
    print(f"Created folder: {OUTPUT_FOLDER}")

# Initialize camera
cap = cv.VideoCapture(CAMERA_INDEX)

if not cap.isOpened():
    print("Error: Could not open camera")
    print("Try changing CAMERA_INDEX to 1, 2, etc. if you have multiple cameras")
    exit(1)

# Set camera resolution (optional - comment out if you want to use default)
# cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
# cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)

# Get actual camera resolution
width = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))
print(f"Camera resolution: {width}x{height}")

# Checkerboard detection parameters (for preview)
CHECKERBOARD_COLS = 7
CHECKERBOARD_ROWS = 4
show_checkerboard = True

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
    
    if not ret:
        print("Error: Failed to capture frame")
        break
    
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
    print("\nYou can now run the calibration script with these images.")
    print("Update the calibrate.py script to use this folder:")
    print(f"  images_folder = '{os.path.abspath(OUTPUT_FOLDER)}/*'")
else:
    print("No images were captured.")