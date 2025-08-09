import cv2 as cv
import glob
import numpy as np
import os
import sys

# Configuration - change this to match your setup
DEFAULT_IMAGE_FOLDER = 'calibration_images'  # Folder created by capture script
CALIBRATION_OUTPUT = 'calibration_data.npz'

# Allow passing folder as command line argument
if len(sys.argv) > 1:
    image_folder = sys.argv[1]
else:
    image_folder = DEFAULT_IMAGE_FOLDER

# Check if folder exists
if not os.path.exists(image_folder):
    print(f"Error: Folder '{image_folder}' does not exist!")
    print(f"Please run capture_calibration_images.py first to capture images.")
    exit(1)

# Load images
images_pattern = os.path.join(image_folder, '*')
images_names = sorted(glob.glob(images_pattern))

# Filter for image files only
image_extensions = ['.png', '.jpg', '.jpeg', '.bmp']
images_names = [img for img in images_names if any(img.lower().endswith(ext) for ext in image_extensions)]

print(f"Found {len(images_names)} image files in '{image_folder}'")

if len(images_names) == 0:
    print(f"No images found in '{image_folder}'")
    print("Please capture some calibration images first.")
    exit(1)

# Load images
images = []
for imname in images_names:
    im = cv.imread(imname, 1)
    if im is not None:
        images.append(im)
        print(f"  Loaded: {os.path.basename(imname)} - {im.shape}")
    else:
        print(f"  Failed to load: {os.path.basename(imname)}")

if len(images) == 0:
    print("ERROR: No images loaded successfully!")
    exit(1)

print(f"\nSuccessfully loaded {len(images)} images")
    
# Criteria used by checkerboard pattern detector
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Common checkerboard patterns to try (columns, rows) format
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

# Try to detect the pattern size automatically
detected_pattern = None
print("\n=== DETECTING CHECKERBOARD PATTERN ===")
print("Trying to detect checkerboard pattern size...")

for cols, rows in patterns_to_try:
    gray = cv.cvtColor(images[0], cv.COLOR_BGR2GRAY)
    ret, _ = cv.findChessboardCorners(gray, (cols, rows), None)
    if ret:
        print(f"✓ Pattern detected: {cols}x{rows} internal corners")
        detected_pattern = (cols, rows)
        
        # Verify pattern works for all images
        valid_count = 0
        for img in images:
            gray_test = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            ret_test, _ = cv.findChessboardCorners(gray_test, (cols, rows), None)
            if ret_test:
                valid_count += 1
        
        print(f"  Pattern found in {valid_count}/{len(images)} images")
        
        if valid_count < len(images) * 0.5:  # If less than 50% of images work, try another pattern
            print(f"  Pattern not reliable enough, trying next...")
            detected_pattern = None
            continue
        else:
            break

if detected_pattern is None:
    print("\nERROR: Could not detect pattern size automatically!")
    print("Possible issues:")
    print("1. Checkerboard not visible in images")
    print("2. Non-standard checkerboard pattern")
    print("3. Poor image quality or lighting")
    print("\nYou can manually specify the pattern size by editing this script.")
    exit(1)

columns, rows = detected_pattern
world_scaling = 1.0  # Change this to the real world square size in your units (e.g., 25.0 for 25mm squares)
 
# Coordinates of squares in the checkerboard world space
objp = np.zeros((rows*columns,3), np.float32)
objp[:,:2] = np.mgrid[0:columns,0:rows].T.reshape(-1,2)
objp = world_scaling * objp
 
# Frame dimensions - frames should be the same size
width = images[0].shape[1]
height = images[0].shape[0]
 
# Arrays to store points from all images
imgpoints = []  # 2d points in image plane
objpoints = []  # 3d points in real world space

print(f"\n=== PROCESSING IMAGES ===")
print(f"Using pattern size: {columns}x{rows}")
print(f"Image dimensions: {width}x{height}")

successful_images = 0
for i, frame in enumerate(images):
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
 
    # Find the checkerboard (OpenCV expects (columns, rows) format)
    ret, corners = cv.findChessboardCorners(gray, (columns, rows), None)
    
    if ret == True:
        successful_images += 1
        print(f"  Image {i+1}/{len(images)}: ✓ Checkerboard detected")
        
        # Improve corner detection accuracy
        conv_size = (11, 11)
        corners = cv.cornerSubPix(gray, corners, conv_size, (-1, -1), criteria)
        
        # Add to calibration data
        objpoints.append(objp)
        imgpoints.append(corners)
        
        # Optional: Show detected corners (comment out for faster processing)
        # cv.drawChessboardCorners(frame, (columns, rows), corners, ret)
        # cv.imshow('Detected Pattern', frame)
        # cv.waitKey(100)
    else:
        print(f"  Image {i+1}/{len(images)}: ✗ No checkerboard detected")

# cv.destroyAllWindows()

print(f"\n=== CALIBRATION RESULTS ===")
print(f"Successfully processed: {successful_images}/{len(images)} images")

if len(objpoints) < 3:
    print("ERROR: Need at least 3 successful detections for calibration!")
    print("Please capture more images with the checkerboard clearly visible.")
    exit(1)

# Perform camera calibration
print("\nRunning camera calibration...")
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, (width, height), None, None)

print(f"\n✓ Calibration successful!")
print(f"  RMS re-projection error: {ret:.4f} pixels")
print(f"  (Lower is better, < 1.0 is good, < 0.5 is excellent)")

print(f"\nCamera matrix:")
print(f"  fx = {mtx[0,0]:.2f}  (focal length in pixels)")
print(f"  fy = {mtx[1,1]:.2f}  (focal length in pixels)")
print(f"  cx = {mtx[0,2]:.2f}  (principal point x)")
print(f"  cy = {mtx[1,2]:.2f}  (principal point y)")

print(f"\nDistortion coefficients:")
print(f"  k1 = {dist[0,0]:.6f}")
print(f"  k2 = {dist[0,1]:.6f}")
print(f"  p1 = {dist[0,2]:.6f}")
print(f"  p2 = {dist[0,3]:.6f}")
print(f"  k3 = {dist[0,4]:.6f}")

# Save calibration results
np.savez(CALIBRATION_OUTPUT, 
         mtx=mtx, 
         dist=dist, 
         rvecs=rvecs, 
         tvecs=tvecs,
         image_size=(width, height),
         rms_error=ret,
         pattern_size=(columns, rows))

print(f"\n✓ Calibration data saved to '{CALIBRATION_OUTPUT}'")

# Calculate and display additional metrics
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2) / len(imgpoints2)
    mean_error += error
mean_error /= len(objpoints)

print(f"\nMean re-projection error: {mean_error:.4f} pixels")

# Optional: Test undistortion on one image
print(f"\n=== TESTING UNDISTORTION ===")
test_img = images[0]
undistorted = cv.undistort(test_img, mtx, dist)

# Save comparison image
comparison = np.hstack([test_img, undistorted])
cv.imwrite('calibration_comparison.png', comparison)
print(f"✓ Saved comparison image to 'calibration_comparison.png'")
print("  (Left: Original, Right: Undistorted)")

print("\n=== CALIBRATION COMPLETE ===")
print(f"You can now use the calibration data from '{CALIBRATION_OUTPUT}'")
print("To load it in your code:")
print("  data = np.load('calibration_data.npz')")
print("  mtx = data['mtx']")
print("  dist = data['dist']")