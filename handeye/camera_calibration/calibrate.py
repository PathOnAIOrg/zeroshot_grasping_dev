import cv2 as cv
import glob
import numpy as np

images_folder = '/home/pathonai/Documents/Github/opensource_dev/handeye/D2/*'
images_names = sorted(glob.glob(images_folder))
print(f"Found {len(images_names)} image files: {images_names}")
images = []
for imname in images_names:
    im = cv.imread(imname, 1)
    if im is not None:
        images.append(im)
        print(f"Loaded {imname}: {im.shape}")
    else:
        print(f"Failed to load {imname}")

if len(images) == 0:
    print("ERROR: No images loaded successfully!")
    exit(1)
    
#criteria used by checkerboard pattern detector.
#Change this if the code can't find the checkerboard
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Common checkerboard patterns to try (columns, rows) format
patterns_to_try = [
    (7, 4), (4, 7),  # Detected pattern
    (7, 5), (5, 7),  # Original guess
    (6, 4), (4, 6),  # Smaller pattern
    (8, 6), (6, 8),  # Medium pattern
    (9, 6), (6, 9),  # Common calibration pattern
    (7, 6), (6, 7),  # Another common size
    (5, 4), (4, 5),  # Small pattern
    (8, 5), (5, 8),  # Another possibility
]

# Try to detect the pattern size automatically
detected_pattern = None
print("\nTrying to detect checkerboard pattern size...")
for cols, rows in patterns_to_try:
    gray = cv.cvtColor(images[0], cv.COLOR_BGR2GRAY)
    ret, _ = cv.findChessboardCorners(gray, (cols, rows), None)
    if ret:
        print(f"✓ Pattern detected: {cols}x{rows} internal corners")
        detected_pattern = (cols, rows)
        break
    else:
        print(f"  {cols}x{rows}: Not detected")

if detected_pattern is None:
    print("\nERROR: Could not detect pattern size automatically!")
    print("Please check your checkerboard pattern and update the script manually.")
    exit(1)

columns, rows = detected_pattern
world_scaling = 1. #change this to the real world square size. Or not.
 
#coordinates of squares in the checkerboard world space
objp = np.zeros((rows*columns,3), np.float32)
objp[:,:2] = np.mgrid[0:columns,0:rows].T.reshape(-1,2)
objp = world_scaling* objp
 
 
#frame dimensions. Frames should be the same size.
width = images[0].shape[1]
height = images[0].shape[0]
 
#Pixel coordinates of checkerboards
imgpoints = [] # 2d points in image plane.
 
#coordinates of the checkerboard in checkerboard world space.
objpoints = [] # 3d point in real world space
 
print(f"\nProcessing images with pattern size {columns}x{rows}...")
for i, frame in enumerate(images):
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
 
    #find the checkerboard (OpenCV expects (columns, rows) format)
    ret, corners = cv.findChessboardCorners(gray, (columns, rows), None)
    print(f"Image {i}: Checkerboard detection = {ret}")
    
    if ret == True:
 
        #Convolution size used to improve corner detection. Don't make this too large.
        conv_size = (11, 11)
 
        #opencv can attempt to improve the checkerboard coordinates
        corners = cv.cornerSubPix(gray, corners, conv_size, (-1, -1), criteria)
        cv.drawChessboardCorners(frame, (columns, rows), corners, ret)
        cv.imshow('img', frame)
        k = cv.waitKey(500)
 
        objpoints.append(objp)
        imgpoints.append(corners)
    

print(f"\nCalibration data:")
print(f"Number of successful detections: {len(objpoints)}")
print(f"Image dimensions: {width}x{height}")

if len(objpoints) > 0:
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, (width, height), None, None)
    print("\nCalibration successful!")
    print(f"RMS re-projection error: {ret}")
    print(f"\nCamera matrix:\n{mtx}")
    print(f"\nDistortion coefficients:\n{dist}")
    
    # Save calibration results
    np.savez('calibration_data.npz', mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)
    print("\nCalibration data saved to 'calibration_data.npz'")
else:
    print("ERROR: No checkerboard patterns detected in any image!")
    print("\nPossible issues:")
    print("1. The checkerboard might have a non-standard size")
    print("2. Image quality or lighting issues")
    print("3. The checkerboard might be partially obscured")