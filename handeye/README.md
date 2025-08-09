# Camera Calibration

Simple camera calibration using a checkerboard pattern.

## Quick Start

### 1. Capture calibration images

**For RealSense cameras:**
```bash
# Try the auto-detect script
python capture_realsense.py

# If that doesn't work, install pyrealsense2:
pip install pyrealsense2
python capture_realsense_dedicated.py
```

**For regular webcam:**
```bash
python capture_calibration_images.py
```

**Controls:**
- Press **SPACE** to capture image
- Press **C** to toggle checkerboard detection
- Press **Q** to quit
- Capture 10-20 images from different angles

### 2. Run calibration
```bash
python calibrate_from_capture.py
```

## Files

| File | Description |
|------|-------------|
| `capture_realsense.py` | Auto-detect camera (RealSense/webcam) |
| `capture_realsense_dedicated.py` | RealSense using pyrealsense2 SDK |
| `capture_calibration_images.py` | Standard webcam capture |
| `calibrate_from_capture.py` | Run calibration on captured images |
| `calibrate.py` | Calibrate from existing images |
| `test_pattern_detection.py` | Test checkerboard detection |

## Output

- `calibration_images/` - Captured images folder
- `calibration_data.npz` - Calibration parameters (camera matrix, distortion coefficients)
- `calibration_comparison.png` - Before/after undistortion comparison

## Checkerboard Pattern

Common patterns (internal corners):
- **9×6** - Standard OpenCV calibration pattern
- **7×4** - Smaller pattern
- **8×6** - Medium pattern

The scripts auto-detect your pattern size.

## Troubleshooting

**Camera not found:**
- RealSense may appear at higher camera indices
- Try `capture_realsense.py` first (auto-detects)
- Install `pyrealsense2` for better RealSense support

**Checkerboard not detected:**
- Ensure good lighting
- Hold pattern flat and steady
- Check pattern size matches your printout
- Run `test_pattern_detection.py` on captured images

## Loading Calibration Data

```python
import numpy as np

# Load calibration
data = np.load('calibration_data.npz')
camera_matrix = data['mtx']
distortion_coeffs = data['dist']
```