#!/usr/bin/env python3
"""Test script to find RealSense camera index for OpenCV"""

import cv2
import sys

def test_camera(index):
    """Test if camera at given index can be opened and read"""
    print(f"\nTesting camera index {index}...")
    cap = cv2.VideoCapture(index)

    if not cap.isOpened():
        print(f"  ❌ Cannot open camera {index}")
        return False

    # Try to read a frame
    ret, frame = cap.read()

    if ret and frame is not None:
        height, width = frame.shape[:2]
        print(f"  ✓ Camera {index} opened successfully!")
        print(f"    Resolution: {width}x{height}")
        print(f"    Backend: {cap.getBackendName()}")

        # Get camera properties
        fps = cap.get(cv2.CAP_PROP_FPS)
        fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
        fourcc_str = "".join([chr((fourcc >> 8 * i) & 0xFF) for i in range(4)])
        print(f"    FPS: {fps}")
        print(f"    Format: {fourcc_str}")

        cap.release()
        return True
    else:
        print(f"  ❌ Camera {index} opened but cannot read frames")
        cap.release()
        return False

def main():
    print("Searching for RealSense camera...")
    print("=" * 50)

    # Test known RealSense indices from v4l2-ctl
    realsense_indices = [4, 6]

    working_indices = []
    for idx in realsense_indices:
        if test_camera(idx):
            working_indices.append(idx)

    print("\n" + "=" * 50)
    if working_indices:
        print(f"\n✓ RealSense camera found at index(es): {working_indices}")
        print(f"\nTo use with OpenCV:")
        print(f"  cap = cv2.VideoCapture({working_indices[0]})")
    else:
        print("\n❌ No working RealSense camera indices found")
        return 1

    return 0

if __name__ == "__main__":
    sys.exit(main())
