#!/usr/bin/env python3
"""
Test RealSense Camera Connection
"""

import numpy as np

def test_realsense():
    """Test RealSense camera connection and functionality"""
    try:
        import pyrealsense2 as rs
    except ImportError:
        print("❌ pyrealsense2 not installed!")
        print("Install with: pip install pyrealsense2")
        return False
    
    print("RealSense Camera Test")
    print("=" * 50)
    
    # Create a context object
    ctx = rs.context()
    
    # Get connected devices
    devices = ctx.query_devices()
    
    if len(devices) == 0:
        print("❌ No RealSense devices found!")
        print("\nTroubleshooting:")
        print("1. Check USB connection (use USB 3.0 port)")
        print("2. Check if camera LED is on")
        print("3. Try a different USB cable")
        print("4. On macOS, check System Preferences > Security & Privacy > Camera")
        return False
    
    print(f"✅ Found {len(devices)} RealSense device(s):")
    
    for i, device in enumerate(devices):
        print(f"\nDevice {i}:")
        print(f"  Name: {device.get_info(rs.camera_info.name)}")
        print(f"  Serial: {device.get_info(rs.camera_info.serial_number)}")
        print(f"  Firmware: {device.get_info(rs.camera_info.firmware_version)}")
        print(f"  USB: {device.get_info(rs.camera_info.usb_type_descriptor)}")
    
    # Test streaming
    print("\n" + "=" * 50)
    print("Testing camera streams...")
    
    pipeline = rs.pipeline()
    config = rs.config()
    
    # Get device serial
    device = devices[0]
    serial = device.get_info(rs.camera_info.serial_number)
    config.enable_device(serial)
    
    # Configure streams
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    
    try:
        # Start streaming
        profile = pipeline.start(config)
        
        print("✅ Successfully started camera streams")
        
        # Get camera intrinsics
        color_profile = profile.get_stream(rs.stream.color)
        intrinsics = color_profile.as_video_stream_profile().get_intrinsics()
        
        print(f"\nColor camera intrinsics:")
        print(f"  Resolution: {intrinsics.width}x{intrinsics.height}")
        print(f"  Principal point: ({intrinsics.ppx:.2f}, {intrinsics.ppy:.2f})")
        print(f"  Focal length: ({intrinsics.fx:.2f}, {intrinsics.fy:.2f})")
        
        # Capture a few frames
        print("\nCapturing test frames...")
        for i in range(5):
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            
            if color_frame and depth_frame:
                print(f"  Frame {i+1}: ✅ Color + Depth")
            else:
                print(f"  Frame {i+1}: ❌ Missing data")
        
        # Get depth scale
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        print(f"\nDepth scale: {depth_scale} meters/unit")
        
        # Stop streaming
        pipeline.stop()
        print("\n✅ Camera test completed successfully!")
        return True
        
    except Exception as e:
        print(f"❌ Error during streaming: {e}")
        return False


def test_opencv_camera():
    """Test generic camera with OpenCV"""
    try:
        import cv2
    except ImportError:
        print("\n❌ OpenCV not installed!")
        print("Install with: pip install opencv-python")
        return False
    
    print("\n" + "=" * 50)
    print("OpenCV Camera Test")
    print("=" * 50)
    
    # Try different camera indices
    for idx in range(3):
        print(f"\nTrying camera index {idx}...")
        cap = cv2.VideoCapture(idx)
        
        if cap.isOpened():
            print(f"✅ Camera {idx} opened successfully")
            
            # Get camera properties
            width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            fps = cap.get(cv2.CAP_PROP_FPS)
            
            print(f"  Resolution: {int(width)}x{int(height)}")
            print(f"  FPS: {fps}")
            
            # Try to read a frame
            ret, frame = cap.read()
            if ret:
                print("  ✅ Successfully captured frame")
            else:
                print("  ❌ Failed to capture frame")
            
            cap.release()
        else:
            print(f"❌ No camera at index {idx}")


def main():
    """Main test function"""
    print("Camera Connection Test")
    print("=" * 60)
    
    # Test RealSense
    realsense_ok = test_realsense()
    
    # Test OpenCV cameras
    test_opencv_camera()
    
    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    
    if realsense_ok:
        print("✅ RealSense camera: Connected and working")
        print("\nNext steps:")
        print("1. Run camera calibration: python scripts/calibrate_camera.py")
        print("2. Test point cloud capture: python scripts/capture_pointcloud.py")
    else:
        print("❌ RealSense camera: Not connected")
        print("\nTroubleshooting steps:")
        print("1. Install RealSense SDK:")
        print("   - macOS: brew install librealsense")
        print("   - Ubuntu: Follow Intel RealSense installation guide")
        print("2. Install Python wrapper: pip install pyrealsense2")
        print("3. Use USB 3.0 port (blue port)")
        print("4. Try running: realsense-viewer")


if __name__ == "__main__":
    main()