#!/usr/bin/env python3
"""
SO-101 Connection Test Script

Tests connection to both the SO-101 robot and RealSense camera.
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from so101_grasp.robot.calibration import RobotCalibrator
from so101_grasp.vision.camera import CameraController


def test_robot_connection():
    """Test SO-101 robot connection."""
    print("🤖 Testing SO-101 Robot Connection")
    print("=" * 50)
    
    # Test common ports
    possible_ports = [
        "/dev/tty.usbmodem5A680107891",
        "/dev/tty.usbmodemSN234567892",
        "/dev/ttyACM0",
        "/dev/ttyACM1",
        "/dev/ttyUSB0",
        "/dev/ttyUSB1"
    ]
    
    for port in possible_ports:
        print(f"\nTesting port: {port}")
        calibrator = RobotCalibrator(port)
        
        if calibrator.test_connection():
            print(f"✅ Robot found on port: {port}")
            calibrator.disconnect()
            return port
        else:
            print(f"❌ No robot on port: {port}")
    
    print("\n❌ Robot not found on any tested port")
    print("\nTroubleshooting:")
    print("1. Ensure robot is powered on")
    print("2. Check USB connection")
    print("3. Verify correct port in code")
    print("4. Try: python -m lerobot.find_port")
    
    return None


def test_camera_connection():
    """Test RealSense camera connection."""
    print("\n📷 Testing RealSense Camera Connection")
    print("=" * 50)
    
    camera = CameraController()
    
    if camera.connect():
        print("✅ Camera connected successfully")
        
        # Test frame capture
        print("\nTesting frame capture...")
        color, depth, intrinsics = camera.capture_rgbd()
        
        if color is not None and depth is not None:
            print(f"✅ Frame capture successful")
            print(f"   Color image: {color.shape}")
            print(f"   Depth image: {depth.shape}")
        else:
            print("❌ Frame capture failed")
        
        camera.disconnect()
        return True
    else:
        print("❌ Camera connection failed")
        print("\nTroubleshooting:")
        print("1. Ensure RealSense camera is connected")
        print("2. Check USB 3.0 connection")
        print("3. Install RealSense SDK")
        print("4. Try: realsense-viewer")
        return False


def main():
    """Main test function."""
    print("SO-101 Grasping System - Connection Test")
    print("=" * 60)
    
    robot_port = test_robot_connection()
    camera_ok = test_camera_connection()
    
    print("\n" + "=" * 60)
    print("CONNECTION TEST SUMMARY")
    print("=" * 60)
    
    if robot_port:
        print(f"✅ Robot: Connected on {robot_port}")
    else:
        print("❌ Robot: Not connected")
    
    if camera_ok:
        print("✅ Camera: Connected")
    else:
        print("❌ Camera: Not connected")
    
    if robot_port and camera_ok:
        print("\n🎉 All systems ready!")
        print(f"Update your configuration files with:")
        print(f"   Robot port: {robot_port}")
        print("\nNext steps:")
        print("1. Run: python scripts/calibrate_robot.py")
        print("2. Run: python scripts/calibrate_camera.py")
        print("3. Run: python scripts/run_demo.py")
    else:
        print("\n⚠️  Some connections failed. Fix issues before proceeding.")


if __name__ == "__main__":
    main()