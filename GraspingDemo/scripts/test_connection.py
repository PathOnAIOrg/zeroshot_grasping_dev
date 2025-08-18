#!/usr/bin/env python3
"""
Simple Connection Test Script

Tests connection to the SO-101 robot and RealSense camera.
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def test_robot_connection():
    """Test SO-101 robot connection."""
    print("🤖 Testing SO-101 Robot Connection")
    print("=" * 50)
    
    try:
        from so101_grasp.robot import SO101ClientRawSimple
        
        # Test common ports
        possible_ports = [
            "/dev/tty.usbmodem5A680107891",
            "/dev/ttyACM0",
            "/dev/ttyACM1",
            "/dev/ttyUSB0",
            "/dev/ttyUSB1"
        ]
        
        for port in possible_ports:
            print(f"\nTesting port: {port}")
            try:
                client = SO101ClientRawSimple(port=port)
                positions = client.read_joints()
                print(f"✅ Robot found on port: {port}")
                print(f"   Current positions: {[f'{p:.2f}' for p in positions]}")
                client.disconnect(disable_torque=False, close_port=True)
                return port
            except Exception as e:
                print(f"❌ No robot on port: {port} ({str(e)[:50]})")
        
        print("\n⚠️ No robot found on any port")
        return None
        
    except ImportError:
        print("❌ Robot control modules not installed")
        return None


def test_camera_connection():
    """Test RealSense camera connection."""
    print("\n📷 Testing RealSense Camera Connection")
    print("=" * 50)
    
    try:
        import pyrealsense2 as rs
        
        # Create a context object
        ctx = rs.context()
        
        # Get list of connected devices
        devices = ctx.query_devices()
        
        if len(devices) == 0:
            print("❌ No RealSense camera detected")
            return False
            
        for i, device in enumerate(devices):
            serial = device.get_info(rs.camera_info.serial_number)
            name = device.get_info(rs.camera_info.name)
            print(f"✅ Camera {i+1} found: {name} (Serial: {serial})")
            
        return True
        
    except ImportError:
        print("❌ pyrealsense2 not installed")
        print("   Install with: pip install pyrealsense2")
        return False
    except Exception as e:
        print(f"❌ Error accessing camera: {e}")
        return False


def main():
    """Run all connection tests."""
    print("\n" + "="*60)
    print("       SO-101 System Connection Test")
    print("="*60)
    
    # Test robot
    robot_port = test_robot_connection()
    
    # Test camera
    camera_ok = test_camera_connection()
    
    # Summary
    print("\n" + "="*60)
    print("📊 Test Summary:")
    print("-" * 40)
    
    if robot_port:
        print(f"✅ Robot: Connected on {robot_port}")
    else:
        print("❌ Robot: Not connected")
        
    if camera_ok:
        print("✅ Camera: Connected")
    else:
        print("❌ Camera: Not connected")
        
    print("="*60)
    
    # Return success if at least one device is connected
    return robot_port is not None or camera_ok


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)