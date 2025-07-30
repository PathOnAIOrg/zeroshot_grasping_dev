"""
Script to collect calibration data for SO-101 robot system
"""

import numpy as np
import json
from pathlib import Path
import pyrealsense2 as rs


def get_robot_port():
    """Find available robot serial ports."""
    import serial.tools.list_ports
    
    print("Available serial ports:")
    ports = serial.tools.list_ports.comports()
    for i, port in enumerate(ports):
        print(f"{i}: {port.device} - {port.description}")
    
    if ports:
        idx = int(input("Select port number: "))
        return ports[idx].device
    return None


def get_camera_info():
    """Get RealSense camera configuration."""
    pipeline = rs.pipeline()
    config = rs.config()
    
    # Get device info
    pipeline_profile = config.resolve(rs.pipeline_wrapper(pipeline))
    device = pipeline_profile.get_device()
    
    # Get depth scale
    depth_sensor = device.first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    
    # Get supported streams
    print("\nCamera Information:")
    print(f"Device: {device.get_info(rs.camera_info.name)}")
    print(f"Serial: {device.get_info(rs.camera_info.serial_number)}")
    print(f"Depth Scale: {depth_scale}")
    
    return {
        "depth_scale": depth_scale,
        "device_name": device.get_info(rs.camera_info.name),
        "serial_number": device.get_info(rs.camera_info.serial_number)
    }


def calibrate_robot_joints():
    """Calibrate robot joint offsets."""
    print("\nRobot Joint Calibration")
    print("Move each joint to its zero position and press Enter")
    
    offsets = []
    joint_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", 
                   "wrist_flex", "wrist_roll", "gripper"]
    
    for joint in joint_names:
        input(f"Move {joint} to zero position and press Enter...")
        # In real implementation, read current encoder value
        offset = float(input(f"Enter offset for {joint} (degrees): "))
        offsets.append(offset)
    
    return offsets


def calibrate_camera_transform():
    """Calibrate camera to robot transform using checkerboard."""
    print("\nCamera-Robot Transform Calibration")
    print("Place a checkerboard at known robot positions")
    
    # Placeholder for actual calibration
    # In real implementation:
    # 1. Move robot to multiple known positions
    # 2. Detect checkerboard in camera
    # 3. Solve for transform matrix
    
    transform = np.eye(4)
    transform[:3, 3] = [0.1, 0.0, 0.2]  # Example translation
    
    return transform


def save_all_calibration_data():
    """Collect and save all calibration data."""
    from config_manager import ConfigManager
    
    config_mgr = ConfigManager()
    
    # 1. Get robot port
    port = get_robot_port()
    if port:
        config_mgr.update_robot_port(port)
    
    # 2. Get camera info
    try:
        camera_info = get_camera_info()
        camera_config = config_mgr.get_camera_config()
        camera_config.update(camera_info)
        config_mgr.save_config("camera", camera_config)
    except:
        print("Camera not connected")
    
    # 3. Calibrate robot joints
    if input("\nCalibrate robot joints? (y/n): ").lower() == 'y':
        offsets = calibrate_robot_joints()
        robot_config = config_mgr.get_robot_config()
        robot_config["calibration_offset"] = offsets
        config_mgr.save_config("robot", robot_config)
    
    # 4. Calibrate camera transform
    if input("\nCalibrate camera transform? (y/n): ").lower() == 'y':
        transform = calibrate_camera_transform()
        config_mgr.save_calibration_data("transform", transform)
    
    print("\n✅ Calibration complete!")
    config_mgr.list_configs()


if __name__ == "__main__":
    save_all_calibration_data()