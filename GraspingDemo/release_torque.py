#!/usr/bin/env python3
"""
Quick script to release all servo torque
Use this when the robot arm is locked and you need to move it manually
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from so101_grasp.robot.so101_client_raw_simple import SO101ClientRawSimple
import time

def main():
    print("SO-101 Torque Release Tool")
    print("=" * 40)
    
    try:
        # Connect to robot
        client = SO101ClientRawSimple(port="/dev/ttyACM0")
        
        # Read current position first
        print("Current position:", [f"{p:.3f}" for p in client.read_joints()])
        
        # Disable all torque
        print("\nReleasing all servos...")
        for i in range(6):
            client.enable_torque(i + 1, False)
            time.sleep(0.01)
        
        print("✅ All servos released - you can move the arm manually")
        
        # Disconnect
        client.disconnect(disable_torque=False, close_port=True)
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()