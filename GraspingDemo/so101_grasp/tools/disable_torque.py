#!/usr/bin/env python3
"""
SO-101 Torque Control Tool

Utility for enabling/disabling motor torque for manual robot positioning.
"""

import sys
import os
import time
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from so101_grasp.robot.so101_client import SO101Client


def main():
    """Main torque control function."""
    print("SO-101 Torque Control Tool")
    print("=" * 40)
    
    port = "/dev/tty.usbmodem5A680107891"  # Update as needed
    
    try:
        print(f"Connecting to robot on port: {port}")
        client = SO101Client(port=port, follower=True, force_calibration=False)
        
        while True:
            print("\nOptions:")
            print("1. Disable torque (allow manual movement)")
            print("2. Enable torque (lock positions)")
            print("3. Read current positions")
            print("4. Exit")
            
            choice = input("\nEnter choice (1-4): ").strip()
            
            if choice == "1":
                print("\nDisabling torque on all motors...")
                try:
                    # For FeetechMotorsBus, use disable_torque method
                    client.robot.bus.disable_torque()
                    print("  ✅ All motors: torque disabled")
                    
                    print("\n✅ All motors can now be moved manually!")
                    print("⚠️  Be careful when moving the robot")
                    print("Press Ctrl+C when done moving to re-enable torque, or choose option 2")
                    
                    try:
                        while True:
                            time.sleep(1)
                    except KeyboardInterrupt:
                        print("\n\nRe-enabling torque...")
                        client.robot.bus.enable_torque()
                        print("✅ Torque re-enabled")
                        
                except Exception as e:
                    print(f"❌ Error disabling torque: {e}")
            
            elif choice == "2":
                print("\nEnabling torque on all motors...")
                try:
                    # For FeetechMotorsBus, use enable_torque method
                    client.robot.bus.enable_torque()
                    print("  ✅ All motors: torque enabled")
                    print("✅ Robot positions are now locked!")
                except Exception as e:
                    print(f"❌ Error enabling torque: {e}")
            
            elif choice == "3":
                print("\nReading current positions...")
                try:
                    positions = client.read_joints()
                    print("Joint positions (radians):")
                    joint_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", 
                                 "wrist_flex", "wrist_roll", "gripper"]
                    for i, (name, pos) in enumerate(zip(joint_names, positions)):
                        print(f"  {name}: {pos:.4f} rad ({np.degrees(pos):.2f}°)")
                except Exception as e:
                    print(f"❌ Error reading positions: {e}")
                    print("Robot may need calibration first.")
            
            elif choice == "4":
                break
            
            else:
                print("Invalid choice!")
        
        client.disconnect()
        print("\n👋 Disconnected from robot")
        
    except Exception as e:
        print(f"❌ Error: {e}")
        print("\nTroubleshooting:")
        print("1. Check robot power and connection")
        print("2. Verify correct port")
        print("3. Ensure robot is calibrated")


if __name__ == "__main__":
    import numpy as np
    main()