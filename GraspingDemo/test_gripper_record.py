#!/usr/bin/env python3
"""
Test gripper recording and replay with debug output
"""

import sys
import os
import time

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from so101_grasp.robot.so101_client_raw_simple import SO101ClientRawSimple

def main():
    print("Gripper Recording Test")
    print("=" * 40)
    
    try:
        client = SO101ClientRawSimple(port="/dev/ttyACM0")
        
        poses = []
        
        # Disable all torque including gripper
        print("\nDisabling ALL torque (including gripper)...")
        for i in range(6):
            client.enable_torque(i + 1, False)
            print(f"  Servo {i+1} torque disabled")
            time.sleep(0.01)
        
        print("\n✅ All 6 servos disabled - move robot AND gripper manually")
        
        # Record 3 poses
        for i in range(3):
            input(f"\nMove to pose {i+1} (including gripper) and press ENTER...")
            pose = client.read_joints()
            poses.append(pose)
            print(f"Recorded pose {i+1}:")
            print(f"  Joints 0-4: {[f'{p:.2f}' for p in pose[:5]]}")
            print(f"  Gripper (joint 5): {pose[5]:.3f} rad")
        
        # Re-enable torque
        print("\nRe-enabling ALL torque...")
        for i in range(6):
            client.enable_torque(i + 1, True)
            print(f"  Servo {i+1} torque enabled")
            time.sleep(0.01)
        
        # Replay
        input("\nPress ENTER to replay...")
        
        for i, pose in enumerate(poses):
            print(f"\nMoving to pose {i+1}...")
            print(f"  Target joints 0-4: {[f'{p:.2f}' for p in pose[:5]]}")
            print(f"  Target gripper: {pose[5]:.3f} rad")
            
            # Move to pose
            client.write_joints(pose)
            time.sleep(2)
            
            # Read actual position
            actual = client.read_joints()
            print(f"  Actual joints 0-4: {[f'{p:.2f}' for p in actual[:5]]}")
            print(f"  Actual gripper: {actual[5]:.3f} rad")
            
            # Check gripper specifically
            gripper_error = abs(actual[5] - pose[5])
            if gripper_error > 0.1:
                print(f"  ⚠️ Gripper error: {gripper_error:.3f} rad")
        
        print("\n✅ Test complete!")
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            # Release all
            for i in range(6):
                client.enable_torque(i + 1, False)
            client.disconnect()
        except:
            pass

if __name__ == "__main__":
    main()