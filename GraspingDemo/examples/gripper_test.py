#!/usr/bin/env python3
"""
Gripper Test Script

Test gripper (jaw) open and close movements.
The gripper is joint index 5 (the 6th joint).
"""

import sys
import os
import time
import numpy as np

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from so101_grasp.robot.so101_client_raw_simple import SO101ClientRawSimple


def main():
    """Test gripper functionality."""
    print("SO-101 Gripper Test")
    print("=" * 40)
    
    try:
        # Connect to robot
        print("Connecting to robot...")
        client = SO101ClientRawSimple(port="/dev/ttyACM0")
        print("✅ Robot connected")
        
        # Read current position
        print("\n📍 Reading current position...")
        current_pos = client.read_joints()
        print(f"Current joints: {[f'{p:.3f}' for p in current_pos[:5]]}")
        print(f"Current gripper: {current_pos[5]:.3f} rad")
        
        print("\n" + "="*40)
        print("GRIPPER TEST SEQUENCE")
        print("="*40)
        print("The gripper will:")
        print("1. Close fully")
        print("2. Open fully")
        print("3. Move to middle position")
        print("4. Test incremental positions")
        print("="*40)
        
        input("\nPress ENTER to start gripper test...")
        
        # Keep first 5 joints unchanged, only move gripper
        test_pos = current_pos.copy()
        
        # Test 1: Close gripper
        print("\n1. Closing gripper...")
        test_pos[5] = -1.5  # Close position (negative value)
        client.write_joints(test_pos)
        time.sleep(2)
        actual = client.read_joints()
        print(f"   Target: {test_pos[5]:.3f} rad")
        print(f"   Actual: {actual[5]:.3f} rad")
        
        # Test 2: Open gripper
        print("\n2. Opening gripper...")
        test_pos[5] = 1.5  # Open position (positive value)
        client.write_joints(test_pos)
        time.sleep(2)
        actual = client.read_joints()
        print(f"   Target: {test_pos[5]:.3f} rad")
        print(f"   Actual: {actual[5]:.3f} rad")
        
        # Test 3: Middle position
        print("\n3. Moving to middle position...")
        test_pos[5] = 0.0  # Middle position
        client.write_joints(test_pos)
        time.sleep(2)
        actual = client.read_joints()
        print(f"   Target: {test_pos[5]:.3f} rad")
        print(f"   Actual: {actual[5]:.3f} rad")
        
        # Test 4: Incremental positions
        print("\n4. Testing incremental positions...")
        positions = [-1.0, -0.5, 0.0, 0.5, 1.0]
        for pos in positions:
            print(f"   Moving to {pos:.1f} rad...")
            test_pos[5] = pos
            client.write_joints(test_pos)
            time.sleep(1.5)
            actual = client.read_joints()
            print(f"      Actual: {actual[5]:.3f} rad")
        
        # Return to original position
        print("\n🏠 Returning to original position...")
        client.write_joints(current_pos)
        time.sleep(2)
        
        print("\n✅ Gripper test completed!")
        
        # Interactive mode
        print("\n" + "="*40)
        print("INTERACTIVE GRIPPER CONTROL")
        print("="*40)
        print("Enter gripper values (-3.14 to 3.14)")
        print("Or commands: 'open', 'close', 'middle', 'quit'")
        print("="*40)
        
        while True:
            cmd = input("\nGripper command: ").strip().lower()
            
            if cmd == 'quit' or cmd == 'q':
                break
            elif cmd == 'open':
                test_pos[5] = 1.5
                client.write_joints(test_pos)
                print("  → Gripper opened")
            elif cmd == 'close':
                test_pos[5] = -1.5
                client.write_joints(test_pos)
                print("  → Gripper closed")
            elif cmd == 'middle':
                test_pos[5] = 0.0
                client.write_joints(test_pos)
                print("  → Gripper at middle")
            else:
                try:
                    value = float(cmd)
                    # Clamp to valid range
                    value = max(-3.14, min(3.14, value))
                    test_pos[5] = value
                    client.write_joints(test_pos)
                    print(f"  → Gripper set to {value:.3f} rad")
                except ValueError:
                    print("  ❌ Invalid input! Use number or command")
            
            # Read actual position
            actual = client.read_joints()
            print(f"  Current gripper: {actual[5]:.3f} rad")
        
    except KeyboardInterrupt:
        print("\n\n⏹️  Test interrupted")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            # Release torque
            print("\n📌 Releasing robot...")
            for i in range(6):
                client.enable_torque(i + 1, False)
                time.sleep(0.01)
            client.disconnect()
            print("✅ Robot released")
        except:
            pass


if __name__ == "__main__":
    main()