#!/usr/bin/env python3
"""
Debug script to test gripper range and find correct open/close values
"""

import sys
import os
import time

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from so101_grasp.robot.so101_client_raw_simple import SO101ClientRawSimple

def main():
    print("Gripper Debug Test")
    print("=" * 40)
    
    try:
        client = SO101ClientRawSimple(port="/dev/ttyACM0")
        
        # Read current position
        current_pos = client.read_joints()
        print(f"\nCurrent gripper value: {current_pos[5]:.3f} rad")
        print(f"Current gripper ticks: {client.radians_to_ticks(current_pos[5])}")
        
        # Test different values
        test_values = [
            ("Maximum close", -3.14),
            ("Medium close", -1.5),
            ("Slight close", -0.5),
            ("Center", 0.0),
            ("Slight open", 0.5),
            ("Medium open", 1.5),
            ("Maximum open", 3.14),
        ]
        
        print("\n" + "="*40)
        print("Testing gripper values...")
        print("="*40)
        
        test_pos = current_pos.copy()
        
        for name, value in test_values:
            print(f"\n{name}: {value:.2f} rad")
            
            # Calculate ticks
            ticks = client.radians_to_ticks(value)
            print(f"  Ticks: {ticks}")
            
            # Move gripper
            test_pos[5] = value
            client.write_joints(test_pos)
            time.sleep(2)
            
            # Read actual position
            actual = client.read_joints()
            actual_ticks = client.radians_to_ticks(actual[5])
            print(f"  Actual: {actual[5]:.3f} rad (ticks: {actual_ticks})")
            
            # Check if gripper moved
            if abs(actual[5] - value) > 0.5:
                print(f"  ⚠️ Large error: {abs(actual[5] - value):.3f} rad")
        
        # Test raw servo commands
        print("\n" + "="*40)
        print("Testing raw servo commands...")
        print("="*40)
        
        servo_id = 6  # Gripper servo
        
        # Test specific tick values
        tick_tests = [
            ("Min (0)", 0),
            ("Quarter (1024)", 1024),
            ("Center (2048)", 2048),
            ("Three-quarter (3072)", 3072),
            ("Max (4095)", 4095),
        ]
        
        for name, ticks in tick_tests:
            print(f"\n{name}: {ticks} ticks")
            
            # Write directly to servo
            success = client.write_servo_position(servo_id, ticks)
            print(f"  Write success: {success}")
            time.sleep(2)
            
            # Read back
            actual_ticks = client.read_servo_position(servo_id)
            if actual_ticks:
                actual_rad = client.ticks_to_radians(actual_ticks)
                print(f"  Actual: {actual_ticks} ticks ({actual_rad:.3f} rad)")
            else:
                print(f"  Failed to read servo")
        
        # Interactive test
        print("\n" + "="*40)
        print("Interactive test - enter tick values (0-4095)")
        print("Type 'q' to quit")
        print("="*40)
        
        while True:
            cmd = input("\nEnter ticks (or 'q'): ").strip()
            if cmd.lower() == 'q':
                break
            
            try:
                ticks = int(cmd)
                if 0 <= ticks <= 4095:
                    print(f"  Writing {ticks} ticks to gripper...")
                    client.write_servo_position(servo_id, ticks)
                    time.sleep(1)
                    
                    actual = client.read_servo_position(servo_id)
                    if actual:
                        print(f"  Actual: {actual} ticks")
                    else:
                        print(f"  Failed to read")
                else:
                    print("  Invalid range! Use 0-4095")
            except ValueError:
                print("  Invalid input!")
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            # Release
            for i in range(6):
                client.enable_torque(i + 1, False)
            client.disconnect()
            print("\n✅ Gripper released")
        except:
            pass

if __name__ == "__main__":
    main()