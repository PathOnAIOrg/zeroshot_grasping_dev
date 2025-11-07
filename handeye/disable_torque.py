#!/usr/bin/env python3
"""Quick script to disable robot torque for manual movement"""

import sys
import os
import time

# Add GraspingDemo to path
grasping_demo_path = os.path.join(os.path.dirname(__file__), '..', 'GraspingDemo')
sys.path.insert(0, grasping_demo_path)

from so101_grasp.robot import SO101ClientRawSimple

print("Connecting to robot...")
robot = SO101ClientRawSimple(port="/dev/ttyACM0")

print("\nDisabling torque on all servos...")
# Disable torque for all 6 servos
for servo_id in range(1, 7):  # Servos 1-6
    success = robot.enable_torque(servo_id, enable=False)
    if success:
        print(f"  ✅ Servo {servo_id} torque disabled")
    else:
        print(f"  ⚠️  Servo {servo_id} - command sent (may have failed)")
    time.sleep(0.01)

print("\n✅ Torque disabled! Robot should now be easy to move by hand.")
print("   You can now move the robot freely for calibration.")
print("\nPress Ctrl+C to exit this script (robot will stay moveable)")

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\nExiting. Robot torque remains disabled.")
    print("To re-enable torque, restart the robot or run: python enable_torque.py")
