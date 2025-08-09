#!/usr/bin/env python3
"""
Simple working solution for SO-101 digital twin.
This publishes robot_description BEFORE starting robot_state_publisher.
"""

import os
import sys
import time
import subprocess
import signal

def main():
    print("=" * 60)
    print("🤖 STARTING SO-101 DIGITAL TWIN")
    print("=" * 60)
    
    # Check arguments
    if len(sys.argv) > 1 and sys.argv[1] == "--simulate":
        simulate = True
        port = "/dev/ttyACM0"
        print("Mode: SIMULATION")
    else:
        simulate = False
        port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
        print(f"Mode: REAL ROBOT on {port}")
    
    processes = []
    
    try:
        # Step 1: Start the main node that publishes robot_description and joint_states
        print("\n1️⃣ Starting main digital twin node...")
        cmd1 = [sys.executable, "launch_complete.py", f"--port={port}"]
        if simulate:
            cmd1.append("--simulate")
        cmd1.append("--no-rviz")  # We'll launch RViz separately
        
        p1 = subprocess.Popen(cmd1)
        processes.append(p1)
        
        # Wait for robot_description to be published
        print("   Waiting for robot_description to be published...")
        time.sleep(3)
        
        # Step 2: Start robot_state_publisher
        print("\n2️⃣ Starting robot_state_publisher...")
        p2 = subprocess.Popen(["ros2", "run", "robot_state_publisher", "robot_state_publisher"])
        processes.append(p2)
        time.sleep(2)
        
        # Step 3: Start RViz2
        print("\n3️⃣ Starting RViz2...")
        rviz_config = "config/so101_digital_twin.rviz"
        if os.path.exists(rviz_config):
            p3 = subprocess.Popen(["rviz2", "-d", rviz_config])
        else:
            p3 = subprocess.Popen(["rviz2"])
        processes.append(p3)
        
        print("\n" + "=" * 60)
        print("✅ DIGITAL TWIN RUNNING!")
        print("=" * 60)
        print("\n📋 In RViz2:")
        print("  1. Set Fixed Frame to 'world' or 'base'")
        print("  2. Add Display → RobotModel")
        print("  3. The robot should appear and move!")
        print("\nPress Ctrl+C to stop all components")
        print("=" * 60)
        
        # Wait for user interrupt
        signal.pause()
        
    except KeyboardInterrupt:
        print("\n\n⏹️ Stopping all components...")
        for p in processes:
            p.terminate()
        time.sleep(1)
        for p in processes:
            p.kill()
        print("✅ Stopped")
    except Exception as e:
        print(f"Error: {e}")
        for p in processes:
            p.terminate()

if __name__ == "__main__":
    main()