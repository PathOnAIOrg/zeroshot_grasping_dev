#!/usr/bin/env python3
"""
Simple Trajectory Demo - Quick Record and Replay

A simplified version for quick demonstration:
1. Run the script
2. Move robot and press Enter to record poses
3. Automatically saves and replays the trajectory
"""

import sys
import os
import time
import json
from datetime import datetime
from pathlib import Path

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from so101_grasp.robot.so101_client_raw_simple import SO101ClientRawSimple


def main():
    """Simple trajectory recording and replay demo."""
    print("SO-101 Simple Trajectory Demo")
    print("=" * 40)
    
    # Create trajectories directory
    traj_dir = Path("trajectories")
    traj_dir.mkdir(exist_ok=True)
    
    try:
        # Connect to robot
        print("Connecting to robot...")
        client = SO101ClientRawSimple(port="/dev/ttyACM0")
        print("✅ Robot connected")
        
        # List of recorded poses
        keyframes = []
        
        print("\n" + "="*40)
        print("QUICK RECORDING MODE")
        print("="*40)
        print("1. Robot torque will be disabled")
        print("2. Move robot to desired pose")
        print("3. Press ENTER to record the pose")
        print("4. Type 'done' when finished")
        print("5. Trajectory will auto-save and replay")
        print("="*40)
        
        input("\nPress ENTER to start...")
        
        # Disable torque for manual movement
        print("\nDisabling torque...")
        for i in range(6):
            client.enable_torque(i + 1, False)
            time.sleep(0.01)
        print("✅ Torque disabled - move robot manually\n")
        
        # Recording loop
        pose_count = 0
        while True:
            cmd = input(f"Pose {pose_count+1}: Press ENTER to record (or 'done'): ").strip().lower()
            
            if cmd == 'done':
                if pose_count < 2:
                    print("⚠️  Need at least 2 poses! Keep recording...")
                    continue
                break
            elif cmd == '':
                # Record current pose (including gripper)
                pose = client.read_joints()
                keyframes.append({
                    'index': pose_count,
                    'pose': [float(p) for p in pose],
                    'timestamp': time.time()
                })
                pose_count += 1
                # Show all 6 joints including gripper (joint 5)
                print(f"  ✅ Recorded! Joints: {[f'{p:.2f}' for p in pose[:5]]}")
                print(f"              Gripper: {pose[5]:.2f} rad")
        
        print(f"\n✅ Recording complete! {len(keyframes)} poses recorded")
        
        # Re-enable torque
        print("\nRe-enabling torque...")
        for i in range(6):
            client.enable_torque(i + 1, True)
            time.sleep(0.01)
        
        # Save trajectory
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = traj_dir / f"demo_trajectory_{timestamp}.json"
        
        data = {
            'name': f"demo_{timestamp}",
            'created': datetime.now().isoformat(),
            'num_keyframes': len(keyframes),
            'keyframes': keyframes
        }
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
        
        print(f"\n💾 Trajectory saved to: {filename}")
        
        # Wait before replay
        print("\n" + "="*40)
        print("REPLAY MODE")
        print("="*40)
        input("Press ENTER to replay the trajectory...")
        
        # Move to start position
        print("\nMoving to start position...")
        current_pose = client.read_joints()
        start_pose = keyframes[0]['pose']
        client.interpolate_waypoint(current_pose, start_pose, steps=50, timestep=0.02)
        time.sleep(1)
        
        # Replay trajectory
        print("\n🎬 Replaying trajectory...")
        for i in range(len(keyframes) - 1):
            print(f"  Moving: Pose {i+1} → Pose {i+2}")
            client.interpolate_waypoint(
                keyframes[i]['pose'],
                keyframes[i+1]['pose'],
                steps=30,
                timestep=0.03
            )
            time.sleep(0.5)  # Pause between moves
        
        print("\n✅ Replay complete!")
        
        # Ask if want to replay again
        while True:
            again = input("\nReplay again? (y/n): ").strip().lower()
            if again == 'y':
                print("\n🎬 Replaying trajectory...")
                # Move to start
                current_pose = client.read_joints()
                client.interpolate_waypoint(current_pose, keyframes[0]['pose'], steps=50, timestep=0.02)
                time.sleep(1)
                
                # Replay
                for i in range(len(keyframes) - 1):
                    print(f"  Moving: Pose {i+1} → Pose {i+2}")
                    client.interpolate_waypoint(
                        keyframes[i]['pose'],
                        keyframes[i+1]['pose'],
                        steps=30,
                        timestep=0.03
                    )
                    time.sleep(0.5)
                print("✅ Replay complete!")
            else:
                break
        
    except KeyboardInterrupt:
        print("\n\n⏹️  Interrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            # Release torque before exit
            print("\n📌 Releasing robot...")
            for i in range(6):
                client.enable_torque(i + 1, False)
                time.sleep(0.01)
            client.disconnect()
            print("✅ Done - robot is free to move")
        except:
            pass


if __name__ == "__main__":
    main()