#!/usr/bin/env python3
"""
Enhanced Trajectory Demo with Gripper Status Display

Shows gripper open/close status during recording and replay.
"""

import sys
import os
import time
import json
from datetime import datetime
from pathlib import Path

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from so101_grasp.robot.so101_client_raw_simple import SO101ClientRawSimple


def get_gripper_status(gripper_value):
    """Convert gripper value to human-readable status."""
    if gripper_value < -1.0:
        return "CLOSED"
    elif gripper_value > 1.0:
        return "OPEN"
    elif -0.5 < gripper_value < 0.5:
        return "MIDDLE"
    elif gripper_value < 0:
        return "CLOSING"
    else:
        return "OPENING"


def main():
    """Trajectory recording and replay with gripper status."""
    print("SO-101 Trajectory Demo with Gripper Info")
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
        print("RECORDING MODE WITH GRIPPER")
        print("="*40)
        print("1. Robot torque will be disabled")
        print("2. Move robot AND GRIPPER to desired pose")
        print("3. Press ENTER to record the pose")
        print("4. Type 'done' when finished")
        print("="*40)
        
        input("\nPress ENTER to start...")
        
        # Disable torque for manual movement
        print("\nDisabling torque (including gripper)...")
        for i in range(6):
            client.enable_torque(i + 1, False)
            time.sleep(0.01)
        print("✅ All 6 joints disabled - move robot and gripper manually\n")
        
        # Recording loop
        pose_count = 0
        while True:
            cmd = input(f"\nPose {pose_count+1}: Press ENTER to record (or 'done'): ").strip().lower()
            
            if cmd == 'done':
                if pose_count < 2:
                    print("⚠️  Need at least 2 poses! Keep recording...")
                    continue
                break
            elif cmd == '':
                # Record current pose
                pose = client.read_joints()
                gripper_status = get_gripper_status(pose[5])
                
                keyframes.append({
                    'index': pose_count,
                    'pose': [float(p) for p in pose],
                    'timestamp': time.time(),
                    'gripper_status': gripper_status
                })
                pose_count += 1
                
                print(f"  ✅ Pose {pose_count} recorded!")
                print(f"     Arm joints: {[f'{p:.2f}' for p in pose[:5]]}")
                print(f"     Gripper: {pose[5]:.3f} rad ({gripper_status})")
        
        print(f"\n✅ Recording complete! {len(keyframes)} poses recorded")
        
        # Show summary
        print("\nRecorded sequence:")
        for i, kf in enumerate(keyframes):
            print(f"  Pose {i+1}: Gripper {kf['gripper_status']}")
        
        # Re-enable torque
        print("\nRe-enabling torque...")
        for i in range(6):
            client.enable_torque(i + 1, True)
            time.sleep(0.01)
        
        # Save trajectory
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = traj_dir / f"gripper_trajectory_{timestamp}.json"
        
        data = {
            'name': f"gripper_demo_{timestamp}",
            'created': datetime.now().isoformat(),
            'num_keyframes': len(keyframes),
            'keyframes': keyframes
        }
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
        
        print(f"\n💾 Trajectory saved to: {filename}")
        
        # Wait before replay
        print("\n" + "="*40)
        print("REPLAY MODE WITH GRIPPER")
        print("="*40)
        input("Press ENTER to replay the trajectory...")
        
        # Move to start position
        print("\nMoving to start position...")
        current_pose = client.read_joints()
        start_pose = keyframes[0]['pose']
        print(f"  Start gripper: {get_gripper_status(start_pose[5])}")
        client.interpolate_waypoint(current_pose, start_pose, steps=50, timestep=0.02)
        time.sleep(1)
        
        # Replay trajectory
        print("\n🎬 Replaying trajectory with gripper actions...")
        for i in range(len(keyframes) - 1):
            kf1 = keyframes[i]
            kf2 = keyframes[i+1]
            
            gripper_from = kf1['gripper_status']
            gripper_to = kf2['gripper_status']
            
            print(f"\n  Pose {i+1} → Pose {i+2}")
            print(f"    Gripper: {gripper_from} → {gripper_to}")
            
            # Show actual values
            print(f"    Gripper value: {kf1['pose'][5]:.3f} → {kf2['pose'][5]:.3f}")
            
            client.interpolate_waypoint(
                kf1['pose'],
                kf2['pose'],
                steps=30,
                timestep=0.03
            )
            
            # Verify final position
            actual = client.read_joints()
            actual_gripper_status = get_gripper_status(actual[5])
            print(f"    Actual gripper: {actual[5]:.3f} ({actual_gripper_status})")
            
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
                
                # Replay with gripper info
                for i in range(len(keyframes) - 1):
                    print(f"  Pose {i+1} → {i+2}: Gripper {keyframes[i]['gripper_status']} → {keyframes[i+1]['gripper_status']}")
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