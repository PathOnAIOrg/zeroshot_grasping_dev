#!/usr/bin/env python3
"""
Replay Saved Trajectories

Load and replay previously saved trajectory files.
"""

import sys
import os
import time
import json
from pathlib import Path

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from so101_grasp.robot.so101_client_raw_simple import SO101ClientRawSimple


def list_trajectories(traj_dir):
    """List all saved trajectory files."""
    files = list(traj_dir.glob("*.json"))
    if not files:
        print("No saved trajectories found!")
        return []
    
    print("\n📁 Available trajectories:")
    print("-" * 40)
    
    trajectories = []
    for i, file in enumerate(sorted(files), 1):
        try:
            with open(file, 'r') as f:
                data = json.load(f)
            print(f"{i}. {file.name}")
            print(f"   Created: {data.get('created', 'Unknown')}")
            print(f"   Poses: {data.get('num_keyframes', 'Unknown')}")
            trajectories.append(file)
        except:
            pass
    
    return trajectories


def load_trajectory(filepath):
    """Load trajectory from JSON file."""
    with open(filepath, 'r') as f:
        data = json.load(f)
    return data


def replay_trajectory(client, keyframes, speed=1.0):
    """Replay a trajectory."""
    if len(keyframes) < 2:
        print("Need at least 2 keyframes!")
        return False
    
    print(f"\n🎬 Starting replay ({len(keyframes)} poses)...")
    
    # Move to start position
    print("Moving to start position...")
    current_pose = client.read_joints()
    start_pose = keyframes[0]['pose']
    client.interpolate_waypoint(current_pose, start_pose, steps=50, timestep=0.02)
    time.sleep(1)
    
    # Replay through keyframes
    for i in range(len(keyframes) - 1):
        print(f"  Pose {i+1} → {i+2}")
        
        # Calculate interpolation based on speed
        steps = int(30 / speed)
        timestep = 0.03 * speed
        
        client.interpolate_waypoint(
            keyframes[i]['pose'],
            keyframes[i+1]['pose'],
            steps=steps,
            timestep=timestep
        )
        
        # Pause between moves (adjusted by speed)
        time.sleep(0.3 / speed)
    
    print("✅ Replay complete!")
    return True


def main():
    """Main replay function."""
    print("SO-101 Trajectory Replay")
    print("=" * 40)
    
    # Check for trajectories directory
    traj_dir = Path("trajectories")
    if not traj_dir.exists():
        print("❌ No trajectories directory found!")
        print("   Run simple_trajectory_demo.py first to record some trajectories.")
        return
    
    try:
        # List available trajectories
        trajectories = list_trajectories(traj_dir)
        if not trajectories:
            return
        
        # Select trajectory
        print("\n" + "-" * 40)
        choice = input("Enter trajectory number to replay (or 'q' to quit): ").strip()
        
        if choice.lower() == 'q':
            return
        
        if not choice.isdigit():
            print("Invalid choice!")
            return
        
        idx = int(choice) - 1
        if idx < 0 or idx >= len(trajectories):
            print("Invalid trajectory number!")
            return
        
        # Load selected trajectory
        selected_file = trajectories[idx]
        print(f"\n📂 Loading: {selected_file.name}")
        data = load_trajectory(selected_file)
        keyframes = data['keyframes']
        
        print(f"✅ Loaded {len(keyframes)} poses")
        
        # Connect to robot
        print("\nConnecting to robot...")
        client = SO101ClientRawSimple(port="/dev/ttyACM0")
        print("✅ Robot connected")
        
        # Replay loop
        while True:
            print("\n" + "="*40)
            print("REPLAY OPTIONS")
            print("="*40)
            print("1. Normal speed (1x)")
            print("2. Slow motion (0.5x)")
            print("3. Fast forward (4x)")
            print("4. Custom speed")
            print("0. Exit")
            print("-" * 40)
            
            option = input("Choose option: ").strip()
            
            if option == '0':
                break
            elif option == '1':
                replay_trajectory(client, keyframes, speed=1.0)
            elif option == '2':
                replay_trajectory(client, keyframes, speed=0.5)
            elif option == '3':
                replay_trajectory(client, keyframes, speed=4.0)
            elif option == '4':
                speed = input("Enter speed (0.1 to 5.0): ").strip()
                try:
                    speed = float(speed)
                    speed = max(0.1, min(5.0, speed))
                    replay_trajectory(client, keyframes, speed=speed)
                except:
                    print("Invalid speed!")
            else:
                print("Invalid option!")
            
            again = input("\nReplay again? (y/n): ").strip().lower()
            if again != 'y':
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