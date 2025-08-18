#!/usr/bin/env python3
"""
Keyframe Recording and Replay for SO-101 Robot

This script allows you to:
1. Record keyframe poses by manually moving the robot
2. Save the recorded trajectory to a file
3. Load and replay saved trajectories
"""

import sys
import os
import time
import json
from datetime import datetime
from pathlib import Path

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from so101_grasp.robot.so101_client_raw_simple import SO101ClientRawSimple
from so101_grasp.utils.config import ConfigManager


class KeyframeRecorder:
    """Record and replay keyframe trajectories for SO-101."""
    
    def __init__(self, client):
        self.client = client
        self.keyframes = []
        self.trajectory_dir = Path("trajectories")
        self.trajectory_dir.mkdir(exist_ok=True)
        
    def record_keyframe(self):
        """Record current robot pose as a keyframe."""
        pose = self.client.read_joints()
        timestamp = time.time()
        keyframe = {
            'pose': pose,
            'timestamp': timestamp,
            'index': len(self.keyframes)
        }
        self.keyframes.append(keyframe)
        return keyframe
    
    def clear_keyframes(self):
        """Clear all recorded keyframes."""
        self.keyframes = []
        print("All keyframes cleared")
    
    def save_trajectory(self, name=None):
        """Save recorded keyframes to a JSON file."""
        if not self.keyframes:
            print("No keyframes to save!")
            return None
            
        if name is None:
            name = datetime.now().strftime("trajectory_%Y%m%d_%H%M%S")
        
        filename = self.trajectory_dir / f"{name}.json"
        
        # Convert numpy arrays to lists for JSON serialization
        data = {
            'name': name,
            'created': datetime.now().isoformat(),
            'num_keyframes': len(self.keyframes),
            'keyframes': [
                {
                    'index': kf['index'],
                    'pose': [float(p) for p in kf['pose']],
                    'timestamp': kf['timestamp']
                }
                for kf in self.keyframes
            ]
        }
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
        
        print(f"✅ Trajectory saved to: {filename}")
        return filename
    
    def load_trajectory(self, filename):
        """Load trajectory from a JSON file."""
        filepath = Path(filename)
        if not filepath.exists():
            filepath = self.trajectory_dir / filename
            if not filepath.suffix:
                filepath = filepath.with_suffix('.json')
        
        if not filepath.exists():
            print(f"❌ File not found: {filepath}")
            return False
        
        with open(filepath, 'r') as f:
            data = json.load(f)
        
        self.keyframes = [
            {
                'index': kf['index'],
                'pose': kf['pose'],
                'timestamp': kf['timestamp']
            }
            for kf in data['keyframes']
        ]
        
        print(f"✅ Loaded trajectory: {data['name']}")
        print(f"   Created: {data['created']}")
        print(f"   Keyframes: {data['num_keyframes']}")
        return True
    
    def replay_trajectory(self, speed=1.0, interpolation_steps=30):
        """
        Replay the recorded trajectory.
        
        Args:
            speed: Playback speed multiplier (1.0 = normal, 2.0 = 2x speed)
            interpolation_steps: Number of interpolation steps between keyframes
        """
        if len(self.keyframes) < 2:
            print("Need at least 2 keyframes to replay!")
            return
        
        print(f"\n🎬 Replaying trajectory with {len(self.keyframes)} keyframes...")
        print(f"   Speed: {speed}x")
        print(f"   Interpolation: {interpolation_steps} steps")
        
        # Move to first keyframe
        print("\nMoving to start position...")
        start_pose = self.keyframes[0]['pose']
        current_pose = self.client.read_joints()
        self.client.interpolate_waypoint(
            current_pose, 
            start_pose, 
            steps=50, 
            timestep=0.02
        )
        time.sleep(1)
        
        # Replay through all keyframes
        for i in range(len(self.keyframes) - 1):
            kf1 = self.keyframes[i]
            kf2 = self.keyframes[i + 1]
            
            print(f"Keyframe {i+1} → {i+2}")
            
            # Calculate time between keyframes (for natural timing)
            time_diff = kf2['timestamp'] - kf1['timestamp']
            adjusted_time = time_diff / speed
            
            # Interpolate between keyframes
            timestep = adjusted_time / interpolation_steps if interpolation_steps > 0 else 0.02
            timestep = max(0.01, min(0.1, timestep))  # Clamp timestep
            
            self.client.interpolate_waypoint(
                kf1['pose'],
                kf2['pose'],
                steps=interpolation_steps,
                timestep=timestep
            )
        
        print("✅ Trajectory replay complete!")
    
    def list_saved_trajectories(self):
        """List all saved trajectory files."""
        files = list(self.trajectory_dir.glob("*.json"))
        if not files:
            print("No saved trajectories found")
            return []
        
        print("\n📁 Saved trajectories:")
        trajectories = []
        for i, file in enumerate(files, 1):
            with open(file, 'r') as f:
                data = json.load(f)
            print(f"  {i}. {file.name}")
            print(f"     Created: {data['created']}")
            print(f"     Keyframes: {data['num_keyframes']}")
            trajectories.append(file.name)
        return trajectories


def main():
    """Main function for keyframe recording and replay."""
    print("SO-101 Keyframe Recorder")
    print("=" * 40)
    
    # Load configuration
    config_manager = ConfigManager()
    robot_config = config_manager.get_robot_config()
    
    try:
        # Connect to robot
        print(f"Connecting to robot on port: {robot_config['port']}")
        client = SO101ClientRawSimple(port=robot_config['port'])
        print("✅ Robot connected (raw mode)")
        
        # Create recorder
        recorder = KeyframeRecorder(client)
        
        # Main menu loop
        while True:
            print("\n" + "="*40)
            print("KEYFRAME RECORDER MENU")
            print("="*40)
            print("1. Start recording keyframes")
            print("2. Replay current trajectory")
            print("3. Save trajectory to file")
            print("4. Load trajectory from file")
            print("5. List saved trajectories")
            print("6. Clear current keyframes")
            print("0. Exit")
            print("-"*40)
            
            choice = input("Enter choice: ").strip()
            
            if choice == '1':
                # Recording mode
                print("\n📹 RECORDING MODE")
                print("Instructions:")
                print("  - Disable torque to move robot manually")
                print("  - Press ENTER to record current pose")
                print("  - Type 'done' to finish recording")
                print("  - Type 'cancel' to abort")
                
                # Disable torque for manual movement
                print("\nDisabling torque...")
                for i in range(6):
                    client.enable_torque(i + 1, False)
                    time.sleep(0.01)
                print("✅ Torque disabled - move robot manually")
                
                recorder.clear_keyframes()
                
                while True:
                    cmd = input("\nPress ENTER to record pose (or 'done'/'cancel'): ").strip().lower()
                    
                    if cmd == 'done':
                        # Re-enable torque
                        print("Re-enabling torque...")
                        for i in range(6):
                            client.enable_torque(i + 1, True)
                            time.sleep(0.01)
                        print(f"✅ Recording complete! {len(recorder.keyframes)} keyframes recorded")
                        break
                    elif cmd == 'cancel':
                        recorder.clear_keyframes()
                        # Re-enable torque
                        for i in range(6):
                            client.enable_torque(i + 1, True)
                            time.sleep(0.01)
                        print("Recording cancelled")
                        break
                    elif cmd == '':
                        # Record keyframe
                        kf = recorder.record_keyframe()
                        print(f"✅ Keyframe {kf['index']+1} recorded")
                        print(f"   Pose: {[f'{p:.3f}' for p in kf['pose']]}")
            
            elif choice == '2':
                # Replay trajectory
                if not recorder.keyframes:
                    print("❌ No keyframes to replay!")
                    continue
                
                print(f"\nCurrent trajectory has {len(recorder.keyframes)} keyframes")
                speed = input("Enter playback speed (default 1.0): ").strip()
                speed = float(speed) if speed else 1.0
                
                steps = input("Interpolation steps between keyframes (default 30): ").strip()
                steps = int(steps) if steps else 30
                
                recorder.replay_trajectory(speed=speed, interpolation_steps=steps)
            
            elif choice == '3':
                # Save trajectory
                if not recorder.keyframes:
                    print("❌ No keyframes to save!")
                    continue
                
                name = input("Enter trajectory name (or press ENTER for auto): ").strip()
                name = name if name else None
                recorder.save_trajectory(name)
            
            elif choice == '4':
                # Load trajectory
                trajectories = recorder.list_saved_trajectories()
                if not trajectories:
                    continue
                
                choice = input("\nEnter file number or name: ").strip()
                
                if choice.isdigit():
                    idx = int(choice) - 1
                    if 0 <= idx < len(trajectories):
                        filename = trajectories[idx]
                    else:
                        print("Invalid selection")
                        continue
                else:
                    filename = choice
                
                if recorder.load_trajectory(filename):
                    print(f"Ready to replay {len(recorder.keyframes)} keyframes")
            
            elif choice == '5':
                # List trajectories
                recorder.list_saved_trajectories()
            
            elif choice == '6':
                # Clear keyframes
                recorder.clear_keyframes()
            
            elif choice == '0':
                # Exit
                print("\nExiting...")
                break
            
            else:
                print("Invalid choice!")
        
    except KeyboardInterrupt:
        print("\n\n⏹️  Interrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
    finally:
        try:
            # Release torque before exit
            print("\n📌 Releasing robot for manual control...")
            for i in range(6):
                client.enable_torque(i + 1, False)
                time.sleep(0.01)
            print("✅ Robot torque disabled")
            client.disconnect(disable_torque=False, close_port=True)
            print("👋 Disconnected from robot")
        except:
            pass


if __name__ == "__main__":
    main()