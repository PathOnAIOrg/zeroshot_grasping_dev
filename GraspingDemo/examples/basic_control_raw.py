#!/usr/bin/env python3
"""
Basic Robot Control with Raw Coordinates (Same as JointStateReader)

Uses the same coordinate system as joint_state_reader.py (raw servo positions)
"""

import sys
import os
import time
import numpy as np
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from so101_grasp.robot.so101_client_raw_simple import SO101ClientRawSimple
from so101_grasp.utils.config import ConfigManager


def main():
    """Basic control demonstration with raw coordinates."""
    print("SO-101 Basic Control (Raw Coordinates)")
    print("=" * 40)
    # Load configuration
    config_manager = ConfigManager()
    robot_config = config_manager.get_robot_config()
    
    try:
        # Connect to robot (no calibration, raw mode)
        print(f"Connecting to robot on port: {robot_config['port']}")
        client = SO101ClientRawSimple(
            port=robot_config['port']
        )
        
        print("✅ Robot connected successfully (raw mode)!")
        
        # Read initial position (raw radians, -π to π)
        print("\n📍 Reading initial position...")
        initial_pos = client.read_joints()
        print(f"Initial position (rad): {[f'{p:.3f}' for p in initial_pos]}")
        
        # Define test positions in raw coordinates (-π to π)
        # These match the JointStateReader coordinate system
        positions = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],      # Center position (2048 ticks)
            [0.5, -0.3, 0.4, 0.2, 0.0, 0.0],     # Test position 1
            [-0.5, -0.3, 0.4, -0.2, 0.0, 0.0],   # Test position 2
            [0.0, -0.5, 0.8, 0.3, 0.0, 0.0],     # Test position 3
        ]
        
        print("\n🎯 Executing test movements (raw coordinates)...")
        for i, pos in enumerate(positions):
            print(f"\nMoving to position {i+1}: {[f'{p:.3f}' for p in pos]}")
            
            # Move to position with interpolation
            client.interpolate_waypoint(
                client.read_joints(), 
                pos, 
                steps=30, 
                timestep=0.05
            )
            
            # Wait and read final position
            time.sleep(1)
            final_pos = client.read_joints()
            print(f"Reached position: {[f'{p:.3f}' for p in final_pos]}")
            
            # Calculate error
            error = np.array(pos) - np.array(final_pos)
            max_error = np.max(np.abs(error))
            print(f"Max error: {max_error:.4f} rad ({np.degrees(max_error):.2f}°)")
            
            time.sleep(2)  # Pause between movements
        
        # Return to initial position
        print("\n🏠 Returning to initial position...")
        client.interpolate_waypoint(
            client.read_joints(),
            initial_pos,
            steps=30,
            timestep=0.05
        )
        
        print("✅ Basic control demonstration completed!")
        
        # Test gripper (raw values)
        print("\n🤏 Testing gripper...")
        current_pos = client.read_joints()
        
        # Close gripper (positive value in raw mode)
        current_pos[5] = 1.0  # Close to ~1 radian
        client.write_joints(current_pos)
        time.sleep(2)
        
        # Open gripper
        current_pos[5] = 0.0   # Center position
        client.write_joints(current_pos)
        time.sleep(1)
        
        print("✅ Gripper test completed!")
        
    except KeyboardInterrupt:
        print("\n⏹️  Movement interrupted by user")
    except Exception as e:
        print(f"\n❌ Error during movement: {e}")
    finally:
        try:
            # Release torque to allow manual movement
            print("📌 Releasing robot for manual control...")
            for i in range(6):
                client.enable_torque(i + 1, False)
                time.sleep(0.01)
            print("✅ Robot torque disabled - you can move it manually")
            client.disconnect(disable_torque=False, close_port=True)
            print("👋 Disconnected from robot")
        except:
            pass


if __name__ == "__main__":
    main()