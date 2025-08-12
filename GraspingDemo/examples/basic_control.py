#!/usr/bin/env python3
"""
Basic Robot Control Example

Demonstrates basic SO-101 robot control operations.
"""

import sys
import os
import time
import numpy as np
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from so101_grasp.robot.so101_client import SO101Client
from so101_grasp.utils.config import ConfigManager


def main():
    """Basic control demonstration."""
    print("SO-101 Basic Control Example")
    print("=" * 40)
    # Load configuration
    config_manager = ConfigManager()
    robot_config = config_manager.get_robot_config()
    
    try:
        # Connect to robot
        print(f"Connecting to robot on port: {robot_config['port']}")
        client = SO101Client(
            port=robot_config['port'],
            follower=True,
            force_calibration=False
        )
        
        print("✅ Robot connected successfully!")
        
        # Read initial position
        print("\n📍 Reading initial position...")
        initial_pos = client.read_joints()
        print(f"Initial position: {[f'{p:.3f}' for p in initial_pos]}")
        
        # Define some safe test positions
        positions = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],      # Neutral position
            # [0.3, -0.2, 0.5, 0.0, 0.0, 0.0],     # Test position 1
            # [-0.3, -0.2, 0.5, 0.0, 0.0, 0.0],    # Test position 2
            # [0.0, -0.5, 1.0, 0.5, 0.0, 0.0],     # Test position 3
        ]
        
        print("\n🎯 Executing test movements...")
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
        
        # Test gripper
        print("\n🤏 Testing gripper...")
        current_pos = client.read_joints()
        
        # Close gripper
        current_pos[5] = 50.0  # Close to 50%
        client.write_joints(current_pos)
        time.sleep(2)
        
        # Open gripper
        current_pos[5] = 0.0   # Fully open
        client.write_joints(current_pos)
        time.sleep(1)
        
        print("✅ Gripper test completed!")
        
    except KeyboardInterrupt:
        print("\n⏹️  Movement interrupted by user")
    except Exception as e:
        print(f"\n❌ Error during movement: {e}")
    finally:
        try:
            client.disconnect()
            print("👋 Disconnected from robot")
        except:
            pass


if __name__ == "__main__":
    main()