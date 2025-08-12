#!/usr/bin/env python3
"""
Basic Robot Control with Keep-Alive (Raw Coordinates)

Keeps the robot active after script execution by maintaining position control.
"""

import sys
import os
import time
import numpy as np
import threading
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from so101_grasp.robot.so101_client_raw_simple import SO101ClientRawSimple
from so101_grasp.utils.config import ConfigManager


class RobotKeepAlive:
    """Maintains robot position after main script ends."""
    
    def __init__(self, client):
        self.client = client
        self.running = False
        self.thread = None
        self.target_position = None
    
    def start(self, position):
        """Start keep-alive thread."""
        self.target_position = position
        self.running = True
        self.thread = threading.Thread(target=self._keep_alive_loop, daemon=False)
        self.thread.start()
    
    def _keep_alive_loop(self):
        """Continuously send position commands to maintain robot state."""
        while self.running:
            try:
                # Re-send position command to maintain torque
                self.client.write_joints(self.target_position)
                time.sleep(0.5)  # Send command every 500ms
            except:
                break
    
    def stop(self):
        """Stop keep-alive thread."""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1)


def main():
    """Basic control demonstration with keep-alive."""
    print("SO-101 Basic Control (Keep-Alive Mode)")
    print("=" * 40)
    
    config_manager = ConfigManager()
    robot_config = config_manager.get_robot_config()
    
    client = None
    keep_alive = None
    
    try:
        # Connect to robot
        print(f"Connecting to robot on port: {robot_config['port']}")
        client = SO101ClientRawSimple(port=robot_config['port'])
        keep_alive = RobotKeepAlive(client)
        
        print("✅ Robot connected successfully!")
        
        # Read initial position
        print("\n📍 Reading initial position...")
        initial_pos = client.read_joints()
        print(f"Initial position (rad): {[f'{p:.3f}' for p in initial_pos]}")
        
        # Test positions
        positions = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],      # Center
            [0.3, -0.2, 0.3, 0.1, 0.0, 0.0],     # Test 1
            [-0.3, -0.2, 0.3, -0.1, 0.0, 0.0],   # Test 2
        ]
        
        print("\n🎯 Executing test movements...")
        for i, pos in enumerate(positions):
            print(f"\nMoving to position {i+1}: {[f'{p:.3f}' for p in pos]}")
            
            # Interpolate to position
            client.interpolate_waypoint(
                client.read_joints(), 
                pos, 
                steps=30, 
                timestep=0.05
            )
            
            time.sleep(1)
            final_pos = client.read_joints()
            print(f"Reached: {[f'{p:.3f}' for p in final_pos]}")
            
            time.sleep(2)
        
        # Return to initial
        print("\n🏠 Returning to initial position...")
        client.interpolate_waypoint(
            client.read_joints(),
            initial_pos,
            steps=30,
            timestep=0.05
        )
        
        # Start keep-alive at final position
        final_position = client.read_joints()
        print(f"\n🔒 Starting keep-alive at position: {[f'{p:.3f}' for p in final_position]}")
        keep_alive.start(final_position)
        
        print("\n✅ Robot will maintain position for 30 seconds...")
        print("Press Ctrl+C to stop")
        
        # Keep robot active for 30 seconds
        time.sleep(30)
        
    except KeyboardInterrupt:
        print("\n⏹️ Stopped by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
    finally:
        if keep_alive:
            print("\nStopping keep-alive...")
            keep_alive.stop()
        if client:
            client.disconnect()
            print("👋 Disconnected")


if __name__ == "__main__":
    main()