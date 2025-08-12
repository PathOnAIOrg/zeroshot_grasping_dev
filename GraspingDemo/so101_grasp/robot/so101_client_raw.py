import sys
import os
# Add the lerobot path
lerobot_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), 'lerobot', 'src')
if lerobot_path not in sys.path:
    sys.path.insert(0, lerobot_path)

from lerobot.robots import make_robot_from_config
from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
from lerobot.teleoperators.so101_leader.config_so101_leader import SO101LeaderConfig
from lerobot.teleoperators.utils import make_teleoperator_from_config
from typing import List, Tuple
import time
import numpy as np
import os
from pathlib import Path


class SO101ClientRaw:
    """
    SO101 Client with raw coordinates matching JointStateReader.
    
    This client uses the same coordinate system as joint_state_reader.py:
    - No calibration offsets
    - Raw servo positions normalized to -π to π
    - Servo center (2048 ticks) = 0 radians
    - No sign inversions
    """
    
    def __init__(self, port: str, limits: List[Tuple[float, float]] = None, follower: bool = True):
        """
        Initialize the SO-101 client in raw mode.

        Args:
            port (str): The port to connect to the SO-101 via USB.
            limits (List[Tuple[float, float]]): The pairs of upper and lower limits of the joints.
            follower (bool): Whether to use follower or leader config.
        """
        self.follower = follower
        if self.follower:
            print("Using SO-101 follower config (raw mode)")
            self.cfg = SO101FollowerConfig(port=port)
            self.cfg.use_degrees = True
            self.robot = make_robot_from_config(self.cfg)
        else:
            print("Using SO-101 leader config (raw mode)")
            self.cfg = SO101LeaderConfig(port=port)
            self.cfg.use_degrees = True
            self.robot = make_teleoperator_from_config(self.cfg)
        
        # Connect without calibration
        self.robot.connect(calibrate=False)
        print("✅ Connected in raw mode (no calibration)")
        
        self.limits = limits
        if self.limits:
            print(f"Limits: {self.limits}")
    

    def write_joints(self, joints: List[float]):
        """
        Send joint command in radians to the robot (raw coordinates).
        
        Maps raw radians (-π to π) to robot's expected format.
        This matches the JointStateReader coordinate system.
        
        Args:
            joints (List[float]): List of joint angles in radians (-π to π).
        """
        if len(joints) != 6:
            raise ValueError(f"Expected 6 joint values, got {len(joints)}: {joints}")

        if self.limits is not None:
            for i in range(len(joints)):
                assert joints[i] >= self.limits[i][0] and joints[i] <= self.limits[i][1], \
                    f"Joint {i} is out of bounds: {joints[i]} for limits: {self.limits[i]}"

        # Convert raw radians to degrees (no offsets, no sign inversion)
        # This matches how JointStateReader would interpret the positions
        degrees = np.degrees(joints)
        
        # Build the action dictionary
        action = {
            "shoulder_pan.pos": degrees[0],
            "shoulder_lift.pos": degrees[1],
            "elbow_flex.pos": degrees[2],
            "wrist_flex.pos": degrees[3],
            "wrist_roll.pos": degrees[4],
            "gripper.pos": degrees[5],
        }

        self.robot.send_action(action)
        
    def read_joints(self) -> List[float]:
        """
        Read the joint angles from the robot in radians (raw coordinates).
        
        Returns positions in the same coordinate system as JointStateReader:
        - Raw servo positions normalized to -π to π
        - No calibration offsets
        - No sign inversions
        
        Returns:
            List[float]: List of joint angles in radians (-π to π).
        """
        if self.follower:
            observation = self.robot.get_observation()
        else:
            observation = self.robot.get_action()
        
        # Get positions in degrees
        degrees = np.array([
            observation['shoulder_pan.pos'], 
            observation['shoulder_lift.pos'], 
            observation['elbow_flex.pos'], 
            observation['wrist_flex.pos'], 
            observation['wrist_roll.pos'], 
            observation['gripper.pos']
        ])
        
        # Convert to radians (no offsets, no sign inversion)
        # This gives us raw positions matching JointStateReader
        radians = np.radians(degrees)
        
        return radians.tolist()

    def interpolate_waypoint(self, waypoint1, waypoint2, steps=50, timestep=0.02):
        """
        Interpolate between two waypoints.
        
        Args:
            waypoint1: Starting joint positions (radians)
            waypoint2: Target joint positions (radians)
            steps: Number of interpolation steps
            timestep: Time between steps in seconds
        """
        for i in range(steps):
            alpha = i / (steps - 1)  # Goes from 0 to 1
            q = [(1 - alpha) * w1 + alpha * w2 for w1, w2 in zip(waypoint1, waypoint2)]
            self.write_joints(q)
            time.sleep(timestep)
    
    def disconnect(self):
        """Safely disconnect the robot."""
        if hasattr(self, 'robot') and self.robot.is_connected:
            self.robot.disconnect()
    
    def __del__(self):
        try:
            self.disconnect()
        except:
            pass  # Ignore errors during cleanup


if __name__ == "__main__":
    # Test the raw client
    client = SO101ClientRaw(port="/dev/ttyACM0")
    
    print("Testing raw coordinate system...")
    
    # Test neutral position (should be all zeros in raw mode)
    neutral = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    print(f"Moving to neutral position: {neutral}")
    client.write_joints(neutral)
    time.sleep(1)
    
    # Read and display position
    pos = client.read_joints()
    print(f"Current position (raw): {[f'{p:.3f}' for p in pos]}")
    
    client.disconnect()