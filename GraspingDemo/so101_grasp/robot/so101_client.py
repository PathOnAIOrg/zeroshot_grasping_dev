from lerobot.robots import make_robot_from_config
from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
from lerobot.teleoperators.so101_leader.config_so101_leader import SO101LeaderConfig
from lerobot.teleoperators.utils import make_teleoperator_from_config
from typing import List, Tuple
import time
import numpy as np
import os
from pathlib import Path

# To account for calibration different between SO-101 URDF and physical SO-101
# Note: These offsets may need to be adjusted for SO-101 specifically
OFFSET = [3.5926674e-02,  1.7880281e+00, -1.5459236e+00, -6.0479313e-01,
  7.6940347e-04,  0]
CALIBRATION_PATH = "config/so101_calibration.json"


class SO101Client:
    def __init__(self, port: str, limits : List[Tuple[float, float]] = None, follower: bool = True, force_calibration: bool = False):
        """
        Initialize the SO-101 client.

        Args:
            port (str): The port to connect to the SO-101 via USB.
            limits (List[Tuple[float, float]]): The pairs of upper and lower limits of the joints.
        """
        self.follower = follower
        if self.follower:
            print("Using SO-101 follower config")
            self.cfg = SO101FollowerConfig(port=port)
            self.cfg.use_degrees = True
            self.robot = make_robot_from_config(self.cfg)
        else:
            print("Using SO-101 leader config")
            self.cfg = SO101LeaderConfig(port=port)
            self.cfg.use_degrees = True
            self.robot = make_teleoperator_from_config(self.cfg)
        self.robot.connect(calibrate=False)
        
        # Handle calibration
        if force_calibration or not os.path.exists(CALIBRATION_PATH):
            print("Robot needs calibration. Please manually move the robot to neutral position.")
            try:
                self.robot.calibrate()
                if not os.path.exists("config"):
                    os.makedirs("config")
                self.robot._save_calibration(Path(CALIBRATION_PATH))
                print(f"Calibration saved to {CALIBRATION_PATH}")
            except Exception as e:
                print(f"Calibration failed: {e}")
                print("Continuing without calibration...")
        else:
            try:
                self.robot._load_calibration(Path(CALIBRATION_PATH))
                print(f"Loaded calibration from {CALIBRATION_PATH}")
            except Exception as e:
                print(f"Failed to load calibration: {e}")
                print("Continuing without calibration...")

        self.limits = limits
        print(f"Limits: {self.limits}")
    

    def write_joints(self, joints : List[float]):
        """Send joint command in radians to the robot.

        The sequence is:
            1. Subtract the calibration *OFFSET* provided at the top of the file
               so that `joints = 0` corresponds to the follower's neutral pose.
            2. Convert the resulting radian values to the [-100, 100] scale.
            3. Dispatch the command dictionary to `self.robot.send_action`.

        Args:
            joints (List[float]): List of joint angles in **radians**.
        """

        if len(joints) != len(OFFSET):
            raise ValueError(
                f"Expected {len(OFFSET)} joint values, got {len(joints)}: {joints}"
            )

        if self.limits is not None:
            for i in range(len(joints)):
                assert joints[i] >= self.limits[i][0] and joints[i] <= self.limits[i][1], f"Joint {i} is out of bounds: {joints[i]} for limits: {self.limits[i]}"

        # 1. Apply offset so that the policy can work in its own reference frame.
        offset_joints = [j - o for j, o in zip(joints, OFFSET)]

        offset_joints[0] = -offset_joints[0]


        normalised = np.degrees(offset_joints)

        # 2. Build the action dictionary.
        action = {
            "shoulder_pan.pos": normalised[0],
            "shoulder_lift.pos": normalised[1],
            "elbow_flex.pos": normalised[2],
            "wrist_flex.pos": normalised[3],
            "wrist_roll.pos": normalised[4],
            "gripper.pos": normalised[5],
        }

        self.robot.send_action(action)
        
    def read_joints(self) -> List[float]:
        """
        Read the joint angles from the robot in radians.

        Returns:
            List[float]: List of joint angles in radians.
        """
        if self.follower:
            observation = self.robot.get_observation()
        else:
            observation = self.robot.get_action()
        res = np.array([observation['shoulder_pan.pos'], 
                observation['shoulder_lift.pos'], 
                observation['elbow_flex.pos'], 
                observation['wrist_flex.pos'], 
                observation['wrist_roll.pos'], 
                observation['gripper.pos']])
        res = np.radians(res)
        offset_joints = [j + o for j, o in zip(res, OFFSET)]
        offset_joints[0] = -offset_joints[0]
        return offset_joints

    def interpolate_waypoint(self, waypoint1, waypoint2, steps=50, timestep=0.02):
        for i in range(steps):
            alpha = i / (steps-1)  # Goes from 0 to 1
            q = [(1-alpha)*w1 + alpha*w2 for w1, w2 in zip(waypoint1, waypoint2)]
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
    client = SO101Client(port="/dev/tty.usbmodem5A680107891")
    action = {
            "shoulder_pan.pos": 0,
            "shoulder_lift.pos": 0,
            "elbow_flex.pos": 0,
            "wrist_flex.pos": 0,
            "wrist_roll.pos": 0,
            "gripper.pos": 0,
        }
        
    client.robot.send_action(action)
    time.sleep(1)
    print(client.robot.get_observation())