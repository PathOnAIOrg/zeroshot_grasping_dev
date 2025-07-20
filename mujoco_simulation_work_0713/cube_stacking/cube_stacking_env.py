"""
Cube Stacking Gym Environment for SO-ARM100 Robot

This environment challenges the robot to pick up the red cube and stack it on top of the blue cube.
Observations include state vector and two camera views (top-down and wrist).
"""

import gymnasium as gym
from gymnasium import spaces
import numpy as np
import mujoco
import mujoco.viewer
from typing import Optional, Tuple, Dict, Any
import cv2
from pathlib import Path


class CubeStackingEnv(gym.Env):
    """
    Gym environment for cube stacking task with SO-ARM100 robot.
    
    Goal: Stack the red cube on top of the blue cube.
    
    Observation Space:
        - State vector (26 dim): 
          - Robot joint positions (6)
          - Robot joint velocities (6)
          - Red cube position (3) + quaternion (4)
          - Blue cube position (3) + quaternion (4)
        - Top-down camera image (84x84x3)
        - Wrist camera image (84x84x3)
    
    Action Space:
        - 6 continuous actions for robot joint targets
    
    Success Criteria:
        - Red cube is on top of blue cube (within tolerance)
        - Both cubes are stable (low velocity)
    """
    
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 30}
    
    def __init__(
        self, 
        xml_path: str | None = None,
        render_mode: Optional[str] = None,
        image_size: int = 256,
        control_dt: float = 0.02,
        max_episode_steps: int = 500,
        success_threshold: float = 0.03,  # 3cm tolerance for stacking
        stability_threshold: float = 0.1,  # velocity threshold for stability
    ):
        super().__init__()
        
        self.xml_path = xml_path
        self.render_mode = render_mode
        self.image_size = image_size
        self.control_dt = control_dt
        self.max_episode_steps = max_episode_steps
        self.success_threshold = success_threshold
        self.stability_threshold = stability_threshold
        
        # Load MuJoCo model
        package_root = Path(__file__).parent                    # …/cube_stacking
        if xml_path is None:
            xml_file = package_root / "trs_so_arm100" / "scene.xml"
        else:
            xml_path = Path(xml_path)
            xml_file = xml_path if xml_path.is_absolute() else package_root / xml_path
        # ------------------------------------------------------------------

        # Load MuJoCo model & data
        self.model = mujoco.MjModel.from_xml_path(str(xml_file))
        self.data = mujoco.MjData(self.model)
        
        # Get camera IDs
        self.topdown_cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "topdown")
        self.wrist_cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "wrist")
        
        # Get body IDs for cubes
        # self.red_cube_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "red_cube")
        # self.blue_cube_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "blue_cube")
        self.red_cube_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "red_cube_geom")
        self.blue_cube_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "blue_cube_geom")
        
        # Define joint limits for the robot (6 joints)
        self.joint_limits = np.array([
            [-2.2, 2.2],         # Rotation
            [-3.14158, 0.2],     # Pitch
            [0.0, 3.14158],      # Elbow
            [-2.0, 1.8],         # Wrist_Pitch
            [-3.14158, 3.14158], # Wrist_Roll
            [-0.2, 2.0]          # Jaw
        ])
        
        # Define observation space
        # State vector: 26 dimensions
        state_low = np.concatenate([
            self.joint_limits[:, 0],  # joint positions (6)
            np.full(6, -10.0),        # joint velocities (6)
            np.full(3, -1.0),         # red cube position (3)
            np.full(4, -1.0),         # red cube quaternion (4)
            np.full(3, -1.0),         # blue cube position (3)
            np.full(4, -1.0),         # blue cube quaternion (4)
        ])
        
        state_high = np.concatenate([
            self.joint_limits[:, 1],  # joint positions (6)
            np.full(6, 10.0),         # joint velocities (6)
            np.full(3, 1.0),          # red cube position (3)
            np.full(4, 1.0),          # red cube quaternion (4)
            np.full(3, 1.0),          # blue cube position (3)
            np.full(4, 1.0),          # blue cube quaternion (4)
        ])
        
        self.observation_space = spaces.Dict({
            "state": spaces.Box(low=state_low, high=state_high, dtype=np.float32),
            "topdown_image": spaces.Box(low=0, high=255, shape=(image_size, image_size, 3), dtype=np.uint8),
            "wrist_image": spaces.Box(low=0, high=255, shape=(image_size, image_size, 3), dtype=np.uint8),
        })
        
        # Define action space (6 joint targets)
        self.action_space = spaces.Box(
            low=self.joint_limits[:, 0],
            high=self.joint_limits[:, 1],
            dtype=np.float32
        )
        
        # Initialize viewer if needed
        self.viewer = None
        self._viewers = {}
        
        # Episode tracking
        self.episode_steps = 0
        self.previous_distance = None
        
    def reset(self, seed: Optional[int] = None, options: Optional[Dict] = None) -> Tuple[Dict, Dict]:
        """Reset the environment to initial state."""
        super().reset(seed=seed)
        
        # Reset simulation
        mujoco.mj_resetData(self.model, self.data)
        
        # Set initial robot position (home position or random)
        if options and options.get("random_init", False):
            # Random initial joint positions
            for i in range(6):
                low, high = self.joint_limits[i]
                self.data.qpos[i] = self.np_random.uniform(low, high)
        else:
            # Home position
            self.data.qpos[0] = 0.0      # Rotation
            self.data.qpos[1] = -1.0     # Pitch
            self.data.qpos[2] = 1.5      # Elbow
            self.data.qpos[3] = 0.5      # Wrist_Pitch
            self.data.qpos[4] = 0.0      # Wrist_Roll
            self.data.qpos[5] = 0.0      # Jaw (open)
        
        # Randomize cube positions slightly
        if options and options.get("randomize_cubes", True):
            # Red cube position
            red_cube_qpos_addr = self.model.jnt_qposadr[self.model.body_jntadr[self.red_cube_body_id]]
            self.data.qpos[red_cube_qpos_addr:red_cube_qpos_addr+3] = [
                0.15 + self.np_random.uniform(-0.05, 0.05),
                -0.15 + self.np_random.uniform(-0.05, 0.05),
                0.025
            ]
            self.data.qpos[red_cube_qpos_addr+3:red_cube_qpos_addr+7] = [1, 0, 0, 0]  # quaternion
            
            # Blue cube position
            blue_cube_qpos_addr = self.model.jnt_qposadr[self.model.body_jntadr[self.blue_cube_body_id]]
            self.data.qpos[blue_cube_qpos_addr:blue_cube_qpos_addr+3] = [
                0.15 + self.np_random.uniform(-0.05, 0.05),
                -0.25 + self.np_random.uniform(-0.05, 0.05),
                0.025
            ]
            self.data.qpos[blue_cube_qpos_addr+3:blue_cube_qpos_addr+7] = [1, 0, 0, 0]  # quaternion
        
        # Forward simulation to compute derived quantities
        mujoco.mj_forward(self.model, self.data)
        
        # Reset episode tracking
        self.episode_steps = 0
        self.previous_distance = self._compute_cube_distance()
        
        observation = self._get_observation()
        info = self._get_info()
        
        return observation, info
    
    def step(self, action: np.ndarray) -> Tuple[Dict, float, bool, bool, Dict]:
        """Execute one timestep of the environment."""
        # Clip actions to joint limits
        action = np.clip(action, self.joint_limits[:, 0], self.joint_limits[:, 1])
        
        # Apply PD control
        self._apply_pd_control(action)
        
        # Step simulation
        mujoco.mj_step(self.model, self.data)
        
        # Increment step counter
        self.episode_steps += 1
        
        # Get observation
        observation = self._get_observation()
        
        # Calculate reward
        reward, success = self._calculate_reward()
        
        # Check termination conditions
        terminated = success  # Episode ends when successful
        truncated = self.episode_steps >= self.max_episode_steps
        
        # Get info
        info = self._get_info()
        info["success"] = success
        info["episode_steps"] = self.episode_steps
        
        return observation, reward, terminated, truncated, info
    
    def _apply_pd_control(self, target_positions: np.ndarray):
        """Apply PD control to reach target joint positions."""
        kp = 50.0  # Proportional gain
        kd = 5.0   # Derivative gain
        
        # Current positions and velocities (only robot joints)
        current_pos = self.data.qpos[:6]
        current_vel = self.data.qvel[:6]
        
        # PD control
        position_error = target_positions - current_pos
        velocity_error = -current_vel
        
        control_torques = kp * position_error + kd * velocity_error
        
        # Apply control
        self.data.ctrl[:] = control_torques
    
    def _get_observation(self) -> Dict[str, np.ndarray]:
        """Get current observation."""
        # State vector
        state = np.concatenate([
            self.data.qpos[:6],  # Robot joint positions
            self.data.qvel[:6],  # Robot joint velocities
            self.data.xpos[self.red_cube_body_id],  # Red cube position
            self.data.xquat[self.red_cube_body_id], # Red cube quaternion
            self.data.xpos[self.blue_cube_body_id], # Blue cube position
            self.data.xquat[self.blue_cube_body_id], # Blue cube quaternion
        ])
        
        # Camera images
        topdown_image = self._render_camera(self.topdown_cam_id)
        wrist_image = self._render_camera(self.wrist_cam_id)
        
        return {
            "state": state.astype(np.float32),
            "topdown_image": topdown_image,
            "wrist_image": wrist_image,
        }
    
    def _render_camera(self, camera_id: int) -> np.ndarray:
        """Render image from specified camera."""
        # Create renderer if not exists
        if camera_id not in self._viewers:
            self._viewers[camera_id] = mujoco.Renderer(self.model, height=self.image_size, width=self.image_size)
        
        renderer = self._viewers[camera_id]
        renderer.update_scene(self.data, camera=camera_id)
        pixels = renderer.render()
        
        return pixels
    
    def _calculate_reward(self) -> Tuple[float, bool]:
        """Calculate reward and check success."""
        # Get cube positions
        red_pos = self.data.xpos[self.red_cube_body_id]
        blue_pos = self.data.xpos[self.blue_cube_body_id]
        
        # Calculate distance between cubes
        horizontal_distance = np.linalg.norm(red_pos[:2] - blue_pos[:2])
        vertical_distance = red_pos[2] - blue_pos[2]
        
        # Target vertical distance (red should be 5cm above blue center)
        target_vertical_distance = 0.05  # 2.5cm + 2.5cm (half heights of cubes)
        
        # Check if stacked successfully
        is_stacked = (
            horizontal_distance < self.success_threshold and
            abs(vertical_distance - target_vertical_distance) < 0.01
        )
        
        # Check stability (low velocities)
        red_cube_vel_addr = self.model.jnt_dofadr[self.model.body_jntadr[self.red_cube_body_id]]
        blue_cube_vel_addr = self.model.jnt_dofadr[self.model.body_jntadr[self.blue_cube_body_id]]
        
        red_vel = np.linalg.norm(self.data.qvel[red_cube_vel_addr:red_cube_vel_addr+6])
        blue_vel = np.linalg.norm(self.data.qvel[blue_cube_vel_addr:blue_cube_vel_addr+6])
        
        is_stable = red_vel < self.stability_threshold and blue_vel < self.stability_threshold
        
        # Success when stacked and stable
        success = is_stacked and is_stable
        
        # Reward components
        reward = 0.0
        
        # Distance reward (encourage getting cubes closer)
        distance_reward = -horizontal_distance
        reward += distance_reward
        
        # Height reward (encourage lifting red cube)
        if horizontal_distance < 0.1:  # Only when horizontally close
            height_reward = -abs(vertical_distance - target_vertical_distance) * 10
            reward += height_reward
        
        # Success bonus
        if success:
            reward += 100.0
        
        # Penalty for dropping cubes
        if red_pos[2] < 0.01 or blue_pos[2] < 0.01:
            reward -= 10.0
        
        # Small penalty for time
        reward -= 0.01
        
        return reward, success
    
    def _compute_cube_distance(self) -> float:
        """Compute distance between red and blue cubes."""
        red_pos = self.data.xpos[self.red_cube_body_id]
        blue_pos = self.data.xpos[self.blue_cube_body_id]
        return np.linalg.norm(red_pos - blue_pos)
    
    def _get_info(self) -> Dict[str, Any]:
        """Get additional information."""
        red_pos = self.data.xpos[self.red_cube_body_id]
        blue_pos = self.data.xpos[self.blue_cube_body_id]
        
        return {
            "red_cube_pos": red_pos.copy(),
            "blue_cube_pos": blue_pos.copy(),
            "cube_distance": self._compute_cube_distance(),
            "gripper_pos": self.data.xpos[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "Fixed_Jaw")],
        }
    
    def render(self):
        """Render the environment."""
        if self.render_mode == "human":
            if self.viewer is None:
                self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.viewer.sync()
        elif self.render_mode == "rgb_array":
            return self._render_camera(self.topdown_cam_id)
        
    def close(self):
        """Clean up resources."""
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None
        for viewer in self._viewers.values():
            viewer.close()
        self._viewers.clear()


# Example usage
if __name__ == "__main__":
    # Create environment
    env = CubeStackingEnv(render_mode="human")
    
    # Test random actions
    observation, info = env.reset()
    print(f"Initial observation shapes:")
    print(f"  State: {observation['state'].shape}")
    print(f"  Top-down image: {observation['topdown_image'].shape}")
    print(f"  Wrist image: {observation['wrist_image'].shape}")
    
    for _ in range(1000):
        # Random action
        action = env.action_space.sample()
        
        # Step environment
        observation, reward, terminated, truncated, info = env.step(action)
        
        # Render
        env.render()
        
        # Print progress
        if info.get("success", False):
            print("Success! Red cube stacked on blue cube!")
            break
        
        if terminated or truncated:
            observation, info = env.reset()
            print("Episode reset")
    
    env.close()