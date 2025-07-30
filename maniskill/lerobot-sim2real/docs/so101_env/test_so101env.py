# Import required packages
import gymnasium as gym
import mani_skill.envs

import os
import sys

current_dir = os.path.dirname(__file__)
so101_env_path = os.path.join(current_dir, "so101_env")     # compute the relative path to the so101_env directory

if so101_env_path not in sys.path:  # add to sys.path if not already there
    sys.path.append(so101_env_path)
import PickCubeSO101


import time
env = gym.make("PickCubeSO101-v1", render_mode="human")
obs, _ = env.reset(seed=0)
env.unwrapped.print_sim_details() # print verbose details about the configuration
done = False
start_time = time.time()
env.render()
time.sleep(10)

env.close()



# import gymnasium as gym
# import mani_skill.envs

# env = gym.make(
#     "PickCubeSO100-v1", # there are more tasks e.g. "PushCube-v1", "PegInsertionSide-v1", ...
#     num_envs=1,
#     obs_mode="state", # there is also "state_dict", "rgbd", ...
#     # control_mode="pd_ee_delta_pose", # there is also "pd_joint_delta_pos", ...
#     render_mode="human"
# )
# print("Observation space", env.observation_space)
# print("Action space", env.action_space)

# obs, _ = env.reset(seed=0) # reset with a seed for determinism
# done = False
# while not done:
#     action = env.action_space.sample()
#     obs, reward, terminated, truncated, info = env.step(action)
#     done = terminated or truncated
#     env.render()  # a display is required to render
# env.close()