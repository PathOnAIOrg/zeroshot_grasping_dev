"""Cube-stacking MuJoCo environment (SO-ARM100)."""

from gymnasium.envs.registration import register
from .cube_stacking_env import CubeStackingEnv

# Register once at import time
register(
    id="CubeStacking-v0",                 # any unique name
    entry_point="cube_stacking:CubeStackingEnv",
    max_episode_steps=500,                # same as env default
)
