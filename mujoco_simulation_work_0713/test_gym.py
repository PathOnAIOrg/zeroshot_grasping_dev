#!/usr/bin/env mjpython
"""
Headed run of CubeStacking-v0.
• front_view.mp4  – what the MuJoCo window shows (free camera)
• step_XXXX_<obs>.png – every RGB obs frame (topdown_image, wrist_image, …)
• step_XXXX_front.png – optional PNG snapshots of the front view
"""

from pathlib import Path
import imageio.v2 as imageio
from PIL import Image
import numpy as np
import gymnasium as gym
import cube_stacking           # registers env
import mujoco

# ---------------------------------------------------------------------- #
# 0.  Settings
# ---------------------------------------------------------------------- #
N_STEPS        = 200
FRONT_SIZE     = (480, 480)     # (width, height) of the video
FPS            = 25
SAVE_FRONT_PNG = False          # True → individual front_XXXX.png frames

# ---------------------------------------------------------------------- #
# 1.  Output directory & video writer
# ---------------------------------------------------------------------- #
out_dir = Path("images_front")
out_dir.mkdir(exist_ok=True)

front_writer = imageio.get_writer(
    out_dir / "front_view.mp4", fps=FPS, codec="libx264"
)

# ---------------------------------------------------------------------- #
# 2.  Environment with viewer (headed) and off-screen renderer
# ---------------------------------------------------------------------- #
env = gym.make("CubeStacking-v0", render_mode="human", image_size=256)
obs, _ = env.reset(seed=42)

front_renderer = mujoco.Renderer(env.unwrapped.model, *FRONT_SIZE)

# ---------------------------------------------------------------------- #
# 3.  Main simulation loop
# ---------------------------------------------------------------------- #
for step in range(N_STEPS):
    # ---- 3a. save every RGB observation image ------------------------- #
    for key, arr in obs.items():
        if isinstance(arr, np.ndarray) and arr.ndim == 3 and arr.dtype == np.uint8:
            Image.fromarray(arr).save(out_dir / f"{step:04d}_{key}.png")

    # ---- 3b. capture front (free-camera) view ------------------------- #
    front_renderer.update_scene(env.unwrapped.data, camera=-1)  # -1 = free cam
    front_frame = front_renderer.render()
    front_writer.append_data(front_frame)

    if SAVE_FRONT_PNG:
        Image.fromarray(front_frame).save(out_dir / f"{step:04d}_front.png")

    # ---- 3c. advance simulation & sync on-screen viewer --------------- #
    env.render()
    obs, _, terminated, truncated, _ = env.step(env.action_space.sample())
    if terminated or truncated:
        obs, _ = env.reset()

# ---------------------------------------------------------------------- #
# 4.  Cleanup
# ---------------------------------------------------------------------- #
front_writer.close()
try:
    front_renderer.close()  # MuJoCo ≥3.1
except AttributeError:
    pass                    # older versions: resources freed on GC
env.close()

print("All files saved to", out_dir.resolve())
