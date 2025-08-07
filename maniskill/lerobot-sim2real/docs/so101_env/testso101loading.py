import os
import sys

current_dir = os.path.dirname(__file__)
so101_env_path = os.path.join(current_dir, "so101_env")     # compute the relative path to the so101_env directory

if so101_env_path not in sys.path:  # add to sys.path if not already there
    sys.path.append(so101_env_path)

import so101_arm
import mani_skill.examples.demo_robot as demo_robot_script

demo_robot_script.main()