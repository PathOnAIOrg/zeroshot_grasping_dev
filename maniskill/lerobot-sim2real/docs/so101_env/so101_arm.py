import os
import sapien
import numpy as np
from mani_skill.agents.base_agent import BaseAgent, Keyframe
from mani_skill.agents.controllers import *
from mani_skill.agents.registration import register_agent


@register_agent()
class MySO101(BaseAgent):
    uid = "my_so101"
    urdf_path = os.path.join(os.path.dirname(__file__), "SO101", "so101_new_calib.urdf")