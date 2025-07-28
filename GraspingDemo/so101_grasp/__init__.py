"""
SO-101 Robot Grasping System

A comprehensive system for autonomous object grasping using the SO-101 robotic arm.
Integrates computer vision, motion planning, and robot control for intelligent manipulation.
"""

__version__ = "1.0.0"
__author__ = "SO-101 Grasping Team"

from . import robot
from . import vision
from . import api
from . import control
from . import visualization
from . import utils
from . import simulation

__all__ = [
    "robot",
    "vision", 
    "api",
    "control",
    "visualization",
    "utils",
    "simulation"
]