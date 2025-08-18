"""
SO-101 Robot Control Core Module

Provides essential robot control and kinematics for the SO-101 robotic arm.
"""

__version__ = "1.0.0"

# Import core modules
from . import robot
from . import utils

__all__ = ["robot", "utils"]