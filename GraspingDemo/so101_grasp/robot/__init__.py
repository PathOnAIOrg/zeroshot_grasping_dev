"""
Robot Control Module

Handles SO-101 robot communication, control, and calibration.
"""

from .so101_client import SO101Client
from .calibration import RobotCalibrator

__all__ = ["SO101Client", "RobotCalibrator"]