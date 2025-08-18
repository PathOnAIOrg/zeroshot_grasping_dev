"""
Robot Control Module

Core robot control and kinematics implementations.
"""

from .so101_client_raw_simple import SO101ClientRawSimple
from .so101_kinematics import SO101Kinematics

try:
    from .so101_kinematics_fast import SO101KinematicsFast
except ImportError:
    SO101KinematicsFast = SO101Kinematics

__all__ = ['SO101ClientRawSimple', 'SO101Kinematics', 'SO101KinematicsFast']