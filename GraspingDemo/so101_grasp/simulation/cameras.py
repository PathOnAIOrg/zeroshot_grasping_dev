"""Camera configs for PyBullet simulation."""

import numpy as np
import pybullet as p



class RealSenseD415():
    """Default configuration with 3 RealSense RGB-D cameras."""

    image_size = (480, 640)
    intrinsics = np.array([[462.14, 0, 320], [0, 450, 240], [0, 0, 1]])
    
    # Camera poses relative to the manipulator base
    front_position = (0, 0.1, 0.2)
    front_rotation = (np.pi/4, np.pi*4/4, np.pi / 2)
    front_rotation = p.getQuaternionFromEuler(front_rotation)

    left_position = (0, 0.5, 0.75)
    left_rotation = (np.pi / 4.5, np.pi, np.pi / 4)
    left_rotation = p.getQuaternionFromEuler(left_rotation)

    right_position = (0, -0.5, 0.75)
    right_rotation = (np.pi / 4.5, np.pi, 3 * np.pi / 4)
    right_rotation = p.getQuaternionFromEuler(right_rotation)

    # Default camera configs for simulation
    CONFIG = [
        {
            'image_size': image_size,
            'intrinsics': intrinsics,
            'position': front_position,
            'rotation': front_rotation,
            'zrange': (0.01, 10.0),  # Near and far clipping planes
            'noise': False,  # Whether to add noise to the images
            'name': 'front'
        },
        {
            'image_size': image_size,
            'intrinsics': intrinsics,
            'position': left_position,
            'rotation': left_rotation,
            'zrange': (0.01, 10.0),
            'noise': False,
            'name': 'left'
        },
        {
            'image_size': image_size,
            'intrinsics': intrinsics,
            'position': right_position,
            'rotation': right_rotation,
            'zrange': (0.01, 10.0),
            'noise': False,
            'name': 'right'
        }
    ]