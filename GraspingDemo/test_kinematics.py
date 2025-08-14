#!/usr/bin/env python3
"""
Test script for SO-101 kinematics
"""

import numpy as np
import sys
import os

# Add path and import directly
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Import kinematics module directly without going through __init__
import importlib.util
spec = importlib.util.spec_from_file_location("so101_kinematics", 
                                               "so101_grasp/robot/so101_kinematics.py")
so101_kinematics = importlib.util.module_from_spec(spec)
spec.loader.exec_module(so101_kinematics)
SO101Kinematics = so101_kinematics.SO101Kinematics

def test_forward_kinematics():
    """Test forward kinematics with various joint configurations"""
    kin = SO101Kinematics()
    
    print("Testing Forward Kinematics")
    print("=" * 50)
    
    # Test configurations
    test_configs = [
        # Home position
        ([0, 0, 0, 0, 0, 0], "Home position"),
        # Base rotation only
        ([np.pi/4, 0, 0, 0, 0, 0], "Base rotated 45°"),
        # Shoulder pitched
        ([0, np.pi/6, 0, 0, 0, 0], "Shoulder pitched 30°"),
        # Elbow bent
        ([0, np.pi/6, np.pi/4, 0, 0, 0], "Elbow bent"),
        # Full configuration
        ([np.pi/4, np.pi/6, np.pi/4, -np.pi/6, np.pi/8, 0], "Full configuration"),
    ]
    
    for joints, description in test_configs:
        pos, rot = kin.forward_kinematics(joints)
        print(f"\n{description}:")
        print(f"  Joints: {[f'{np.degrees(j):.1f}°' for j in joints[:5]]}")
        print(f"  Position (mm): [{pos[0]*1000:.1f}, {pos[1]*1000:.1f}, {pos[2]*1000:.1f}]")
        print(f"  Distance from base: {np.linalg.norm(pos)*1000:.1f}mm")

def test_inverse_kinematics():
    """Test inverse kinematics"""
    kin = SO101Kinematics()
    
    print("\n\nTesting Inverse Kinematics")
    print("=" * 50)
    
    # Test target positions (in meters)
    test_targets = [
        (np.array([0.2, 0.0, 0.2]), "Forward reach"),
        (np.array([0.15, 0.15, 0.2]), "Forward-left"),
        (np.array([0.1, 0.0, 0.3]), "High position"),
        (np.array([0.25, 0.0, 0.15]), "Low forward"),
    ]
    
    current_joints = [0, 0, 0, 0, 0, 0]
    
    for target_pos, description in test_targets:
        print(f"\n{description}:")
        print(f"  Target (mm): [{target_pos[0]*1000:.1f}, {target_pos[1]*1000:.1f}, {target_pos[2]*1000:.1f}]")
        
        # Calculate IK
        new_joints = kin.inverse_kinematics(target_pos, current_joints)
        
        if new_joints is not None:
            # Verify with forward kinematics
            actual_pos, _ = kin.forward_kinematics(new_joints)
            error = np.linalg.norm(actual_pos - target_pos) * 1000
            
            print(f"  Solution found!")
            print(f"  Joints: {[f'{np.degrees(j):.1f}°' for j in new_joints[:5]]}")
            print(f"  Actual pos (mm): [{actual_pos[0]*1000:.1f}, {actual_pos[1]*1000:.1f}, {actual_pos[2]*1000:.1f}]")
            print(f"  Error: {error:.2f}mm")
        else:
            print(f"  No solution found (unreachable)")

def test_cartesian_moves():
    """Test Cartesian movement directions"""
    kin = SO101Kinematics()
    
    print("\n\nTesting Cartesian Moves")
    print("=" * 50)
    
    # Start from a nominal position
    start_joints = [0, np.pi/6, np.pi/6, 0, 0, 0]
    start_pos, start_rot = kin.forward_kinematics(start_joints)
    
    print(f"Starting position (mm): [{start_pos[0]*1000:.1f}, {start_pos[1]*1000:.1f}, {start_pos[2]*1000:.1f}]")
    
    # Test moves in each direction
    moves = [
        ("X+", np.array([0.01, 0, 0])),  # 10mm forward
        ("X-", np.array([-0.01, 0, 0])), # 10mm backward
        ("Y+", np.array([0, 0.01, 0])),  # 10mm left
        ("Y-", np.array([0, -0.01, 0])), # 10mm right
        ("Z+", np.array([0, 0, 0.01])),  # 10mm up
        ("Z-", np.array([0, 0, -0.01])), # 10mm down
    ]
    
    for direction, delta in moves:
        target_pos = start_pos + delta
        new_joints = kin.inverse_kinematics(target_pos, start_joints, start_rot)
        
        if new_joints is not None:
            actual_pos, _ = kin.forward_kinematics(new_joints)
            actual_delta = actual_pos - start_pos
            print(f"\n{direction} move (10mm):")
            print(f"  Target delta: {delta*1000} mm")
            print(f"  Actual delta: [{actual_delta[0]*1000:.2f}, {actual_delta[1]*1000:.2f}, {actual_delta[2]*1000:.2f}] mm")
            print(f"  Joint change: {[f'{np.degrees(new_joints[i]-start_joints[i]):.1f}°' for i in range(5)]}")
        else:
            print(f"\n{direction} move: Unreachable")

if __name__ == "__main__":
    print("SO-101 Kinematics Test")
    print("=" * 70)
    
    test_forward_kinematics()
    test_inverse_kinematics()
    test_cartesian_moves()
    
    print("\n" + "=" * 70)
    print("Tests completed!")