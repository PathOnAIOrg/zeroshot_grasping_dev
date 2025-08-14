#!/usr/bin/env python3
"""
Compare performance between standard and fast IK solvers
"""

import numpy as np
import time
import sys
import os

# Add path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Import both kinematics solvers
from so101_grasp.robot.so101_kinematics import SO101Kinematics
from so101_grasp.robot.so101_kinematics_fast import SO101KinematicsFast


def benchmark_solvers():
    """Compare performance of standard vs fast IK solver"""
    
    # Initialize both solvers
    kin_standard = SO101Kinematics()
    kin_fast = SO101KinematicsFast()
    
    # Test positions covering workspace
    test_positions = [
        # Easy positions (center of workspace)
        np.array([0.2, 0.0, 0.2]),
        np.array([0.15, 0.15, 0.25]),
        np.array([0.18, -0.05, 0.22]),
        
        # Medium difficulty
        np.array([0.25, 0.1, 0.15]),
        np.array([0.1, 0.2, 0.3]),
        np.array([0.22, -0.08, 0.18]),
        
        # Near limits
        np.array([0.3, 0.0, 0.2]),
        np.array([0.05, 0.05, 0.35]),
        np.array([0.28, 0.05, 0.12]),
        
        # Edge cases
        np.array([0.1, 0.0, 0.1]),
    ]
    
    current_joints = [0, 0, 0, 0, 0, 0]
    
    print("\n" + "="*70)
    print("IK SOLVER PERFORMANCE COMPARISON")
    print("="*70)
    
    # Collect statistics
    standard_times = []
    fast_times = []
    standard_errors = []
    fast_errors = []
    standard_success = 0
    fast_success = 0
    
    print("\nTesting {} positions...\n".format(len(test_positions)))
    print("{:<4} {:<35} {:<35}".format("Pos", "Standard IK", "Fast IK"))
    print("-"*70)
    
    for i, target in enumerate(test_positions, 1):
        # Standard IK
        start = time.perf_counter()
        solution_std = kin_standard.inverse_kinematics(target, current_joints)
        std_time = (time.perf_counter() - start) * 1000
        
        if solution_std:
            pos_std, _ = kin_standard.forward_kinematics(solution_std)
            error_std = np.linalg.norm(pos_std - target) * 1000
            standard_times.append(std_time)
            standard_errors.append(error_std)
            standard_success += 1
            std_result = f"{std_time:6.2f}ms, Error: {error_std:5.1f}mm"
        else:
            std_result = "Failed"
        
        # Fast IK
        start = time.perf_counter()
        solution_fast = kin_fast.inverse_kinematics(target, current_joints)
        fast_time = (time.perf_counter() - start) * 1000
        
        if solution_fast:
            pos_fast, _ = kin_fast.forward_kinematics(solution_fast)
            error_fast = np.linalg.norm(pos_fast - target) * 1000
            fast_times.append(fast_time)
            fast_errors.append(error_fast)
            fast_success += 1
            fast_result = f"{fast_time:6.2f}ms, Error: {error_fast:5.1f}mm"
        else:
            fast_result = "Failed"
        
        print("{:<4} {:<35} {:<35}".format(i, std_result, fast_result))
    
    # Statistics
    print("\n" + "="*70)
    print("SUMMARY STATISTICS")
    print("="*70)
    
    print(f"\nSuccess Rate:")
    print(f"  Standard IK: {standard_success}/{len(test_positions)} ({standard_success*100/len(test_positions):.0f}%)")
    print(f"  Fast IK:     {fast_success}/{len(test_positions)} ({fast_success*100/len(test_positions):.0f}%)")
    
    if standard_times and fast_times:
        print(f"\nComputation Time (successful solutions):")
        print(f"  Standard IK: Avg = {np.mean(standard_times):.2f}ms, Max = {np.max(standard_times):.2f}ms")
        print(f"  Fast IK:     Avg = {np.mean(fast_times):.2f}ms, Max = {np.max(fast_times):.2f}ms")
        
        speedup = np.mean(standard_times) / np.mean(fast_times)
        print(f"  Speedup:     {speedup:.1f}x faster")
        
        print(f"\nPosition Error (successful solutions):")
        print(f"  Standard IK: Avg = {np.mean(standard_errors):.1f}mm, Max = {np.max(standard_errors):.1f}mm")
        print(f"  Fast IK:     Avg = {np.mean(fast_errors):.1f}mm, Max = {np.max(fast_errors):.1f}mm")
    
    # Test cache performance
    print("\n" + "="*70)
    print("CACHE PERFORMANCE TEST")
    print("="*70)
    
    test_pos = np.array([0.2, 0.0, 0.2])
    
    # First call (no cache)
    start = time.perf_counter()
    kin_fast.inverse_kinematics(test_pos, current_joints)
    first_call = (time.perf_counter() - start) * 1000
    
    # Second call (cached)
    start = time.perf_counter()
    kin_fast.inverse_kinematics(test_pos, current_joints)
    cached_call = (time.perf_counter() - start) * 1000
    
    print(f"\nSame position called twice:")
    print(f"  First call:  {first_call:.3f}ms")
    print(f"  Cached call: {cached_call:.3f}ms")
    print(f"  Cache speedup: {first_call/cached_call:.0f}x faster")
    
    # Batch processing test
    print("\n" + "="*70)
    print("BATCH PROCESSING TEST (100 random positions)")
    print("="*70)
    
    np.random.seed(42)
    batch_positions = []
    for _ in range(100):
        # Random positions within workspace
        r = np.random.uniform(0.1, 0.25)
        theta = np.random.uniform(-np.pi/2, np.pi/2)
        z = np.random.uniform(0.1, 0.3)
        batch_positions.append(np.array([r*np.cos(theta), r*np.sin(theta), z]))
    
    # Standard IK batch
    start = time.perf_counter()
    for pos in batch_positions:
        kin_standard.inverse_kinematics(pos, current_joints)
    standard_batch_time = (time.perf_counter() - start) * 1000
    
    # Fast IK batch
    start = time.perf_counter()
    for pos in batch_positions:
        kin_fast.inverse_kinematics(pos, current_joints)
    fast_batch_time = (time.perf_counter() - start) * 1000
    
    print(f"\nBatch processing time:")
    print(f"  Standard IK: {standard_batch_time:.1f}ms total, {standard_batch_time/100:.2f}ms per position")
    print(f"  Fast IK:     {fast_batch_time:.1f}ms total, {fast_batch_time/100:.2f}ms per position")
    print(f"  Speedup:     {standard_batch_time/fast_batch_time:.1f}x faster")
    
    print("\n" + "="*70)
    print("CONCLUSION")
    print("="*70)
    print(f"\nThe Fast IK solver is approximately {speedup:.1f}x faster than the standard solver")
    print(f"with comparable accuracy ({np.mean(fast_errors):.1f}mm avg error).")
    print(f"\nRecommended for real-time control applications.")
    print("="*70 + "\n")


if __name__ == "__main__":
    benchmark_solvers()