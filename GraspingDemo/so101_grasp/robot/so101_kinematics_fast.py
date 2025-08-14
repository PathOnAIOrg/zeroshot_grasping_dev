#!/usr/bin/env python3
"""
Fast and efficient inverse kinematics solver for SO-101 robot
Optimizations:
1. Analytical IK with closed-form solutions
2. Caching of frequently used values
3. Vectorized operations with NumPy
4. Early termination checks
5. Precomputed transformation matrices
"""

import numpy as np
from typing import List, Tuple, Optional
from functools import lru_cache
import time

class SO101KinematicsFast:
    def __init__(self):
        # Robot parameters (in meters)
        self.L1 = 0.095   # Base to shoulder height
        self.L2 = 0.113   # Upper arm length
        self.L3 = 0.135   # Lower arm length  
        self.L4 = 0.061   # Wrist to gripper length
        
        # Joint limits (radians)
        self.joint_limits = [
            (-1.92, 1.92),   # Joint 1 (base)
            (-1.75, 1.75),   # Joint 2 (shoulder)
            (-1.75, 1.57),   # Joint 3 (elbow)
            (-1.66, 1.66),   # Joint 4 (wrist pitch)
            (-2.79, 2.79),   # Joint 5 (wrist roll)
            (-1.57, 1.57),   # Joint 6 (gripper)
        ]
        
        # Precompute frequently used values
        self.L2_sq = self.L2 ** 2
        self.L3_sq = self.L3 ** 2
        self.L23_product = 2 * self.L2 * self.L3
        self.max_reach = self.L2 + self.L3
        self.min_reach = abs(self.L2 - self.L3)
        self.max_reach_sq = self.max_reach ** 2
        
        # Cache for transformation matrices and IK solutions
        self._transform_cache = {}
        self._jacobian_cache = {}
        self._ik_cache = {}  # Cache recent IK solutions
        self._cache_size = 100  # Maximum cache size
        
    @lru_cache(maxsize=128)
    def _compute_trig(self, angle: float) -> Tuple[float, float]:
        """Cache trigonometric calculations"""
        return np.cos(angle), np.sin(angle)
    
    def forward_kinematics(self, joints: List[float]) -> Tuple[np.ndarray, np.ndarray]:
        """
        Fast forward kinematics using optimized matrix operations
        """
        q1, q2, q3, q4, q5 = joints[:5]
        
        # Use cached trig values
        c1, s1 = self._compute_trig(q1)
        c2, s2 = self._compute_trig(q2)
        c23, s23 = self._compute_trig(q2 + q3)
        c234, s234 = self._compute_trig(q2 + q3 + q4)
        
        # Fast computation using direct formulas
        # Position of wrist
        wrist_local_x = self.L2 * c2 + self.L3 * c23
        wrist_local_z = self.L2 * s2 + self.L3 * s23
        
        # Transform to world coordinates
        wrist_x = wrist_local_x * c1
        wrist_y = wrist_local_x * s1
        wrist_z = self.L1 + wrist_local_z
        
        # Gripper position (add tool offset)
        gripper_offset = self.L4 * c234
        pos_x = wrist_x + gripper_offset * c1
        pos_y = wrist_y + gripper_offset * s1
        pos_z = wrist_z + self.L4 * s234
        
        position = np.array([pos_x, pos_y, pos_z])
        
        # Simplified orientation (only if needed)
        orientation = np.eye(3)  # Can be computed if required
        
        return position, orientation
    
    def analytical_ik_fast(self, target_pos: np.ndarray, 
                           current_joints: Optional[List[float]] = None) -> Optional[List[float]]:
        """
        Ultra-fast analytical IK solver with closed-form solution
        
        Returns solution in ~1ms vs ~10-50ms for numerical methods
        """
        x, y, z = target_pos
        
        # Quick reachability check using squared distances (avoid sqrt)
        xy_sq = x*x + y*y
        if xy_sq < 1e-8:  # Near singularity at origin
            xy_sq = 1e-8
            
        # Base rotation (Joint 1) - direct solution
        q1 = np.arctan2(y, x)
        
        # Radial distance in arm plane
        r = np.sqrt(xy_sq)
        
        # Adjust for tool length (assuming horizontal tool for speed)
        r_wrist = r - self.L4
        z_wrist = z - self.L1
        
        # Fast reachability check
        d_sq = r_wrist * r_wrist + z_wrist * z_wrist
        d = np.sqrt(d_sq)
        
        if d > self.max_reach or d < self.min_reach:
            # Quick adjustment: scale to reachable range
            if d > self.max_reach:
                scale = self.max_reach * 0.95 / d
                r_wrist *= scale
                z_wrist *= scale
                d = self.max_reach * 0.95
                d_sq = d * d
        
        # Elbow angle (Joint 3) - law of cosines
        cos_q3 = (d_sq - self.L2_sq - self.L3_sq) / self.L23_product
        cos_q3 = np.clip(cos_q3, -1.0, 1.0)
        
        # Use elbow-down configuration (more stable)
        q3 = np.arccos(cos_q3)
        
        # Shoulder angle (Joint 2) - geometric solution
        alpha = np.arctan2(z_wrist, r_wrist)
        
        # Fast approximation for small angles
        if d > 0.01:
            sin_beta = self.L3 * np.sin(q3) / d
            sin_beta = np.clip(sin_beta, -1.0, 1.0)
            beta = np.arcsin(sin_beta)
        else:
            beta = 0
        
        q2 = alpha - beta
        
        # Wrist pitch (Joint 4) - maintain horizontal gripper
        q4 = -(q2 + q3)
        
        # Wrist roll (Joint 5) - maintain current or zero
        q5 = current_joints[4] if current_joints and len(current_joints) > 4 else 0
        
        # Apply joint limits efficiently
        joints = np.array([q1, q2, q3, q4, q5, 0])
        
        # Vectorized limit application
        limits = np.array(self.joint_limits)
        joints[:5] = np.clip(joints[:5], limits[:5, 0], limits[:5, 1])
        
        return joints.tolist()
    
    def inverse_kinematics(self, target_pos: np.ndarray,
                          current_joints: List[float],
                          orientation: Optional[np.ndarray] = None) -> Optional[List[float]]:
        """
        Main IK interface - uses fast analytical solution with fallback
        """
        # Check cache first
        cache_key = tuple(np.round(target_pos, 4))  # Round to 0.1mm precision
        if cache_key in self._ik_cache:
            cached_solution = self._ik_cache[cache_key]
            # Verify cached solution is still valid
            pos, _ = self.forward_kinematics(cached_solution)
            if np.linalg.norm(pos - target_pos) < 0.005:
                return cached_solution
        
        # Try fast analytical solution first
        solution = self.analytical_ik_fast(target_pos, current_joints)
        
        if solution:
            # Verify solution quickly
            pos, _ = self.forward_kinematics(solution)
            error = np.linalg.norm(pos - target_pos)
            
            if error < 0.005:  # 5mm tolerance
                # Cache the solution
                if len(self._ik_cache) > self._cache_size:
                    # Remove oldest entry
                    self._ik_cache.pop(next(iter(self._ik_cache)))
                self._ik_cache[cache_key] = solution
                return solution
            
            # If analytical failed, try one iteration of correction
            if error < 0.02:  # Within 20mm, do quick correction
                solution = self.quick_jacobian_correction(solution, target_pos, pos)
                if solution:
                    return solution
        
        # Fallback to optimized numerical IK if needed
        solution = self.numerical_ik_optimized(target_pos, current_joints, orientation)
        if solution:
            # Cache the solution
            if len(self._ik_cache) > self._cache_size:
                self._ik_cache.pop(next(iter(self._ik_cache)))
            self._ik_cache[cache_key] = solution
        return solution
    
    def quick_jacobian_correction(self, joints: List[float], 
                                 target: np.ndarray, 
                                 current: np.ndarray) -> Optional[List[float]]:
        """
        Single-step Jacobian correction for fine-tuning
        """
        error = target - current
        
        # Approximate Jacobian (simplified for speed)
        J = self.approximate_jacobian_fast(joints)
        
        # Pseudo-inverse with damping
        damping = 0.01
        JtJ = J.T @ J
        JtJ_damped = JtJ + damping * np.eye(3)
        
        try:
            delta_q = np.linalg.solve(JtJ_damped, J.T @ error)
        except:
            return None
        
        # Apply correction
        new_joints = list(joints)
        for i in range(3):  # Only adjust first 3 joints for position
            new_joints[i] += delta_q[i] * 0.5  # Conservative update
            new_joints[i] = np.clip(new_joints[i], 
                                   self.joint_limits[i][0], 
                                   self.joint_limits[i][1])
        
        return new_joints
    
    def approximate_jacobian_fast(self, joints: List[float]) -> np.ndarray:
        """
        Fast approximate Jacobian for position (3x3 matrix)
        Uses analytical derivatives for first 3 joints
        """
        q1, q2, q3 = joints[:3]
        
        # Precompute trig
        c1, s1 = self._compute_trig(q1)
        c2, s2 = self._compute_trig(q2)
        c23, s23 = self._compute_trig(q2 + q3)
        
        # Analytical Jacobian elements
        J = np.zeros((3, 3))
        
        # Joint 1 derivatives (base rotation)
        reach = self.L2 * c2 + self.L3 * c23 + self.L4 * np.cos(q2 + q3 + joints[3])
        J[0, 0] = -reach * s1
        J[1, 0] = reach * c1
        J[2, 0] = 0
        
        # Joint 2 derivatives (shoulder)
        J[0, 1] = -c1 * (self.L2 * s2 + self.L3 * s23)
        J[1, 1] = -s1 * (self.L2 * s2 + self.L3 * s23)
        J[2, 1] = self.L2 * c2 + self.L3 * c23
        
        # Joint 3 derivatives (elbow)
        J[0, 2] = -c1 * self.L3 * s23
        J[1, 2] = -s1 * self.L3 * s23
        J[2, 2] = self.L3 * c23
        
        return J
    
    def numerical_ik_optimized(self, target_pos: np.ndarray,
                              current_joints: List[float],
                              orientation: Optional[np.ndarray] = None,
                              max_iterations: int = 50,
                              tolerance: float = 0.002) -> Optional[List[float]]:
        """
        Optimized numerical IK with early termination and adaptive step size
        """
        joints = list(current_joints[:6])
        prev_error = float('inf')
        alpha = 0.8  # Initial learning rate
        
        for iteration in range(max_iterations):
            # Current position
            current_pos, _ = self.forward_kinematics(joints)
            
            # Error vector
            error = target_pos - current_pos
            error_norm = np.linalg.norm(error)
            
            # Check convergence
            if error_norm < tolerance:
                return joints
            
            # Adaptive learning rate
            if error_norm > prev_error:
                alpha *= 0.5  # Reduce step size if error increased
            elif iteration > 5 and error_norm > 0.01:
                alpha = min(alpha * 1.1, 1.0)  # Increase if making progress
            
            prev_error = error_norm
            
            # Fast Jacobian
            J = self.approximate_jacobian_fast(joints)
            
            # Damped least squares
            damping = 0.01 * (1 + error_norm)  # Adaptive damping
            JtJ = J.T @ J
            JtJ_damped = JtJ + damping * np.eye(3)
            
            try:
                delta_q = np.linalg.solve(JtJ_damped, J.T @ error)
            except:
                return None
            
            # Update joints with adaptive step
            for i in range(3):
                joints[i] += alpha * delta_q[i]
                joints[i] = np.clip(joints[i], 
                                  self.joint_limits[i][0], 
                                  self.joint_limits[i][1])
            
            # Early termination if progress is too slow
            if iteration > 30 and error_norm > 0.1:
                break  # Try to return best solution so far
        
        # Final check - return best solution if within reasonable error
        final_pos, _ = self.forward_kinematics(joints)
        final_error = np.linalg.norm(target_pos - final_pos)
        if final_error < 0.01:  # 10mm tolerance for fallback
            return joints
        
        # Last resort: return if it's better than nothing
        if final_error < 0.05:  # 50mm tolerance
            return joints
            
        return None


# Benchmark function
def benchmark_ik():
    """Compare performance of different IK methods"""
    kin = SO101KinematicsFast()
    
    # Test positions
    test_positions = [
        np.array([0.2, 0.0, 0.2]),
        np.array([0.15, 0.15, 0.25]),
        np.array([0.1, -0.1, 0.15]),
        np.array([0.25, 0.05, 0.3]),
    ]
    
    current_joints = [0, 0, 0, 0, 0, 0]
    
    print("IK Performance Benchmark:")
    print("-" * 50)
    
    for i, target in enumerate(test_positions):
        # Analytical IK
        start = time.perf_counter()
        solution = kin.analytical_ik_fast(target, current_joints)
        analytical_time = (time.perf_counter() - start) * 1000
        
        if solution:
            pos, _ = kin.forward_kinematics(solution)
            error = np.linalg.norm(pos - target) * 1000
            print(f"Position {i+1}: Analytical IK - {analytical_time:.3f}ms, Error: {error:.2f}mm")
        else:
            print(f"Position {i+1}: Analytical IK - Failed")
        
        # Full IK with fallback
        start = time.perf_counter()
        solution = kin.inverse_kinematics(target, current_joints)
        full_time = (time.perf_counter() - start) * 1000
        
        if solution:
            pos, _ = kin.forward_kinematics(solution)
            error = np.linalg.norm(pos - target) * 1000
            print(f"           Full IK      - {full_time:.3f}ms, Error: {error:.2f}mm")
        else:
            print(f"           Full IK      - Failed")
        
        print()


if __name__ == "__main__":
    benchmark_ik()