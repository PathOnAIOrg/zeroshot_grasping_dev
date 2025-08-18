#!/usr/bin/env python3
"""
SO-101 Robot Kinematics Module
Handles forward and inverse kinematics for cartesian control
"""

import numpy as np
from typing import List, Tuple, Optional


class SO101Kinematics:
    """
    Kinematics calculations for SO-101 6-DOF robot arm.
    Based on DH parameters and geometric relationships.
    """
    
    def __init__(self):
        """Initialize robot parameters and DH table"""
        # Robot link lengths (in meters) - SO-101 actual measurements
        # These need to be calibrated for your specific robot
        self.L1 = 0.095  # Base to shoulder height (95mm)
        self.L2 = 0.105  # Upper arm length (105mm)
        self.L3 = 0.098  # Forearm length (98mm)
        self.L4 = 0.135  # Wrist to gripper length (135mm including gripper)
        
        # Joint limits in radians
        self.joint_limits = [
            (-np.pi, np.pi),      # Joint 1: Base rotation
            (-np.pi/2, np.pi/2),  # Joint 2: Shoulder pitch
            (-np.pi/2, np.pi/2),  # Joint 3: Elbow
            (-np.pi/2, np.pi/2),  # Joint 4: Wrist pitch
            (-np.pi, np.pi),      # Joint 5: Wrist roll
            (-1.57, 1.57)         # Joint 6: Gripper (not used in FK/IK)
        ]
        
    def dh_transform(self, theta, d, a, alpha):
        """
        Calculate DH transformation matrix
        
        Args:
            theta: Joint angle (radians)
            d: Link offset
            a: Link length
            alpha: Link twist
            
        Returns:
            4x4 transformation matrix
        """
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        
        return np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])
    
    def forward_kinematics(self, joints: List[float]) -> Tuple[np.ndarray, np.ndarray]:
        """
        Calculate end-effector position and orientation from joint angles
        Position is returned in world/base_link frame
        
        Args:
            joints: List of 6 joint angles in radians [j1, j2, j3, j4, j5, j6]
                   j6 (gripper) is ignored for position calculation
            
        Returns:
            position: 3D position vector [x, y, z] in meters (world frame)
            orientation: 3x3 rotation matrix (world frame)
        """
        if len(joints) < 5:
            raise ValueError("Need at least 5 joint angles")
        
        # Extract joint angles (ignore gripper)
        q1, q2, q3, q4, q5 = joints[:5]
        
        # Simple geometric forward kinematics for SO-101
        # Base rotation affects X-Y position
        # Shoulder, elbow affect reach and height
        
        # Calculate arm extension in local frame
        # Shoulder angle (q2) and elbow angle (q3) determine reach
        shoulder_angle = q2
        elbow_angle = q3
        
        # Upper arm endpoint
        upper_x = self.L2 * np.cos(shoulder_angle)
        upper_z = self.L2 * np.sin(shoulder_angle)
        
        # Forearm endpoint (relative to upper arm)
        forearm_x = self.L3 * np.cos(shoulder_angle + elbow_angle)
        forearm_z = self.L3 * np.sin(shoulder_angle + elbow_angle)
        
        # Wrist extension
        wrist_angle = shoulder_angle + elbow_angle + q4
        wrist_x = self.L4 * np.cos(wrist_angle)
        wrist_z = self.L4 * np.sin(wrist_angle)
        
        # Total reach in local frame (before base rotation)
        local_reach = upper_x + forearm_x + wrist_x
        local_height = self.L1 + upper_z + forearm_z + wrist_z
        
        # Apply base rotation to get world coordinates
        # q1 rotates around Z axis
        world_x = local_reach * np.cos(q1)
        world_y = local_reach * np.sin(q1)
        world_z = local_height
        
        position = np.array([world_x, world_y, world_z])
        
        # Calculate orientation matrix
        # Base rotation
        R_base = np.array([
            [np.cos(q1), -np.sin(q1), 0],
            [np.sin(q1), np.cos(q1), 0],
            [0, 0, 1]
        ])
        
        # Combined pitch from shoulder, elbow, wrist
        total_pitch = shoulder_angle + elbow_angle + q4
        R_pitch = np.array([
            [np.cos(total_pitch), 0, np.sin(total_pitch)],
            [0, 1, 0],
            [-np.sin(total_pitch), 0, np.cos(total_pitch)]
        ])
        
        # Wrist roll
        R_roll = np.array([
            [1, 0, 0],
            [0, np.cos(q5), -np.sin(q5)],
            [0, np.sin(q5), np.cos(q5)]
        ])
        
        # Combined orientation
        orientation = R_base @ R_pitch @ R_roll
        
        return position, orientation
    
    def inverse_kinematics(self, target_pos: np.ndarray, 
                          current_joints: List[float],
                          orientation: Optional[np.ndarray] = None) -> Optional[List[float]]:
        """
        Calculate joint angles to reach target position
        Uses analytical solution for SO-101 robot
        
        Args:
            target_pos: Target 3D position [x, y, z] in meters
            current_joints: Current joint configuration (for initial guess)
            orientation: Optional target orientation (3x3 matrix)
            
        Returns:
            Joint angles [j1, j2, j3, j4, j5, gripper] or None if unreachable
        """
        x, y, z = target_pos
        
        # Calculate base rotation (Joint 1)
        # Handle case when x and y are both very small
        if abs(x) < 0.001 and abs(y) < 0.001:
            q1 = current_joints[0]  # Keep current base angle
            r = 0.001  # Small value to avoid division by zero
        else:
            q1 = np.arctan2(y, x)
            r = np.sqrt(x**2 + y**2)
        
        # Height from base to target
        z_target = z - self.L1
        
        # For a 3-DOF arm (shoulder, elbow, wrist pitch), we need to solve for the arm plane
        # The wrist position needs to be calculated considering the tool length
        # We want the gripper at the target, so wrist is L4 back from target
        
        # If we have a desired orientation, use it to determine wrist position
        if orientation is not None:
            # Use the z-axis of orientation to determine tool direction
            tool_dir = orientation[:, 2]
            wrist_pos = target_pos - self.L4 * tool_dir
            r_wrist = np.sqrt(wrist_pos[0]**2 + wrist_pos[1]**2)
            z_wrist = wrist_pos[2] - self.L1
        else:
            # Simple approach: assume tool pointing horizontally forward
            # This means wrist is directly behind the target by L4
            r_wrist = r - self.L4
            z_wrist = z_target
        
        # Now solve 2-DOF IK for shoulder and elbow
        # Distance from shoulder to wrist
        d = np.sqrt(r_wrist**2 + z_wrist**2)
        
        # Check reachability
        max_reach = self.L2 + self.L3
        min_reach = abs(self.L2 - self.L3)
        
        if d > max_reach or d < min_reach:
            # Target unreachable, try numerical IK
            return self.numerical_ik(target_pos, current_joints, orientation)
        
        # Calculate elbow angle (Joint 3) using law of cosines
        cos_q3 = (d**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
        cos_q3 = np.clip(cos_q3, -1, 1)
        
        # Two solutions for elbow (elbow up or elbow down)
        # Choose elbow down (positive angle) for more natural motion
        q3 = np.arccos(cos_q3)
        
        # Calculate shoulder angle (Joint 2)
        # Angle from horizontal to line connecting shoulder to wrist
        alpha = np.arctan2(z_wrist, r_wrist)
        
        # Angle from upper arm to line connecting shoulder to wrist
        if d > 0.001:
            beta = np.arcsin(self.L3 * np.sin(q3) / d)
        else:
            beta = 0
        
        q2 = alpha - beta
        
        # Calculate wrist pitch (Joint 4) to maintain desired orientation
        if orientation is not None:
            # Calculate desired pitch from orientation matrix
            desired_pitch = np.arctan2(-orientation[2, 0], 
                                      np.sqrt(orientation[2, 1]**2 + orientation[2, 2]**2))
            q4 = desired_pitch - q2 - q3
        else:
            # Keep gripper roughly horizontal
            q4 = -(q2 + q3)
        
        # Wrist roll (Joint 5) - maintain current or set to zero
        if len(current_joints) >= 5:
            q5 = current_joints[4]  # Maintain current roll
        else:
            q5 = 0
        
        # Keep gripper state
        gripper = current_joints[5] if len(current_joints) > 5 else 0
        
        # Check joint limits
        joints = [q1, q2, q3, q4, q5, gripper]
        
        # Apply joint limits
        for i in range(5):
            joints[i] = np.clip(joints[i], self.joint_limits[i][0], self.joint_limits[i][1])
        
        return joints
    
    def numerical_ik(self, target_pos: np.ndarray,
                    current_joints: List[float],
                    orientation: Optional[np.ndarray] = None,
                    max_iterations: int = 100,
                    tolerance: float = 0.001) -> Optional[List[float]]:
        """
        Numerical inverse kinematics using Jacobian pseudo-inverse method
        
        Args:
            target_pos: Target position
            current_joints: Initial joint configuration
            orientation: Optional target orientation
            max_iterations: Maximum iterations
            tolerance: Position error tolerance in meters
            
        Returns:
            Joint configuration or None if failed
        """
        joints = list(current_joints[:6])
        
        for iteration in range(max_iterations):
            # Current end-effector position
            current_pos, current_rot = self.forward_kinematics(joints)
            
            # Position error
            pos_error = target_pos - current_pos
            
            # Check convergence
            if np.linalg.norm(pos_error) < tolerance:
                return joints
            
            # Calculate Jacobian
            J = self.calculate_jacobian(joints)
            
            # Use only position part of Jacobian (3x5)
            J_pos = J[:3, :5]
            
            # Pseudo-inverse
            J_pinv = np.linalg.pinv(J_pos)
            
            # Joint update
            delta_q = J_pinv @ pos_error
            
            # Apply update with damping
            alpha = 0.5  # Learning rate
            for i in range(5):
                joints[i] += alpha * delta_q[i]
                # Enforce joint limits
                joints[i] = np.clip(joints[i], self.joint_limits[i][0], self.joint_limits[i][1])
        
        # Check final error
        final_pos, _ = self.forward_kinematics(joints)
        if np.linalg.norm(target_pos - final_pos) < tolerance * 10:
            return joints
        
        return None  # Failed to converge
    
    def calculate_jacobian(self, joints: List[float], delta: float = 0.0001) -> np.ndarray:
        """
        Calculate Jacobian matrix using numerical differentiation
        
        Args:
            joints: Current joint configuration
            delta: Small angle for numerical differentiation
            
        Returns:
            6x5 Jacobian matrix (position and orientation w.r.t first 5 joints)
        """
        J = np.zeros((6, 5))
        
        # Reference position and orientation
        pos0, rot0 = self.forward_kinematics(joints)
        
        # Calculate derivatives for each joint
        for i in range(5):
            # Perturb joint i
            joints_plus = list(joints)
            joints_plus[i] += delta
            
            # Calculate new position
            pos_plus, rot_plus = self.forward_kinematics(joints_plus)
            
            # Position derivatives
            J[:3, i] = (pos_plus - pos0) / delta
            
            # Orientation derivatives (using rotation vector)
            # Simplified: use euler angle rates
            rot_diff = rot_plus @ rot0.T
            angle = np.arccos(np.clip((np.trace(rot_diff) - 1) / 2, -1, 1))
            if abs(angle) > 0.001:
                axis = np.array([
                    rot_diff[2, 1] - rot_diff[1, 2],
                    rot_diff[0, 2] - rot_diff[2, 0],
                    rot_diff[1, 0] - rot_diff[0, 1]
                ])
                axis = axis / (2 * np.sin(angle))
                J[3:6, i] = axis * angle / delta
        
        return J
    
    def cartesian_move(self, current_joints: List[float],
                      direction: str,
                      distance: float) -> Optional[List[float]]:
        """
        Calculate joints for moving in cartesian direction
        All movements are in world/base_link coordinate frame
        
        World frame convention:
        - X: Forward (away from base)
        - Y: Left (when facing forward)
        - Z: Up (vertical)
        
        Args:
            current_joints: Current joint configuration
            direction: Movement direction ('x', 'y', 'z', 'left', 'right', 'forward', 'back', 'up', 'down')
            distance: Distance to move in meters
            
        Returns:
            New joint configuration or None if unreachable
        """
        # Get current position in world frame
        current_pos, current_rot = self.forward_kinematics(current_joints)
        
        # Calculate target position based on direction in world frame
        target_pos = current_pos.copy()
        
        # World frame movements (independent of robot orientation)
        if direction in ['forward', 'front']:
            # Move forward in world X (away from base)
            target_pos[0] += distance
        elif direction in ['back', 'backward']:
            # Move backward in world X (toward base)
            target_pos[0] -= distance
        elif direction == 'left':
            # Move left in world Y
            target_pos[1] += distance
        elif direction == 'right':
            # Move right in world Y
            target_pos[1] -= distance
        elif direction == 'up':
            # Move up in world Z
            target_pos[2] += distance
        elif direction == 'down':
            # Move down in world Z
            target_pos[2] -= distance
        elif direction == 'x':
            # Direct world X movement
            target_pos[0] += distance
        elif direction == 'y':
            # Direct world Y movement
            target_pos[1] += distance
        elif direction == 'z':
            # Direct world Z movement
            target_pos[2] += distance
        else:
            raise ValueError(f"Unknown direction: {direction}")
        
        # Calculate new joint configuration
        return self.inverse_kinematics(target_pos, current_joints)
    
    def rotate_gripper(self, current_joints: List[float],
                       axis: str,
                       angle: float) -> Optional[List[float]]:
        """
        Rotate gripper around specified axis
        
        Args:
            current_joints: Current joint configuration
            axis: Rotation axis ('roll', 'pitch', 'yaw', 'wrist_roll', 'wrist_pitch')
            angle: Rotation angle in radians
            
        Returns:
            New joint configuration or None if unreachable
        """
        joints = list(current_joints)
        
        if axis == 'yaw' or axis == 'base':
            # Rotate base (Joint 1)
            joints[0] += angle
        elif axis == 'wrist_pitch':
            # Rotate wrist pitch (Joint 4)
            joints[3] += angle
        elif axis == 'wrist_roll' or axis == 'roll':
            # Rotate wrist roll (Joint 5)
            joints[4] += angle
        elif axis == 'pitch':
            # Distribute pitch between shoulder and elbow
            # This maintains gripper position while changing orientation
            joints[1] += angle * 0.5
            joints[2] += angle * 0.5
            joints[3] -= angle  # Compensate with wrist to maintain gripper angle
        else:
            raise ValueError(f"Unknown rotation axis: {axis}")
        
        # Apply joint limits
        for i in range(5):
            joints[i] = np.clip(joints[i], self.joint_limits[i][0], self.joint_limits[i][1])
        
        return joints
    
    def get_gripper_pose(self, joints: List[float]) -> dict:
        """
        Get gripper pose in workspace
        
        Args:
            joints: Joint configuration
            
        Returns:
            Dictionary with position, orientation, and gripper state
        """
        pos, rot = self.forward_kinematics(joints)
        
        # Extract euler angles from rotation matrix
        # Using ZYX convention
        sy = np.sqrt(rot[0, 0]**2 + rot[1, 0]**2)
        singular = sy < 1e-6
        
        if not singular:
            roll = np.arctan2(rot[2, 1], rot[2, 2])
            pitch = np.arctan2(-rot[2, 0], sy)
            yaw = np.arctan2(rot[1, 0], rot[0, 0])
        else:
            roll = np.arctan2(-rot[1, 2], rot[1, 1])
            pitch = np.arctan2(-rot[2, 0], sy)
            yaw = 0
        
        gripper_state = "open" if joints[5] > 1.0 else "closed" if joints[5] < -1.0 else "middle"
        
        return {
            'position': {
                'x': float(pos[0]),
                'y': float(pos[1]),
                'z': float(pos[2])
            },
            'orientation': {
                'roll': float(roll),
                'pitch': float(pitch),
                'yaw': float(yaw)
            },
            'gripper': gripper_state,
            'joints': [float(j) for j in joints]
        }


if __name__ == "__main__":
    # Test kinematics
    kin = SO101Kinematics()
    
    # Test forward kinematics
    print("Testing Forward Kinematics:")
    test_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    pos, rot = kin.forward_kinematics(test_joints)
    print(f"Joints: {test_joints}")
    print(f"Position: {pos}")
    print(f"Orientation:\n{rot}")
    
    # Test inverse kinematics
    print("\nTesting Inverse Kinematics:")
    target = np.array([0.2, 0.1, 0.15])
    result = kin.inverse_kinematics(target, test_joints)
    if result:
        print(f"Target: {target}")
        print(f"Solution: {result}")
        # Verify
        verify_pos, _ = kin.forward_kinematics(result)
        print(f"Verification: {verify_pos}")
        print(f"Error: {np.linalg.norm(target - verify_pos):.6f} m")
    
    # Test cartesian move
    print("\nTesting Cartesian Move:")
    move_result = kin.cartesian_move(test_joints, 'left', 0.01)
    if move_result:
        print(f"Move left 1cm: {move_result}")
        new_pos, _ = kin.forward_kinematics(move_result)
        print(f"New position: {new_pos}")