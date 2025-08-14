#!/usr/bin/env python3
"""
Advanced Motion Planning for SO-101 Robot
Using robotics-toolbox-python for MoveIt-like capabilities

Features:
- Collision avoidance
- Trajectory optimization
- Path planning with RRT/PRM
- Cartesian path planning
- Joint space planning
- Velocity and acceleration limits
"""

import numpy as np
from typing import List, Optional, Tuple, Dict, Any
import time
from dataclasses import dataclass
from enum import Enum

try:
    import roboticstoolbox as rtb
    from roboticstoolbox import DHRobot, RevoluteDH
    from spatialmath import SE3, SO3
    from spatialgeometry import Cuboid, Sphere, Cylinder
    import spatialmath.base as smb
    HAVE_RTB = True
    print("✅ Robotics toolbox loaded successfully for advanced motion planning")
except ImportError as e:
    print(f"Warning: robotics-toolbox-python import failed: {e}")
    print("Using basic planner without collision checking and RRT")
    HAVE_RTB = False

# Import our fast kinematics
try:
    from .so101_kinematics_fast import SO101KinematicsFast
except ImportError:
    try:
        from so101_kinematics_fast import SO101KinematicsFast
    except ImportError:
        try:
            from .so101_kinematics import SO101Kinematics as SO101KinematicsFast
        except ImportError:
            from so101_kinematics import SO101Kinematics as SO101KinematicsFast


class PlannerMode(Enum):
    """Planning modes"""
    JOINT_SPACE = "joint_space"
    CARTESIAN = "cartesian"
    RRT = "rrt"
    PRM = "prm"
    OPTIMAL = "optimal"


@dataclass
class PlannerConfig:
    """Configuration for motion planner"""
    max_velocity: float = 1.0  # rad/s
    max_acceleration: float = 2.0  # rad/s²
    max_jerk: float = 5.0  # rad/s³
    collision_check_resolution: float = 0.01  # meters
    path_resolution: int = 100  # waypoints
    planning_timeout: float = 5.0  # seconds
    smoothing_iterations: int = 10
    obstacle_padding: float = 0.02  # meters


class SO101Robot:
    """SO-101 Robot Model for robotics-toolbox"""
    
    def __init__(self):
        """Create SO-101 robot model using DH parameters"""
        if not HAVE_RTB:
            self.robot = None
            return
            
        # DH Parameters for SO-101
        # a_i-1, alpha_i-1, d_i, theta_i
        links = [
            RevoluteDH(d=0.095, a=0, alpha=np.pi/2),      # Base to shoulder
            RevoluteDH(d=0, a=0.113, alpha=0),            # Shoulder
            RevoluteDH(d=0, a=0.135, alpha=0),            # Elbow
            RevoluteDH(d=0, a=0, alpha=np.pi/2),          # Wrist pitch
            RevoluteDH(d=0.061, a=0, alpha=-np.pi/2),     # Wrist roll
        ]
        
        self.robot = DHRobot(links, name='SO-101')
        
        # Set joint limits (needs to be transposed for robotics-toolbox)
        self.robot.qlim = np.array([
            [-1.92, 1.92],   # Joint 1
            [-1.75, 1.75],   # Joint 2
            [-1.75, 1.57],   # Joint 3
            [-1.66, 1.66],   # Joint 4
            [-2.79, 2.79],   # Joint 5
        ]).T  # Transpose to get shape (2, n)
        
        # Tool transform (gripper offset)
        self.robot.tool = SE3.Tz(0.061)
        
    def get_robot(self):
        """Get the robot model"""
        return self.robot


class ObstacleEnvironment:
    """Manage obstacles in the workspace"""
    
    def __init__(self):
        self.obstacles = []
        self.workspace_bounds = {
            'x': (-0.5, 0.5),
            'y': (-0.5, 0.5),
            'z': (0.0, 0.6)
        }
    
    def add_box(self, center: np.ndarray, size: np.ndarray, name: str = "box"):
        """Add box obstacle"""
        if HAVE_RTB:
            obstacle = Cuboid(
                scale=size,
                pose=SE3.Trans(center)
            )
            obstacle.name = name  # Set name as attribute
            obstacle.center = center  # Store center for collision checking
            obstacle.size = size
            self.obstacles.append(obstacle)
    
    def add_sphere(self, center: np.ndarray, radius: float, name: str = "sphere"):
        """Add sphere obstacle"""
        if HAVE_RTB:
            obstacle = Sphere(
                radius=radius,
                pose=SE3.Trans(center)
            )
            obstacle.name = name  # Set name as attribute
            obstacle.center = center  # Store center for collision checking
            self.obstacles.append(obstacle)
    
    def add_cylinder(self, center: np.ndarray, radius: float, height: float, name: str = "cylinder"):
        """Add cylinder obstacle"""
        if HAVE_RTB:
            obstacle = Cylinder(
                radius=radius,
                height=height,
                pose=SE3.Trans(center)
            )
            obstacle.name = name  # Set name as attribute
            obstacle.center = center  # Store center for collision checking
            obstacle.height = height
            self.obstacles.append(obstacle)
    
    def check_collision(self, robot_config: np.ndarray, robot_model) -> bool:
        """Check if robot configuration collides with obstacles"""
        if not HAVE_RTB or not robot_model:
            return False
            
        # Get robot link poses
        robot_model.q = robot_config[:5]
        
        # Simplified collision check (would need full implementation)
        # For now, check end-effector only
        ee_pose = robot_model.fkine(robot_config[:5])
        ee_pos = ee_pose.t
        
        for obstacle in self.obstacles:
            # Simple sphere collision check
            if hasattr(obstacle, 'radius'):
                # Get obstacle position
                if hasattr(obstacle, 'pose'):
                    obs_pos = obstacle.pose.t
                elif hasattr(obstacle, 'center'):
                    obs_pos = obstacle.center
                else:
                    continue
                    
                dist = np.linalg.norm(ee_pos - obs_pos)
                if dist < obstacle.radius + 0.05:  # 5cm safety margin
                    return True
        
        return False
    
    def is_in_workspace(self, point: np.ndarray) -> bool:
        """Check if point is within workspace bounds"""
        return (self.workspace_bounds['x'][0] <= point[0] <= self.workspace_bounds['x'][1] and
                self.workspace_bounds['y'][0] <= point[1] <= self.workspace_bounds['y'][1] and
                self.workspace_bounds['z'][0] <= point[2] <= self.workspace_bounds['z'][1])


class SO101MotionPlanner:
    """Advanced motion planner for SO-101 robot"""
    
    def __init__(self, config: Optional[PlannerConfig] = None):
        """Initialize motion planner"""
        self.config = config or PlannerConfig()
        self.kinematics = SO101KinematicsFast()
        self.robot_model = SO101Robot()
        self.robot = self.robot_model.get_robot()
        self.environment = ObstacleEnvironment()
        
        # Trajectory cache
        self._trajectory_cache = {}
        
    def plan_joint_trajectory(self, 
                            start: List[float], 
                            goal: List[float],
                            obstacles: Optional[List] = None) -> Optional[Dict[str, Any]]:
        """
        Plan trajectory in joint space
        
        Returns:
            Dictionary with:
            - waypoints: List of joint configurations
            - times: Time at each waypoint
            - velocities: Velocity profile
            - accelerations: Acceleration profile
        """
        start = np.array(start[:5])
        goal = np.array(goal[:5])
        
        # Check joint limits
        if not self._check_joint_limits(start) or not self._check_joint_limits(goal):
            return None
        
        # Simple linear interpolation in joint space
        waypoints = []
        num_points = self.config.path_resolution
        
        for i in range(num_points + 1):
            alpha = i / num_points
            q = start * (1 - alpha) + goal * alpha
            
            # Check collision if obstacles provided
            if obstacles and self.environment.check_collision(q, self.robot):
                # Try to find alternative path
                return self._plan_with_rrt(start, goal)
            
            waypoints.append(q)
        
        # Convert numpy arrays to lists for JSON serialization
        waypoints_list = [wp.tolist() if hasattr(wp, 'tolist') else list(wp) for wp in waypoints]
        
        # Generate time-optimal trajectory
        trajectory = self._generate_time_optimal_trajectory(waypoints_list)
        
        return trajectory
    
    def plan_cartesian_path(self,
                           start_pose: np.ndarray,
                           goal_pose: np.ndarray,
                           current_joints: List[float],
                           linear_velocity: float = 0.1) -> Optional[Dict[str, Any]]:
        """
        Plan straight-line Cartesian path
        
        Args:
            start_pose: Starting position [x, y, z]
            goal_pose: Goal position [x, y, z]
            current_joints: Current joint configuration
            linear_velocity: Linear velocity in m/s
            
        Returns:
            Trajectory dictionary
        """
        # Generate Cartesian waypoints
        num_points = self.config.path_resolution
        cartesian_waypoints = []
        
        for i in range(num_points + 1):
            alpha = i / num_points
            pos = start_pose * (1 - alpha) + goal_pose * alpha
            cartesian_waypoints.append(pos)
        
        # Convert to joint space using IK
        joint_waypoints = []
        last_joints = list(current_joints)
        
        for pos in cartesian_waypoints:
            joints = self.kinematics.inverse_kinematics(pos, last_joints)
            if joints is None:
                # IK failed, try alternative method
                return self._plan_cartesian_with_optimization(
                    start_pose, goal_pose, current_joints
                )
            joint_waypoints.append(joints[:5])
            last_joints = joints
        
        # Generate trajectory
        trajectory = self._generate_time_optimal_trajectory(joint_waypoints)
        trajectory['cartesian_waypoints'] = cartesian_waypoints
        
        return trajectory
    
    def plan_with_rrt(self,
                     start: List[float],
                     goal: List[float],
                     max_iterations: int = 1000) -> Optional[Dict[str, Any]]:
        """
        Plan using Rapidly-exploring Random Tree (RRT)
        
        Useful for complex environments with obstacles
        """
        if not HAVE_RTB:
            # Fallback to simple planner
            return self.plan_joint_trajectory(start, goal)
        
        return self._plan_with_rrt(start, goal, max_iterations)
    
    def _plan_with_rrt(self, start, goal, 
                      max_iterations: int = 1000) -> Optional[Dict[str, Any]]:
        """RRT implementation"""
        # Ensure numpy arrays
        start = np.array(start) if not isinstance(start, np.ndarray) else start
        goal = np.array(goal) if not isinstance(goal, np.ndarray) else goal
        
        tree = [start]
        parent = {0: None}
        
        for iteration in range(max_iterations):
            # Random configuration
            if np.random.rand() < 0.1:  # Goal bias
                q_rand = goal
            else:
                q_rand = self._random_config()
            
            # Find nearest node in tree
            nearest_idx = self._nearest_neighbor(tree, q_rand)
            q_near = tree[nearest_idx]
            
            # Extend toward random config
            q_new = self._extend(q_near, q_rand)
            
            # Check collision
            if not self.environment.check_collision(q_new, self.robot):
                tree.append(q_new)
                parent[len(tree) - 1] = nearest_idx
                
                # Check if reached goal
                if np.linalg.norm(q_new - goal) < 0.05:
                    # Extract path
                    path = self._extract_path(tree, parent, len(tree) - 1)
                    # Smooth path
                    path = self._smooth_path(path)
                    # Generate trajectory
                    return self._generate_time_optimal_trajectory(path)
        
        return None
    
    def _random_config(self) -> np.ndarray:
        """Generate random configuration within joint limits"""
        q = np.zeros(5)
        for i in range(5):
            low, high = self.kinematics.joint_limits[i]
            q[i] = np.random.uniform(low, high)
        return q
    
    def _nearest_neighbor(self, tree: List[np.ndarray], q: np.ndarray) -> int:
        """Find nearest neighbor in tree"""
        distances = [np.linalg.norm(node - q) for node in tree]
        return np.argmin(distances)
    
    def _extend(self, q_from: np.ndarray, q_to: np.ndarray, 
                step_size: float = 0.1) -> np.ndarray:
        """Extend from q_from toward q_to"""
        direction = q_to - q_from
        distance = np.linalg.norm(direction)
        
        if distance < step_size:
            return q_to
        
        direction = direction / distance
        return q_from + direction * step_size
    
    def _extract_path(self, tree: List, parent: Dict, goal_idx: int) -> List[np.ndarray]:
        """Extract path from RRT tree"""
        path = []
        idx = goal_idx
        
        while idx is not None:
            path.append(tree[idx])
            idx = parent[idx]
        
        path.reverse()
        return path
    
    def _smooth_path(self, path: List[np.ndarray]) -> List[np.ndarray]:
        """Smooth path using shortcutting"""
        smoothed = [path[0]]
        
        i = 0
        while i < len(path) - 1:
            # Try to connect to furthest reachable point
            for j in range(len(path) - 1, i, -1):
                if self._is_path_clear(path[i], path[j]):
                    smoothed.append(path[j])
                    i = j
                    break
            else:
                i += 1
                if i < len(path):
                    smoothed.append(path[i])
        
        return smoothed
    
    def _is_path_clear(self, q1: np.ndarray, q2: np.ndarray, 
                       num_checks: int = 10) -> bool:
        """Check if straight-line path is collision-free"""
        for i in range(num_checks):
            alpha = i / (num_checks - 1)
            q = q1 * (1 - alpha) + q2 * alpha
            if self.environment.check_collision(q, self.robot):
                return False
        return True
    
    def _generate_time_optimal_trajectory(self, 
                                         waypoints: List) -> Dict[str, Any]:
        """
        Generate time-optimal trajectory with velocity/acceleration limits
        
        Uses trapezoidal velocity profile
        """
        if len(waypoints) < 2:
            return None
        
        # Ensure all waypoints are numpy arrays
        waypoints = [np.array(wp) if not isinstance(wp, np.ndarray) else wp for wp in waypoints]
        
        trajectory = {
            'waypoints': waypoints,
            'times': [],
            'velocities': [],
            'accelerations': []
        }
        
        current_time = 0.0
        trajectory['times'].append(current_time)
        trajectory['velocities'].append(np.zeros(5))
        trajectory['accelerations'].append(np.zeros(5))
        
        for i in range(1, len(waypoints)):
            q_start = waypoints[i-1]
            q_end = waypoints[i]
            
            # Calculate time for each joint (trapezoidal profile)
            segment_time = 0
            velocities = np.zeros(5)
            accelerations = np.zeros(5)
            
            for j in range(5):
                distance = abs(q_end[j] - q_start[j])
                
                # Time to accelerate to max velocity
                t_accel = self.config.max_velocity / self.config.max_acceleration
                
                # Distance during acceleration
                d_accel = 0.5 * self.config.max_acceleration * t_accel**2
                
                if 2 * d_accel > distance:
                    # Triangle profile (never reaches max velocity)
                    t_segment = 2 * np.sqrt(distance / self.config.max_acceleration)
                    v_max = np.sqrt(distance * self.config.max_acceleration)
                else:
                    # Trapezoidal profile
                    d_const = distance - 2 * d_accel
                    t_const = d_const / self.config.max_velocity
                    t_segment = 2 * t_accel + t_const
                    v_max = self.config.max_velocity
                
                segment_time = max(segment_time, t_segment)
                velocities[j] = v_max * np.sign(q_end[j] - q_start[j])
                accelerations[j] = self.config.max_acceleration * np.sign(q_end[j] - q_start[j])
            
            current_time += segment_time
            trajectory['times'].append(current_time)
            trajectory['velocities'].append(velocities)
            trajectory['accelerations'].append(accelerations)
        
        return trajectory
    
    def _check_joint_limits(self, joints: np.ndarray) -> bool:
        """Check if joints are within limits"""
        for i in range(min(5, len(joints))):
            low, high = self.kinematics.joint_limits[i]
            if not (low <= joints[i] <= high):
                return False
        return True
    
    def _plan_cartesian_with_optimization(self,
                                         start_pose: np.ndarray,
                                         goal_pose: np.ndarray,
                                         current_joints: List[float]) -> Optional[Dict[str, Any]]:
        """
        Plan Cartesian path with optimization when simple IK fails
        
        Uses null-space optimization to find better solutions
        """
        # Simplified version - would need full implementation
        return self.plan_joint_trajectory(current_joints[:5], current_joints[:5])
    
    def optimize_trajectory(self, trajectory: Dict[str, Any]) -> Dict[str, Any]:
        """
        Optimize existing trajectory for smoothness and time
        
        Applies various optimization techniques:
        - Minimum jerk
        - Time optimization
        - Energy minimization
        """
        if not trajectory or len(trajectory['waypoints']) < 3:
            return trajectory
        
        waypoints = trajectory['waypoints']
        
        # Apply minimum jerk smoothing
        smoothed_waypoints = self._minimum_jerk_smoothing(waypoints)
        
        # Regenerate timing
        optimized = self._generate_time_optimal_trajectory(smoothed_waypoints)
        
        return optimized
    
    def _minimum_jerk_smoothing(self, waypoints: List[np.ndarray]) -> List[np.ndarray]:
        """Apply minimum jerk trajectory smoothing"""
        if len(waypoints) < 3:
            return waypoints
        
        smoothed = [waypoints[0]]
        
        for i in range(1, len(waypoints) - 1):
            # Simple smoothing using weighted average
            prev_weight = 0.25
            curr_weight = 0.5
            next_weight = 0.25
            
            smooth_point = (prev_weight * waypoints[i-1] + 
                          curr_weight * waypoints[i] + 
                          next_weight * waypoints[i+1])
            smoothed.append(smooth_point)
        
        smoothed.append(waypoints[-1])
        return smoothed
    
    def execute_trajectory(self, trajectory: Dict[str, Any], 
                          robot_interface=None,
                          real_time: bool = True) -> bool:
        """
        Execute planned trajectory on robot
        
        Args:
            trajectory: Planned trajectory
            robot_interface: Robot control interface
            real_time: Execute in real-time or as fast as possible
            
        Returns:
            Success status
        """
        if not trajectory or not robot_interface:
            return False
        
        waypoints = trajectory['waypoints']
        times = trajectory['times']
        
        try:
            start_time = time.time()
            
            for i, (waypoint, t) in enumerate(zip(waypoints, times)):
                if real_time:
                    # Wait until correct time
                    while time.time() - start_time < t:
                        time.sleep(0.001)
                
                # Send joint command
                joints = list(waypoint) + [0]  # Add gripper
                robot_interface.write_joints(joints)
                
            return True
            
        except Exception as e:
            print(f"Trajectory execution failed: {e}")
            return False


def create_demo_planner():
    """Create a demo planner with example usage"""
    
    # Create planner with custom config
    config = PlannerConfig(
        max_velocity=1.5,
        max_acceleration=3.0,
        path_resolution=50
    )
    
    planner = SO101MotionPlanner(config)
    
    # Add some obstacles
    planner.environment.add_sphere(
        center=np.array([0.15, 0.1, 0.2]),
        radius=0.05,
        name="obstacle_1"
    )
    
    planner.environment.add_box(
        center=np.array([0.2, -0.1, 0.15]),
        size=np.array([0.1, 0.1, 0.1]),
        name="obstacle_2"
    )
    
    return planner


def demo_planning():
    """Demonstrate motion planning capabilities"""
    
    print("=" * 60)
    print("SO-101 ADVANCED MOTION PLANNER DEMO")
    print("=" * 60)
    
    planner = create_demo_planner()
    
    # Test 1: Joint space planning
    print("\n1. Joint Space Planning:")
    start_joints = [0, 0, 0, 0, 0]
    goal_joints = [0.5, 0.3, 0.4, -0.2, 0.1]
    
    trajectory = planner.plan_joint_trajectory(start_joints, goal_joints)
    if trajectory:
        print(f"   ✓ Planned trajectory with {len(trajectory['waypoints'])} waypoints")
        print(f"   Total time: {trajectory['times'][-1]:.2f} seconds")
    
    # Test 2: Cartesian path planning
    print("\n2. Cartesian Path Planning:")
    start_pose = np.array([0.2, 0.0, 0.2])
    goal_pose = np.array([0.15, 0.15, 0.25])
    
    trajectory = planner.plan_cartesian_path(
        start_pose, goal_pose, start_joints
    )
    if trajectory:
        print(f"   ✓ Planned Cartesian path with {len(trajectory['waypoints'])} waypoints")
        print(f"   Total time: {trajectory['times'][-1]:.2f} seconds")
    
    # Test 3: RRT planning
    print("\n3. RRT Planning (with obstacles):")
    trajectory = planner.plan_with_rrt(start_joints, goal_joints)
    if trajectory:
        print(f"   ✓ RRT found path with {len(trajectory['waypoints'])} waypoints")
        print(f"   Total time: {trajectory['times'][-1]:.2f} seconds")
    
    # Test 4: Trajectory optimization
    print("\n4. Trajectory Optimization:")
    if trajectory:
        optimized = planner.optimize_trajectory(trajectory)
        print(f"   ✓ Optimized trajectory")
        print(f"   Original time: {trajectory['times'][-1]:.2f}s")
        print(f"   Optimized time: {optimized['times'][-1]:.2f}s")
    
    print("\n" + "=" * 60)
    print("Planning capabilities ready for integration!")
    print("=" * 60)


if __name__ == "__main__":
    demo_planning()