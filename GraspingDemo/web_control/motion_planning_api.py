#!/usr/bin/env python3
"""
Motion Planning API for Web Control Interface

Provides advanced motion planning capabilities through REST API
"""

from flask import Blueprint, request, jsonify
import numpy as np
import sys
import os
import time
from typing import List, Dict, Any, Optional

# Add path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# Import motion planner
try:
    from so101_grasp.robot.so101_motion_planner import (
        SO101MotionPlanner, 
        PlannerConfig, 
        PlannerMode,
        ObstacleEnvironment
    )
    HAVE_PLANNER = True
except ImportError as e:
    print(f"Warning: Motion planner not available: {e}")
    HAVE_PLANNER = False

# Create Blueprint
motion_api = Blueprint('motion_api', __name__)

# Global planner instance
planner = None
current_trajectory = None
obstacles = []

# Motion queue for sequential execution
motion_queue = []
queue_executing = False
current_queue_index = 0

# Motion history for completed actions
motion_history = []
max_history_size = 100  # Keep last 100 actions


def init_planner():
    """Initialize motion planner"""
    global planner
    if HAVE_PLANNER:
        config = PlannerConfig(
            max_velocity=1.5,
            max_acceleration=3.0,
            path_resolution=50,
            planning_timeout=5.0
        )
        planner = SO101MotionPlanner(config)
        return True
    return False


@motion_api.route('/api/motion/plan/joint', methods=['POST'])
def plan_joint_trajectory():
    """Plan trajectory in joint space"""
    if not planner:
        if not init_planner():
            return jsonify({'success': False, 'message': 'Motion planner not available'})
    
    data = request.get_json() or {}
    start_joints = data.get('start_joints')
    goal_joints = data.get('goal_joints')
    
    if not start_joints or not goal_joints:
        return jsonify({'success': False, 'message': 'Missing start or goal joints'})
    
    try:
        # Plan trajectory
        trajectory = planner.plan_joint_trajectory(start_joints, goal_joints, obstacles)
        
        if trajectory:
            global current_trajectory
            current_trajectory = trajectory
            
            return jsonify({
                'success': True,
                'trajectory': {
                    'num_waypoints': len(trajectory['waypoints']),
                    'total_time': trajectory['times'][-1],
                    'waypoints': [wp.tolist() for wp in trajectory['waypoints']],
                    'times': trajectory['times']
                },
                'message': f'Planned trajectory with {len(trajectory["waypoints"])} waypoints'
            })
        else:
            return jsonify({'success': False, 'message': 'Failed to plan trajectory'})
            
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})


@motion_api.route('/api/motion/plan/cartesian', methods=['POST'])
def plan_cartesian_path():
    """Plan straight-line Cartesian path"""
    if not planner:
        if not init_planner():
            return jsonify({'success': False, 'message': 'Motion planner not available'})
    
    data = request.get_json() or {}
    start_pos = data.get('start_position')  # [x, y, z] in meters
    goal_pos = data.get('goal_position')
    current_joints = data.get('current_joints', [0, 0, 0, 0, 0, 0])
    velocity = data.get('velocity', 0.1)  # m/s
    
    if not start_pos or not goal_pos:
        return jsonify({'success': False, 'message': 'Missing start or goal position'})
    
    try:
        # Convert to numpy arrays
        start_pos = np.array(start_pos)
        goal_pos = np.array(goal_pos)
        
        # Plan Cartesian path
        trajectory = planner.plan_cartesian_path(
            start_pos, goal_pos, current_joints, velocity
        )
        
        if trajectory:
            global current_trajectory
            current_trajectory = trajectory
            
            return jsonify({
                'success': True,
                'trajectory': {
                    'num_waypoints': len(trajectory['waypoints']),
                    'total_time': trajectory['times'][-1],
                    'waypoints': [wp.tolist() for wp in trajectory['waypoints']],
                    'times': trajectory['times'],
                    'cartesian_waypoints': [wp.tolist() for wp in trajectory.get('cartesian_waypoints', [])]
                },
                'message': f'Planned Cartesian path with {len(trajectory["waypoints"])} waypoints'
            })
        else:
            return jsonify({'success': False, 'message': 'Failed to plan Cartesian path'})
            
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})


@motion_api.route('/api/motion/plan/rrt', methods=['POST'])
def plan_with_rrt():
    """Plan using RRT algorithm (good for obstacles)"""
    if not planner:
        if not init_planner():
            return jsonify({'success': False, 'message': 'Motion planner not available'})
    
    data = request.get_json() or {}
    start_joints = data.get('start_joints')
    goal_joints = data.get('goal_joints')
    max_iterations = data.get('max_iterations', 1000)
    
    if not start_joints or not goal_joints:
        return jsonify({'success': False, 'message': 'Missing start or goal joints'})
    
    try:
        # Plan with RRT
        trajectory = planner.plan_with_rrt(
            start_joints, goal_joints, max_iterations
        )
        
        if trajectory:
            global current_trajectory
            current_trajectory = trajectory
            
            return jsonify({
                'success': True,
                'trajectory': {
                    'num_waypoints': len(trajectory['waypoints']),
                    'total_time': trajectory['times'][-1],
                    'waypoints': [wp.tolist() for wp in trajectory['waypoints']],
                    'times': trajectory['times']
                },
                'message': f'RRT planned path with {len(trajectory["waypoints"])} waypoints'
            })
        else:
            return jsonify({'success': False, 'message': 'RRT planning failed'})
            
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})


@motion_api.route('/api/motion/optimize', methods=['POST'])
def optimize_trajectory():
    """Optimize current trajectory"""
    if not planner or not current_trajectory:
        return jsonify({'success': False, 'message': 'No trajectory to optimize'})
    
    try:
        optimized = planner.optimize_trajectory(current_trajectory)
        
        if optimized:
            time_saved = current_trajectory['times'][-1] - optimized['times'][-1]
            current_trajectory = optimized
            
            return jsonify({
                'success': True,
                'trajectory': {
                    'num_waypoints': len(optimized['waypoints']),
                    'total_time': optimized['times'][-1],
                    'time_saved': time_saved,
                    'waypoints': [wp.tolist() for wp in optimized['waypoints']],
                    'times': optimized['times']
                },
                'message': f'Optimized trajectory, saved {time_saved:.2f} seconds'
            })
        else:
            return jsonify({'success': False, 'message': 'Optimization failed'})
            
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})


@motion_api.route('/api/motion/obstacles/add', methods=['POST'])
def add_obstacle():
    """Add obstacle to environment"""
    if not planner:
        if not init_planner():
            return jsonify({'success': False, 'message': 'Motion planner not available'})
    
    data = request.get_json() or {}
    obstacle_type = data.get('type', 'sphere')
    
    try:
        if obstacle_type == 'sphere':
            center = np.array(data.get('center', [0, 0, 0]))
            radius = data.get('radius', 0.05)
            name = data.get('name', f'sphere_{len(obstacles)}')
            
            planner.environment.add_sphere(center, radius, name)
            obstacles.append({
                'type': 'sphere',
                'center': center.tolist(),
                'radius': radius,
                'name': name
            })
            
        elif obstacle_type == 'box':
            center = np.array(data.get('center', [0, 0, 0]))
            size = np.array(data.get('size', [0.1, 0.1, 0.1]))
            name = data.get('name', f'box_{len(obstacles)}')
            
            planner.environment.add_box(center, size, name)
            obstacles.append({
                'type': 'box',
                'center': center.tolist(),
                'size': size.tolist(),
                'name': name
            })
            
        elif obstacle_type == 'cylinder':
            center = np.array(data.get('center', [0, 0, 0]))
            radius = data.get('radius', 0.05)
            height = data.get('height', 0.2)
            name = data.get('name', f'cylinder_{len(obstacles)}')
            
            planner.environment.add_cylinder(center, radius, height, name)
            obstacles.append({
                'type': 'cylinder',
                'center': center.tolist(),
                'radius': radius,
                'height': height,
                'name': name
            })
        else:
            return jsonify({'success': False, 'message': f'Unknown obstacle type: {obstacle_type}'})
        
        return jsonify({
            'success': True,
            'message': f'Added {obstacle_type} obstacle',
            'total_obstacles': len(obstacles)
        })
        
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})


@motion_api.route('/api/motion/obstacles/clear', methods=['POST'])
def clear_obstacles():
    """Clear all obstacles"""
    global obstacles
    
    if planner:
        planner.environment.obstacles = []
    obstacles = []
    
    return jsonify({'success': True, 'message': 'Cleared all obstacles'})


@motion_api.route('/api/motion/obstacles/list', methods=['GET'])
def list_obstacles():
    """List all obstacles"""
    return jsonify({
        'success': True,
        'obstacles': obstacles,
        'count': len(obstacles)
    })


@motion_api.route('/api/motion/execute', methods=['POST'])
def execute_trajectory():
    """Execute planned trajectory on robot"""
    if not current_trajectory:
        return jsonify({'success': False, 'message': 'No trajectory to execute'})
    
    data = request.get_json() or {}
    real_time = data.get('real_time', True)
    
    # Get robot interface from main app
    from app import robot
    
    if not robot or not robot.connected:
        return jsonify({'success': False, 'message': 'Robot not connected'})
    
    try:
        success = planner.execute_trajectory(
            current_trajectory, 
            robot, 
            real_time
        )
        
        if success:
            return jsonify({
                'success': True,
                'message': 'Trajectory executed successfully',
                'duration': current_trajectory['times'][-1]
            })
        else:
            return jsonify({'success': False, 'message': 'Trajectory execution failed'})
            
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})


@motion_api.route('/api/motion/visualize', methods=['GET'])
def visualize_trajectory():
    """Get trajectory visualization data"""
    if not current_trajectory:
        return jsonify({'success': False, 'message': 'No trajectory to visualize'})
    
    try:
        # Convert trajectory to visualization format
        waypoints = current_trajectory['waypoints']
        times = current_trajectory['times']
        
        # Calculate Cartesian positions for each waypoint
        cartesian_points = []
        for wp in waypoints:
            # Use forward kinematics to get position
            from so101_grasp.robot.so101_kinematics_fast import SO101KinematicsFast
            kin = SO101KinematicsFast()
            pos, _ = kin.forward_kinematics(list(wp) + [0])
            cartesian_points.append(pos.tolist())
        
        return jsonify({
            'success': True,
            'visualization': {
                'joint_waypoints': [wp.tolist() for wp in waypoints],
                'cartesian_waypoints': cartesian_points,
                'times': times,
                'obstacles': obstacles
            }
        })
        
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})


@motion_api.route('/api/motion/queue/add', methods=['POST'])
def add_to_motion_queue():
    """Add motion instruction to queue"""
    global motion_queue
    
    data = request.get_json() or {}
    instruction_type = data.get('type')  # 'pose', 'joint', 'relative'
    
    if instruction_type == 'pose':
        instruction = {
            'id': len(motion_queue) + 1,
            'type': 'pose',
            'target_pose': data.get('pose', [0.2, 0.0, 0.2]),
            'description': data.get('description', f'Move to pose {len(motion_queue) + 1}'),
            'speed': data.get('speed', 0.1),
            'status': 'pending'
        }
    elif instruction_type == 'joint':
        instruction = {
            'id': len(motion_queue) + 1,
            'type': 'joint',
            'target_joints': data.get('joints', [0, 0, 0, 0, 0]),
            'description': data.get('description', f'Move joints {len(motion_queue) + 1}'),
            'speed': data.get('speed', 1.0),
            'status': 'pending'
        }
    elif instruction_type == 'relative':
        instruction = {
            'id': len(motion_queue) + 1,
            'type': 'relative',
            'relative_move': data.get('relative', [0, 0, 0]),
            'description': data.get('description', f'Relative move {len(motion_queue) + 1}'),
            'speed': data.get('speed', 0.1),
            'status': 'pending'
        }
    elif instruction_type == 'gripper':
        instruction = {
            'id': len(motion_queue) + 1,
            'type': 'gripper',
            'gripper_action': data.get('action', 'close'),  # 'open', 'close', 'position'
            'gripper_value': data.get('value', 0.0),
            'description': data.get('description', f'Gripper {data.get("action", "action")}'),
            'status': 'pending'
        }
    elif instruction_type == 'wait':
        instruction = {
            'id': len(motion_queue) + 1,
            'type': 'wait',
            'duration': data.get('duration', 1.0),
            'description': data.get('description', f'Wait {data.get("duration", 1.0)}s'),
            'status': 'pending'
        }
    else:
        return jsonify({'success': False, 'message': f'Unknown instruction type: {instruction_type}'})
    
    motion_queue.append(instruction)
    
    return jsonify({
        'success': True,
        'message': f'Added {instruction_type} instruction to queue',
        'instruction': instruction,
        'queue_length': len(motion_queue)
    })


@motion_api.route('/api/motion/queue/list', methods=['GET'])
def get_motion_queue():
    """Get current motion queue"""
    return jsonify({
        'success': True,
        'queue': motion_queue,
        'queue_length': len(motion_queue),
        'executing': queue_executing,
        'current_index': current_queue_index
    })


@motion_api.route('/api/motion/queue/clear', methods=['POST'])
def clear_motion_queue():
    """Clear motion queue"""
    global motion_queue, queue_executing, current_queue_index
    
    if queue_executing:
        return jsonify({'success': False, 'message': 'Cannot clear queue while executing'})
    
    motion_queue = []
    current_queue_index = 0
    
    return jsonify({
        'success': True,
        'message': 'Motion queue cleared'
    })


@motion_api.route('/api/motion/queue/remove/<int:instruction_id>', methods=['DELETE'])
def remove_from_queue(instruction_id):
    """Remove specific instruction from queue"""
    global motion_queue
    
    if queue_executing:
        return jsonify({'success': False, 'message': 'Cannot modify queue while executing'})
    
    # Find and remove instruction
    motion_queue = [instr for instr in motion_queue if instr['id'] != instruction_id]
    
    return jsonify({
        'success': True,
        'message': f'Removed instruction {instruction_id}',
        'queue_length': len(motion_queue)
    })


@motion_api.route('/api/motion/queue/execute', methods=['POST'])
def execute_motion_queue():
    """Execute motion queue sequentially"""
    global queue_executing, current_queue_index
    
    if not motion_queue:
        return jsonify({'success': False, 'message': 'Queue is empty'})
    
    if queue_executing:
        return jsonify({'success': False, 'message': 'Queue is already executing'})
    
    # Get robot interface from main app
    try:
        from app import robot
        if not robot or not robot.connected:
            return jsonify({'success': False, 'message': 'Robot not connected'})
    except ImportError:
        return jsonify({'success': False, 'message': 'Robot interface not available'})
    
    queue_executing = True
    current_queue_index = 0
    
    # Start execution in background (would need threading for real implementation)
    # For now, just mark as started
    return jsonify({
        'success': True,
        'message': 'Queue execution started',
        'queue_length': len(motion_queue)
    })


@motion_api.route('/api/motion/queue/stop', methods=['POST'])
def stop_motion_queue():
    """Stop motion queue execution"""
    global queue_executing
    
    queue_executing = False
    
    return jsonify({
        'success': True,
        'message': 'Queue execution stopped'
    })


@motion_api.route('/api/motion/queue/status', methods=['GET'])
def get_queue_status():
    """Get queue execution status"""
    return jsonify({
        'success': True,
        'executing': queue_executing,
        'current_index': current_queue_index,
        'total_instructions': len(motion_queue),
        'current_instruction': motion_queue[current_queue_index] if current_queue_index < len(motion_queue) else None
    })


@motion_api.route('/api/motion/history/list', methods=['GET'])
def get_motion_history():
    """Get motion history"""
    return jsonify({
        'success': True,
        'history': motion_history,
        'history_length': len(motion_history),
        'max_size': max_history_size
    })


@motion_api.route('/api/motion/history/add', methods=['POST'])
def add_to_motion_history():
    """Add completed instruction to history"""
    global motion_history
    
    data = request.get_json() or {}
    instruction = data.get('instruction')
    
    if not instruction:
        return jsonify({'success': False, 'message': 'No instruction provided'})
    
    # Generate unique ID for this history entry
    history_id = len(motion_history) + 1
    
    # Create comprehensive history entry
    history_entry = {
        'id': history_id,
        'instruction': instruction,
        'executed_at': data.get('executed_at', time.time()),
        'execution_time': data.get('execution_time', 0),
        'success': data.get('success', True),
        'error_message': data.get('error_message', None),
        
        # Enhanced robot state information
        'joint_state': data.get('joint_state', {}),
        'pose_state': data.get('pose_state', {}),
        'trajectory': data.get('trajectory', None),
        'robot_status': data.get('robot_status', {}),
        'context': data.get('context', {}),
        
        # Add basic instruction info for backward compatibility
        'type': instruction.get('type', 'unknown'),
        'description': instruction.get('description', 'No description'),
        'target_pose': instruction.get('pose') or instruction.get('target_pose'),
        'target_joints': instruction.get('joints') or instruction.get('target_joints'),
        'relative_move': instruction.get('relative') or instruction.get('relative_move'),
        'gripper_action': instruction.get('action') or instruction.get('gripper_action'),
        'gripper_value': instruction.get('value') or instruction.get('gripper_value'),
        'duration': instruction.get('duration'),
        'speed': instruction.get('speed')
    }
    
    motion_history.append(history_entry)
    
    # Keep only the last max_history_size entries
    if len(motion_history) > max_history_size:
        motion_history = motion_history[-max_history_size:]
    
    return jsonify({
        'success': True,
        'message': 'Added to history',
        'history_length': len(motion_history),
        'history_id': history_id
    })


@motion_api.route('/api/motion/history/clear', methods=['POST'])
def clear_motion_history():
    """Clear motion history"""
    global motion_history
    
    motion_history = []
    
    return jsonify({
        'success': True,
        'message': 'Motion history cleared'
    })


@motion_api.route('/api/motion/history/export', methods=['GET'])
def export_motion_history():
    """Export motion history as JSON"""
    import json
    from datetime import datetime
    
    export_data = {
        'exported_at': datetime.now().isoformat(),
        'total_actions': len(motion_history),
        'history': motion_history
    }
    
    response = jsonify(export_data)
    response.headers['Content-Disposition'] = f'attachment; filename=motion_history_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json'
    return response


@motion_api.route('/api/motion/history/update_final_state', methods=['POST'])
def update_history_final_state():
    """Update history entry with final robot state"""
    global motion_history
    
    data = request.get_json() or {}
    history_id = data.get('history_id')
    final_joint_state = data.get('final_joint_state')
    final_pose_state = data.get('final_pose_state')
    
    if not history_id:
        return jsonify({'success': False, 'message': 'No history ID provided'})
    
    # Find and update the history entry
    for entry in motion_history:
        if entry.get('id') == history_id:
            if 'joint_state' not in entry:
                entry['joint_state'] = {}
            if 'pose_state' not in entry:
                entry['pose_state'] = {}
                
            entry['joint_state']['after'] = final_joint_state
            entry['pose_state']['after'] = final_pose_state
            entry['updated_at'] = time.time()
            break
    
    return jsonify({
        'success': True,
        'message': 'Final state updated'
    })


@motion_api.route('/api/motion/history/detailed/<int:entry_id>', methods=['GET'])
def get_detailed_history_entry():
    """Get detailed information for a specific history entry"""
    global motion_history
    
    entry_id = request.view_args['entry_id']
    
    # Find the entry
    entry = None
    for hist_entry in motion_history:
        if hist_entry.get('id') == entry_id:
            entry = hist_entry
            break
    
    if not entry:
        return jsonify({'success': False, 'message': 'Entry not found'})
    
    return jsonify({
        'success': True,
        'entry': entry
    })


# Initialize planner on import
if HAVE_PLANNER:
    init_planner()
    print("Motion planning API initialized")