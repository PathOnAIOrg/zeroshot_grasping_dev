#!/usr/bin/env python3
"""
Web UI for SO-101 Robot Control
A simple web interface with buttons to control the robot
"""

from flask import Flask, render_template, jsonify, request, send_from_directory
import subprocess
import json
import os
import time
import threading
from pathlib import Path
from datetime import datetime
import sys
import numpy as np

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from so101_grasp.robot.so101_client_raw_simple import SO101ClientRawSimple
from so101_grasp.robot.so101_kinematics import SO101Kinematics

app = Flask(__name__)

# Global robot client and kinematics
robot = None
kinematics = SO101Kinematics()
recording_keyframes = []
is_recording = False
current_status = "Disconnected"
is_moving = False
current_thread = None
motors_enabled = False  # Track motor state

# Trajectories directory
TRAJ_DIR = Path.home() / "Documents/Github/opensource_dev/GraspingDemo/trajectories"
TRAJ_DIR.mkdir(exist_ok=True)


def connect_robot():
    """Connect to robot"""
    global robot, current_status
    try:
        robot = SO101ClientRawSimple(port="/dev/ttyACM0")
        current_status = "Connected"
        return True
    except Exception as e:
        current_status = f"Connection failed: {e}"
        return False


def disconnect_robot():
    """Disconnect from robot"""
    global robot, current_status
    if robot:
        robot.disconnect()
        robot = None
    current_status = "Disconnected"


@app.route('/')
def index():
    """Main page"""
    return render_template('index.html')


@app.route('/api/status')
def get_status():
    """Get robot status"""
    global current_status, is_moving, motors_enabled
    
    if robot:
        try:
            # Try to read with validation disabled for status updates
            positions = robot.read_joints(validate=False)
            gripper_status = "OPEN" if positions[5] > 1.0 else "CLOSED" if positions[5] < -1.0 else "MIDDLE"
            
            # Check if any position is exactly 0.0 (might indicate read failure)
            read_quality = "good"
            zero_count = sum(1 for p in positions if p == 0.0)
            if zero_count >= 3:  # If 3 or more joints read as 0, likely a problem
                read_quality = "poor"
            elif zero_count >= 1:
                read_quality = "fair"
                
            return jsonify({
                'connected': True,
                'status': current_status,
                'positions': [round(p, 3) for p in positions],
                'gripper': gripper_status,
                'is_moving': is_moving,
                'read_quality': read_quality,
                'motors_enabled': motors_enabled
            })
        except Exception as e:
            return jsonify({'connected': False, 'status': current_status, 'error': str(e)})
    else:
        return jsonify({'connected': False, 'status': current_status})


@app.route('/api/connect', methods=['POST'])
def connect():
    """Connect to robot"""
    if connect_robot():
        return jsonify({'success': True, 'message': 'Connected to robot'})
    else:
        return jsonify({'success': False, 'message': current_status})


@app.route('/api/disconnect', methods=['POST'])
def disconnect():
    """Disconnect from robot"""
    disconnect_robot()
    return jsonify({'success': True, 'message': 'Disconnected from robot'})


@app.route('/api/enable_torque', methods=['POST'])
def enable_torque():
    """Enable all servos"""
    global motors_enabled
    
    if not robot:
        return jsonify({'success': False, 'message': 'Robot not connected'})
    
    try:
        for i in range(6):
            robot.enable_torque(i + 1, True)
            time.sleep(0.01)
        motors_enabled = True
        return jsonify({'success': True, 'message': 'All servos enabled'})
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})


@app.route('/api/disable_torque', methods=['POST'])
def disable_torque():
    """Disable all servos"""
    global motors_enabled
    
    if not robot:
        return jsonify({'success': False, 'message': 'Robot not connected'})
    
    try:
        for i in range(6):
            robot.enable_torque(i + 1, False)
            time.sleep(0.01)
        motors_enabled = False
        return jsonify({'success': True, 'message': 'All servos disabled - move manually'})
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})


@app.route('/api/emergency_stop', methods=['POST'])
def emergency_stop():
    """Emergency stop - immediately stop all movement"""
    global is_moving, current_thread
    
    if not robot:
        return jsonify({'success': False, 'message': 'Robot not connected'})
    
    try:
        # Set flag to stop movement
        is_moving = False
        
        # Stop current thread if exists
        if current_thread and current_thread.is_alive():
            # This will cause the thread to check is_moving flag
            current_thread = None
        
        # Read current position and hold it
        current = robot.read_joints()
        robot.write_joints(current)
        
        return jsonify({'success': True, 'message': '🛑 EMERGENCY STOP ACTIVATED'})
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})


@app.route('/api/set_home', methods=['POST'])
def set_home_position():
    """Set home position with validation"""
    if not robot:
        return jsonify({'success': False, 'message': 'Robot not connected'})
    
    try:
        # Try to read positions with validation
        positions = robot.read_joints(validate=True)
        
        # Additional validation - check for reasonable values
        # Joints should not all be at 0 (unless robot is actually at origin)
        zero_count = sum(1 for p in positions[:5] if abs(p) < 0.01)  # Check first 5 joints
        if zero_count >= 3:
            return jsonify({
                'success': False, 
                'message': 'Position reading unreliable - too many zero values. Please try again.'
            })
        
        # Check for extreme values that might indicate bad reads
        for i, pos in enumerate(positions[:5]):
            if abs(pos) > 3.0:  # More than ~172 degrees
                return jsonify({
                    'success': False,
                    'message': f'Joint {i+1} reading seems incorrect ({pos:.2f} rad). Please try again.'
                })
        
        return jsonify({
            'success': True,
            'positions': [round(p, 3) for p in positions],
            'message': 'Home position validated and ready to save'
        })
        
    except ValueError as e:
        return jsonify({'success': False, 'message': str(e)})
    except Exception as e:
        return jsonify({'success': False, 'message': f'Error reading position: {e}'})


@app.route('/api/home', methods=['POST'])
def go_home():
    """Go to home position"""
    global is_moving, motors_enabled
    
    if not robot:
        return jsonify({'success': False, 'message': 'Robot not connected'})
    
    # Check if already moving
    if is_moving:
        return jsonify({'success': False, 'message': '⚠️ Robot is already moving! Use Emergency Stop first.'})
    
    try:
        # Enable motors if not already enabled (required for movement)
        if not motors_enabled:
            for i in range(6):
                robot.enable_torque(i + 1, True)
                time.sleep(0.01)
            motors_enabled = True
        
        # Get home position from request or use default
        data = request.get_json() or {}
        home_position = data.get('position', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        # Ensure home_position has 6 values (including gripper)
        if len(home_position) < 6:
            home_position = home_position + [0.0] * (6 - len(home_position))
        
        # Get speed and convert to float
        speed_value = data.get('speed', 100)
        if isinstance(speed_value, str):
            speed_value = float(speed_value)
        speed = speed_value / 100.0  # Convert percentage to multiplier
        
        current = robot.read_joints()
        steps = int(50 / speed) if speed > 0 else 50  # Adjust steps based on speed
        timestep = 0.02 * speed if speed > 0 else 0.02  # Adjust timestep based on speed
        
        # Set moving flag
        is_moving = True
        try:
            robot.interpolate_waypoint(current, home_position, steps=steps, timestep=timestep)
            return jsonify({'success': True, 'message': 'Moved to home position'})
        finally:
            is_moving = False
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})


@app.route('/api/record/start', methods=['POST'])
def start_recording():
    """Start recording"""
    global is_recording, recording_keyframes, motors_enabled
    
    if not robot:
        return jsonify({'success': False, 'message': 'Robot not connected'})
    
    is_recording = True
    recording_keyframes = []
    
    # Disable torque for manual movement
    for i in range(6):
        robot.enable_torque(i + 1, False)
        time.sleep(0.01)
    motors_enabled = False
    
    return jsonify({'success': True, 'message': 'Recording started - move robot manually'})


@app.route('/api/record/keyframe', methods=['POST'])
def record_keyframe():
    """Record a keyframe"""
    global recording_keyframes
    
    if not robot or not is_recording:
        return jsonify({'success': False, 'message': 'Not recording'})
    
    try:
        pose = robot.read_joints()
        keyframe = {
            'index': len(recording_keyframes),
            'pose': [float(p) for p in pose],
            'timestamp': time.time()
        }
        recording_keyframes.append(keyframe)
        
        gripper_status = "OPEN" if pose[5] > 1.0 else "CLOSED" if pose[5] < -1.0 else "MIDDLE"
        
        return jsonify({
            'success': True,
            'message': f'Keyframe {len(recording_keyframes)} saved',
            'keyframe_count': len(recording_keyframes),
            'gripper': gripper_status
        })
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})


@app.route('/api/record/stop', methods=['POST'])
def stop_recording():
    """Stop recording and save"""
    global is_recording, recording_keyframes, motors_enabled
    
    if not is_recording:
        return jsonify({'success': False, 'message': 'Not recording'})
    
    is_recording = False
    
    # Re-enable torque
    for i in range(6):
        robot.enable_torque(i + 1, True)
        time.sleep(0.01)
    motors_enabled = True
    
    if len(recording_keyframes) < 2:
        return jsonify({'success': False, 'message': 'Need at least 2 keyframes'})
    
    # Save trajectory
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = TRAJ_DIR / f"web_trajectory_{timestamp}.json"
    
    data = {
        'name': f"web_{timestamp}",
        'created': datetime.now().isoformat(),
        'num_keyframes': len(recording_keyframes),
        'keyframes': recording_keyframes
    }
    
    with open(filename, 'w') as f:
        json.dump(data, f, indent=2)
    
    return jsonify({
        'success': True,
        'message': f'Saved {len(recording_keyframes)} keyframes',
        'filename': filename.name
    })


@app.route('/api/trajectories')
def list_trajectories():
    """List saved trajectories"""
    files = list(TRAJ_DIR.glob("*.json"))
    trajectories = []
    
    for file in sorted(files, reverse=True)[:20]:  # Last 20 files
        try:
            with open(file, 'r') as f:
                data = json.load(f)
            trajectories.append({
                'filename': file.name,
                'name': data.get('name', 'Unknown'),
                'created': data.get('created', 'Unknown'),
                'keyframes': data.get('num_keyframes', len(data.get('keyframes', [])))
            })
        except:
            pass
    
    return jsonify(trajectories)


@app.route('/api/replay', methods=['POST'])
def replay_trajectory():
    """Replay a trajectory"""
    global is_moving, current_thread
    
    if not robot:
        return jsonify({'success': False, 'message': 'Robot not connected'})
    
    # Check if already moving
    if is_moving:
        return jsonify({'success': False, 'message': '⚠️ Robot is already moving! Use Emergency Stop first.'})
    
    data = request.get_json() or {}
    filename = data.get('filename')
    
    # Get speed and convert to float
    speed_value = data.get('speed', 100)
    if isinstance(speed_value, str):
        speed_value = float(speed_value)
    speed = speed_value / 100.0  # Convert percentage to multiplier
    
    if not filename:
        return jsonify({'success': False, 'message': 'No filename provided'})
    
    filepath = TRAJ_DIR / filename
    if not filepath.exists():
        return jsonify({'success': False, 'message': 'File not found'})
    
    try:
        with open(filepath, 'r') as f:
            traj_data = json.load(f)
        
        keyframes = traj_data.get('keyframes', [])
        if len(keyframes) < 2:
            return jsonify({'success': False, 'message': 'Invalid trajectory'})
        
        # Run replay in background thread
        def replay_thread():
            global is_moving
            is_moving = True
            try:
                # Move to start position
                current = robot.read_joints()
                start = keyframes[0]['pose']
                steps = int(50 / speed)
                timestep = 0.02 * speed
                
                # Check if should stop
                if not is_moving:
                    return
                    
                robot.interpolate_waypoint(current, start, steps=steps, timestep=timestep)
                time.sleep(1 / speed)
                
                # Replay through keyframes
                for i in range(len(keyframes) - 1):
                    # Check if should stop
                    if not is_moving:
                        break
                        
                    steps = int(30 / speed)
                    timestep = 0.03 * speed
                    robot.interpolate_waypoint(
                        keyframes[i]['pose'],
                        keyframes[i+1]['pose'],
                        steps=steps,
                        timestep=timestep
                    )
                    time.sleep(0.5 / speed)
            except:
                pass
            finally:
                is_moving = False
        
        current_thread = threading.Thread(target=replay_thread)
        current_thread.daemon = True
        current_thread.start()
        
        return jsonify({'success': True, 'message': f'Replaying {len(keyframes)} keyframes at {int(speed*100)}% speed'})
        
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})


@app.route('/api/gripper/<action>', methods=['POST'])
def control_gripper(action):
    """Control gripper only (joint 6)"""
    if not robot:
        return jsonify({'success': False, 'message': 'Robot not connected'})
    
    try:
        # Set gripper position
        if action == 'open':
            gripper_pos = 1.5
        elif action == 'close':
            gripper_pos = -1.5
        elif action == 'middle':
            gripper_pos = 0.0
        else:
            return jsonify({'success': False, 'message': 'Invalid action'})
        
        # Only write to gripper servo (servo ID 6)
        # Using direct servo control to avoid moving other joints
        servo_id = 6
        ticks = robot.radians_to_ticks(gripper_pos)
        
        # Use the write_servo_position method to control only the gripper
        robot.write_servo_position(servo_id, ticks, speed=500)
        
        return jsonify({'success': True, 'message': f'Gripper {action}'})
        
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})


@app.route('/api/delete', methods=['POST'])
def delete_trajectory():
    """Delete a trajectory file"""
    filename = request.json.get('filename')
    if not filename:
        return jsonify({'success': False, 'message': 'No filename provided'})
    
    filepath = TRAJ_DIR / filename
    if not filepath.exists():
        return jsonify({'success': False, 'message': 'File not found'})
    
    try:
        filepath.unlink()
        return jsonify({'success': True, 'message': f'Deleted {filename}'})
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})


@app.route('/api/gripper_pose')
def get_gripper_pose():
    """Get current gripper pose in cartesian coordinates"""
    if not robot:
        return jsonify({'success': False, 'message': 'Robot not connected'})
    
    try:
        # Read current joint positions
        joints = robot.read_joints(validate=False)
        
        # Calculate gripper pose
        pose = kinematics.get_gripper_pose(joints)
        
        return jsonify({
            'success': True,
            'pose': pose
        })
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})


@app.route('/api/cartesian_move', methods=['POST'])
def cartesian_move():
    """Move gripper in cartesian space"""
    global is_moving, motors_enabled
    
    if not robot:
        return jsonify({'success': False, 'message': 'Robot not connected'})
    
    # Check if already moving
    if is_moving:
        return jsonify({'success': False, 'message': '⚠️ Robot is already moving! Use Emergency Stop first.'})
    
    data = request.get_json() or {}
    direction = data.get('direction')
    distance = data.get('distance', 0.01)  # Default 1cm
    
    if not direction:
        return jsonify({'success': False, 'message': 'No direction specified'})
    
    try:
        # Enable motors if not already enabled (required for movement)
        if not motors_enabled:
            for i in range(6):
                robot.enable_torque(i + 1, True)
                time.sleep(0.01)
            motors_enabled = True
        
        # Convert distance to meters if given in cm
        if data.get('unit') == 'cm':
            distance = distance / 100.0
        elif data.get('unit') == 'mm':
            distance = distance / 1000.0
        
        # Get current joint positions
        current_joints = robot.read_joints()
        
        # Calculate new joint positions for cartesian move
        new_joints = kinematics.cartesian_move(current_joints, direction, distance)
        
        if new_joints is None:
            return jsonify({'success': False, 'message': 'Target position unreachable'})
        
        # Set moving flag
        is_moving = True
        try:
            # Move to new position
            robot.interpolate_waypoint(current_joints, new_joints, steps=30, timestep=0.02)
            
            # Get new gripper pose
            new_pose = kinematics.get_gripper_pose(new_joints)
            
            return jsonify({
                'success': True,
                'message': f'Moved {direction} {distance*1000:.1f}mm',
                'new_pose': new_pose,
                'motors_enabled': motors_enabled
            })
        finally:
            is_moving = False
            
    except Exception as e:
        is_moving = False
        return jsonify({'success': False, 'message': str(e)})


@app.route('/api/rotate_gripper', methods=['POST'])
def rotate_gripper():
    """Rotate gripper around specified axis"""
    global is_moving, motors_enabled
    
    if not robot:
        return jsonify({'success': False, 'message': 'Robot not connected'})
    
    # Check if already moving
    if is_moving:
        return jsonify({'success': False, 'message': '⚠️ Robot is already moving! Use Emergency Stop first.'})
    
    data = request.get_json() or {}
    axis = data.get('axis')
    angle = data.get('angle', 10)  # Default 10 degrees
    
    if not axis:
        return jsonify({'success': False, 'message': 'No rotation axis specified'})
    
    try:
        # Enable motors if not already enabled (required for movement)
        if not motors_enabled:
            for i in range(6):
                robot.enable_torque(i + 1, True)
                time.sleep(0.01)
            motors_enabled = True
        
        # Convert angle to radians if given in degrees
        if data.get('unit') == 'degrees' or data.get('unit') == 'deg':
            angle_rad = np.radians(angle)
        else:
            angle_rad = angle
        
        # Get current joint positions
        current_joints = robot.read_joints()
        
        # Calculate new joint positions for rotation
        new_joints = kinematics.rotate_gripper(current_joints, axis, angle_rad)
        
        if new_joints is None:
            return jsonify({'success': False, 'message': 'Rotation unreachable'})
        
        # Set moving flag
        is_moving = True
        try:
            # Move to new position
            robot.interpolate_waypoint(current_joints, new_joints, steps=20, timestep=0.02)
            
            # Get new gripper pose
            new_pose = kinematics.get_gripper_pose(new_joints)
            
            return jsonify({
                'success': True,
                'message': f'Rotated {axis} by {angle}°',
                'new_pose': new_pose,
                'motors_enabled': motors_enabled
            })
        finally:
            is_moving = False
            
    except Exception as e:
        is_moving = False
        return jsonify({'success': False, 'message': str(e)})


@app.route('/api/move_to_position', methods=['POST'])
def move_to_position():
    """Move gripper to specific cartesian position"""
    global is_moving, motors_enabled
    
    if not robot:
        return jsonify({'success': False, 'message': 'Robot not connected'})
    
    # Check if already moving
    if is_moving:
        return jsonify({'success': False, 'message': '⚠️ Robot is already moving! Use Emergency Stop first.'})
    
    data = request.get_json() or {}
    target = data.get('target')
    
    if not target or 'x' not in target or 'y' not in target or 'z' not in target:
        return jsonify({'success': False, 'message': 'Invalid target position'})
    
    try:
        import numpy as np
        
        # Enable motors if not already enabled (required for movement)
        if not motors_enabled:
            for i in range(6):
                robot.enable_torque(i + 1, True)
                time.sleep(0.01)
            motors_enabled = True
        
        # Get target position
        target_pos = np.array([target['x'], target['y'], target['z']])
        
        # Convert from cm/mm to meters if needed
        if data.get('unit') == 'cm':
            target_pos = target_pos / 100.0
        elif data.get('unit') == 'mm':
            target_pos = target_pos / 1000.0
        
        # Get current joints
        current_joints = robot.read_joints()
        
        # Calculate IK solution
        new_joints = kinematics.inverse_kinematics(target_pos, current_joints)
        
        if new_joints is None:
            return jsonify({'success': False, 'message': 'Target position unreachable'})
        
        # Set moving flag
        is_moving = True
        try:
            # Move to target
            robot.interpolate_waypoint(current_joints, new_joints, steps=50, timestep=0.02)
            
            # Get actual pose
            final_pose = kinematics.get_gripper_pose(new_joints)
            
            return jsonify({
                'success': True,
                'message': f'Moved to position',
                'pose': final_pose,
                'motors_enabled': motors_enabled
            })
        finally:
            is_moving = False
            
    except Exception as e:
        is_moving = False
        return jsonify({'success': False, 'message': str(e)})


if __name__ == '__main__':
    print("Starting SO-101 Web Control...")
    print("Open browser at: http://localhost:5000")
    app.run(host='0.0.0.0', port=5000, debug=False)