#!/usr/bin/env python3
"""
Web UI for SO-101 Robot Control
A simple web interface with buttons to control the robot
"""

from flask import Flask, render_template, jsonify, request, send_from_directory, send_file
import subprocess
import json
import os
import time
import threading
from pathlib import Path
from datetime import datetime
import sys
import numpy as np
import base64

# Try to import cv2 for image handling
try:
    import cv2
    cv2_available = True
except ImportError:
    print("Warning: OpenCV not installed. Some features may be limited.")
    cv2_available = False

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import the improved main client with built-in reliability features
from so101_grasp.robot.so101_client_raw_simple import SO101ClientRawSimple
from so101_grasp.robot.so101_kinematics import SO101Kinematics

print("Using SO-101 client with enhanced reliability features")

# Import camera handlers
from realsense_handler import RealSenseHandler, SimpleWebcamHandler
from pointcloud_viewer import (
    load_point_cloud_from_capture, 
    downsample_pointcloud, 
    vis_pcd_plotly,
    generate_plotly_html
)
from coordinate_transform import CoordinateTransform

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

# Global camera handlers
realsense_camera = None
webcam = None
active_camera = None  # 'realsense', 'webcam', or None
camera_streaming = False

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
    global robot, current_status, motors_enabled
    if robot:
        robot.disconnect()
        robot = None
    motors_enabled = False
    current_status = "Disconnected"


@app.route('/')
def index():
    """Main page"""
    return render_template('index.html')


@app.route('/camera')
def camera_interface():
    """Camera interface page"""
    return render_template('index_with_camera.html')


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
        
        # Always enable torque for gripper before moving it
        # This allows gripper to work even in release mode
        robot.enable_torque(servo_id, True)
        time.sleep(0.01)
        
        ticks = robot.radians_to_ticks(gripper_pos)
        
        # Use the write_servo_position method to control only the gripper
        robot.write_servo_position(servo_id, ticks, speed=500)
        
        # If in release mode (motors_enabled=False), disable gripper torque after movement
        # Give time for movement to complete
        if not motors_enabled:
            time.sleep(0.5)  # Wait for gripper to reach position
            robot.enable_torque(servo_id, False)
        
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


# Camera control routes
@app.route('/api/camera/connect', methods=['POST'])
def connect_camera():
    """Connect to camera (RealSense or webcam)"""
    global realsense_camera, webcam, active_camera, camera_streaming
    
    data = request.get_json() or {}
    camera_type = data.get('type', 'auto')  # 'realsense', 'webcam', or 'auto'
    
    # Disconnect any existing camera
    if active_camera:
        disconnect_camera()
    
    # Try RealSense first if auto or specifically requested
    if camera_type in ['auto', 'realsense']:
        try:
            realsense_camera = RealSenseHandler()
            success, message = realsense_camera.connect()
            if success:
                active_camera = 'realsense'
                camera_streaming = True
                return jsonify({'success': True, 'message': message, 'type': 'realsense'})
        except Exception as e:
            if camera_type == 'realsense':
                return jsonify({'success': False, 'message': f'RealSense error: {str(e)}'})
    
    # Fall back to webcam if auto or specifically requested
    if camera_type in ['auto', 'webcam']:
        try:
            webcam = SimpleWebcamHandler()
            success, message = webcam.connect()
            if success:
                active_camera = 'webcam'
                camera_streaming = True
                return jsonify({'success': True, 'message': message, 'type': 'webcam'})
        except Exception as e:
            return jsonify({'success': False, 'message': f'Webcam error: {str(e)}'})
    
    return jsonify({'success': False, 'message': 'No camera available'})


@app.route('/api/camera/disconnect', methods=['POST'])
def disconnect_camera():
    """Disconnect camera"""
    global realsense_camera, webcam, active_camera, camera_streaming
    
    try:
        if active_camera == 'realsense' and realsense_camera:
            success, message = realsense_camera.disconnect()
            realsense_camera = None
        elif active_camera == 'webcam' and webcam:
            success, message = webcam.disconnect()
            webcam = None
        else:
            return jsonify({'success': False, 'message': 'No camera connected'})
        
        active_camera = None
        camera_streaming = False
        return jsonify({'success': success, 'message': message})
        
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})


@app.route('/api/camera/status')
def camera_status():
    """Get camera status"""
    global active_camera, camera_streaming
    
    status = {
        'connected': active_camera is not None,
        'type': active_camera,
        'streaming': camera_streaming
    }
    
    if active_camera == 'realsense' and realsense_camera:
        info = realsense_camera.get_camera_info()
        if info:
            status['info'] = info
    
    return jsonify(status)


@app.route('/api/camera/rgb')
def get_rgb_stream():
    """Get RGB image stream"""
    global active_camera, realsense_camera, webcam
    
    if not active_camera:
        return jsonify({'success': False, 'message': 'No camera connected'})
    
    try:
        if active_camera == 'realsense' and realsense_camera:
            frame_data = realsense_camera.get_rgb_frame()
        elif active_camera == 'webcam' and webcam:
            frame_data = webcam.get_rgb_frame()
        else:
            return jsonify({'success': False, 'message': 'Camera not available'})
        
        if frame_data:
            return jsonify({'success': True, 'image': frame_data})
        else:
            return jsonify({'success': False, 'message': 'No frame available'})
            
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})


@app.route('/api/camera/depth')
def get_depth_stream():
    """Get depth image stream (RealSense only)"""
    global active_camera, realsense_camera
    
    if active_camera != 'realsense' or not realsense_camera:
        return jsonify({'success': False, 'message': 'Depth only available with RealSense'})
    
    try:
        frame_data = realsense_camera.get_depth_frame()
        if frame_data:
            return jsonify({'success': True, 'image': frame_data})
        else:
            return jsonify({'success': False, 'message': 'No depth frame available'})
            
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})


@app.route('/api/camera/pointcloud')
def get_pointcloud():
    """Get point cloud data (RealSense only)"""
    global active_camera, realsense_camera
    
    if active_camera != 'realsense' or not realsense_camera:
        return jsonify({'success': False, 'message': 'Point cloud only available with RealSense'})
    
    try:
        # Get downsample factor from request (Three.js default settings)
        downsample = request.args.get('downsample', 4, type=int)
        max_points = request.args.get('max_points', 20000, type=int)
        
        pc_data = realsense_camera.get_point_cloud(downsample=downsample, max_points=max_points)
        if pc_data:
            return jsonify({'success': True, 'pointcloud': pc_data})
        else:
            return jsonify({'success': False, 'message': 'No point cloud available'})
            
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})


@app.route('/api/camera/capture', methods=['POST'])
def capture_camera():
    """Capture current camera frame(s) with robot state"""
    global active_camera, realsense_camera, webcam, robot
    
    if not active_camera:
        return jsonify({'success': False, 'message': 'No camera connected'})
    
    data = request.get_json() or {}
    include_pointcloud = data.get('pointcloud', True)
    
    # Get robot state if connected
    robot_state = None
    if robot:
        try:
            joints = robot.read_joints(validate=False)
            robot_state = {
                'joint_positions': [float(j) for j in joints],
                'joint_names': ['base', 'shoulder', 'elbow', 'wrist_pitch', 'wrist_roll', 'gripper'],
                'gripper_state': 'open' if joints[5] > 1.0 else 'closed' if joints[5] < -1.0 else 'middle',
                'timestamp': time.time()
            }
        except:
            robot_state = None
    
    try:
        if active_camera == 'realsense' and realsense_camera:
            success, capture_info = realsense_camera.capture(include_pointcloud=include_pointcloud, robot_state=robot_state)
        elif active_camera == 'webcam' and webcam:
            success, capture_info = webcam.capture(robot_state=robot_state)
        else:
            return jsonify({'success': False, 'message': 'Camera not available'})
        
        return jsonify({'success': success, 'message': capture_info, 'capture_info': capture_info})
        
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})


@app.route('/api/camera/list_captures')
def list_captures():
    """List captured images with metadata"""
    capture_dir = Path.home() / "Documents/Github/opensource_dev/GraspingDemo/captures"
    if not capture_dir.exists():
        return jsonify([])
    
    # First filter to get only directories, then sort
    folders = [f for f in capture_dir.iterdir() if f.is_dir()]
    folders.sort(reverse=True)  # Sort by name (newest first for timestamp format)
    
    captures = []
    for folder in folders[:20]:  # Get latest 20 captures
        # Only process folders that match the timestamp format (YYYYMMDD_HHMMSS)
        if len(folder.name) == 15 and folder.name[8] == '_':
            capture_info = {
                'name': folder.name,
                'path': str(folder),
                'has_rgb': (folder / 'rgb.png').exists(),
                'has_depth': (folder / 'depth.npy').exists(),
                'has_pointcloud': (folder / 'pointcloud.ply').exists(),
                'has_metadata': (folder / 'metadata.json').exists()
            }
            
            # Load metadata if available
            metadata_path = folder / 'metadata.json'
            if metadata_path.exists():
                try:
                    with open(metadata_path, 'r') as f:
                        metadata = json.load(f)
                        capture_info['timestamp'] = metadata.get('timestamp')
                        capture_info['camera_type'] = metadata.get('camera_type')
                        if 'robot_state' in metadata:
                            capture_info['has_robot_state'] = True
                            capture_info['gripper_state'] = metadata['robot_state'].get('gripper_state')
                except:
                    pass
            
            captures.append(capture_info)
    
    return jsonify(captures)


@app.route('/api/camera/capture/<capture_name>')
def get_capture_details(capture_name):
    """Get detailed information about a specific capture"""
    capture_dir = Path.home() / "Documents/Github/opensource_dev/GraspingDemo/captures" / capture_name
    
    if not capture_dir.exists():
        return jsonify({'success': False, 'message': 'Capture not found'})
    
    result = {
        'success': True,
        'name': capture_name,
        'files': {}
    }
    
    # Load metadata
    metadata_path = capture_dir / 'metadata.json'
    if metadata_path.exists():
        with open(metadata_path, 'r') as f:
            result['metadata'] = json.load(f)
    
    # Get images as base64 if cv2 is available
    if cv2_available:
        # Get RGB image as base64
        rgb_path = capture_dir / 'rgb.png'
        if rgb_path.exists():
            try:
                img = cv2.imread(str(rgb_path))
                if img is not None:
                    _, buffer = cv2.imencode('.jpg', img)
                    img_base64 = base64.b64encode(buffer).decode('utf-8')
                    result['files']['rgb'] = img_base64
            except Exception as e:
                print(f"Error reading RGB image: {e}")
        
        # Get depth colorized as base64
        depth_vis_path = capture_dir / 'depth_colorized.png'
        if depth_vis_path.exists():
            try:
                img = cv2.imread(str(depth_vis_path))
                if img is not None:
                    _, buffer = cv2.imencode('.jpg', img)
                    img_base64 = base64.b64encode(buffer).decode('utf-8')
                    result['files']['depth_colorized'] = img_base64
            except Exception as e:
                print(f"Error reading depth image: {e}")
    else:
        # Alternative: read files directly as base64 without cv2
        rgb_path = capture_dir / 'rgb.png'
        if rgb_path.exists():
            try:
                with open(rgb_path, 'rb') as f:
                    img_base64 = base64.b64encode(f.read()).decode('utf-8')
                    result['files']['rgb'] = img_base64
            except Exception as e:
                print(f"Error reading RGB image: {e}")
        
        depth_vis_path = capture_dir / 'depth_colorized.png'
        if depth_vis_path.exists():
            try:
                with open(depth_vis_path, 'rb') as f:
                    img_base64 = base64.b64encode(f.read()).decode('utf-8')
                    result['files']['depth_colorized'] = img_base64
            except Exception as e:
                print(f"Error reading depth image: {e}")
    
    return jsonify(result)


@app.route('/api/camera/capture/<capture_name>/<file_type>')
def get_capture_file(capture_name, file_type):
    """Serve capture file directly"""
    capture_dir = Path.home() / "Documents/Github/opensource_dev/GraspingDemo/captures" / capture_name
    
    if file_type == 'rgb':
        file_path = capture_dir / 'rgb.png'
    elif file_type == 'depth':
        file_path = capture_dir / 'depth_colorized.png'
    else:
        return jsonify({'success': False, 'message': 'Invalid file type'})
    
    if file_path.exists():
        return send_file(str(file_path), mimetype='image/png')
    else:
        return jsonify({'success': False, 'message': 'File not found'})


@app.route('/api/camera/capture/<capture_name>/pointcloud')
def get_capture_pointcloud(capture_name):
    """Get point cloud visualization for a capture"""
    capture_dir = Path.home() / "Documents/Github/opensource_dev/GraspingDemo/captures" / capture_name
    
    if not capture_dir.exists():
        return jsonify({'success': False, 'message': 'Capture not found'})
    
    try:
        # Get parameters from request
        density = request.args.get('density', 8, type=int)  # Higher density = less points
        max_points = request.args.get('max_points', 20000, type=int)
        point_size = request.args.get('size', 1, type=float)
        show_camera = request.args.get('show_camera', 'true').lower() == 'true'
        show_robot = request.args.get('show_robot', 'true').lower() == 'true'
        
        # Load point cloud data
        pcd_data, metadata = load_point_cloud_from_capture(str(capture_dir))
        
        if pcd_data is None:
            return jsonify({'success': False, 'message': 'No point cloud data available'})
        
        vertices = np.array(pcd_data['vertices'])
        colors = np.array(pcd_data['colors'])
        
        # Convert from meters to centimeters
        vertices = vertices * 100  # Convert to cm
        
        # Only filter out extremely far points
        if len(vertices) > 0:
            # Remove points that are unreasonably far
            distances = np.linalg.norm(vertices, axis=1)
            valid_mask = distances < 500  # Keep points within 5 meters
            vertices = vertices[valid_mask]
            colors = colors[valid_mask]
        
        # Apply density-based downsampling
        if density > 1 and len(vertices) > 0:
            # Use stride-based sampling for predictable density control
            indices = np.arange(0, len(vertices), density)
            vertices = vertices[indices]
            colors = colors[indices]
        
        # Further limit points if exceeding max_points
        if len(vertices) > max_points:
            indices = np.random.choice(len(vertices), max_points, replace=False)
            vertices = vertices[indices]
            colors = colors[indices]
        
        # Add robot state if available
        robot_joints = None
        if metadata and 'robot_state' in metadata:
            robot_joints = metadata['robot_state']['joint_positions']
        
        # Create Plotly visualization
        pointclouds = [{
            'vertices': vertices,
            'colors': colors,
            'name': f'Capture: {capture_name}'
        }]
        
        # Add robot visualization if available
        if robot_joints:
            # Simple robot end-effector position (would need FK for full robot)
            ee_pos = np.array(robot_joints[:3]).reshape(1, 3)
            pointclouds.append({
                'vertices': ee_pos,
                'colors': np.array([[1, 0, 0]]),  # Red for robot
                'name': 'Robot Position'
            })
        
        # Get camera info from metadata or use defaults
        camera_info = None
        if metadata and 'camera_intrinsics' in metadata:
            intrinsics = metadata['camera_intrinsics']
            camera_info = {
                'fx': intrinsics.get('fx', 615.0),
                'fy': intrinsics.get('fy', 615.0),
                'cx': intrinsics.get('cx', intrinsics.get('ppx', 320.0)),
                'cy': intrinsics.get('cy', intrinsics.get('ppy', 240.0)),
                'width': intrinsics.get('width', 640),
                'height': intrinsics.get('height', 480),
                'scale': intrinsics.get('scale', 1000.0)
            }
        
        # Get robot joint angles from metadata if available
        robot_joint_angles = None
        robot_position = None
        if metadata and 'robot_state' in metadata:
            robot_state = metadata['robot_state']
            if 'joint_positions' in robot_state:
                # Convert joint positions to dict with proper names (SO101 uses numbers)
                positions = robot_state['joint_positions']
                robot_joint_angles = {
                    '1': positions[0] if len(positions) > 0 else 0,
                    '2': positions[1] if len(positions) > 1 else 0,
                    '3': positions[2] if len(positions) > 2 else 0,
                    '4': positions[3] if len(positions) > 3 else 0,
                    '5': positions[4] if len(positions) > 4 else 0,
                    '6': positions[5] if len(positions) > 5 else 0
                }
        
        # Use point_size from request
        fig = vis_pcd_plotly(pointclouds, size_ls=[point_size, 5], 
                           title=f"Point Cloud - {capture_name}",
                           show_camera=show_camera, camera_info=camera_info,
                           show_robot=show_robot, robot_joint_angles=robot_joint_angles,
                           robot_position=robot_position)
        
        # Generate HTML
        html_content = generate_plotly_html(fig)
        
        return html_content, 200, {'Content-Type': 'text/html'}
        
    except Exception as e:
        print(f"Error generating point cloud visualization: {e}")
        return jsonify({'success': False, 'message': str(e)})


@app.route('/api/camera/pointcloud/live')
def get_live_pointcloud():
    """Get live point cloud visualization from RealSense with robot"""
    global realsense_camera, robot
    
    if not realsense_camera or not realsense_camera.is_streaming:
        return jsonify({'success': False, 'message': 'RealSense not streaming'})
    
    try:
        # Get parameters
        show_camera = request.args.get('show_camera', 'true').lower() == 'true'
        show_robot = request.args.get('show_robot', 'true').lower() == 'true'
        
        # Get current point cloud
        pcd_data = realsense_camera.get_point_cloud(downsample=4)
        
        if pcd_data is None:
            return jsonify({'success': False, 'message': 'No point cloud data'})
        
        vertices = np.array(pcd_data['vertices'])
        colors = np.array(pcd_data['colors']) / 255.0  # Normalize colors
        
        # Convert from meters to centimeters for better visualization
        vertices = vertices * 100  # Convert to cm
        
        # Only filter out extremely far points (noise)
        # Remove points that are unreasonably far (more than 500 cm away)
        distances = np.linalg.norm(vertices, axis=1)
        valid_mask = distances < 500  # Keep points within 5 meters
        vertices = vertices[valid_mask]
        colors = colors[valid_mask]
        
        # Remove statistical outliers if there are enough points
        if len(vertices) > 100:
            # Remove points that are too isolated
            from pointcloud_viewer import remove_statistical_outliers
            vertices, colors = remove_statistical_outliers(vertices, colors, n_neighbors=10, std_ratio=1.5)
        
        # Further downsample if needed (voxel size in cm now)
        if len(vertices) > 20000:
            vertices, colors = downsample_pointcloud(vertices, colors, voxel_size=1.0)  # 1cm voxel size
        
        # Create Plotly visualization
        pointclouds = [{
            'vertices': vertices,
            'colors': colors,
            'name': 'Live Point Cloud'
        }]
        
        # Get camera info if available
        camera_info = None
        if active_camera == 'realsense' and realsense_camera and realsense_camera.intrinsics:
            intrinsics = realsense_camera.intrinsics
            camera_info = {
                'fx': intrinsics.fx,
                'fy': intrinsics.fy,
                'cx': intrinsics.ppx,
                'cy': intrinsics.ppy,
                'width': intrinsics.width,
                'height': intrinsics.height,
                'scale': getattr(realsense_camera, 'depth_scale', 0.001) * 1000  # Convert to scale
            }
        
        # Get robot joint states
        robot_joint_angles = None
        if show_robot and robot:
            # Check if joint states were passed as parameter
            joint_states_str = request.args.get('joint_states')
            if joint_states_str:
                try:
                    import json
                    robot_joint_angles = json.loads(joint_states_str)
                except:
                    pass
            
            # If not passed or invalid, try to read from robot
            if robot_joint_angles is None:
                try:
                    joint_positions = robot.read_joints(validate=False)
                    robot_joint_angles = {
                        '1': joint_positions[0] if len(joint_positions) > 0 else 0,
                        '2': joint_positions[1] if len(joint_positions) > 1 else 0,
                        '3': joint_positions[2] if len(joint_positions) > 2 else 0,
                        '4': joint_positions[3] if len(joint_positions) > 3 else 0,
                        '5': joint_positions[4] if len(joint_positions) > 4 else 0,
                        '6': joint_positions[5] if len(joint_positions) > 5 else 0
                    }
                except:
                    pass  # Use default angles if can't get from robot
        
        # Create visualization with both point cloud and robot
        fig = vis_pcd_plotly(
            pointclouds, 
            size_ls=[1], 
            title="Live Scene",
            show_camera=show_camera, 
            camera_info=camera_info,
            show_robot=show_robot,
            robot_joint_angles=robot_joint_angles,
            robot_position=np.array([
                float(request.args.get('robot_pos_x', 0)),
                float(request.args.get('robot_pos_y', 0)),
                float(request.args.get('robot_pos_z', 0))
            ]),
            robot_rotation=np.array([
                float(request.args.get('robot_rot_roll', 0)),
                float(request.args.get('robot_rot_pitch', 0)),
                float(request.args.get('robot_rot_yaw', 0))
            ])
        )
        
        # Generate HTML
        html_content = generate_plotly_html(fig, div_id="live-pointcloud")
        
        return html_content, 200, {'Content-Type': 'text/html'}
        
    except Exception as e:
        print(f"Error generating live point cloud: {e}")
        return jsonify({'success': False, 'message': str(e)})


@app.route('/api/camera/pointcloud_data')
def get_pointcloud_data():
    """Get raw point cloud data for Three.js visualization - direct stream"""
    global realsense_camera
    
    if not realsense_camera or not realsense_camera.is_streaming:
        return jsonify({'success': False, 'message': 'Camera not streaming'})
    
    try:
        # Get point cloud directly from RealSense stream with built-in downsampling
        pcd_data = realsense_camera.get_point_cloud(downsample=8)  # More aggressive downsampling
        
        if pcd_data is None:
            return jsonify({'success': False, 'message': 'No point cloud data'})
        
        # Data is already in the right format from get_point_cloud
        return jsonify({
            'success': True,
            'vertices': pcd_data['vertices'],  # Already a list
            'colors': pcd_data['colors'],      # Already a list
            'num_points': pcd_data.get('num_points', len(pcd_data['vertices']))
        })
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})


@app.route('/api/camera/capture/<capture_name>/rgb')
def get_capture_rgb(capture_name):
    """Serve RGB image from a capture"""
    capture_dir = Path.home() / "Documents/Github/opensource_dev/GraspingDemo/captures" / capture_name
    rgb_path = capture_dir / 'rgb.png'
    
    if rgb_path.exists():
        from flask import send_file
        return send_file(str(rgb_path), mimetype='image/png')
    else:
        return jsonify({'success': False, 'message': 'RGB image not found'}), 404


@app.route('/api/camera/capture/<capture_name>/depth')
def get_capture_depth(capture_name):
    """Serve depth visualization from a capture"""
    capture_dir = Path.home() / "Documents/Github/opensource_dev/GraspingDemo/captures" / capture_name
    depth_vis_path = capture_dir / 'depth_colorized.png'
    
    if depth_vis_path.exists():
        from flask import send_file
        return send_file(str(depth_vis_path), mimetype='image/png')
    else:
        # Try to generate from depth.npy
        depth_path = capture_dir / 'depth.npy'
        if depth_path.exists():
            try:
                depth = np.load(str(depth_path))
                # Normalize and colorize
                depth_norm = depth.copy()
                depth_norm[depth_norm == 0] = np.nan
                depth_norm = (depth_norm - np.nanmin(depth_norm)) / (np.nanmax(depth_norm) - np.nanmin(depth_norm))
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_norm * 255, alpha=1), cv2.COLORMAP_JET)
                
                # Convert to PNG
                _, buffer = cv2.imencode('.png', depth_colormap)
                from io import BytesIO
                return send_file(BytesIO(buffer), mimetype='image/png')
            except:
                pass
        
        return jsonify({'success': False, 'message': 'Depth image not found'}), 404


@app.route('/api/camera/capture/<capture_name>/pointcloud_data')
def get_capture_pointcloud_data(capture_name):
    """Get point cloud data from a capture for Three.js"""
    capture_dir = Path.home() / "Documents/Github/opensource_dev/GraspingDemo/captures" / capture_name
    
    if not capture_dir.exists():
        return jsonify({'success': False, 'message': 'Capture not found'})
    
    try:
        # Load point cloud data
        pcd_data, metadata = load_point_cloud_from_capture(str(capture_dir))
        
        if pcd_data is None:
            return jsonify({'success': False, 'message': 'No point cloud data'})
        
        vertices = pcd_data['vertices']
        colors = pcd_data['colors']
        
        # Ensure colors are in 0-255 range for Three.js
        if colors.max() <= 1.0:
            colors = (colors * 255).astype(int)
        
        # Downsample for web
        if len(vertices) > 10000:
            vertices, colors = downsample_pointcloud(vertices, colors, voxel_size=0.005)
        
        return jsonify({
            'success': True,
            'vertices': vertices.tolist(),
            'colors': colors.tolist()
        })
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})


@app.route('/api/camera/replay/<capture_name>', methods=['POST'])
def replay_capture(capture_name):
    """Replay robot position from a capture"""
    global robot, is_moving, motors_enabled
    
    if not robot:
        return jsonify({'success': False, 'message': 'Robot not connected'})
    
    capture_dir = Path.home() / "Documents/Github/opensource_dev/GraspingDemo/captures" / capture_name
    metadata_path = capture_dir / 'metadata.json'
    
    if not metadata_path.exists():
        return jsonify({'success': False, 'message': 'No metadata found for this capture'})
    
    try:
        with open(metadata_path, 'r') as f:
            metadata = json.load(f)
        
        if 'robot_state' not in metadata:
            return jsonify({'success': False, 'message': 'No robot state in this capture'})
        
        robot_state = metadata['robot_state']
        target_joints = robot_state['joint_positions']
        
        # Enable motors if not already enabled
        if not motors_enabled:
            for i in range(6):
                robot.enable_torque(i + 1, True)
                time.sleep(0.01)
            motors_enabled = True
        
        # Move to captured position
        current = robot.read_joints()
        is_moving = True
        try:
            robot.interpolate_waypoint(current, target_joints, steps=50, timestep=0.02)
            return jsonify({
                'success': True, 
                'message': f'Moved to position from {capture_name}',
                'robot_state': robot_state
            })
        finally:
            is_moving = False
            
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})


@app.route('/api/coordinate/info')
def get_coordinate_info():
    """Get coordinate transformation information"""
    global realsense_camera, active_camera
    
    # Initialize coordinate transformer
    transform = CoordinateTransform()
    
    # Get camera intrinsics if available
    if active_camera == 'realsense' and realsense_camera and realsense_camera.intrinsics:
        intrinsics = realsense_camera.intrinsics
        transform.camera_intrinsics = {
            'fx': intrinsics.fx,
            'fy': intrinsics.fy,
            'cx': intrinsics.ppx,
            'cy': intrinsics.ppy,
            'width': intrinsics.width,
            'height': intrinsics.height,
            'depth_scale': getattr(realsense_camera, 'depth_scale', 0.001)
        }
    else:
        # Use default values
        transform.camera_intrinsics = {
            'fx': 615.0,
            'fy': 615.0,
            'cx': 320.0,
            'cy': 240.0,
            'width': 640,
            'height': 480,
            'depth_scale': 0.001
        }
    
    # Get transformation info
    info = transform.get_transform_info()
    
    return jsonify({'status': 'success', 'data': info})


@app.route('/api/coordinate/transform', methods=['POST'])
def set_coordinate_transform():
    """Set camera to world transformation"""
    data = request.get_json() or {}
    
    # Initialize transformer
    transform = CoordinateTransform()
    
    # Get transformation parameters
    rotation = data.get('rotation', None)
    translation = data.get('translation', None)
    
    try:
        if rotation:
            rotation = np.array(rotation)
        if translation:
            translation = np.array(translation)
        
        transform.set_camera_to_world_transform(rotation, translation)
        
        return jsonify({
            'success': True,
            'transform_matrix': transform.camera_to_world.tolist()
        })
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})


@app.route('/api/coordinate/visualize')
def get_coordinate_visualization():
    """Get coordinate frame visualization data"""
    global realsense_camera, active_camera
    
    # Initialize coordinate transformer
    transform = CoordinateTransform()
    
    # Set camera intrinsics
    if active_camera == 'realsense' and realsense_camera and realsense_camera.intrinsics:
        intrinsics = realsense_camera.intrinsics
        transform.camera_intrinsics = {
            'fx': intrinsics.fx,
            'fy': intrinsics.fy,
            'cx': intrinsics.ppx,
            'cy': intrinsics.ppy,
            'width': intrinsics.width,
            'height': intrinsics.height,
            'depth_scale': getattr(realsense_camera, 'depth_scale', 0.001)
        }
    
    # Get visualization data
    viz_data = transform.visualize_coordinate_frames()
    
    return jsonify(viz_data)


@app.route('/api/robot/move_joints', methods=['POST'])
def move_robot_joints():
    """Move robot to specific joint positions"""
    global robot, is_moving, motors_enabled
    
    if not robot:
        return jsonify({'success': False, 'message': 'Robot not connected'})
    
    if is_moving:
        return jsonify({'success': False, 'message': 'Robot is already moving'})
    
    data = request.get_json() or {}
    target_joints = data.get('joints', None)
    
    if not target_joints or len(target_joints) != 6:
        return jsonify({'success': False, 'message': 'Must provide 6 joint angles'})
    
    try:
        # Enable motors if not already enabled
        if not motors_enabled:
            for i in range(6):
                robot.enable_torque(i + 1, True)
                time.sleep(0.01)
            motors_enabled = True
            time.sleep(0.1)  # Give motors time to enable
        
        # Get current positions
        current_joints = robot.read_joints()
        
        # Set moving flag
        is_moving = True
        
        try:
            # Move to target positions
            steps = data.get('steps', 50)
            timestep = data.get('timestep', 0.02)
            
            # Interpolate movement
            robot.interpolate_waypoint(current_joints, target_joints, steps=steps, timestep=timestep)
            
            # Read final position
            final_joints = robot.read_joints()
            
            return jsonify({
                'success': True,
                'message': 'Moved to target joint positions',
                'initial_joints': current_joints,
                'target_joints': target_joints,
                'final_joints': final_joints,
                'motors_enabled': motors_enabled
            })
            
        finally:
            is_moving = False
            
    except Exception as e:
        is_moving = False
        motors_enabled = False
        return jsonify({'success': False, 'message': str(e)})


@app.route('/api/robot/test_movement', methods=['POST'])
def test_robot_movement():
    """Test robot movement with small motion"""
    global robot, motors_enabled
    
    if not robot:
        return jsonify({'success': False, 'message': 'Robot not connected'})
    
    try:
        # Step 1: Enable all motors with detailed feedback
        print("=" * 50)
        print("TEST MOVEMENT SEQUENCE STARTING")
        print("=" * 50)
        
        print("Step 1: Enabling motors...")
        enable_results = []
        for i in range(6):
            servo_id = i + 1
            success = robot.enable_torque(servo_id, True)
            enable_results.append(success)
            print(f"  Servo {servo_id}: {'✅ Enabled' if success else '❌ Failed'}")
            time.sleep(0.05)
        
        if not all(enable_results):
            failed_servos = [i+1 for i, success in enumerate(enable_results) if not success]
            return jsonify({
                'success': False, 
                'message': f'Failed to enable servos: {failed_servos}',
                'details': {
                    'enable_results': enable_results,
                    'failed_servos': failed_servos
                }
            })
        
        motors_enabled = True
        print("  All motors enabled successfully")
        
        # Step 2: Wait for motors to stabilize
        print("Step 2: Waiting for motors to stabilize...")
        time.sleep(0.5)
        
        # Step 3: Read current position with validation
        print("Step 3: Reading current position...")
        try:
            current = robot.read_joints(validate=False)
            print(f"  Position (rad): {[f'{p:.3f}' for p in current]}")
            print(f"  Position (deg): {[f'{p*57.3:.1f}' for p in current]}")
            
            # Check if position seems valid
            zero_count = sum(1 for p in current if abs(p) < 0.001)
            if zero_count >= 4:
                print("  ⚠️  Warning: Many joints read as zero, position may be invalid")
                
        except Exception as e:
            print(f"  ❌ Failed to read position: {e}")
            return jsonify({
                'success': False,
                'message': f'Failed to read current position: {str(e)}'
            })
        
        # Step 4: Calculate test movement
        print("Step 4: Calculating test movement...")
        test_target = current.copy()
        test_target[1] += 0.15  # Move shoulder joint by 0.15 radians (~8.6 degrees)
        
        print(f"  Current: {[f'{p:.3f}' for p in current]}")
        print(f"  Target:  {[f'{p:.3f}' for p in test_target]}")
        print(f"  Delta:   {[f'{t-c:.3f}' for c, t in zip(current, test_target)]}")
        
        # Step 5: Execute movement
        print("Step 5: Executing movement...")
        print("  Moving forward (30 steps, 1.5 seconds)...")
        
        # Move forward
        robot.interpolate_waypoint(current, test_target, steps=30, timestep=0.05)
        
        # Step 6: Verify movement
        print("Step 6: Verifying movement...")
        time.sleep(0.5)
        final = robot.read_joints(validate=False)
        print(f"  Final:   {[f'{p:.3f}' for p in final]}")
        
        actual_delta = [f-c for f, c in zip(final, current)]
        print(f"  Actual delta: {[f'{d:.3f}' for d in actual_delta]}")
        
        # Check if movement occurred
        movement_detected = abs(actual_delta[1]) > 0.05
        print(f"  Movement detected: {'✅ Yes' if movement_detected else '❌ No'}")
        
        # Step 7: Return to original position
        print("Step 7: Returning to original position...")
        robot.interpolate_waypoint(final, current, steps=30, timestep=0.05)
        
        print("=" * 50)
        print("TEST COMPLETE")
        print("=" * 50)
        
        return jsonify({
            'success': True,
            'message': 'Test movement completed' if movement_detected else 'Test completed but no movement detected',
            'movement_detected': movement_detected,
            'initial': [float(p) for p in current],
            'target': [float(p) for p in test_target],
            'final': [float(p) for p in final],
            'actual_delta': [float(d) for d in actual_delta],
            'motors_enabled': motors_enabled
        })
        
    except Exception as e:
        import traceback
        error_details = traceback.format_exc()
        print(f"❌ Test failed with error: {e}")
        print(error_details)
        return jsonify({
            'success': False, 
            'message': f'Test failed: {str(e)}',
            'error_details': error_details
        })


@app.route('/api/robot/reliability', methods=['GET'])
def get_reliability_stats():
    """Get data reliability statistics"""
    global robot
    
    if not robot:
        return jsonify({'error': 'Robot not connected'})
    
    try:
        # Get statistics from client
        stats = robot.get_stats() if hasattr(robot, 'get_stats') else {}
        
        # Get position history info
        history_info = {}
        if hasattr(robot, 'position_history'):
            for servo_id in range(1, 7):
                hist = robot.position_history.get(servo_id, [])
                if hist:
                    # Convert ticks to radians for statistics
                    radians = [robot.ticks_to_radians(t) for t in hist]
                    history_info[f'servo_{servo_id}'] = {
                        'samples': len(hist),
                        'mean_rad': float(np.mean(radians)),
                        'std_rad': float(np.std(radians)),
                        'min_rad': float(np.min(radians)),
                        'max_rad': float(np.max(radians))
                    }
        
        return jsonify({
            'stats': stats,
            'position_history': history_info,
            'last_valid_positions': [float(p) for p in robot.last_valid_positions] if hasattr(robot, 'last_valid_positions') else None,
            'reliability_features': True
        })
        
    except Exception as e:
        return jsonify({'error': str(e)})

@app.route('/api/robot/joint_states')
def get_current_joint_states():
    """Get current robot joint states"""
    global robot
    
    if not robot:
        return jsonify({'success': False, 'message': 'Robot not connected'})
    
    try:
        # Get current joint positions using read_joints
        joint_positions = robot.read_joints()
        
        # Convert to dictionary format for URDF visualization
        joint_states = {
            '1': joint_positions[0] if len(joint_positions) > 0 else 0,
            '2': joint_positions[1] if len(joint_positions) > 1 else 0,
            '3': joint_positions[2] if len(joint_positions) > 2 else 0,
            '4': joint_positions[3] if len(joint_positions) > 3 else 0,
            '5': joint_positions[4] if len(joint_positions) > 4 else 0,
            '6': joint_positions[5] if len(joint_positions) > 5 else 0
        }
        
        return jsonify({
            'success': True,
            'joint_states': joint_states,
            'joint_positions': joint_positions,
            'timestamp': time.time()
        })
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})


@app.route('/api/robot/urdf_data')
def get_urdf_data():
    """Get URDF data for Three.js visualization"""
    try:
        # Parse URDF directly to get full mesh information
        import xml.etree.ElementTree as ET
        urdf_path = "/home/pathonai/Documents/Github/opensource_dev/GraspingDemo/robot_description/urdf/so101_base.xacro"
        
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        
        urdf_data = {
            'links': {},
            'joints': {}
        }
        
        # Extract links with visual information
        for link in root.findall('.//link'):
            link_name = link.get('name')
            link_data = {'name': link_name, 'visual': None}
            
            # Get visual element
            visual = link.find('visual')
            if visual:
                visual_data = {'geometry': {}, 'origin': {}}
                
                # Get origin
                origin = visual.find('origin')
                if origin:
                    visual_data['origin'] = {
                        'xyz': [float(x) for x in origin.get('xyz', '0 0 0').split()],
                        'rpy': [float(x) for x in origin.get('rpy', '0 0 0').split()]
                    }
                else:
                    visual_data['origin'] = {'xyz': [0, 0, 0], 'rpy': [0, 0, 0]}
                
                # Get geometry
                geometry = visual.find('geometry')
                if geometry:
                    mesh = geometry.find('mesh')
                    if mesh:
                        # Get original filename
                        original_filename = mesh.get('filename', '')
                        # Extract just the STL filename
                        stl_filename = original_filename.split('/')[-1] if '/' in original_filename else original_filename
                        
                        visual_data['geometry']['mesh'] = {
                            'filename': stl_filename,  # Just the filename
                            'scale': [float(x) for x in mesh.get('scale', '1 1 1').split()]
                        }
                
                link_data['visual'] = visual_data
            
            urdf_data['links'][link_name] = link_data
        
        # Extract joints
        for joint in root.findall('.//joint'):
            joint_name = joint.get('name')
            joint_type = joint.get('type')
            
            parent = joint.find('parent')
            child = joint.find('child')
            
            if parent is not None and child is not None:
                joint_data = {
                    'name': joint_name,
                    'type': joint_type,
                    'parent': parent.get('link'),
                    'child': child.get('link'),
                    'axis': [0, 0, 1]  # Default Z-axis like Plotly
                }
                
                # Get joint origin
                origin = joint.find('origin')
                if origin:
                    joint_data['origin'] = {
                        'xyz': [float(x) for x in origin.get('xyz', '0 0 0').split()],
                        'rpy': [float(x) for x in origin.get('rpy', '0 0 0').split()]
                    }
                else:
                    joint_data['origin'] = {'xyz': [0, 0, 0], 'rpy': [0, 0, 0]}
                
                # Get joint limits if available
                limit = joint.find('limit')
                if limit:
                    joint_data['limits'] = {
                        'lower': float(limit.get('lower', '-3.14')),
                        'upper': float(limit.get('upper', '3.14'))
                    }
                
                urdf_data['joints'][joint_name] = joint_data
        
        return jsonify({
            'success': True,
            'urdf_data': urdf_data
        })
            
    except Exception as e:
        return jsonify({
            'success': False,
            'message': f'Failed to load URDF: {e}'
        })


@app.route('/static/meshes/<path:filename>')
def serve_mesh(filename):
    """Serve STL mesh files from robot_description/meshes directory"""
    mesh_dir = '/home/pathonai/Documents/Github/opensource_dev/GraspingDemo/robot_description/meshes/so101'
    return send_from_directory(mesh_dir, filename)


@app.route('/api/robot/update_visualization', methods=['POST'])
def update_robot_visualization():
    """Update robot visualization with specific joint states"""
    data = request.get_json() or {}
    
    # Get joint states from request
    joint_states = data.get('joint_states', None)
    
    if joint_states is None:
        # If no states provided, get current states from robot
        if robot:
            joint_positions = robot.read_joints()
            joint_states = {
                '1': joint_positions[0] if len(joint_positions) > 0 else 0,
                '2': joint_positions[1] if len(joint_positions) > 1 else 0,
                '3': joint_positions[2] if len(joint_positions) > 2 else 0,
                '4': joint_positions[3] if len(joint_positions) > 3 else 0,
                '5': joint_positions[4] if len(joint_positions) > 4 else 0,
                '6': joint_positions[5] if len(joint_positions) > 5 else 0
            }
        else:
            # Use default states
            joint_states = {'1': 0, '2': 0, '3': 0, '4': 0, '5': 0, '6': 0}
    
    # Get visualization parameters
    show_camera = data.get('show_camera', True)
    show_robot = data.get('show_robot', True)
    robot_position = data.get('robot_position', [0, 0, 0])
    robot_rotation = data.get('robot_rotation', [0, 0, 0])  # Roll, Pitch, Yaw in degrees
    
    try:
        # Create visualization with current joint states
        from pointcloud_viewer import vis_pcd_plotly, generate_plotly_html
        
        # Create empty point cloud if needed
        pointclouds = []
        
        # Get camera info if available
        camera_info = None
        if active_camera == 'realsense' and realsense_camera and realsense_camera.intrinsics:
            intrinsics = realsense_camera.intrinsics
            camera_info = {
                'fx': intrinsics.fx,
                'fy': intrinsics.fy,
                'cx': intrinsics.ppx,
                'cy': intrinsics.ppy,
                'width': intrinsics.width,
                'height': intrinsics.height,
                'scale': getattr(realsense_camera, 'depth_scale', 0.001) * 1000
            }
        
        # Create figure with robot at current joint states
        fig = vis_pcd_plotly(
            pointclouds, 
            size_ls=[1],
            title="Robot Visualization",
            show_camera=show_camera,
            camera_info=camera_info,
            show_robot=show_robot,
            robot_joint_angles=joint_states,
            robot_position=np.array(robot_position),
            robot_rotation=np.array(robot_rotation)
        )
        
        # Generate HTML
        html_content = generate_plotly_html(fig)
        
        return html_content, 200, {'Content-Type': 'text/html'}
        
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})


@app.route('/api/plotly/live_with_robot')
def get_live_plotly_with_robot():
    """Get live point cloud with robot visualization"""
    global realsense_camera, active_camera, robot
    
    # Get robot position from request parameters
    robot_pos_x = float(request.args.get('robot_pos_x', 0))
    robot_pos_y = float(request.args.get('robot_pos_y', 0)) 
    robot_pos_z = float(request.args.get('robot_pos_z', 0))
    
    # Get robot rotation from request parameters (in degrees)
    robot_rot_roll = float(request.args.get('robot_rot_roll', 0))
    robot_rot_pitch = float(request.args.get('robot_rot_pitch', 0))
    robot_rot_yaw = float(request.args.get('robot_rot_yaw', 0))
    
    if active_camera != 'realsense' or not realsense_camera:
        # Return robot-only visualization if no camera
        return update_robot_visualization()
    
    try:
        # Get point cloud
        pcd_data = realsense_camera.get_point_cloud(downsample=8, max_points=10000)
        
        if pcd_data:
            vertices = np.array(pcd_data['vertices'])
            colors = np.array(pcd_data['colors'])
            
            # Convert to centimeters
            vertices = vertices * 100
            
            pointclouds = [{
                'vertices': vertices,
                'colors': colors,
                'name': 'Live Point Cloud'
            }]
        else:
            pointclouds = []
        
        # Get camera info
        camera_info = None
        if realsense_camera and realsense_camera.intrinsics:
            intrinsics = realsense_camera.intrinsics
            camera_info = {
                'fx': intrinsics.fx,
                'fy': intrinsics.fy,
                'cx': intrinsics.ppx,
                'cy': intrinsics.ppy,
                'width': intrinsics.width,
                'height': intrinsics.height,
                'scale': getattr(realsense_camera, 'depth_scale', 0.001) * 1000
            }
        
        # Get current robot joint states
        robot_joint_angles = None
        if robot:
            try:
                joint_positions = robot.read_joints()
                robot_joint_angles = {
                    '1': joint_positions[0] if len(joint_positions) > 0 else 0,
                    '2': joint_positions[1] if len(joint_positions) > 1 else 0,
                    '3': joint_positions[2] if len(joint_positions) > 2 else 0,
                    '4': joint_positions[3] if len(joint_positions) > 3 else 0,
                    '5': joint_positions[4] if len(joint_positions) > 4 else 0,
                    '6': joint_positions[5] if len(joint_positions) > 5 else 0
                }
            except:
                pass  # Use default angles if can't get from robot
        
        # Create visualization with both point cloud and robot
        fig = vis_pcd_plotly(
            pointclouds, 
            size_ls=[1],
            title="Live Scene with Robot",
            show_camera=True,
            camera_info=camera_info,
            show_robot=True,
            robot_joint_angles=robot_joint_angles,
            robot_position=np.array([robot_pos_x, robot_pos_y, robot_pos_z]),
            robot_rotation=np.array([robot_rot_roll, robot_rot_pitch, robot_rot_yaw])
        )
        
        # Generate HTML
        html_content = generate_plotly_html(fig, div_id="live-scene")
        
        return html_content, 200, {'Content-Type': 'text/html'}
        
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})


@app.route('/api/coordinate/pixel_to_3d', methods=['POST'])
def pixel_to_3d():
    """Convert pixel coordinates to 3D coordinates"""
    data = request.get_json() or {}
    
    u = data.get('u', 320)
    v = data.get('v', 240)
    depth = data.get('depth', 1000)  # Default 1 meter
    
    # Initialize transformer
    transform = CoordinateTransform()
    
    # Set camera intrinsics if available
    global realsense_camera, active_camera
    if active_camera == 'realsense' and realsense_camera and realsense_camera.intrinsics:
        intrinsics = realsense_camera.intrinsics
        transform.camera_intrinsics = {
            'fx': intrinsics.fx,
            'fy': intrinsics.fy,
            'cx': intrinsics.ppx,
            'cy': intrinsics.ppy,
            'depth_scale': getattr(realsense_camera, 'depth_scale', 0.001)
        }
    
    # Convert to camera coordinates
    camera_point = transform.pixel_to_camera(u, v, depth)
    
    # Convert to world coordinates
    world_point = transform.transform_point_to_world(camera_point)
    
    return jsonify({
        'pixel': {'u': u, 'v': v},
        'depth_mm': depth,
        'camera_coords': {
            'x': float(camera_point[0]),
            'y': float(camera_point[1]),
            'z': float(camera_point[2]),
            'units': 'meters'
        },
        'world_coords': {
            'X': float(world_point[0]),
            'Y': float(world_point[1]),
            'Z': float(world_point[2]),
            'units': 'meters'
        }
    })


if __name__ == '__main__':
    import sys
    
    # Allow port to be specified as command line argument
    port = 5000
    if len(sys.argv) > 1:
        try:
            port = int(sys.argv[1])
        except ValueError:
            print(f"Invalid port number: {sys.argv[1]}, using default port 5000")
    
    print(f"Starting SO-101 Web Control on port {port}...")
    print(f"Open browser at: http://localhost:{port}")
    
    try:
        app.run(host='0.0.0.0', port=port, debug=False)
    except OSError as e:
        if "Address already in use" in str(e):
            print(f"\nError: Port {port} is already in use!")
            print(f"Try a different port: python app.py 5001")
        else:
            raise