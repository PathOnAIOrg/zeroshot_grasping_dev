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
import io
import logging

# Try to import cv2 for image handling
try:
    import cv2
    cv2_available = True
except ImportError:
    print("Warning: OpenCV not installed. Some features may be limited.")
    cv2_available = False

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import the improved main client with built-in reliability features
try:
    from so101_grasp.robot.so101_client_raw_simple import SO101ClientRawSimple
    
    # Try to use fast kinematics first
    try:
        from so101_grasp.robot.so101_kinematics_fast import SO101KinematicsFast as SO101Kinematics
        print("✅ Using optimized fast kinematics solver")
    except ImportError:
        from so101_grasp.robot.so101_kinematics import SO101Kinematics
        print("ℹ️ Using standard kinematics solver")
    
    HAS_ROBOT = True
except (ImportError, ModuleNotFoundError) as e:
    print(f"⚠️ Warning: Robot control modules not available: {e}")
    SO101ClientRawSimple = None
    SO101Kinematics = None
    HAS_ROBOT = False
    
    # Create dummy kinematics class
    class SO101Kinematics:
        def __init__(self):
            pass

# Import hand-eye calibration modules
handeye_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'handeye', 'handeye_calibration')
sys.path.insert(0, handeye_path)
print(f"Adding handeye path: {handeye_path}")

try:
    from handeye_manual_calibration import ManualHandEyeCalibrator
    from handeye_manual_realsense import RealSenseCalibrator
    from handeye_manual_realsense_stationary import StationaryRealSenseCalibrator
    calibration_available = True
    print("✅ Hand-eye calibration modules loaded successfully")
except ImportError as e:
    print(f"⚠️ Warning: Hand-eye calibration modules not available: {e}")
    calibration_available = False
    ManualHandEyeCalibrator = None
    RealSenseCalibrator = None
    StationaryRealSenseCalibrator = None

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

app = Flask(__name__, static_folder='static', static_url_path='/static')

# Configure logging to suppress request logs for frequent endpoints
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)  # Only show errors, not regular requests

# If you want to selectively log certain endpoints, you can use a custom filter
class RequestFilter(logging.Filter):
    def filter(self, record):
        # Filter out frequent status and streaming endpoints
        if hasattr(record, 'getMessage'):
            message = record.getMessage()
            # List of endpoints to suppress logging for
            suppress_endpoints = [
                '/api/status',
                '/api/robot/joint_states',
                '/api/camera/rgb',
                '/api/camera/depth',
                '/api/camera/pointcloud'
            ]
            for endpoint in suppress_endpoints:
                if endpoint in message:
                    return False
        return True

# Apply the filter to werkzeug logger
werkzeug_logger = logging.getLogger('werkzeug')
werkzeug_logger.addFilter(RequestFilter())

# Import and register motion planning API if available
try:
    from motion_planning_api import motion_api
    app.register_blueprint(motion_api)
    print("✅ Motion planning API registered")
    HAS_MOTION_PLANNER = True
except ImportError as e:
    print(f"⚠️ Motion planning API not available: {e}")
    HAS_MOTION_PLANNER = False

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

# Shared RealSense pipeline for all operations
shared_realsense_pipeline = None
shared_realsense_config = None
shared_realsense_lock = threading.Lock()

# Global calibration handler
current_calibrator = None
calibration_session = {}

def get_shared_realsense():
    """Get or create shared RealSense pipeline"""
    global shared_realsense_pipeline, shared_realsense_config
    
    with shared_realsense_lock:
        if shared_realsense_pipeline is None:
            try:
                import pyrealsense2 as rs
                shared_realsense_pipeline = rs.pipeline()
                shared_realsense_config = rs.config()
                shared_realsense_config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
                shared_realsense_config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
                shared_realsense_pipeline.start(shared_realsense_config)
                print("✅ Shared RealSense pipeline initialized")
            except Exception as e:
                print(f"❌ Failed to initialize shared RealSense: {e}")
                return None
        return shared_realsense_pipeline

def release_shared_realsense():
    """Release shared RealSense pipeline"""
    global shared_realsense_pipeline
    
    with shared_realsense_lock:
        if shared_realsense_pipeline:
            try:
                shared_realsense_pipeline.stop()
                print("✅ Shared RealSense pipeline released")
            except:
                pass
            shared_realsense_pipeline = None

# Trajectories directory
TRAJ_DIR = Path.home() / "Documents/Github/opensource_dev/GraspingDemo/trajectories"
TRAJ_DIR.mkdir(exist_ok=True)


def connect_robot():
    """Connect to robot"""
    global robot, current_status
    
    if not HAS_ROBOT or SO101ClientRawSimple is None:
        current_status = "Robot control module not available"
        return False
        
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
    """Main page - classic UI"""
    return render_template('index.html')


@app.route('/favicon.ico')
def favicon():
    """Serve a simple favicon to avoid 404 errors"""
    import io
    import base64
    from flask import send_file
    
    # Robot icon as favicon (base64 encoded PNG, 32x32)
    favicon_base64 = "iVBORw0KGgoAAAANSUhEUgAAACAAAAAgCAYAAABzenr0AAAABHNCSVQICAgIfAhkiAAAAAlwSFlzAAAA7AAAAOwBeShxvQAAABl0RVh0U29mdHdhcmUAd3d3Lmlua3NjYXBlLm9yZ5vuPBoAAAJFSURBVFiFxZe9jtNAEMc/r+04jhMnF3EREqJBQkI0NDRQQ0NHxwNQ8QI8AS0VDRUPQMUDUFFRIKWgoEFCQkI6JM5HnNiO7bU3FLuzXjuXO+4QI61kvTM7/5nZnZ0d8N13lFIopRARRATXdRERAFzXRUSw1iIiAKRpirUWYwzWWowxWGsxxmCMwVqLMQZrLdZalFIURUGe50RRhIig63qZnmVZBX6SJKRpSpZlFEVBURTkeU6WZeR5TpZlFVqe52RZRpqmpGlKkiQkScLg7u5u+uHDhxGASqmS5VEUATCfz0mSpFTkeR5BEBCGIb7v43keYRji+z6u6yIiRFFEEATlO4Du9XplVsdxSJKELMsq4BtskWVZOZ7neaXK8zwnCAJc16Xb7VbStCiKaDQaZUYcx0EphVIKx3FQSlWA5jiOtDqxcxxHRKQ1UUqJUkocxxGlVDmmta78aa3r2nHWAr7ZdWx37eYHqOs6tm3bqOV5XqmgtY7W2lp0SqkqWOvAnNj9r5BqiCpgNVJyXRegtppPTk4AWCwWtGJd1xkMBgBsbW2xs7MDwPHxMZeXl7X7m80mW1tbrFYrZrNZvRhjODo6wmqtaTabKKXY3d1ld3cXgMPDQ7LsD5ZzQCml3r171394aqytVqsNJGzfuKfT6crUB4MBvu+ztbWFtZbdbMb5+fm6JHw8PiGJk9+W+uJLnHVQjhhjODg4IMsyXNclDMN/Pl4uut0uvV6P/f39G82BwcHBAbPZjOl0yvPnzzeFmKIovskvIYQQQvwEP/XpMXQFCrIAAAAASUVORK5CYII="
    
    # Decode base64 to bytes
    favicon_bytes = base64.b64decode(favicon_base64)
    
    # Return as PNG image
    return send_file(
        io.BytesIO(favicon_bytes),
        mimetype='image/png'
    )


@app.route('/modern')
def modern():
    """Modern UI version"""
    return render_template('index_modern.html')

@app.route('/unified')
def unified():
    """Unified UI with theme switching"""
    return render_template('index_unified.html')


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


@app.route('/api/motor/status')
def get_motor_status():
    """Get current motor lock status"""
    global motors_enabled, robot
    
    return jsonify({
        'connected': robot is not None,
        'motors_locked': motors_enabled,
        'status': 'Locked' if motors_enabled else 'Unlocked (Manual Mode)',
        'can_move': motors_enabled,
        'timestamp': time.time()
    })


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
        return jsonify({'success': True, 'message': 'All servos enabled', 'motors_locked': True})
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
        return jsonify({'success': True, 'message': 'All servos disabled - move manually', 'motors_locked': False})
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
    """Get current gripper pose in cartesian coordinates using forward kinematics"""
    if not robot:
        return jsonify({'success': False, 'message': 'Robot not connected'})
    
    try:
        # Read current joint positions
        joints = robot.read_joints(validate=False)
        
        # Calculate gripper pose using forward kinematics
        position, orientation = kinematics.forward_kinematics(joints)
        
        # Convert orientation matrix to Euler angles and quaternion for display
        from scipy.spatial.transform import Rotation
        r = Rotation.from_matrix(orientation)
        euler_angles = r.as_euler('xyz', degrees=True)
        quaternion = r.as_quat()  # Returns [x, y, z, w]
        
        pose = {
            'position': {
                'x': float(position[0]),
                'y': float(position[1]),
                'z': float(position[2])
            },
            'orientation': {
                'roll': float(euler_angles[0]),
                'pitch': float(euler_angles[1]),
                'yaw': float(euler_angles[2])
            },
            'quaternion': {
                'w': float(quaternion[3]),
                'x': float(quaternion[0]),
                'y': float(quaternion[1]),
                'z': float(quaternion[2])
            },
            'position_mm': {
                'x': float(position[0] * 1000),
                'y': float(position[1] * 1000),
                'z': float(position[2] * 1000)
            }
        }
        
        return jsonify({
            'success': True,
            'pose': pose,
            'joints': joints
        })
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})


@app.route('/api/cartesian_move', methods=['POST'])
def cartesian_move():
    """Move gripper in cartesian space using forward/inverse kinematics"""
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
        
        # Calculate current end-effector position using forward kinematics
        current_pos, current_rot = kinematics.forward_kinematics(current_joints)
        
        # Calculate target position based on direction
        target_pos = current_pos.copy()
        if direction in ['forward', 'x+']:
            target_pos[0] += distance
        elif direction in ['backward', 'back', 'x-']:
            target_pos[0] -= distance
        elif direction in ['left', 'y-']:
            target_pos[1] -= distance  # Left is -Y in robot frame
        elif direction in ['right', 'y+']:
            target_pos[1] += distance  # Right is +Y in robot frame
        elif direction in ['up', 'z+']:
            target_pos[2] += distance
        elif direction in ['down', 'z-']:
            target_pos[2] -= distance
        else:
            return jsonify({'success': False, 'message': f'Unknown direction: {direction}'})
        
        # Calculate new joint positions using inverse kinematics
        new_joints = kinematics.inverse_kinematics(target_pos, current_joints, current_rot)
        
        if new_joints is None:
            return jsonify({'success': False, 'message': 'Target position unreachable'})
        
        # Set moving flag
        is_moving = True
        try:
            # Move to new position smoothly
            robot.interpolate_waypoint(current_joints, new_joints, steps=50, timestep=0.02)
            
            # Get new gripper pose
            new_pos, new_rot = kinematics.forward_kinematics(new_joints)
            
            return jsonify({
                'success': True,
                'message': f'Moved {direction} {distance*1000:.1f}mm',
                'new_pose': {
                    'position': new_pos.tolist(),
                    'joints': new_joints
                },
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
        new_joints = list(current_joints)  # Make a copy
        
        # Handle specific joint rotations
        if axis == 'base':
            # Rotate joint 1 (base rotation)
            new_joints[0] += angle_rad
            # Clamp to joint limits
            new_joints[0] = max(-1.92, min(1.92, new_joints[0]))
            
        elif axis == 'wrist_pitch':
            # Rotate joint 4 (wrist pitch)
            new_joints[3] += angle_rad
            # Clamp to joint limits
            new_joints[3] = max(-1.66, min(1.66, new_joints[3]))
            
        elif axis == 'wrist_roll':
            # Rotate joint 5 (wrist roll)
            new_joints[4] += angle_rad
            # Clamp to joint limits
            new_joints[4] = max(-2.79, min(2.79, new_joints[4]))
            
        elif axis in ['x', 'roll', 'y', 'pitch', 'z', 'yaw']:
            # Use IK for end-effector rotations
            current_pos, current_rot = kinematics.forward_kinematics(current_joints)
            
            # Create rotation matrix for the desired rotation
            from scipy.spatial.transform import Rotation
            if axis in ['x', 'roll']:
                rot_matrix = Rotation.from_euler('x', angle_rad).as_matrix()
            elif axis in ['y', 'pitch']:
                rot_matrix = Rotation.from_euler('y', angle_rad).as_matrix()
            elif axis in ['z', 'yaw']:
                rot_matrix = Rotation.from_euler('z', angle_rad).as_matrix()
            
            # Apply rotation to current orientation
            new_orientation = current_rot @ rot_matrix
            
            # Calculate new joint positions using IK with new orientation
            ik_solution = kinematics.inverse_kinematics(current_pos, current_joints, new_orientation)
            if ik_solution is None:
                return jsonify({'success': False, 'message': 'Rotation unreachable'})
            new_joints = ik_solution
        else:
            return jsonify({'success': False, 'message': f'Unknown axis: {axis}'})
        
        # Set moving flag
        is_moving = True
        try:
            # Move to new position
            robot.interpolate_waypoint(current_joints, new_joints, steps=20, timestep=0.02)
            
            # Get new gripper pose using forward kinematics
            new_pos, new_rot = kinematics.forward_kinematics(new_joints)
            
            # Convert to pose format
            r = Rotation.from_matrix(new_rot)
            euler_angles = r.as_euler('xyz', degrees=False)  # Return in radians
            
            new_pose = {
                'position': {
                    'x': float(new_pos[0]),
                    'y': float(new_pos[1]),
                    'z': float(new_pos[2])
                },
                'orientation': {
                    'roll': float(euler_angles[0]),
                    'pitch': float(euler_angles[1]),
                    'yaw': float(euler_angles[2])
                },
                'position_mm': {
                    'x': float(new_pos[0] * 1000),
                    'y': float(new_pos[1] * 1000),
                    'z': float(new_pos[2] * 1000)
                }
            }
            
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
    """Move gripper to specific cartesian position using inverse kinematics"""
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
        
        # Get current orientation (optional, to maintain it)
        _, current_orientation = kinematics.forward_kinematics(current_joints)
        
        # Calculate IK solution
        new_joints = kinematics.inverse_kinematics(target_pos, current_joints, current_orientation)
        
        if new_joints is None:
            return jsonify({'success': False, 'message': 'Target position unreachable'})
        
        # Set moving flag
        is_moving = True
        try:
            # Move to target
            robot.interpolate_waypoint(current_joints, new_joints, steps=50, timestep=0.02)
            
            # Get actual pose using forward kinematics
            final_pos, final_rot = kinematics.forward_kinematics(new_joints)
            
            # Convert to pose format
            from scipy.spatial.transform import Rotation
            r = Rotation.from_matrix(final_rot)
            euler_angles = r.as_euler('xyz', degrees=True)
            
            final_pose = {
                'position': final_pos.tolist(),
                'orientation': euler_angles.tolist(),
                'position_mm': (final_pos * 1000).tolist()
            }
            
            return jsonify({
                'success': True,
                'message': f'Moved to position',
                'pose': final_pose,
                'joint_solution': new_joints[:5],  # Return first 5 joints for IK display
                'motors_enabled': motors_enabled
            })
        finally:
            is_moving = False
            
    except Exception as e:
        is_moving = False
        return jsonify({'success': False, 'message': str(e)})


@app.route('/api/inverse_kinematics', methods=['POST'])
def inverse_kinematics():
    """Compute inverse kinematics for a target position"""
    try:
        data = request.get_json() or {}
        position = data.get('position', [0.2, 0.0, 0.2])
        current_joints = data.get('current_joints', [0, 0, 0, 0, 0, 0])
        
        # Validate input
        if not isinstance(position, (list, tuple)) or len(position) != 3:
            return jsonify({
                'success': False,
                'message': 'Position must be [x, y, z] in meters'
            })
        
        # Convert to numpy array
        target_pos = np.array(position)
        
        # Compute IK solution
        solution = kinematics.inverse_kinematics(target_pos, current_joints)
        
        if solution is None:
            return jsonify({
                'success': False,
                'message': 'No IK solution found for target position'
            })
        
        # Verify solution with forward kinematics
        pos, rot = kinematics.forward_kinematics(solution)
        error = np.linalg.norm(pos - target_pos)
        
        return jsonify({
            'success': True,
            'joints': solution[:6],  # Return all 6 joints
            'error_mm': float(error * 1000),
            'verified_position': pos.tolist()
        })
        
    except Exception as e:
        return jsonify({
            'success': False,
            'message': f'IK computation failed: {str(e)}'
        })


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


@app.route('/api/camera/shared_status')
def shared_camera_status():
    """Get shared RealSense camera status"""
    global shared_realsense_pipeline
    
    return jsonify({
        'connected': shared_realsense_pipeline is not None,
        'type': 'RealSense (Shared)',
        'info': {
            'resolution': '640x480',
            'fps': 30,
            'streams': ['color', 'depth']
        } if shared_realsense_pipeline else None
    })

@app.route('/api/camera/shared_connect', methods=['POST'])
def shared_camera_connect():
    """Initialize shared RealSense camera"""
    pipeline = get_shared_realsense()
    if pipeline:
        return jsonify({'success': True, 'message': 'Shared camera initialized'})
    else:
        return jsonify({'success': False, 'message': 'Failed to initialize shared camera'})

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
        
        # Add grasp pose visualization if provided
        show_grasp = request.args.get('show_grasp', 'false').lower() == 'true'
        if show_grasp:
            grasp_pose_str = request.args.get('grasp_pose')
            if grasp_pose_str:
                try:
                    import plotly.graph_objects as go
                    grasp_pose = json.loads(grasp_pose_str)
                    
                    # Convert position from mm to cm
                    pos_cm = [x / 10 for x in grasp_pose['xyz']]
                    rot = grasp_pose['rot']
                    grasp_depth = grasp_pose.get('dep', 0.05)  # Grasp depth/width
                    
                    # Create two-finger gripper visualization
                    gripper_width = grasp_depth * 100  # Convert to cm
                    finger_length = 8  # 8cm finger length
                    finger_width = 1.5  # 1.5cm finger width
                    palm_height = 3    # 3cm palm height
                    
                    # Add gripper palm/base as a box
                    palm_x = [pos_cm[0] - 3, pos_cm[0] + 3, pos_cm[0] + 3, pos_cm[0] - 3,
                             pos_cm[0] - 3, pos_cm[0] + 3, pos_cm[0] + 3, pos_cm[0] - 3]
                    palm_y = [pos_cm[1] - 0.5, pos_cm[1] - 0.5, pos_cm[1] + 0.5, pos_cm[1] + 0.5,
                             pos_cm[1] - 0.5, pos_cm[1] - 0.5, pos_cm[1] + 0.5, pos_cm[1] + 0.5]
                    palm_z = [pos_cm[2], pos_cm[2], pos_cm[2], pos_cm[2],
                             pos_cm[2] + palm_height, pos_cm[2] + palm_height, pos_cm[2] + palm_height, pos_cm[2] + palm_height]
                    
                    # Add gripper palm mesh
                    fig.add_trace(go.Mesh3d(
                        x=palm_x, y=palm_y, z=palm_z,
                        i=[0, 0, 0, 0, 4, 4, 1, 2],
                        j=[1, 2, 4, 3, 5, 7, 5, 6],
                        k=[2, 3, 5, 7, 6, 3, 6, 7],
                        color='gray',
                        opacity=0.7,
                        name='Gripper Palm',
                        showlegend=False
                    ))
                    
                    # Add left finger as lines
                    left_finger_x = [
                        pos_cm[0] - gripper_width/2,  # Base
                        pos_cm[0] - gripper_width/2,  # Tip
                        None,
                        pos_cm[0] - gripper_width/2 - finger_width/2,  # Side edge 1
                        pos_cm[0] - gripper_width/2 - finger_width/2,
                        None,
                        pos_cm[0] - gripper_width/2 + finger_width/2,  # Side edge 2
                        pos_cm[0] - gripper_width/2 + finger_width/2,
                    ]
                    left_finger_y = [
                        pos_cm[1], pos_cm[1], None,
                        pos_cm[1], pos_cm[1], None,
                        pos_cm[1], pos_cm[1]
                    ]
                    left_finger_z = [
                        pos_cm[2], pos_cm[2] - finger_length, None,
                        pos_cm[2], pos_cm[2] - finger_length, None,
                        pos_cm[2], pos_cm[2] - finger_length
                    ]
                    
                    fig.add_trace(go.Scatter3d(
                        x=left_finger_x, y=left_finger_y, z=left_finger_z,
                        mode='lines',
                        line=dict(color='green', width=8),
                        name='Left Finger',
                        showlegend=False
                    ))
                    
                    # Add right finger as lines
                    right_finger_x = [
                        pos_cm[0] + gripper_width/2,  # Base
                        pos_cm[0] + gripper_width/2,  # Tip
                        None,
                        pos_cm[0] + gripper_width/2 - finger_width/2,  # Side edge 1
                        pos_cm[0] + gripper_width/2 - finger_width/2,
                        None,
                        pos_cm[0] + gripper_width/2 + finger_width/2,  # Side edge 2
                        pos_cm[0] + gripper_width/2 + finger_width/2,
                    ]
                    right_finger_y = left_finger_y  # Same Y positions
                    right_finger_z = left_finger_z  # Same Z positions
                    
                    fig.add_trace(go.Scatter3d(
                        x=right_finger_x, y=right_finger_y, z=right_finger_z,
                        mode='lines',
                        line=dict(color='green', width=8),
                        name='Right Finger',
                        showlegend=False
                    ))
                    
                    # Add contact points on fingers
                    contact_points_x = [
                        pos_cm[0] - gripper_width/2,  # Left finger contact
                        pos_cm[0] + gripper_width/2   # Right finger contact
                    ]
                    contact_points_y = [pos_cm[1], pos_cm[1]]
                    contact_points_z = [pos_cm[2] - finger_length * 0.7, pos_cm[2] - finger_length * 0.7]
                    
                    fig.add_trace(go.Scatter3d(
                        x=contact_points_x,
                        y=contact_points_y,
                        z=contact_points_z,
                        mode='markers',
                        marker=dict(size=10, color='gold', symbol='circle',
                                  line=dict(color='orange', width=2)),
                        name='Contact Points',
                        showlegend=False
                    ))
                    
                    # Add grasp width indicator
                    fig.add_trace(go.Scatter3d(
                        x=[pos_cm[0] - gripper_width/2, pos_cm[0] + gripper_width/2],
                        y=[pos_cm[1], pos_cm[1]],
                        z=[pos_cm[2] - finger_length * 0.5, pos_cm[2] - finger_length * 0.5],
                        mode='lines+text',
                        line=dict(color='cyan', width=3, dash='dash'),
                        text=['', f'{gripper_width:.1f}cm'],
                        textposition='top center',
                        name='Grasp Width',
                        showlegend=False
                    ))
                    
                    # Add approach arrow (smaller cone)
                    fig.add_trace(go.Cone(
                        x=[pos_cm[0]],
                        y=[pos_cm[1]],
                        z=[pos_cm[2] + finger_length],
                        u=[0],
                        v=[0],
                        w=[-5],  # Pointing down
                        sizemode='absolute',
                        sizeref=3,
                        anchor='tip',
                        colorscale=[[0, 'red'], [1, 'red']],
                        showscale=False,
                        name='Approach Direction',
                        opacity=0.6
                    ))
                    
                    # Add coordinate axes
                    axis_length = 10  # 10cm
                    
                    # X-axis (red)
                    fig.add_trace(go.Scatter3d(
                        x=[pos_cm[0], pos_cm[0] + rot[0][0] * axis_length],
                        y=[pos_cm[1], pos_cm[1] + rot[1][0] * axis_length],
                        z=[pos_cm[2], pos_cm[2] + rot[2][0] * axis_length],
                        mode='lines',
                        line=dict(color='red', width=5),
                        name='Grasp X',
                        showlegend=False
                    ))
                    
                    # Y-axis (green)
                    fig.add_trace(go.Scatter3d(
                        x=[pos_cm[0], pos_cm[0] + rot[0][1] * axis_length],
                        y=[pos_cm[1], pos_cm[1] + rot[1][1] * axis_length],
                        z=[pos_cm[2], pos_cm[2] + rot[2][1] * axis_length],
                        mode='lines',
                        line=dict(color='green', width=5),
                        name='Grasp Y',
                        showlegend=False
                    ))
                    
                    # Z-axis (blue) - approach direction
                    fig.add_trace(go.Scatter3d(
                        x=[pos_cm[0], pos_cm[0] + rot[0][2] * axis_length],
                        y=[pos_cm[1], pos_cm[1] + rot[1][2] * axis_length],
                        z=[pos_cm[2], pos_cm[2] + rot[2][2] * axis_length],
                        mode='lines',
                        line=dict(color='blue', width=5),
                        name='Grasp Z (approach)',
                        showlegend=False
                    ))
                    
                    # Add grasp point marker
                    fig.add_trace(go.Scatter3d(
                        x=[pos_cm[0]],
                        y=[pos_cm[1]],
                        z=[pos_cm[2]],
                        mode='markers+text',
                        marker=dict(size=15, color='lime', symbol='diamond', 
                                  line=dict(color='darkgreen', width=2)),
                        text=['Grasp Target'],
                        textposition='top center',
                        textfont=dict(size=14, color='lime'),
                        name='Grasp Point'
                    ))
                    
                    # Add arm visualization if joint states are provided
                    grasp_joint_states_str = request.args.get('grasp_joint_states')
                    if grasp_joint_states_str:
                        try:
                            joint_states = json.loads(grasp_joint_states_str)
                            
                            # Robot arm parameters (simplified SO-ARM100 dimensions in cm)
                            link_lengths = [10, 20, 20, 10]  # Base to shoulder, upper arm, forearm, wrist
                            
                            # Extract joint angles (assuming they're in radians)
                            joints = [
                                joint_states.get('joint_1', 0),
                                joint_states.get('joint_2', 0),
                                joint_states.get('joint_3', 0),
                                joint_states.get('joint_4', 0),
                                joint_states.get('joint_5', 0),
                                joint_states.get('joint_6', 0)
                            ]
                            
                            # Calculate forward kinematics (simplified)
                            joint_positions = []
                            
                            # Base position
                            base_pos = [0, 0, 0]
                            joint_positions.append(base_pos)
                            
                            # Shoulder position (vertical from base)
                            shoulder_pos = [0, 0, link_lengths[0]]
                            joint_positions.append(shoulder_pos)
                            
                            # Elbow position (using shoulder and elbow angles)
                            elbow_x = link_lengths[1] * np.cos(joints[1]) * np.cos(joints[0])
                            elbow_y = link_lengths[1] * np.cos(joints[1]) * np.sin(joints[0])
                            elbow_z = shoulder_pos[2] + link_lengths[1] * np.sin(joints[1])
                            joint_positions.append([elbow_x, elbow_y, elbow_z])
                            
                            # Wrist position
                            wrist_angle = joints[1] + joints[2]
                            wrist_x = elbow_x + link_lengths[2] * np.cos(wrist_angle) * np.cos(joints[0])
                            wrist_y = elbow_y + link_lengths[2] * np.cos(wrist_angle) * np.sin(joints[0])
                            wrist_z = elbow_z + link_lengths[2] * np.sin(wrist_angle)
                            joint_positions.append([wrist_x, wrist_y, wrist_z])
                            
                            # End effector position
                            end_angle = wrist_angle + joints[3]
                            end_x = wrist_x + link_lengths[3] * np.cos(end_angle) * np.cos(joints[0])
                            end_y = wrist_y + link_lengths[3] * np.cos(end_angle) * np.sin(joints[0])
                            end_z = wrist_z + link_lengths[3] * np.sin(end_angle)
                            joint_positions.append([end_x, end_y, end_z])
                            
                            # Draw arm links
                            for i in range(len(joint_positions) - 1):
                                start = joint_positions[i]
                                end = joint_positions[i + 1]
                                
                                # Add link as a line
                                fig.add_trace(go.Scatter3d(
                                    x=[start[0], end[0]],
                                    y=[start[1], end[1]],
                                    z=[start[2], end[2]],
                                    mode='lines',
                                    line=dict(color='orange', width=10),
                                    name=f'Link {i+1}',
                                    showlegend=False
                                ))
                                
                                # Add joint as a sphere
                                fig.add_trace(go.Scatter3d(
                                    x=[end[0]],
                                    y=[end[1]],
                                    z=[end[2]],
                                    mode='markers',
                                    marker=dict(size=12, color='gold', symbol='circle',
                                              line=dict(color='darkorange', width=2)),
                                    name=f'Joint {i+1}',
                                    showlegend=False
                                ))
                            
                            # Add base joint
                            fig.add_trace(go.Scatter3d(
                                x=[base_pos[0]],
                                y=[base_pos[1]],
                                z=[base_pos[2]],
                                mode='markers',
                                marker=dict(size=15, color='darkred', symbol='square'),
                                name='Base',
                                showlegend=False
                            ))
                            
                            # Add trajectory from end effector to grasp target
                            fig.add_trace(go.Scatter3d(
                                x=[end_x, pos_cm[0]],
                                y=[end_y, pos_cm[1]],
                                z=[end_z, pos_cm[2]],
                                mode='lines+markers',
                                line=dict(color='lime', width=3, dash='dash'),
                                marker=dict(size=8, color='lime'),
                                name='Grasp Trajectory',
                                text=['End Effector', 'Grasp Target'],
                                textposition='top center'
                            ))
                            
                            # Add distance annotation
                            distance = np.sqrt((end_x - pos_cm[0])**2 + 
                                             (end_y - pos_cm[1])**2 + 
                                             (end_z - pos_cm[2])**2)
                            
                            fig.add_trace(go.Scatter3d(
                                x=[(end_x + pos_cm[0])/2],
                                y=[(end_y + pos_cm[1])/2],
                                z=[(end_z + pos_cm[2])/2],
                                mode='text',
                                text=[f'Distance: {distance:.1f}cm'],
                                textfont=dict(size=12, color='yellow'),
                                showlegend=False
                            ))
                            
                            print(f"Added arm configuration visualization with joints: {joints}")
                            
                        except Exception as e:
                            print(f"Error adding arm visualization: {e}")
                    
                    # Update camera to focus on grasp
                    fig.update_layout(
                        scene_camera=dict(
                            eye=dict(x=1.5, y=1.5, z=1.0),
                            center=dict(x=pos_cm[0]/100, y=pos_cm[1]/100, z=pos_cm[2]/100)
                        )
                    )
                    
                    print(f"Added grasp pose visualization at: {pos_cm}")
                    
                except Exception as e:
                    print(f"Error adding grasp visualization: {e}")
        
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


@app.route('/api/set_joints', methods=['POST'])
def set_joints():
    """Set robot joint positions from IK solution"""
    global robot, is_moving, motors_enabled
    
    if not robot:
        return jsonify({'success': False, 'message': 'Robot not connected'})
    
    if is_moving:
        return jsonify({'success': False, 'message': 'Robot is already moving'})
    
    data = request.get_json() or {}
    joints_dict = data.get('joints', {})
    
    # Convert dictionary to array
    target_joints = [
        joints_dict.get('joint_1', 0),
        joints_dict.get('joint_2', 0),
        joints_dict.get('joint_3', 0),
        joints_dict.get('joint_4', 0),
        joints_dict.get('joint_5', 0),
        robot.read_joints()[5] if robot else 0  # Keep current gripper position
    ]
    
    try:
        # Enable motors if not already enabled
        if not motors_enabled:
            for i in range(6):
                robot.enable_torque(i + 1, True)
                time.sleep(0.01)
            motors_enabled = True
            time.sleep(0.1)
        
        # Get current positions
        current_joints = robot.read_joints()
        
        # Set moving flag
        is_moving = True
        
        try:
            # Move to target positions
            robot.interpolate_waypoint(current_joints, target_joints, steps=50, timestep=0.02)
            
            # Read final position
            final_joints = robot.read_joints()
            
            return jsonify({
                'success': True,
                'message': 'Applied IK joint states successfully',
                'final_joints': final_joints,
                'motors_enabled': motors_enabled
            })
            
        finally:
            is_moving = False
            
    except Exception as e:
        is_moving = False
        return jsonify({'success': False, 'message': str(e)})


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


def transform_camera_to_world(position_camera, camera_intrinsics, depth_array, depth_scale, robot_transform=None):
    """
    Transform position from camera coordinates to world coordinates.
    
    Args:
        position_camera: [x, y, z] in camera frame (meters)
        camera_intrinsics: Camera intrinsic parameters (not used in simple transform)
        depth_array: Depth image array (not used in simple transform)
        depth_scale: Scale factor to convert depth units to meters
        robot_transform: Not used - grasp stays in world coordinates
    
    Returns:
        Position in world coordinates [x, y, z] in meters
    """
    # Camera frame to world frame transformation
    # Typical transformation for robotic applications:
    # Camera: X-right, Y-down, Z-forward (RealSense convention)
    # World: X-forward, Y-left, Z-up (robot/point cloud convention)
    
    x_cam, y_cam, z_cam = position_camera
    
    # Apply coordinate frame transformation
    # This is a common transformation for RGB-D cameras in robotics
    x_world = z_cam   # Forward in world is depth in camera
    y_world = -x_cam  # Left in world is negative right in camera  
    z_world = -y_cam  # Up in world is negative down in camera
    
    # NOTE: We do NOT apply robot transform here
    # The grasp pose should be in world/camera coordinates
    # The robot transform is only for visualizing where the robot is relative to the camera
    
    return np.array([x_world, y_world, z_world])


def calculate_grasp_joint_states(xyz, rot_matrix, robot_transform=None):
    """Calculate robot joint states for a given grasp pose using inverse kinematics
    
    Args:
        xyz: Grasp position in world/camera coordinates (mm)
        rot_matrix: 3x3 rotation matrix for grasp orientation
        robot_transform: Robot base transform relative to camera
    """
    global robot
    
    try:
        # Initialize kinematics if available
        kinematics = SO101Kinematics()
        
        # Convert position from mm to meters
        grasp_position_world = np.array(xyz) / 1000.0
        
        # Apply robot transform to get grasp position relative to robot base
        if robot_transform:
            # Robot transform contains position offset of robot base from camera origin
            robot_base_offset = np.array([
                robot_transform.get('position', {}).get('x', 0) / 1000.0,  # mm to m
                robot_transform.get('position', {}).get('y', 0) / 1000.0,
                robot_transform.get('position', {}).get('z', 0) / 1000.0
            ])
            
            # TODO: Also apply rotation transform if robot base is rotated
            # For now, just subtract the robot base position to get grasp relative to robot
            grasp_position_robot = grasp_position_world - robot_base_offset
        else:
            # No transform provided, assume robot at origin
            grasp_position_robot = grasp_position_world
        
        # Use the robot-relative position for IK
        robot_position = grasp_position_robot
        
        # Convert rotation matrix to quaternion or euler angles
        from scipy.spatial.transform import Rotation
        r = Rotation.from_matrix(np.array(rot_matrix))
        euler = r.as_euler('xyz', degrees=False)
        
        # Calculate IK
        try:
            joint_angles = kinematics.inverse_kinematics(
                robot_position[0], robot_position[1], robot_position[2],
                euler[0], euler[1], euler[2]
            )
            
            # Convert to degrees for display
            joint_angles_deg = [np.rad2deg(angle) for angle in joint_angles]
            
            return {
                'joint_1': joint_angles[0] if len(joint_angles) > 0 else 0,
                'joint_2': joint_angles[1] if len(joint_angles) > 1 else 0,
                'joint_3': joint_angles[2] if len(joint_angles) > 2 else 0,
                'joint_4': joint_angles[3] if len(joint_angles) > 3 else 0,
                'joint_5': joint_angles[4] if len(joint_angles) > 4 else 0,
                'joint_6': joint_angles[5] if len(joint_angles) > 5 else 0,
                'joint_angles_deg': joint_angles_deg,
                'success': True
            }
        except Exception as e:
            print(f"IK calculation failed: {e}")
            # Return approximate joint states
            return {
                'joint_1': 0,
                'joint_2': -0.5,
                'joint_3': 0.8,
                'joint_4': -0.3,
                'joint_5': 0,
                'joint_6': 0,
                'joint_angles_deg': [0, -28.6, 45.8, -17.2, 0, 0],
                'success': False,
                'message': 'Using approximate joint states'
            }
            
    except Exception as e:
        print(f"Error calculating joint states: {e}")
        return {
            'joint_1': 0,
            'joint_2': 0,
            'joint_3': 0,
            'joint_4': 0,
            'joint_5': 0,
            'joint_6': 0,
            'joint_angles_deg': [0, 0, 0, 0, 0, 0],
            'success': False,
            'message': str(e)
        }


@app.route('/api/grasp/detect', methods=['POST'])
def detect_grasp_pose():
    """Detect grasp pose using ThinkGrasp API"""
    try:
        data = request.get_json() or {}
        capture_name = data.get('capture_name')
        grasp_text = data.get('grasp_text', 'pick up the object')
        # Get robot transform for IK calculation (not for grasp detection)
        robot_transform = data.get('robot_transform', None)
        
        if not capture_name:
            return jsonify({'success': False, 'message': 'No capture name provided'})
        
        # Get capture paths
        capture_dir = Path.home() / "Documents/Github/opensource_dev/GraspingDemo/captures" / capture_name
        if not capture_dir.exists():
            # Try relative path as fallback
            capture_dir = Path('captures') / capture_name
            if not capture_dir.exists():
                return jsonify({'success': False, 'message': f'Capture {capture_name} not found'})
        
        rgb_path = capture_dir / 'rgb.png'
        depth_path = capture_dir / 'depth.png'
        text_path = capture_dir / 'grasp_text.txt'
        
        # Save grasp text
        with open(text_path, 'w') as f:
            f.write(grasp_text)
        
        # Call ThinkGrasp API
        import requests
        thinkgrasp_url = 'http://localhost:5010/grasp_pose'
        
        # Log the request for debugging
        print(f"Calling ThinkGrasp API with:")
        print(f"  RGB: {rgb_path.absolute()}")
        print(f"  Depth: {depth_path.absolute()}")
        print(f"  Text: {grasp_text}")
        
        try:
            response = requests.post(thinkgrasp_url, json={
                'image_path': str(rgb_path.absolute()),
                'depth_path': str(depth_path.absolute()),
                'text_path': str(text_path.absolute())
            }, timeout=30)
            
            # Log response for debugging
            print(f"ThinkGrasp response status: {response.status_code}")
            print(f"Response headers: {response.headers.get('content-type', 'unknown')}")
            
            if response.status_code == 200:
                # Check if response is JSON
                content_type = response.headers.get('content-type', '')
                if 'application/json' in content_type:
                    grasp_data = response.json()
                    
                    # Extract grasp pose (in camera coordinates)
                    xyz_camera = grasp_data.get('xyz', [0, 0, 0])
                    rot = grasp_data.get('rot', [[1,0,0],[0,1,0],[0,0,1]])
                    dep = grasp_data.get('dep', 0)
                    
                    # Transform grasp pose from camera to world coordinates
                    # The grasp pose from ThinkGrasp is in camera frame
                    # Need to transform to world/robot frame for visualization
                    
                    # Camera to world transformation
                    # Camera frame: X-right, Y-down, Z-forward (RealSense convention)
                    # World frame: X-forward, Y-left, Z-up (robot/point cloud convention)
                    
                    # Convert mm to meters for consistency
                    x_cam = xyz_camera[0] / 1000.0  # Convert mm to m
                    y_cam = xyz_camera[1] / 1000.0
                    z_cam = xyz_camera[2] / 1000.0
                    
                    # Apply coordinate frame transformation
                    # This transformation aligns camera frame with world frame
                    x_world = z_cam   # Forward in world is depth in camera
                    y_world = -x_cam  # Left in world is negative right in camera
                    z_world = -y_cam  # Up in world is negative down in camera
                    
                    # Grasp pose is in camera/world coordinates
                    # No transform needed - grasp shows where object is in real world
                    camera_offset_x = 0.0
                    camera_offset_y = 0.0
                    camera_offset_z = 0.0
                    
                    x_world += camera_offset_x
                    y_world += camera_offset_y
                    z_world += camera_offset_z
                    
                    # Convert back to mm for consistency with rest of system
                    xyz_world = [x_world * 1000, y_world * 1000, z_world * 1000]
                    
                    # Calculate joint states using inverse kinematics
                    # Pass robot transform so IK knows where robot base is
                    joint_states = calculate_grasp_joint_states(xyz_world, rot, robot_transform)
                    
                    return jsonify({
                        'success': True,
                        'grasp_pose': {
                            'xyz': xyz_world,  # World coordinates for visualization
                            'xyz_camera': xyz_camera,  # Original camera coordinates
                            'rot': rot,
                            'dep': dep,
                            'coordinate_frame': 'world'  # Indicate we're using world frame
                        },
                        'joint_states': joint_states,
                        'message': 'Grasp pose detected successfully'
                    })
                else:
                    # Response is not JSON (probably HTML error page)
                    error_text = response.text[:500]  # First 500 chars
                    print(f"Non-JSON response from ThinkGrasp: {error_text}")
                    return jsonify({
                        'success': False,
                        'message': f'ThinkGrasp API returned non-JSON response. Make sure the API is running correctly.'
                    })
            else:
                # Try to parse error message
                try:
                    error_data = response.json()
                    error_msg = error_data.get('error', 'Unknown error')
                except:
                    error_msg = f'HTTP {response.status_code}: {response.text[:200]}'
                
                return jsonify({
                    'success': False,
                    'message': f'ThinkGrasp API error: {error_msg}'
                })
                
        except requests.exceptions.RequestException as e:
            return jsonify({
                'success': False,
                'message': f'Failed to connect to ThinkGrasp API: {str(e)}. Make sure ThinkGrasp is running on port 5010.'
            })
            
    except Exception as e:
        return jsonify({
            'success': False,
            'message': f'Error detecting grasp pose: {str(e)}'
        })


@app.route('/api/grasp/visualize', methods=['POST'])
def visualize_grasp():
    """Generate Plotly visualization for grasp pose and trajectory"""
    global realsense_camera, active_camera
    try:
        import plotly.graph_objects as go
        from pointcloud_viewer import vis_pcd_plotly, generate_plotly_html, load_point_cloud_from_capture
        
        data = request.get_json() or {}
        grasp_pose = data.get('grasp_pose')
        show_trajectory = data.get('show_trajectory', False)
        capture_name = data.get('capture_name', 'latest')
        
        # Additional visualization options
        show_camera = data.get('show_camera', True)
        show_robot = data.get('show_robot', True)
        robot_joint_angles = data.get('robot_joint_angles')
        robot_position = data.get('robot_position', [0, 0, 0])
        robot_rotation = data.get('robot_rotation', [0, 0, 0])
        camera_on_hand = data.get('camera_on_hand', False)  # Eye-in-hand vs Eye-to-hand
        
        if not grasp_pose:
            return jsonify({'success': False, 'message': 'No grasp pose provided'})
        
        # Load point cloud if capture specified
        pointclouds = []
        
        # Option to use test/demo point cloud if needed
        use_demo_pointcloud = data.get('use_demo_pointcloud', False)
        
        if use_demo_pointcloud:
            # Generate a demo point cloud for testing
            print("Using demo point cloud for testing...")
            demo_points = []
            demo_colors = []
            # Create a simple cube point cloud
            for x in np.linspace(-50, 50, 20):
                for y in np.linspace(-50, 50, 20):
                    for z in np.linspace(10, 60, 10):
                        demo_points.append([x, y, z])
                        # Color based on height
                        demo_colors.append([z/60, 0.5, 1-z/60])
            
            pointclouds.append({
                'vertices': np.array(demo_points),
                'colors': np.array(demo_colors),
                'name': 'Demo Point Cloud'
            })
            print(f"Created demo point cloud with {len(demo_points)} points")
        elif capture_name and capture_name != 'none':
            try:
                # Handle "latest" capture name by finding the most recent capture
                if capture_name == 'latest':
                    captures_base = Path.home() / "Documents/Github/opensource_dev/GraspingDemo/captures"
                    if not captures_base.exists():
                        captures_base = Path('captures')
                    
                    if captures_base.exists():
                        # Find the most recent capture directory
                        capture_dirs = [d for d in captures_base.iterdir() if d.is_dir()]
                        if capture_dirs:
                            # Sort by modification time and get the most recent
                            capture_dir = max(capture_dirs, key=lambda d: d.stat().st_mtime)
                            print(f"Using latest capture: {capture_dir.name}")
                        else:
                            print("No capture directories found")
                            capture_dir = captures_base / 'latest'  # Fallback
                    else:
                        print("Captures directory does not exist")
                        capture_dir = Path('captures') / 'latest'
                else:
                    capture_dir = Path.home() / "Documents/Github/opensource_dev/GraspingDemo/captures" / capture_name
                    if not capture_dir.exists():
                        capture_dir = Path('captures') / capture_name
                
                if capture_dir.exists():
                    from pointcloud_viewer import load_point_cloud_from_capture
                    print(f"Loading point cloud from: {capture_dir}")
                    pcd_data, metadata = load_point_cloud_from_capture(str(capture_dir))
                    if pcd_data:
                        vertices = pcd_data['vertices'] * 100  # Convert to cm
                        colors = pcd_data['colors']
                        print(f"Loaded point cloud with {len(vertices)} points")
                        
                        # Use same filtering method as live stream
                        # Only filter out extremely far points (noise)
                        distances = np.linalg.norm(vertices, axis=1)
                        valid_mask = distances < 500  # Keep points within 5 meters
                        vertices = vertices[valid_mask]
                        colors = colors[valid_mask]
                        
                        # Remove statistical outliers if there are enough points
                        if len(vertices) > 100:
                            from pointcloud_viewer import remove_statistical_outliers
                            vertices, colors = remove_statistical_outliers(vertices, colors, n_neighbors=10, std_ratio=1.5)
                        
                        # Further downsample if needed (voxel size in cm)
                        if len(vertices) > 20000:
                            from pointcloud_viewer import downsample_pointcloud
                            vertices, colors = downsample_pointcloud(vertices, colors, voxel_size=1.0)  # 1cm voxel size
                        
                        print(f"After processing: {len(vertices)} points")
                        print(f"Point cloud range - X: [{vertices[:, 0].min():.2f}, {vertices[:, 0].max():.2f}] cm")
                        print(f"Point cloud range - Y: [{vertices[:, 1].min():.2f}, {vertices[:, 1].max():.2f}] cm")
                        print(f"Point cloud range - Z: [{vertices[:, 2].min():.2f}, {vertices[:, 2].max():.2f}] cm")
                        
                        pointclouds.append({
                            'vertices': vertices,
                            'colors': colors,
                            'name': f'Capture: {capture_name}'
                        })
                    else:
                        print(f"No point cloud data returned for {capture_name}")
                else:
                    print(f"Capture directory does not exist: {capture_dir}")
                    print("Attempting to generate point cloud from current camera data...")
                    
                    # Try to get current camera data to generate point cloud
                    if active_camera == 'realsense' and realsense_camera:
                        try:
                            # Get current depth and RGB frames
                            depth_frame = realsense_camera.get_depth_data()
                            rgb_frame = realsense_camera.get_rgb_data()
                            
                            if depth_frame is not None:
                                from pointcloud_viewer import generate_pointcloud_from_depth
                                
                                # Get camera intrinsics
                                camera_intrinsics = {
                                    'fx': 615.0,
                                    'fy': 615.0, 
                                    'ppx': 320.0,
                                    'ppy': 240.0,
                                    'depth_scale': 0.001
                                }
                                
                                if hasattr(realsense_camera, 'intrinsics'):
                                    intr = realsense_camera.intrinsics
                                    camera_intrinsics.update({
                                        'fx': intr.fx,
                                        'fy': intr.fy,
                                        'ppx': intr.ppx,
                                        'ppy': intr.ppy
                                    })
                                
                                # Generate point cloud from current camera data
                                pcd_data = generate_pointcloud_from_depth(
                                    depth_frame, 
                                    {'camera_intrinsics': camera_intrinsics},
                                    rgb_frame
                                )
                                
                                if pcd_data:
                                    vertices = pcd_data['vertices'] * 100  # Convert to cm
                                    colors = pcd_data['colors']
                                    print(f"Generated {len(vertices)} points from live camera")
                                    
                                    # Apply same filtering as live stream
                                    distances = np.linalg.norm(vertices, axis=1)
                                    valid_mask = distances < 500  # Keep points within 5 meters
                                    vertices = vertices[valid_mask]
                                    colors = colors[valid_mask]
                                    
                                    # Remove statistical outliers
                                    if len(vertices) > 100:
                                        from pointcloud_viewer import remove_statistical_outliers
                                        vertices, colors = remove_statistical_outliers(vertices, colors, n_neighbors=10, std_ratio=1.5)
                                    
                                    # Downsample if needed
                                    if len(vertices) > 20000:
                                        from pointcloud_viewer import downsample_pointcloud
                                        vertices, colors = downsample_pointcloud(vertices, colors, voxel_size=1.0)
                                    
                                    print(f"After processing: {len(vertices)} points from live camera")
                                    
                                    pointclouds.append({
                                        'vertices': vertices,
                                        'colors': colors,
                                        'name': 'Live Camera Data'
                                    })
                            else:
                                print("No depth data available from camera")
                        except Exception as e:
                            print(f"Failed to generate point cloud from camera: {e}")
                            import traceback
                            traceback.print_exc()
                    else:
                        print("RealSense camera not available for live point cloud generation")
            except Exception as e:
                print(f"Could not load point cloud: {e}")
                import traceback
                traceback.print_exc()
        
        # Extract grasp pose components
        xyz = grasp_pose.get('xyz', [0, 0, 0])
        rot = np.array(grasp_pose.get('rot', [[1,0,0],[0,1,0],[0,0,1]]))
        dep = grasp_pose.get('dep', 0.05)
        
        # Convert position from mm to cm for visualization
        pos_cm = [x/10 for x in xyz]
        
        # Transform grasp pose based on camera mounting configuration
        if camera_on_hand and robot_position:
            # Eye-in-hand: Camera moves with robot
            # Grasp pose is relative to camera, which is on the robot
            # Need to transform grasp pose to world frame using robot pose
            
            robot_pos_cm = [robot_position[0] * 100, robot_position[1] * 100, robot_position[2] * 100]
            
            # If robot rotation is provided, apply rotation transformation
            if robot_rotation:
                # Convert rotation from degrees to radians
                roll, pitch, yaw = [np.radians(angle) for angle in robot_rotation]
                
                # Create rotation matrix from Euler angles
                from scipy.spatial.transform import Rotation as R
                robot_rot = R.from_euler('xyz', [roll, pitch, yaw])
                
                # Transform grasp position relative to robot
                grasp_pos_relative = np.array(pos_cm)
                grasp_pos_rotated = robot_rot.apply(grasp_pos_relative)
                pos_cm = grasp_pos_rotated.tolist()
                
                # Also rotate the grasp orientation
                rot = robot_rot.as_matrix() @ rot
            
            # Add robot position offset
            pos_cm[0] += robot_pos_cm[0]
            pos_cm[1] += robot_pos_cm[1]
            pos_cm[2] += robot_pos_cm[2]
            
            print(f"Eye-in-hand mode: Grasp transformed relative to robot")
        else:
            # Eye-to-hand: Camera is stationary
            # Grasp pose is already in world coordinates
            # No transformation needed
            print(f"Eye-to-hand mode: Grasp in world coordinates")
        
        print(f"Grasp position (after transform): {pos_cm} cm")
        
        # Decide whether to use vis_pcd_plotly for complete visualization
        if show_camera or show_robot:
            # Use vis_pcd_plotly for complete scene visualization
            from pointcloud_viewer import vis_pcd_plotly
            
            # Get camera info if available
            camera_info = None
            if show_camera:
                # Use default camera info for visualization
                camera_info = {
                    'fx': 615.0,
                    'fy': 615.0,
                    'cx': 320.0,
                    'cy': 240.0,
                    'width': 640,
                    'height': 480
                }
                try:
                    # Try to get actual intrinsics if RealSense is connected
                    if active_camera == 'realsense' and realsense_camera:
                        if hasattr(realsense_camera, 'intrinsics'):
                            intr = realsense_camera.intrinsics
                            camera_info.update({
                                'fx': intr.fx,
                                'fy': intr.fy,
                                'cx': intr.ppx,
                                'cy': intr.ppy,
                                'width': intr.width,
                                'height': intr.height
                            })
                except Exception as e:
                    print(f"Could not get camera intrinsics: {e}")
            
            # Convert robot joint angles to dict format if needed
            if robot_joint_angles and isinstance(robot_joint_angles, list):
                joint_dict = {}
                for i, angle in enumerate(robot_joint_angles[:6]):
                    joint_dict[str(i+1)] = angle
                robot_joint_angles = joint_dict
            
            # Create figure with full visualization including point cloud
            print(f"Creating vis_pcd_plotly with {len(pointclouds)} point clouds")
            print(f"show_camera={show_camera}, show_robot={show_robot}")
            if pointclouds:
                print(f"First pointcloud has {len(pointclouds[0]['vertices'])} vertices")
            else:
                print("WARNING: No pointclouds to visualize!")
            
            fig = vis_pcd_plotly(
                pointclouds=pointclouds if pointclouds else [],
                size_ls=[2] if pointclouds else [],  # Increased marker size from 1 to 2
                title=f"Grasp Visualization - {capture_name}",
                show_camera=show_camera,
                camera_info=camera_info,
                show_robot=show_robot,
                robot_joint_angles=robot_joint_angles,
                robot_position=robot_position if robot_position else None,
                robot_rotation=robot_rotation if robot_rotation else None,
                show_grid=True  # Show ground grid
            )
        else:
            # Simple visualization without camera/robot
            print(f"Using simple visualization, {len(pointclouds)} point clouds available")
            fig = go.Figure()
            
            # Add ground grid
            from pointcloud_viewer import create_ground_grid_traces
            grid_traces = create_ground_grid_traces(grid_size=20, grid_spacing=10, z_level=0)
            for trace in grid_traces:
                fig.add_trace(trace)
            
            # Add point cloud if available
            if pointclouds:
                pcd = pointclouds[0]
                vertices = pcd['vertices']
                colors = pcd['colors']
                
                # Convert colors to RGB strings
                if colors.max() <= 1.0:
                    colors = (colors * 255).astype(int)
                color_strings = ['rgb({},{},{})'.format(r, g, b) for r, g, b in colors]
                
                fig.add_trace(go.Scatter3d(
                    x=vertices[:, 0],
                    y=vertices[:, 1],
                    z=vertices[:, 2],
                    mode='markers',
                    marker=dict(size=2, color=color_strings),  # Increased marker size from 1 to 2
                    name='Point Cloud'
                ))
            
            # Set proper scene parameters for simple visualization
            fig.update_layout(
                scene=dict(
                    xaxis=dict(title='X (cm)', gridcolor='lightgray', showbackground=True),
                    yaxis=dict(title='Y (cm)', gridcolor='lightgray', showbackground=True),
                    zaxis=dict(title='Z (cm)', gridcolor='lightgray', showbackground=True),
                    aspectmode='cube',  # Ensure uniform scaling
                    camera=dict(
                        eye=dict(x=1.5, y=1.5, z=1.0),
                        center=dict(x=0, y=0, z=0)
                    )
                )
            )
        
        # Add grasp coordinate frame
        axis_length = dep * 100  # Convert to cm
        axis_colors = ['red', 'green', 'blue']
        axis_names = ['X', 'Y', 'Z']
        
        for i, (color, name) in enumerate(zip(axis_colors, axis_names)):
            axis_dir = rot[:, i] * axis_length
            fig.add_trace(go.Scatter3d(
                x=[pos_cm[0], pos_cm[0] + axis_dir[0]],
                y=[pos_cm[1], pos_cm[1] + axis_dir[1]],
                z=[pos_cm[2], pos_cm[2] + axis_dir[2]],
                mode='lines+markers',
                line=dict(color=color, width=5),
                marker=dict(size=[3, 6], color=color),
                name=f'Grasp {name}-axis',
                hovertemplate=f'Grasp {name}-axis<br>Direction: {axis_dir}'
            ))
        
        # Add gripper visualization
        gripper_width = dep * 100  # Convert to cm
        finger_length = gripper_width * 0.8
        finger_thickness = 1  # 1cm thickness
        
        # Create gripper fingers in local frame then transform
        for side in [-1, 1]:  # Left and right fingers
            # Finger base and tip in local frame
            finger_base = np.array([side * gripper_width/2, 0, 0])
            finger_tip = np.array([side * gripper_width/2, finger_length, 0])
            
            # Transform to world frame
            finger_base_world = rot @ finger_base + pos_cm
            finger_tip_world = rot @ finger_tip + pos_cm
            
            fig.add_trace(go.Scatter3d(
                x=[finger_base_world[0], finger_tip_world[0]],
                y=[finger_base_world[1], finger_tip_world[1]],
                z=[finger_base_world[2], finger_tip_world[2]],
                mode='lines',
                line=dict(color='yellow', width=8),
                name=f'Gripper {"left" if side < 0 else "right"} finger',
                showlegend=(side > 0)  # Only show one in legend
            ))
        
        # Add gripper palm connection
        left_base = rot @ np.array([-gripper_width/2, 0, 0]) + pos_cm
        right_base = rot @ np.array([gripper_width/2, 0, 0]) + pos_cm
        
        fig.add_trace(go.Scatter3d(
            x=[left_base[0], right_base[0]],
            y=[left_base[1], right_base[1]],
            z=[left_base[2], right_base[2]],
            mode='lines',
            line=dict(color='orange', width=6),
            name='Gripper palm',
            showlegend=False
        ))
        
        # Add robot end-effector current position marker
        if robot_position:
            robot_pos_cm = [robot_position[0] * 100, robot_position[1] * 100, robot_position[2] * 100]
            tcp_offset = 20  # 20cm from robot base to gripper
            tcp_pos = [robot_pos_cm[0], robot_pos_cm[1], robot_pos_cm[2] + tcp_offset]
            
            # Add marker for current robot position
            fig.add_trace(go.Scatter3d(
                x=[tcp_pos[0]],
                y=[tcp_pos[1]],
                z=[tcp_pos[2]],
                mode='markers+text',
                marker=dict(size=10, color='purple', symbol='square'),
                text=['Robot TCP'],
                textposition='top center',
                name='Robot Current Position',
                hovertemplate='Robot TCP<br>X: %{x:.1f} cm<br>Y: %{y:.1f} cm<br>Z: %{z:.1f} cm'
            ))
        
        # Add trajectory if requested
        if show_trajectory:
            # Current robot position - use actual robot position if provided
            if robot_position:
                # robot_position is in meters, convert to cm
                start_pos = [robot_position[0] * 100, robot_position[1] * 100, robot_position[2] * 100]
                # Add a reasonable TCP offset (tool center point)
                tcp_offset = 20  # 20cm from robot base to gripper
                start_pos[2] += tcp_offset
            else:
                start_pos = [30, 0, 20]  # Default start position in cm
            
            # Pre-grasp position (approach from above)
            pre_grasp_offset = 10  # 10cm above grasp point
            pre_grasp_pos = [pos_cm[0], pos_cm[1], pos_cm[2] + pre_grasp_offset]
            
            # Generate trajectory waypoints
            num_points = 20
            trajectory_points = []
            
            # Approach trajectory (from start to pre-grasp)
            for i in range(num_points//2):
                t = i / (num_points//2 - 1) if num_points > 2 else 0
                point = [
                    start_pos[0] + t * (pre_grasp_pos[0] - start_pos[0]),
                    start_pos[1] + t * (pre_grasp_pos[1] - start_pos[1]),
                    start_pos[2] + t * (pre_grasp_pos[2] - start_pos[2])
                ]
                trajectory_points.append(point)
            
            # Grasp trajectory (from pre-grasp to grasp)
            for i in range(num_points//2):
                t = i / (num_points//2 - 1) if num_points > 2 else 0
                point = [
                    pre_grasp_pos[0] + t * (pos_cm[0] - pre_grasp_pos[0]),
                    pre_grasp_pos[1] + t * (pos_cm[1] - pre_grasp_pos[1]),
                    pre_grasp_pos[2] + t * (pos_cm[2] - pre_grasp_pos[2])
                ]
                trajectory_points.append(point)
            
            # Add trajectory line
            traj_array = np.array(trajectory_points)
            fig.add_trace(go.Scatter3d(
                x=traj_array[:, 0],
                y=traj_array[:, 1],
                z=traj_array[:, 2],
                mode='lines+markers',
                line=dict(color='cyan', width=4),
                marker=dict(
                    size=4,
                    color=np.linspace(0, 1, len(trajectory_points)),
                    colorscale='Viridis',
                    showscale=False
                ),
                name='Grasp Trajectory',
                hovertemplate='Waypoint %{pointNumber}<br>Position: (%{x:.1f}, %{y:.1f}, %{z:.1f}) cm'
            ))
            
            # Add arrow at the end
            if len(trajectory_points) > 1:
                last_point = trajectory_points[-1]
                second_last = trajectory_points[-2]
                direction = np.array(last_point) - np.array(second_last)
                direction = direction / np.linalg.norm(direction) * 3  # 3cm arrow
                
                fig.add_trace(go.Cone(
                    x=[last_point[0]],
                    y=[last_point[1]],
                    z=[last_point[2]],
                    u=[direction[0]],
                    v=[direction[1]],
                    w=[direction[2]],
                    sizemode='absolute',
                    sizeref=2,
                    colorscale=[[0, 'cyan'], [1, 'cyan']],
                    showscale=False,
                    name='Approach Direction'
                ))
        
        # Update layout - only update title to preserve existing scene settings from vis_pcd_plotly
        # The vis_pcd_plotly function already sets proper scene parameters including aspectmode='cube'
        # which ensures uniform scaling for all axes
        fig.update_layout(
            title=f"Grasp Pose Visualization - {capture_name}",
            showlegend=True,
            width=900,
            height=700
        )
        
        
        # Generate HTML
        html_content = generate_plotly_html(fig)
        
        return html_content, 200, {'Content-Type': 'text/html'}
        
    except Exception as e:
        return jsonify({
            'success': False,
            'message': f'Error generating visualization: {str(e)}'
        })


@app.route('/thinkgrasp/results')
def thinkgrasp_results_viewer():
    """Serve the ThinkGrasp results viewer HTML"""
    import os
    thinkgrasp_viewer_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(__file__))),
        'ThinkGrasp', 'results_viewer.html'
    )
    
    if os.path.exists(thinkgrasp_viewer_path):
        with open(thinkgrasp_viewer_path, 'r') as f:
            return f.read()
    else:
        return "ThinkGrasp results viewer not found", 404


@app.route('/list_results')
def list_thinkgrasp_results():
    """List available ThinkGrasp results"""
    import os
    import glob
    
    thinkgrasp_outputs_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(__file__))),
        'ThinkGrasp', 'outputs'
    )
    
    # Find all result directories (format: YYYYMMDD_HHMMSS_NNN)
    result_pattern = os.path.join(thinkgrasp_outputs_dir, '[0-9]*_[0-9]*_[0-9]*')
    result_dirs = glob.glob(result_pattern)
    
    # Extract timestamps from directory names
    timestamps = []
    for dir_path in result_dirs:
        dirname = os.path.basename(dir_path)
        # Validate it's a proper timestamp format (YYYYMMDD_HHMMSS_NNN)
        parts = dirname.split('_')
        if len(parts) == 3 and len(parts[0]) == 8 and len(parts[1]) == 6 and len(parts[2]) == 3:
            timestamps.append(dirname)
    
    # Sort by timestamp (newest first)
    timestamps.sort(reverse=True)
    
    return jsonify({'timestamps': timestamps})


@app.route('/results/<timestamp>/<filename>')
def serve_thinkgrasp_result_file(timestamp, filename):
    """Serve a specific file from ThinkGrasp results"""
    import os
    from flask import send_file, abort
    
    thinkgrasp_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(__file__))),
        'ThinkGrasp', 'outputs', timestamp
    )
    
    file_path = os.path.join(thinkgrasp_dir, filename)
    
    if os.path.exists(file_path):
        # Handle JSON files specially to ensure proper content type
        if filename.endswith('.json'):
            return send_file(file_path, mimetype='application/json')
        elif filename.endswith('.txt'):
            return send_file(file_path, mimetype='text/plain')
        else:
            return send_file(file_path)
    else:
        abort(404)


@app.route('/steps/<timestamp>')
def get_thinkgrasp_steps(timestamp):
    """Get available steps for a ThinkGrasp result"""
    import os
    import glob
    
    thinkgrasp_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(__file__))),
        'ThinkGrasp', 'outputs', timestamp
    )
    
    steps = []
    # Look for step folders (format: XX_step_name)
    step_pattern = os.path.join(thinkgrasp_dir, '[0-9][0-9]_*')
    step_dirs = glob.glob(step_pattern)
    
    for step_dir in step_dirs:
        dirname = os.path.basename(step_dir)
        # Try to load metadata for the step
        metadata_path = os.path.join(step_dir, 'metadata.json')
        num_grasps = 0
        if os.path.exists(metadata_path):
            try:
                with open(metadata_path, 'r') as f:
                    metadata = json.load(f)
                    num_grasps = len(metadata.get('grasps', []))
            except:
                pass
        
        steps.append({
            'folder': dirname,
            'step_name': dirname,
            'num_grasps': num_grasps
        })
    
    return jsonify({'steps': sorted(steps, key=lambda x: x['folder'])})


@app.route('/step_files/<timestamp>/<step_folder>/<filename>')
def serve_thinkgrasp_step_file(timestamp, step_folder, filename):
    """Serve a file from a ThinkGrasp step folder"""
    import os
    from flask import send_file, abort
    
    file_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(__file__))),
        'ThinkGrasp', 'outputs', timestamp, step_folder, filename
    )
    
    if os.path.exists(file_path):
        if filename.endswith('.json'):
            return send_file(file_path, mimetype='application/json')
        else:
            return send_file(file_path)
    else:
        abort(404)


@app.route('/convert_ply/<timestamp>/<filename>')
def convert_ply_to_json(timestamp, filename):
    """Convert PLY file to JSON format for Three.js"""
    import os
    
    ply_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(__file__))),
        'ThinkGrasp', 'outputs', timestamp, filename
    )
    
    if not os.path.exists(ply_path):
        return jsonify({'error': 'PLY file not found'}), 404
    
    try:
        import open3d as o3d
        pcd = o3d.io.read_point_cloud(ply_path)
        points = np.asarray(pcd.points).tolist()
        colors = np.asarray(pcd.colors).tolist() if pcd.has_colors() else []
        
        return jsonify({
            'points': points,
            'colors': colors,
            'count': len(points)
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/api/grasp/execute', methods=['POST'])
def execute_grasp_pose():
    """Execute a detected grasp pose on the robot"""
    global robot, is_moving
    
    try:
        data = request.get_json() or {}
        grasp_pose = data.get('grasp_pose')
        
        if not grasp_pose:
            return jsonify({'success': False, 'message': 'No grasp pose provided'})
        
        if not robot:
            return jsonify({'success': False, 'message': 'Robot not connected'})
        
        if is_moving:
            return jsonify({'success': False, 'message': 'Robot is already moving'})
        
        # Extract pose data
        xyz = grasp_pose.get('xyz', [0, 0, 0])
        rot_matrix = grasp_pose.get('rot', [[1,0,0],[0,1,0],[0,0,1]])
        depth = grasp_pose.get('dep', 0)
        
        # Convert grasp pose to robot joint angles
        # This would involve inverse kinematics calculation
        # For now, we'll just move to a predefined grasp position
        
        # Example: Move to grasp position (you would calculate actual IK here)
        is_moving = True
        try:
            # Pre-grasp position
            pre_grasp_joints = [0, -0.5, 0.8, -0.3, 0, 0.5]  # Open gripper
            robot.write_joints(pre_grasp_joints)
            time.sleep(2)
            
            # Grasp position (would be calculated from IK)
            # grasp_joints = calculate_ik(xyz, rot_matrix)
            
            # Close gripper
            robot.write_joints([0, -0.5, 0.8, -0.3, 0, 0.0])  # Close gripper
            time.sleep(1)
            
            # Lift object
            lift_joints = [0, -0.3, 0.6, -0.3, 0, 0.0]
            robot.write_joints(lift_joints)
            
            return jsonify({
                'success': True,
                'message': 'Grasp executed successfully'
            })
            
        finally:
            is_moving = False
            
    except Exception as e:
        return jsonify({
            'success': False,
            'message': f'Error executing grasp: {str(e)}'
        })


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


# ============================================
# Hand-Eye Calibration API Routes
# ============================================

@app.route('/api/calibration/start', methods=['POST'])
def start_calibration():
    """Start hand-eye calibration session using shared RealSense"""
    global current_calibrator, calibration_session
    
    if not calibration_available:
        return jsonify({'success': False, 'error': 'Calibration modules not available'})
    
    data = request.get_json() or {}
    mode = data.get('mode', 'eye-in-hand')
    pattern = data.get('pattern', '7x4')
    square_size = float(data.get('square_size', 25.0))
    
    # Parse pattern
    try:
        cols, rows = map(int, pattern.split('x'))
        checkerboard_size = (cols, rows)
    except:
        return jsonify({'success': False, 'error': 'Invalid pattern format'})
    
    # Create appropriate calibrator
    try:
        if mode == 'eye-in-hand':
            if RealSenseCalibrator is None:
                return jsonify({'success': False, 'error': 'RealSenseCalibrator not available'})
            current_calibrator = RealSenseCalibrator(
                checkerboard_size=checkerboard_size,
                square_size=square_size
            )
        else:
            if StationaryRealSenseCalibrator is None:
                return jsonify({'success': False, 'error': 'StationaryRealSenseCalibrator not available'})
            current_calibrator = StationaryRealSenseCalibrator(
                checkerboard_size=checkerboard_size,
                square_size=square_size
            )
        
        # Use shared RealSense pipeline instead of initializing new one
        pipeline = get_shared_realsense()
        if pipeline is None:
            return jsonify({'success': False, 'error': 'Failed to access camera. Please ensure RealSense is connected.'})
        
        # Override calibrator's pipeline with shared one
        current_calibrator.pipeline = pipeline
        current_calibrator.camera_matrix = None  # Will be set from intrinsics
        current_calibrator.dist_coeffs = np.zeros(5)  # Default distortion
        
        # Get camera intrinsics from shared pipeline
        try:
            import pyrealsense2 as rs
            profile = pipeline.get_active_profile()
            color_stream = profile.get_stream(rs.stream.color)
            intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
            
            # Convert to OpenCV format
            current_calibrator.camera_matrix = np.array([
                [intrinsics.fx, 0, intrinsics.ppx],
                [0, intrinsics.fy, intrinsics.ppy],
                [0, 0, 1]
            ])
            current_calibrator.dist_coeffs = np.array(intrinsics.coeffs)
        except Exception as e:
            print(f"Warning: Could not get camera intrinsics: {e}")
            # Use default values
            current_calibrator.camera_matrix = np.array([
                [600, 0, 320],
                [0, 600, 240],
                [0, 0, 1]
            ])
        
        # Load camera calibration if available (will override intrinsics)
        current_calibrator.load_camera_calibration('calibration_data.npz')
        
        # Initialize session
        calibration_session = {
            'mode': mode,
            'pattern': pattern,
            'captures': 0,
            'active': True
        }
        
        return jsonify({'success': True, 'mode': mode})
        
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})


@app.route('/api/calibration/detect_pattern', methods=['POST'])
def detect_pattern():
    """Auto-detect checkerboard pattern size using shared RealSense"""
    
    # Patterns to test (from test_checkerboard_detection.py)
    patterns_to_try = [
        (7, 4), (4, 7),  # Common pattern
        (9, 6), (6, 9),  # Standard OpenCV calibration pattern
        (8, 6), (6, 8),  # Another common pattern
        (7, 5), (5, 7),  
        (7, 6), (6, 7),  
        (6, 4), (4, 6),  
        (5, 4), (4, 5),  
        (8, 5), (5, 8),  
        (10, 7), (7, 10),  # Larger patterns
        (11, 8), (8, 11),
        (12, 9), (9, 12),
        (13, 9), (9, 13),
    ]
    
    detected_patterns = []
    
    try:
        # Use shared RealSense pipeline
        pipeline = get_shared_realsense()
        if pipeline is None:
            return jsonify({'success': False, 'error': 'Failed to access camera. Please ensure RealSense is connected.'})
        
        # Try multiple frames
        with shared_realsense_lock:
            for _ in range(10):
                try:
                    frames = pipeline.wait_for_frames(timeout_ms=1000)
                    color_frame = frames.get_color_frame()
                    if not color_frame:
                        continue
                    
                    frame = np.asanyarray(color_frame.get_data())
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    
                    # Try each pattern
                    for pattern in patterns_to_try:
                        ret, corners = cv2.findChessboardCorners(gray, pattern, None)
                        if ret and pattern not in detected_patterns:
                            detected_patterns.append(pattern)
                            print(f"Detected pattern: {pattern}")
                except Exception as e:
                    print(f"Frame capture error: {e}")
                    continue
        
        if detected_patterns:
            # Return the first detected pattern (most likely)
            pattern = detected_patterns[0]
            return jsonify({
                'success': True, 
                'pattern': f"{pattern[0]}x{pattern[1]}",
                'all_patterns': [f"{p[0]}x{p[1]}" for p in detected_patterns],
                'message': f'Detected {len(detected_patterns)} pattern(s)'
            })
        else:
            return jsonify({
                'success': False,
                'error': 'No checkerboard pattern detected. Please ensure checkerboard is visible to camera.'
            })
            
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})


@app.route('/api/calibration/detect_frame', methods=['GET'])
def detect_frame():
    """Get camera frame with pattern detection overlay for preview"""
    
    # Get pattern size from query params
    pattern_str = request.args.get('pattern', '9x6')
    try:
        parts = pattern_str.split('x')
        pattern_size = (int(parts[0]), int(parts[1]))
    except:
        pattern_size = (9, 6)  # Default
    
    try:
        # Use shared RealSense pipeline
        pipeline = get_shared_realsense()
        if pipeline is None:
            return jsonify({'success': False, 'error': 'Camera not available'})
        
        with shared_realsense_lock:
            frames = pipeline.wait_for_frames(timeout_ms=1000)
            color_frame = frames.get_color_frame()
            if not color_frame:
                return jsonify({'success': False, 'error': 'Failed to get frame'})
            
            frame = np.asanyarray(color_frame.get_data())
            display_frame = frame.copy()
            
            # Convert to grayscale for detection
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Try to detect checkerboard
            ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
            
            if ret:
                # Draw the detected corners
                cv2.drawChessboardCorners(display_frame, pattern_size, corners, ret)
                
                # Add status text
                text = f"DETECTED: {pattern_size[0]}x{pattern_size[1]} corners"
                cv2.putText(display_frame, text, (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            else:
                # Add status text
                text = f"Searching for {pattern_size[0]}x{pattern_size[1]} pattern..."
                cv2.putText(display_frame, text, (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            
            # Convert to JPEG
            _, buffer = cv2.imencode('.jpg', display_frame)
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')
            
            return jsonify({
                'success': True, 
                'image': jpg_as_text,
                'detected': ret,
                'pattern': pattern_str
            })
            
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})


@app.route('/api/calibration/detect', methods=['GET'])
def detect_checkerboard():
    """Detect checkerboard in current frame using shared pipeline"""
    global current_calibrator
    
    if not current_calibrator:
        return jsonify({'success': False, 'error': 'No calibration session'})
    
    try:
        # Get frame from shared RealSense pipeline
        pipeline = get_shared_realsense()
        if pipeline is None:
            return jsonify({'success': False, 'error': 'Camera not available'})
        
        with shared_realsense_lock:
            frames = pipeline.wait_for_frames(timeout_ms=1000)
            color_frame = frames.get_color_frame()
            if not color_frame:
                return jsonify({'success': False, 'error': 'Failed to get frame'})
            
            frame = np.asanyarray(color_frame.get_data())
        
        # Detect checkerboard
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret_corners, corners = cv2.findChessboardCorners(
            gray, current_calibrator.checkerboard_size, None
        )
        
        detected = False
        distance = None
        
        if ret_corners:
            # Refine corners
            corners = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), current_calibrator.criteria
            )
            
            # Solve PnP to get distance
            ret_pnp, rvec, tvec = cv2.solvePnP(
                current_calibrator.objp, corners, 
                current_calibrator.camera_matrix, 
                current_calibrator.dist_coeffs
            )
            
            if ret_pnp:
                detected = True
                distance = float(tvec[2, 0])  # Distance in mm
                
                # Store temporarily for capture
                current_calibrator.temp_rvec = rvec
                current_calibrator.temp_tvec = tvec
                
                # Draw checkerboard
                cv2.drawChessboardCorners(frame, current_calibrator.checkerboard_size, corners, True)
        
        # Get robot joints
        robot_joints = [0, 0, 0, 0, 0, 0]
        if robot:
            try:
                positions = robot.read_position()
                if positions:
                    robot_joints = positions[:6]
            except:
                pass
        
        # Encode image
        _, buffer = cv2.imencode('.jpg', frame)
        img_base64 = base64.b64encode(buffer).decode('utf-8')
        
        return jsonify({
            'success': True,
            'detected': detected,
            'image': img_base64,
            'distance': distance,
            'robot_joints': robot_joints,
            'pattern': f"{current_calibrator.checkerboard_size[0]}x{current_calibrator.checkerboard_size[1]}"
        })
        
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})


@app.route('/api/calibration/capture', methods=['POST'])
def capture_calibration_point():
    """Capture calibration data point"""
    global current_calibrator, calibration_session
    
    if not current_calibrator:
        return jsonify({'success': False, 'error': 'No calibration session'})
    
    try:
        # Check if we have a valid detection
        if not hasattr(current_calibrator, 'temp_rvec'):
            return jsonify({'success': False, 'error': 'No checkerboard detected'})
        
        # Get robot joints
        robot_joints = [0, 0, 0, 0, 0, 0]
        if robot:
            try:
                positions = robot.read_position()
                if positions:
                    robot_joints = positions[:6]
            except:
                pass
        
        # Store calibration data
        robot_pose = current_calibrator.joints_to_pose_matrix(robot_joints)
        current_calibrator.robot_poses.append(robot_pose)
        current_calibrator.rvecs.append(current_calibrator.temp_rvec)
        current_calibrator.tvecs.append(current_calibrator.temp_tvec)
        current_calibrator.robot_joints_history.append(list(robot_joints))
        
        # Clear temp data
        delattr(current_calibrator, 'temp_rvec')
        delattr(current_calibrator, 'temp_tvec')
        
        calibration_session['captures'] = len(current_calibrator.robot_poses)
        
        return jsonify({
            'success': True,
            'captured_count': len(current_calibrator.robot_poses),
            'joints': robot_joints
        })
        
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})


@app.route('/api/calibration/delete_last', methods=['POST'])
def delete_last_capture():
    """Delete last captured calibration point"""
    global current_calibrator, calibration_session
    
    if not current_calibrator:
        return jsonify({'success': False, 'error': 'No calibration session'})
    
    try:
        if len(current_calibrator.robot_poses) > 0:
            current_calibrator.robot_poses.pop()
            current_calibrator.rvecs.pop()
            current_calibrator.tvecs.pop()
            current_calibrator.robot_joints_history.pop()
            calibration_session['captures'] = len(current_calibrator.robot_poses)
        
        return jsonify({
            'success': True,
            'captured_count': len(current_calibrator.robot_poses)
        })
        
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})


@app.route('/api/calibration/compute', methods=['POST'])
def compute_calibration():
    """Compute hand-eye calibration"""
    global current_calibrator, calibration_session
    
    if not current_calibrator:
        return jsonify({'success': False, 'error': 'No calibration session'})
    
    if len(current_calibrator.robot_poses) < 3:
        return jsonify({'success': False, 'error': 'Need at least 3 positions'})
    
    try:
        # Use the same computation as original
        if isinstance(current_calibrator, StationaryRealSenseCalibrator):
            T_cam2base, T_base2cam = current_calibrator.compute_hand_eye_calibration()
            
            # Save using original method
            output_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'handeye', 'output')
            os.makedirs(output_dir, exist_ok=True)
            output_file = os.path.join(output_dir, 'handeye_realsense_stationary.npz')
            
            current_calibrator.save_calibration(T_cam2base, T_base2cam, output_file)
            
            result = {
                'camera_to_base': T_cam2base.tolist(),
                'base_to_camera': T_base2cam.tolist(),
                'type': 'eye_to_hand'
            }
        else:
            T_cam2gripper = current_calibrator.compute_hand_eye_calibration()
            
            # Save using original method
            output_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'handeye', 'output')
            os.makedirs(output_dir, exist_ok=True)
            output_file = os.path.join(output_dir, 'handeye_realsense.npz')
            
            current_calibrator.save_calibration(T_cam2gripper, output_file)
            
            result = {
                'transformation_matrix': T_cam2gripper.tolist(),
                'type': 'camera_to_gripper'
            }
        
        return jsonify({'success': True, 'result': result})
        
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})


@app.route('/api/calibration/save', methods=['POST'])
def save_calibration():
    """Save calibration results"""
    # Already saved in compute_calibration
    return jsonify({'success': True, 'message': 'Calibration saved'})


@app.route('/api/calibration/list', methods=['GET'])
def list_calibrations():
    """List saved calibrations"""
    try:
        output_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'handeye', 'output')
        calibrations = []
        
        if os.path.exists(output_dir):
            for file in os.listdir(output_dir):
                if file.endswith('.json'):
                    filepath = os.path.join(output_dir, file)
                    with open(filepath, 'r') as f:
                        data = json.load(f)
                        calibrations.append({
                            'id': file.replace('.json', ''),
                            'name': file,
                            'mode': 'eye-to-hand' if 'stationary' in file else 'eye-in-hand',
                            'timestamp': data.get('timestamp', '')
                        })
        
        return jsonify(calibrations)
        
    except Exception as e:
        return jsonify([])


@app.route('/api/calibration/load/<calibration_id>')
def load_calibration(calibration_id):
    """Load a saved calibration"""
    try:
        output_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'handeye', 'output')
        
        # Try to load npz file
        npz_file = os.path.join(output_dir, f"{calibration_id}.npz")
        json_file = os.path.join(output_dir, f"{calibration_id}.json")
        
        if os.path.exists(npz_file):
            data = np.load(npz_file)
            
            # Check which type of calibration it is
            if 'T_cam2gripper' in data and data['T_cam2gripper'] is not None:
                transformation_matrix = data['T_cam2gripper'].tolist()
                calib_type = 'eye-in-hand'
            elif 'T_cam2base' in data:
                transformation_matrix = data['T_cam2base'].tolist()
                calib_type = 'eye-to-hand'
            else:
                return jsonify({'success': False, 'error': 'Invalid calibration file'})
            
            return jsonify({
                'success': True,
                'transformation_matrix': transformation_matrix,
                'type': calib_type
            })
            
        elif os.path.exists(json_file):
            with open(json_file, 'r') as f:
                data = json.load(f)
                return jsonify({
                    'success': True,
                    'transformation_matrix': data.get('transformation_matrix', data.get('camera_to_base_matrix')),
                    'type': 'eye-to-hand' if 'camera_to_base' in data else 'eye-in-hand'
                })
        else:
            return jsonify({'success': False, 'error': 'Calibration file not found'})
            
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})


@app.route('/api/calibration/delete/<calibration_id>', methods=['DELETE'])
def delete_calibration(calibration_id):
    """Delete a saved calibration"""
    try:
        output_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'handeye', 'output')
        
        # Delete both npz and json files
        npz_file = os.path.join(output_dir, f"{calibration_id}.npz")
        json_file = os.path.join(output_dir, f"{calibration_id}.json")
        
        deleted = False
        if os.path.exists(npz_file):
            os.remove(npz_file)
            deleted = True
        
        if os.path.exists(json_file):
            os.remove(json_file)
            deleted = True
        
        if deleted:
            return jsonify({'success': True, 'message': 'Calibration deleted'})
        else:
            return jsonify({'success': False, 'error': 'Calibration file not found'})
            
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})


@app.route('/api/calibration/stop', methods=['POST'])
def stop_calibration():
    """Stop calibration session (keeps shared pipeline running)"""
    global current_calibrator, calibration_session
    
    try:
        # Clear calibrator (but don't stop shared pipeline)
        if current_calibrator:
            current_calibrator.pipeline = None  # Remove reference to shared pipeline
            current_calibrator = None
        
        # Clear session
        if calibration_session:
            calibration_session['active'] = False
        
        return jsonify({'success': True, 'message': 'Calibration stopped'})
        
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})


@app.route('/api/calibration/test_checkerboard', methods=['GET'])
def test_checkerboard_detection():
    """Test checkerboard pattern detection with current camera frame"""
    
    if not cv2_available:
        return jsonify({'success': False, 'error': 'OpenCV not available'})
    
    # Patterns to test (from test_checkerboard_detection.py)
    patterns_to_try = [
        (7, 4), (4, 7),  # Common pattern
        (9, 6), (6, 9),  # Standard OpenCV calibration pattern
        (8, 6), (6, 8),  # Another common pattern
        (7, 5), (5, 7),  
        (7, 6), (6, 7),  
        (6, 4), (4, 6),  
        (5, 4), (4, 5),  
        (8, 5), (5, 8),  
        (10, 7), (7, 10),  # Larger patterns
        (11, 8), (8, 11),
        (12, 9), (9, 12),
        (13, 9), (9, 13),
    ]
    
    try:
        # Get shared RealSense pipeline
        pipeline = get_shared_realsense()
        if pipeline is None:
            return jsonify({'success': False, 'error': 'Camera not available. Please connect RealSense camera.'})
        
        # Get current frame
        import pyrealsense2 as rs
        frames = pipeline.wait_for_frames(5000)  # 5 second timeout
        color_frame = frames.get_color_frame()
        
        if not color_frame:
            return jsonify({'success': False, 'error': 'No frame available from camera'})
        
        # Convert to numpy array
        frame = np.asanyarray(color_frame.get_data())
        if frame is None or frame.size == 0:
            return jsonify({'success': False, 'error': 'Invalid frame data'})
            
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Test each pattern
        detected_patterns = []
        for pattern in patterns_to_try:
            ret, corners = cv2.findChessboardCorners(gray, pattern, None)
            if ret:
                detected_patterns.append({
                    'pattern': f"{pattern[0]}x{pattern[1]}",
                    'corners': pattern[0] * pattern[1],
                    'size': pattern
                })
        
        # Return the frame as base64 for display
        _, buffer = cv2.imencode('.jpg', frame)
        frame_b64 = base64.b64encode(buffer).decode('utf-8')
        
        return jsonify({
            'success': True,
            'detected_patterns': detected_patterns,
            'frame': frame_b64,
            'recommendations': get_pattern_recommendations(detected_patterns)
        })
        
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})


def get_pattern_recommendations(detected_patterns):
    """Get recommendations based on detected patterns"""
    if not detected_patterns:
        return [
            "No checkerboard pattern detected",
            "Ensure checkerboard is fully visible",
            "Check lighting conditions",
            "Try different angles or distances"
        ]
    elif len(detected_patterns) == 1:
        pattern = detected_patterns[0]
        return [
            f"Perfect! Your checkerboard is {pattern['pattern']} internal corners",
            f"Use this in calibration settings: {pattern['pattern']}",
            "Pattern detected successfully"
        ]
    else:
        return [
            f"Multiple patterns detected ({len(detected_patterns)} patterns)",
            "This might indicate detection uncertainty",
            "Try positioning checkerboard more clearly",
            "Most likely patterns: " + ", ".join([p['pattern'] for p in detected_patterns[:3]])
        ]


@app.route('/api/calibration/manual/start', methods=['POST'])
def start_manual_calibration():
    """Start manual hand-eye calibration using standard webcam"""
    global current_calibrator, calibration_session
    
    if not calibration_available or not ManualHandEyeCalibrator:
        return jsonify({'success': False, 'error': 'Manual calibration not available'})
    
    data = request.get_json() or {}
    pattern = data.get('pattern', '7x4')
    square_size = float(data.get('square_size', 25.0))
    camera_id = int(data.get('camera_id', 0))
    
    # Parse pattern
    try:
        cols, rows = map(int, pattern.split('x'))
        checkerboard_size = (cols, rows)
    except:
        return jsonify({'success': False, 'error': 'Invalid pattern format'})
    
    try:
        # Create manual calibrator
        current_calibrator = ManualHandEyeCalibrator(
            checkerboard_size=checkerboard_size,
            square_size=square_size
        )
        
        # Try to load camera calibration (optional)
        try:
            current_calibrator.load_camera_calibration('calibration_data.npz')
        except FileNotFoundError:
            print("Warning: Camera calibration file not found, using default values")
        except Exception as e:
            print(f"Warning: Could not load camera calibration: {e}")
        
        # Initialize session
        calibration_session = {
            'mode': 'manual',
            'pattern': pattern,
            'captures': 0,
            'active': True,
            'camera_id': camera_id
        }
        
        return jsonify({
            'success': True, 
            'mode': 'manual',
            'message': 'Manual calibration initialized. Use external camera window for capture.'
        })
        
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})


@app.route('/api/calibration/manual/capture_session', methods=['POST'])
def manual_capture_session():
    """Run the manual calibration capture session (blocking)"""
    global current_calibrator, robot
    
    if not current_calibrator:
        return jsonify({'success': False, 'error': 'Manual calibration not started. Call /api/calibration/manual/start first.'})
    
    if not calibration_session.get('active'):
        return jsonify({'success': False, 'error': 'Calibration session not active'})
    
    data = request.get_json() or {}
    min_positions = int(data.get('min_positions', 3))
    max_positions = int(data.get('max_positions', 30))
    camera_id = calibration_session.get('camera_id', 0)
    
    try:
        # Check if robot is available
        if robot is None:
            return jsonify({'success': False, 'error': 'Robot not connected. Please connect robot first.'})
        
        print(f"Starting manual capture session with camera {camera_id}")
        
        # Run the manual capture session (this will block until user finishes)
        num_captured = current_calibrator.manual_calibration_capture(
            robot_client=robot,
            camera_id=camera_id,
            min_positions=min_positions,
            max_positions=max_positions
        )
        
        if num_captured >= min_positions:
            # Compute calibration
            T_cam2gripper = current_calibrator.compute_hand_eye_calibration()
            
            # Save results
            current_calibrator.save_calibration(T_cam2gripper)
            
            # Update session
            calibration_session['captures'] = num_captured
            calibration_session['completed'] = True
            
            return jsonify({
                'success': True,
                'captures': num_captured,
                'message': f'Manual calibration completed successfully with {num_captured} poses',
                'transform_matrix': T_cam2gripper.tolist()
            })
        else:
            return jsonify({
                'success': False,
                'captures': num_captured,
                'error': f'Insufficient captures ({num_captured} < {min_positions}). Please capture more poses.'
            })
            
    except KeyboardInterrupt:
        return jsonify({'success': False, 'error': 'Calibration interrupted by user'})
    except Exception as e:
        print(f"Error in manual capture session: {e}")
        import traceback
        traceback.print_exc()
        return jsonify({'success': False, 'error': f'Calibration failed: {str(e)}'})


@app.route('/api/calibration/ros2/connect', methods=['POST'])
def connect_ros2_tf():
    """Start ROS2 TF connection with proper calibration file"""
    data = request.get_json() or {}
    mode = data.get('mode', 'eye-in-hand')
    calibration_file = data.get('calibration_file', None)
    
    try:
        # Build command based on mode
        if mode == 'eye-to-hand' or mode == 'stationary':
            base_cmd = 'ros2 run ros2_digital_twin connect_camera_to_robot_stationary.py'
            default_file = '/home/pathonai/Documents/Github/opensource_dev/handeye/output/handeye_realsense_stationary.npz'
        else:
            base_cmd = 'ros2 run ros2_digital_twin connect_camera_to_robot.py'
            default_file = '/home/pathonai/Documents/Github/opensource_dev/handeye/output/handeye_realsense.npz'
        
        # Add calibration file parameter
        if calibration_file:
            cmd = f"{base_cmd} --ros-args -p calibration_file:={calibration_file}"
        else:
            cmd = f"{base_cmd} --ros-args -p calibration_file:={default_file}"
        
        # Try to start the process
        import subprocess
        process = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        return jsonify({
            'success': True, 
            'command': cmd,
            'mode': mode,
            'pid': process.pid if process else None,
            'verify_commands': [
                'ros2 run tf2_ros tf2_echo base camera_link',
                'ros2 run tf2_tools view_frames',
                'ros2 topic echo /tf_static'
            ]
        })
        
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})


@app.route('/api/calibration/workflow/start', methods=['POST'])
def start_calibration_workflow():
    """Start complete calibration workflow (eye-in-hand or eye-to-hand)"""
    global current_calibrator, calibration_session
    
    data = request.get_json() or {}
    workflow_type = data.get('type', 'eye-in-hand')  # or 'eye-to-hand'
    pattern_size = data.get('pattern_size', None)  # e.g., [7, 4]
    square_size = data.get('square_size', 25.0)
    
    try:
        # Import appropriate calibrator
        if workflow_type == 'eye-to-hand':
            from handeye_manual_realsense_stationary import StationaryRealSenseCalibrator
            current_calibrator = StationaryRealSenseCalibrator(
                checkerboard_size=tuple(pattern_size) if pattern_size else None,
                square_size=square_size
            )
        else:
            from handeye_manual_realsense import RealSenseCalibrator
            current_calibrator = RealSenseCalibrator(
                checkerboard_size=tuple(pattern_size) if pattern_size else None,
                square_size=square_size
            )
        
        # Initialize RealSense
        if hasattr(current_calibrator, 'init_realsense'):
            success = current_calibrator.init_realsense()
            if not success:
                return jsonify({'success': False, 'error': 'Failed to initialize RealSense'})
        
        # Initialize session
        calibration_session = {
            'type': workflow_type,
            'pattern_size': pattern_size,
            'square_size': square_size,
            'captures': 0,
            'active': True,
            'robot_poses': [],
            'camera_poses': []
        }
        
        return jsonify({
            'success': True,
            'type': workflow_type,
            'message': f'Started {workflow_type} calibration workflow'
        })
        
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})


@app.route('/api/calibration/workflow/capture', methods=['POST'])
def capture_workflow_calibration_point():
    """Capture single calibration point in workflow"""
    global current_calibrator, calibration_session, robot
    
    if not current_calibrator or not calibration_session.get('active'):
        return jsonify({'success': False, 'error': 'No active calibration session'})
    
    try:
        # Get current frame
        frame = current_calibrator.get_frame() if hasattr(current_calibrator, 'get_frame') else None
        if frame is None:
            return jsonify({'success': False, 'error': 'Failed to get camera frame'})
        
        # Get robot joints
        robot_joints = robot.read_joints() if robot and robot.connected else [0, 0, 0, 0, 0, 0]
        
        # Detect checkerboard
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        pattern_size = current_calibrator.checkerboard_size
        
        if pattern_size:
            ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
            
            if ret:
                # Refine corners
                corners = cv2.cornerSubPix(
                    gray, corners, (11, 11), (-1, -1), 
                    current_calibrator.criteria
                )
                
                # Solve PnP
                ret_pnp, rvec, tvec = cv2.solvePnP(
                    current_calibrator.objp, corners,
                    current_calibrator.camera_matrix,
                    current_calibrator.dist_coeffs
                )
                
                if ret_pnp:
                    # Store calibration data
                    robot_pose = current_calibrator.joints_to_pose_matrix(robot_joints)
                    current_calibrator.robot_poses.append(robot_pose)
                    current_calibrator.rvecs.append(rvec)
                    current_calibrator.tvecs.append(tvec)
                    current_calibrator.robot_joints_history.append(list(robot_joints))
                    
                    calibration_session['captures'] += 1
                    
                    return jsonify({
                        'success': True,
                        'captures': calibration_session['captures'],
                        'distance': float(tvec[2, 0]),
                        'joints': robot_joints
                    })
        
        return jsonify({'success': False, 'error': 'Checkerboard not detected'})
        
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})


@app.route('/api/calibration/workflow/compute', methods=['POST'])
def compute_calibration_workflow():
    """Compute calibration from captured data"""
    global current_calibrator, calibration_session
    
    if not current_calibrator:
        return jsonify({'success': False, 'error': 'No calibration data'})
    
    try:
        workflow_type = calibration_session.get('type', 'eye-in-hand')
        
        if workflow_type == 'eye-to-hand':
            # Compute eye-to-hand calibration
            T_cam2base, T_base2cam = current_calibrator.compute_hand_eye_calibration()
            
            # Save calibration
            output_file = '../output/handeye_realsense_stationary.npz'
            current_calibrator.save_calibration(T_cam2base, T_base2cam, output_file)
            
            result = {
                'camera_to_base': T_cam2base.tolist(),
                'base_to_camera': T_base2cam.tolist()
            }
        else:
            # Compute eye-in-hand calibration
            T_cam2gripper = current_calibrator.compute_hand_eye_calibration()
            
            # Save calibration
            output_file = 'output/handeye_realsense.npz'
            current_calibrator.save_calibration(T_cam2gripper, output_file)
            
            result = {
                'camera_to_gripper': T_cam2gripper.tolist()
            }
        
        return jsonify({
            'success': True,
            'type': workflow_type,
            'result': result,
            'output_file': output_file,
            'captures': calibration_session.get('captures', 0)
        })
        
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})


# ============================================
# Camera Stream API Routes
# ============================================

@app.route('/api/camera/stream')
def camera_stream():
    """Provide camera stream for external viewing"""
    if not cv2_available:
        return jsonify({'error': 'OpenCV not available'})
        
    try:
        # Get shared RealSense pipeline
        pipeline = get_shared_realsense()
        if pipeline is None:
            # Try webcam as fallback
            return get_webcam_frame()
        
        # Get current frame from RealSense
        import pyrealsense2 as rs
        frames = pipeline.wait_for_frames(3000)  # 3 second timeout
        color_frame = frames.get_color_frame()
        
        if not color_frame:
            return get_webcam_frame()
        
        # Convert to numpy array and encode as JPEG
        frame = np.asanyarray(color_frame.get_data())
        if frame is None or frame.size == 0:
            return get_webcam_frame()
            
        success, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not success:
            return get_webcam_frame()
        
        return send_file(
            io.BytesIO(buffer.tobytes()),
            mimetype='image/jpeg',
            as_attachment=False
        )
        
    except Exception as e:
        print(f"Error in camera stream: {e}")
        return get_webcam_frame()

def get_webcam_frame():
    """Fallback to webcam if RealSense not available"""
    if not cv2_available:
        return jsonify({'error': 'OpenCV not available'})
        
    try:
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            cap.release()
            return jsonify({'error': 'Webcam not accessible'})
            
        ret, frame = cap.read()
        cap.release()
        
        if ret and frame is not None:
            success, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if success:
                return send_file(
                    io.BytesIO(buffer.tobytes()),
                    mimetype='image/jpeg',
                    as_attachment=False
                )
        
        # Return error as JSON if frame capture failed
        return jsonify({'error': 'No camera available'})
            
    except Exception as e:
        print(f"Webcam fallback failed: {e}")
        return jsonify({'error': f'Camera error: {str(e)}'})


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