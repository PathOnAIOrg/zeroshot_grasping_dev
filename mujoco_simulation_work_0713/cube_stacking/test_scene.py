"""
MuJoCo SO-ARM100 Scene Control - Complete Scene with Cubes

Loads the complete scene including the robot arm, environment, and cubes.
Uses threading to handle input without blocking the viewer.

Usage:
    mjpython test_scene.py --control threaded_input
    mjpython test_scene.py --control pause_input
    mjpython test_scene.py --control continuous
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
import argparse
import threading
import queue
import sys

# Load the complete scene XML model (includes arm + environment + cubes)
model = mujoco.MjModel.from_xml_path("trs_so_arm100/scene.xml")
data = mujoco.MjData(model)

def reset_simulation():
    """Reset the simulation to initial state"""
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)

# Global variables for control
current_targets = np.zeros(6)
control_increment = 0.05
simulation_running = True
joint_limits = [
    (-2.2, 2.2),         # Rotation
    (-3.14158, 0.2),     # Pitch
    (0.0, 3.14158),      # Elbow
    (-2.0, 1.8),         # Wrist_Pitch
    (-3.14158, 3.14158), # Wrist_Roll
    (-0.2, 2.0)          # Jaw
]

joint_names = ["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll", "Jaw"]

def print_scene_info():
    """Print information about the loaded scene"""
    print("\n" + "="*60)
    print("🌍 SCENE INFORMATION")
    print("="*60)
    print(f"Model name: {model.names}")
    print(f"Number of bodies: {model.nbody}")
    print(f"Number of joints: {model.njnt}")
    print(f"Number of geoms: {model.ngeom}")
    print(f"Number of actuators: {model.nu}")
    
    print("\n📦 Bodies in scene:")
    for i in range(model.nbody):
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        if body_name:
            print(f"  {i}: {body_name}")
    
    print("\n🔧 Geoms in scene:")
    for i in range(model.ngeom):
        geom_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
        if geom_name:
            geom_type = model.geom_type[i]
            print(f"  {i}: {geom_name} (type: {geom_type})")
    
    print("="*60)

def apply_pd_control():
    """Apply PD control to reach target positions"""
    kp = 50.0  # Proportional gain
    kd = 5.0   # Derivative gain
    
    # Only control the robot joints, not the cubes
    robot_joints = min(6, model.nv)
    current_pos = data.qpos[:robot_joints]
    current_vel = data.qvel[:robot_joints]
    
    position_error = current_targets[:len(current_pos)] - current_pos
    velocity_error = -current_vel
    
    control_torques = kp * position_error + kd * velocity_error
    data.ctrl[:model.nu] = control_torques[:model.nu]

def update_joint_target(joint_idx, direction):
    """Update target position for a specific joint"""
    global current_targets, control_increment
    
    if joint_idx < len(current_targets):
        delta = direction * control_increment
        new_target = current_targets[joint_idx] + delta
        
        # Apply joint limits
        min_limit, max_limit = joint_limits[joint_idx]
        current_targets[joint_idx] = np.clip(new_target, min_limit, max_limit)
        
        print(f"🎮 {joint_names[joint_idx]}: {current_targets[joint_idx]:.3f}")

def get_cube_positions():
    """Get current positions of the cubes"""
    cube_info = {}
    
    # Find cube bodies
    for i in range(model.nbody):
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        if body_name and 'cube' in body_name.lower():
            # Get body position
            body_pos = data.xpos[i]
            cube_info[body_name] = body_pos.copy()
    
    return cube_info

def reset_cubes():
    """Reset cubes to their initial positions"""
    # Find and reset cube positions
    red_cube_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "red_cube")
    blue_cube_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "blue_cube")
    
    if red_cube_id >= 0:
        # Reset red cube position and velocity
        qpos_start = model.body_jntadr[red_cube_id] if model.body_jntnum[red_cube_id] > 0 else -1
        if qpos_start >= 0:
            data.qpos[qpos_start:qpos_start+7] = [0.2, 0.1, 0.1, 1, 0, 0, 0]  # pos + quat
            data.qvel[qpos_start:qpos_start+6] = 0  # linear + angular velocity
    
    if blue_cube_id >= 0:
        # Reset blue cube position and velocity
        qpos_start = model.body_jntadr[blue_cube_id] if model.body_jntnum[blue_cube_id] > 0 else -1
        if qpos_start >= 0:
            data.qpos[qpos_start:qpos_start+7] = [-0.2, 0.1, 0.1, 1, 0, 0, 0]  # pos + quat
            data.qvel[qpos_start:qpos_start+6] = 0  # linear + angular velocity
    
    mujoco.mj_forward(model, data)
    print("🔄 Cubes reset to initial positions")

def input_thread(command_queue):
    """Background thread to handle user input"""
    global simulation_running
    
    print("\n🎮 SCENE CONTROL STARTED!")
    print("Enter commands (type 'help' for options):")
    
    while simulation_running:
        try:
            cmd = input("> ").strip().lower()
            command_queue.put(cmd)
            if cmd in ['quit', 'exit']:
                simulation_running = False
                break
        except EOFError:
            simulation_running = False
            break
        except KeyboardInterrupt:
            simulation_running = False
            break

def process_command(cmd, command_queue):
    """Process a single command"""
    global control_increment, current_targets, simulation_running
    
    if cmd == 'help':
        print("\n" + "="*50)
        print("🎮 SCENE CONTROL COMMANDS")
        print("="*50)
        print("Robot Joint Controls:")
        print("  q+ / q-  : Rotation joint")
        print("  w+ / w-  : Pitch joint")
        print("  e+ / e-  : Elbow joint")
        print("  r+ / r-  : Wrist Pitch joint")
        print("  t+ / t-  : Wrist Roll joint")
        print("  y+ / y-  : Jaw joint")
        print("\nScene Commands:")
        print("  cubes    : Show cube positions")
        print("  reset_cubes : Reset cubes to initial positions")
        print("  scene_info  : Show scene information")
        print("\nUtility Commands:")
        print("  reset    : Reset robot to zero position")
        print("  status   : Show current robot positions")
        print("  speed X  : Set speed (e.g., 'speed 0.1')")
        print("  help     : Show this help")
        print("  quit     : Exit")
        print("="*50)
        
    elif cmd in ['quit', 'exit']:
        print("🛑 Exiting...")
        simulation_running = False
        return False
        
    elif cmd == 'reset':
        current_targets.fill(0)
        print("🔄 Reset robot joints to zero")
        
    elif cmd == 'reset_cubes':
        reset_cubes()
        
    elif cmd == 'cubes':
        cube_positions = get_cube_positions()
        print("📦 Cube positions:")
        for name, pos in cube_positions.items():
            print(f"   {name}: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
        
    elif cmd == 'scene_info':
        print_scene_info()
        
    elif cmd == 'status':
        print(f"📊 Robot target positions:")
        for i, name in enumerate(joint_names):
            if i < len(current_targets):
                print(f"   {name}: {current_targets[i]:.3f}")
        print(f"📊 Robot current positions: {data.qpos[:6]}")
        print(f"📊 Control speed: {control_increment}")
        
        # Also show cube positions
        cube_positions = get_cube_positions()
        if cube_positions:
            print("📦 Cube positions:")
            for name, pos in cube_positions.items():
                print(f"   {name}: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
        
    elif cmd.startswith('speed '):
        try:
            new_speed = float(cmd.split()[1])
            control_increment = max(0.001, min(0.2, new_speed))
            print(f"⚡ Speed set to: {control_increment}")
        except:
            print("❌ Invalid speed. Use: speed 0.05")
            
    elif len(cmd) == 2 and cmd[1] in ['+', '-']:
        # Joint control commands
        joint_char = cmd[0]
        direction = 1 if cmd[1] == '+' else -1
        
        joint_map = {'q': 0, 'w': 1, 'e': 2, 'r': 3, 't': 4, 'y': 5}
        if joint_char in joint_map:
            update_joint_target(joint_map[joint_char], direction)
        else:
            print("❌ Unknown joint. Use: q, w, e, r, t, y")
            
    elif cmd == '':
        pass  # Empty command, do nothing
        
    else:
        print("❌ Unknown command. Type 'help' for available commands.")
    
    return True

def threaded_input_control():
    """Method 1: Threaded input control - Real user input with continuous simulation"""
    print("\n=== Threaded Input Control - Complete Scene ===")
    print("This runs the complete scene simulation with robot arm and cubes.")
    
    reset_simulation()
    print_scene_info()
    
    global current_targets, simulation_running
    current_targets = data.qpos[:6].copy()  # Only robot joints
    simulation_running = True
    
    # Create command queue for thread communication
    command_queue = queue.Queue()
    
    # Start input thread
    input_thread_obj = threading.Thread(target=input_thread, args=(command_queue,), daemon=True)
    input_thread_obj.start()
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        step = 0
        last_status_time = time.time()
        
        while viewer.is_running() and simulation_running:
            # Check for new commands
            try:
                while True:
                    cmd = command_queue.get_nowait()
                    if not process_command(cmd, command_queue):
                        simulation_running = False
                        break
            except queue.Empty:
                pass
            
            # Apply control and step simulation
            apply_pd_control()
            mujoco.mj_step(model, data)
            viewer.sync()
            
            # Occasional status update
            if time.time() - last_status_time > 15.0:
                print(f"📈 Step {step} - Scene simulation running. Type 'status' for info.")
                last_status_time = time.time()
            
            step += 1
            time.sleep(0.01)
    
    simulation_running = False

def pause_input_control():
    """Method 2: Pause-and-input control - Simulation pauses for each command"""
    print("\n=== Pause Input Control - Complete Scene ===")
    print("Scene simulation pauses while you enter commands.")
    
    reset_simulation()
    print_scene_info()
    
    global current_targets
    current_targets = data.qpos[:6].copy()
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("\n🚀 Scene simulation started!")
        print("Type 'help' for commands.")
        
        while viewer.is_running():
            # Get user input
            try:
                cmd = input("\n> ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                break
            
            # Process command
            if not process_command(cmd, None):
                break
            
            # Run simulation for a bit to show the change
            print("⚙️  Applying command...")
            for _ in range(200):  # Run for 2 seconds at 100Hz
                if not viewer.is_running():
                    return
                apply_pd_control()
                mujoco.mj_step(model, data)
                viewer.sync()
                time.sleep(0.01)

def continuous_control():
    """Method 3: Continuous movement control"""
    print("\n=== Continuous Control - Complete Scene ===")
    print("Enter a sequence of commands to execute in the complete scene.")
    
    reset_simulation()
    print_scene_info()
    
    global current_targets
    current_targets = data.qpos[:6].copy()
    
    # Get sequence of commands
    print("\nEnter commands separated by spaces (e.g., 'q+ w+ cubes reset_cubes r+')")
    print("Or type individual commands and press Enter:")
    
    commands = []
    
    try:
        while True:
            cmd_input = input("Command (or 'done' to start): ").strip().lower()
            if cmd_input == 'done':
                break
            elif cmd_input == 'quit':
                return
            elif cmd_input:
                if ' ' in cmd_input:
                    commands.extend(cmd_input.split())
                else:
                    commands.append(cmd_input)
                print(f"Added: {cmd_input}")
    except (EOFError, KeyboardInterrupt):
        return
    
    if not commands:
        print("No commands entered!")
        return
    
    print(f"\n🚀 Executing sequence in scene: {' '.join(commands)}")
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        for cmd in commands:
            if not viewer.is_running():
                break
                
            print(f"\n⚙️  Executing: {cmd}")
            process_command(cmd, None)
            
            # Run simulation for each command
            for _ in range(300):  # 3 seconds per command
                if not viewer.is_running():
                    return
                apply_pd_control()
                mujoco.mj_step(model, data)
                viewer.sync()
                time.sleep(0.01)
        
        print("\n✅ Sequence completed!")
        print("Scene simulation will continue running. Close viewer to exit.")
        
        # Keep running
        while viewer.is_running():
            apply_pd_control()
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(0.01)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='MuJoCo Scene Control - Complete Scene with Cubes')
    parser.add_argument('--control', 
                       choices=['threaded_input', 'pause_input', 'continuous'], 
                       default='threaded_input', 
                       help='Control method to use')
    
    args = parser.parse_args()
    
    print(f"🌍 Starting {args.control} control method for complete scene...")
    
    try:
        if args.control == 'threaded_input':
            threaded_input_control()
        elif args.control == 'pause_input':
            pause_input_control()
        elif args.control == 'continuous':
            continuous_control()
            
    except KeyboardInterrupt:
        print("\n🛑 Control interrupted by user")
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
    
    print("👋 Scene control session ended") 