#!/usr/bin/env python3
"""
Simple SO101 Client with Raw Servo Communication
Matches exactly the coordinate system used by joint_state_reader.py
"""

import serial
import time
import struct
import numpy as np
from typing import List, Tuple


class SO101ClientRawSimple:
    """
    Direct servo control matching joint_state_reader.py exactly.
    
    - Reads raw servo ticks (0-4095)
    - Converts to radians: (ticks - 2048) / 2048 * π
    - No calibration, no offsets, no sign inversions
    """
    
    def __init__(self, port: str = "/dev/ttyACM0", limits: List[Tuple[float, float]] = None):
        """
        Initialize direct servo connection.
        
        Args:
            port: Serial port for the robot
            limits: Optional joint limits in radians
        """
        self.port = port
        self.limits = limits
        self.joint_names = [
            'Rotation',      # Base rotation
            'Pitch',         # Shoulder pitch  
            'Elbow',         # Elbow
            'Wrist_Pitch',   # Wrist pitch
            'Wrist_Roll',    # Wrist roll
            'Jaw'            # Gripper
        ]
        
        # Connect to serial port
        try:
            self.serial_port = serial.Serial(port, 1000000, timeout=0.1)
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            time.sleep(0.1)
            print(f"✅ Connected to SO-101 on {port} (raw mode)")
            
            # Enable all servos on startup
            self.enable_all_servos()
        except Exception as e:
            raise ConnectionError(f"Failed to connect to robot: {e}")
    
    def read_servo_position(self, servo_id):
        """Read position from STS3215 servo (matching joint_state_reader.py)"""
        try:
            # STS3215 position read command
            length = 4
            instruction = 0x02  # Read
            address = 0x38      # PRESENT_POSITION_L
            read_length = 0x02  # 2 bytes
            
            # Calculate checksum
            checksum = (~(servo_id + length + instruction + address + read_length)) & 0xFF
            
            # Build command
            cmd = bytes([0xFF, 0xFF, servo_id, length, instruction, address, read_length, checksum])
            
            # Send and receive
            self.serial_port.reset_input_buffer()
            self.serial_port.write(cmd)
            time.sleep(0.002)
            response = self.serial_port.read(8)
            
            if len(response) >= 7:
                # Validate response
                if response[0] == 0xFF and response[1] == 0xFF and response[2] == servo_id:
                    # Extract position (little endian)
                    pos = struct.unpack('<H', response[5:7])[0]
                    if 0 <= pos <= 4095:
                        return pos
            return None
        except:
            return None
    
    def enable_torque(self, servo_id, enable=True):
        """Enable or disable servo torque"""
        try:
            # STS3215 torque enable command
            length = 4
            instruction = 0x03  # Write
            address = 0x28      # TORQUE_ENABLE register
            value = 0x01 if enable else 0x00
            
            # Calculate checksum
            checksum = (~(servo_id + length + instruction + address + value)) & 0xFF
            
            # Build command
            cmd = bytes([0xFF, 0xFF, servo_id, length, instruction, address, value, checksum])
            
            # Send command
            self.serial_port.write(cmd)
            time.sleep(0.002)
            return True
        except:
            return False
    
    def enable_all_servos(self):
        """Enable torque for all servos"""
        print("Enabling all servos...")
        for i in range(6):
            servo_id = i + 1
            self.enable_torque(servo_id, True)
            time.sleep(0.01)
        print("All servos enabled")
    
    def hold_current_position(self):
        """Command robot to hold its current position"""
        print("Holding current position...")
        current_pos = self.read_joints()
        self.write_joints(current_pos)
        print("Position locked")
    
    def write_servo_position(self, servo_id, ticks, speed=500, enable_torque_first=True):
        """Write position to STS3215 servo in ticks (0-4095)"""
        try:
            # Optionally enable torque first (can be disabled for smoother transitions)
            if enable_torque_first:
                self.enable_torque(servo_id, True)
            
            # Clamp to valid range
            ticks = int(max(0, min(4095, ticks)))
            
            # STS3215 position write command with speed
            length = 7
            instruction = 0x03  # Write
            address = 0x2A      # GOAL_POSITION_L
            
            # Convert position to bytes (little endian)
            pos_l = ticks & 0xFF
            pos_h = (ticks >> 8) & 0xFF
            
            # Speed (0-1023, 0 = max speed)
            # Use moderate speed for safety
            speed = min(1023, max(0, speed))
            speed_l = speed & 0xFF
            speed_h = (speed >> 8) & 0xFF
            
            # Calculate checksum
            checksum = (~(servo_id + length + instruction + address + 
                         pos_l + pos_h + speed_l + speed_h)) & 0xFF
            
            # Build command
            cmd = bytes([0xFF, 0xFF, servo_id, length, instruction, address,
                        pos_l, pos_h, speed_l, speed_h, checksum])
            
            # Send command
            self.serial_port.write(cmd)
            time.sleep(0.002)  # Small delay for command processing
            return True
        except Exception as e:
            print(f"Error writing to servo {servo_id}: {e}")
            return False
    
    def ticks_to_radians(self, ticks):
        """Convert servo ticks to radians (matching joint_state_reader.py exactly)"""
        if ticks is None:
            return 0.0
        # Normalize to -1 to 1, then to -π to π
        normalized = (ticks - 2048) / 2048.0
        return normalized * np.pi
    
    def radians_to_ticks(self, radians):
        """Convert radians to servo ticks"""
        # From -π to π → -1 to 1 → 0 to 4095
        normalized = radians / np.pi
        ticks = normalized * 2048.0 + 2048
        return int(max(0, min(4095, ticks)))
    
    def read_joints(self) -> List[float]:
        """
        Read all joint positions in radians.
        Matches joint_state_reader.py coordinate system exactly.
        """
        positions = []
        for i in range(len(self.joint_names)):
            servo_id = i + 1
            ticks = self.read_servo_position(servo_id)
            radians = self.ticks_to_radians(ticks)
            positions.append(radians)
            time.sleep(0.01)  # Same delay as joint_state_reader
        return positions
    
    def write_joints(self, joints: List[float]):
        """
        Write joint positions in radians.
        Uses same coordinate system as joint_state_reader.py.
        """
        if len(joints) != 6:
            raise ValueError(f"Expected 6 joint values, got {len(joints)}")
        
        # Check limits if provided
        if self.limits:
            for i, (joint, limit) in enumerate(zip(joints, self.limits)):
                if not (limit[0] <= joint <= limit[1]):
                    raise ValueError(f"Joint {i} out of bounds: {joint} not in {limit}")
        
        # Write each servo
        for i, radians in enumerate(joints):
            servo_id = i + 1
            ticks = self.radians_to_ticks(radians)
            self.write_servo_position(servo_id, ticks)
            time.sleep(0.01)  # Small delay between servos
    
    def interpolate_waypoint(self, waypoint1, waypoint2, steps=50, timestep=0.02):
        """Interpolate between two waypoints"""
        for i in range(steps):
            alpha = i / (steps - 1)
            q = [(1 - alpha) * w1 + alpha * w2 for w1, w2 in zip(waypoint1, waypoint2)]
            self.write_joints(q)
            time.sleep(timestep)
    
    def disconnect(self, disable_torque=False, close_port=True):
        """Disconnect from robot"""
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            if disable_torque:
                # Optionally disable all servos
                for i in range(6):
                    self.enable_torque(i + 1, False)
                    time.sleep(0.01)
                print("All servos disabled")
            
            if close_port:
                self.serial_port.close()
                print("👋 Disconnected from robot (servos remain enabled)")
            else:
                print("📌 Keeping serial connection open (servos remain active)")
    
    def __del__(self):
        try:
            # Don't disable torque on cleanup to keep robot active
            self.disconnect(disable_torque=False)
        except:
            pass


if __name__ == "__main__":
    # Test the client
    client = SO101ClientRawSimple()
    
    print("Reading current position...")
    pos = client.read_joints()
    print(f"Current position (rad): {[f'{p:.3f}' for p in pos]}")
    
    # Test small movement
    print("\nMoving to neutral position [0, 0, 0, 0, 0, 0]...")
    client.write_joints([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    time.sleep(2)
    
    print("Reading new position...")
    pos = client.read_joints()
    print(f"New position (rad): {[f'{p:.3f}' for p in pos]}")
    
    client.disconnect()