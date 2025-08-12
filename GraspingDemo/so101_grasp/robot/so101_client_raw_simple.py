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
        Initialize direct servo connection with reliability improvements.
        
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
        
        # Position tracking for reliability
        self.last_valid_positions = [0.0] * 6
        self.position_history = {i: [] for i in range(1, 7)}
        self.max_history = 10
        
        # Communication statistics
        self.stats = {
            'total_reads': 0,
            'failed_reads': 0,
            'recovered_reads': 0
        }
        
        # Connect to serial port with better error handling
        self._connect_with_retry()
        
        # Initialize servos and get initial positions
        self._initialize()
    
    def _connect_with_retry(self, max_attempts=3):
        """Connect to serial port with retry logic"""
        for attempt in range(max_attempts):
            try:
                self.serial_port = serial.Serial(
                    self.port, 
                    1000000, 
                    timeout=0.15,  # Increased timeout
                    write_timeout=0.1,
                    inter_byte_timeout=0.01
                )
                self.serial_port.reset_input_buffer()
                self.serial_port.reset_output_buffer()
                time.sleep(0.1)
                
                # Test connection by reading from servo 1
                test_pos = self.read_servo_position(1, retries=2)
                if test_pos is not None:
                    print(f"✅ Connected to SO-101 on {self.port} (enhanced reliability mode)")
                    return
                else:
                    self.serial_port.close()
                    if attempt < max_attempts - 1:
                        print(f"  Connection test failed, retrying...")
                        time.sleep(0.5)
                    
            except Exception as e:
                if attempt < max_attempts - 1:
                    print(f"  Connection attempt {attempt + 1} failed: {e}")
                    time.sleep(0.5)
                else:
                    raise ConnectionError(f"Failed to connect after {max_attempts} attempts: {e}")
        
        raise ConnectionError(f"Could not establish reliable connection to robot")
    
    def _initialize(self):
        """Initialize servos and get initial positions"""
        print("Initializing robot...")
        
        # Try to get initial positions
        for attempt in range(3):
            try:
                positions = self.read_joints(validate=False)
                self.last_valid_positions = positions
                print(f"  Initial positions: {[f'{p:.3f}' for p in positions]}")
                break
            except:
                if attempt < 2:
                    time.sleep(0.5)
                else:
                    print("  ⚠️ Could not read initial positions")
        
        # Enable all servos
        self.enable_all_servos()
    
    def read_servo_position(self, servo_id, retries=5):
        """Read position from STS3215 servo with enhanced retry logic and validation"""
        valid_readings = []
        
        for attempt in range(retries):
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
                
                # Clear buffer before sending
                self.serial_port.reset_input_buffer()
                
                # Send command
                self.serial_port.write(cmd)
                self.serial_port.flush()  # Ensure data is sent
                
                # Optimal delay for STS3215 response
                time.sleep(0.008)  # Increased from 0.005 for better reliability
                
                # Read response with timeout
                response = self.serial_port.read(8)
                
                if len(response) >= 7:
                    # Validate response header
                    if response[0] == 0xFF and response[1] == 0xFF and response[2] == servo_id:
                        # Verify checksum if we have full response
                        if len(response) >= 8:
                            calc_checksum = (~sum(response[2:7])) & 0xFF
                            if response[7] != calc_checksum:
                                if attempt < retries - 1:
                                    time.sleep(0.015)
                                continue
                        
                        # Extract position (little endian)
                        pos = struct.unpack('<H', response[5:7])[0]
                        if 0 <= pos <= 4095:
                            valid_readings.append(pos)
                            
                            # If we have 2+ consistent readings, return immediately
                            if len(valid_readings) >= 2:
                                # Check if readings are consistent
                                if max(valid_readings) - min(valid_readings) <= 5:
                                    return int(np.median(valid_readings))
                
                # Wait before retry with exponential backoff
                if attempt < retries - 1:
                    time.sleep(0.01 * (1.5 ** attempt))  # Exponential backoff
                    
            except Exception as e:
                if attempt < retries - 1:
                    time.sleep(0.02)
                elif attempt == retries - 1:
                    print(f"  Error reading servo {servo_id}: {e}")
        
        # If we got any valid readings, return the median
        if valid_readings:
            result = int(np.median(valid_readings))
            if len(valid_readings) < 2:
                print(f"  ⚠️ Servo {servo_id}: Only {len(valid_readings)} valid reading(s), data may be unreliable")
            return result
        
        # If we have position history, use last known good value
        if hasattr(self, 'last_valid_positions') and servo_id <= len(self.last_valid_positions):
            last_pos = self.last_valid_positions[servo_id - 1]
            print(f"  ⚠️ Servo {servo_id}: No response, using last known position")
            return self.radians_to_ticks(last_pos)
        
        print(f"  ❌ Failed to read servo {servo_id} after {retries} attempts")
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
        """Write position to STS3215 servo with verification"""
        try:
            # Optionally enable torque first
            if enable_torque_first:
                self.enable_torque(servo_id, True)
            
            # Clamp to valid range
            ticks = int(max(0, min(4095, ticks)))
            
            # Try multiple times for reliability
            for attempt in range(3):
                try:
                    # STS3215 position write command with speed
                    length = 7
                    instruction = 0x03  # Write
                    address = 0x2A      # GOAL_POSITION_L
                    
                    # Convert position to bytes (little endian)
                    pos_l = ticks & 0xFF
                    pos_h = (ticks >> 8) & 0xFF
                    
                    # Speed (0-1023, 0 = max speed)
                    speed = min(1023, max(0, speed))
                    speed_l = speed & 0xFF
                    speed_h = (speed >> 8) & 0xFF
                    
                    # Calculate checksum
                    checksum = (~(servo_id + length + instruction + address + 
                                pos_l + pos_h + speed_l + speed_h)) & 0xFF
                    
                    # Build command
                    cmd = bytes([0xFF, 0xFF, servo_id, length, instruction, address,
                               pos_l, pos_h, speed_l, speed_h, checksum])
                    
                    # Clear buffer and send command
                    self.serial_port.reset_input_buffer()
                    self.serial_port.write(cmd)
                    self.serial_port.flush()
                    time.sleep(0.003)  # Small delay for command processing
                    
                    return True
                    
                except Exception as e:
                    if attempt < 2:
                        time.sleep(0.01)
                    else:
                        raise e
                        
        except Exception as e:
            print(f"  Error writing to servo {servo_id}: {e}")
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
    
    def read_joints(self, validate=True) -> List[float]:
        """
        Read all joint positions in radians with enhanced validation and recovery.
        """
        positions = []
        failed_reads = []
        recovered_count = 0
        
        self.stats['total_reads'] += 1
        
        for i in range(len(self.joint_names)):
            servo_id = i + 1
            ticks = self.read_servo_position(servo_id)
            
            if ticks is None:
                failed_reads.append(servo_id)
                # Use last valid position as fallback
                radians = self.last_valid_positions[i]
                print(f"    Using fallback for servo {servo_id}: {radians:.3f} rad")
                recovered_count += 1
            else:
                radians = self.ticks_to_radians(ticks)
                
                # Validate against history for outlier detection
                if self.position_history[servo_id]:
                    recent = self.position_history[servo_id][-5:]  # Last 5 readings
                    if recent:
                        avg_rad = np.mean([self.ticks_to_radians(t) for t in recent])
                        if abs(radians - avg_rad) > 0.5:  # More than 0.5 rad difference
                            print(f"    ⚠️ Potential outlier on servo {servo_id}: {radians:.3f} vs avg {avg_rad:.3f}")
                            # Try reading again
                            retry_ticks = self.read_servo_position(servo_id, retries=2)
                            if retry_ticks is not None:
                                radians = self.ticks_to_radians(retry_ticks)
                                print(f"    Recovered with retry: {radians:.3f}")
                                recovered_count += 1
                
                # Update history
                self.position_history[servo_id].append(ticks)
                if len(self.position_history[servo_id]) > self.max_history:
                    self.position_history[servo_id].pop(0)
                
                # Update last valid position
                self.last_valid_positions[i] = radians
                
            positions.append(radians)
            time.sleep(0.01)  # Small delay between reads
        
        # Update statistics
        if failed_reads:
            self.stats['failed_reads'] += 1
        if recovered_count > 0:
            self.stats['recovered_reads'] += recovered_count
        
        # Validation
        if validate and len(failed_reads) >= 4:  # Allow up to 3 failed reads
            error_msg = f"Too many failed reads ({len(failed_reads)}/6): {failed_reads}. "
            error_msg += f"Stats: {self.get_stats()}"
            raise ValueError(error_msg)
        elif failed_reads:
            print(f"  ⚠️ Some reads failed but recovered: {failed_reads}")
            
        return positions
    
    def write_joints(self, joints: List[float]):
        """
        Write joint positions in radians with validation and safety checks.
        """
        if len(joints) != 6:
            raise ValueError(f"Expected 6 joint values, got {len(joints)}")
        
        # Check limits if provided
        if self.limits:
            for i, (joint, limit) in enumerate(zip(joints, self.limits)):
                if not (limit[0] <= joint <= limit[1]):
                    raise ValueError(f"Joint {i} out of bounds: {joint:.3f} not in [{limit[0]:.3f}, {limit[1]:.3f}]")
        
        # Safety check for large movements
        max_delta = 0.7  # Maximum 0.7 rad (~40 degrees) per command
        for i, (current, target) in enumerate(zip(self.last_valid_positions, joints)):
            delta = abs(target - current)
            if delta > max_delta:
                print(f"  ⚠️ Large movement on joint {i+1}: {delta:.3f} rad")
                # Could add confirmation or interpolation here
        
        # Write each servo with verification
        success_count = 0
        for i, radians in enumerate(joints):
            servo_id = i + 1
            ticks = self.radians_to_ticks(radians)
            
            # Try to write with verification
            write_success = self.write_servo_position(servo_id, ticks)
            if write_success:
                success_count += 1
                # Update last commanded position
                self.last_valid_positions[i] = radians
            else:
                print(f"  ⚠️ Failed to write servo {servo_id}")
            
            time.sleep(0.01)  # Small delay between servos
        
        if success_count < 4:
            print(f"  ⚠️ Only {success_count}/6 servos responded to command")
    
    def interpolate_waypoint(self, waypoint1, waypoint2, steps=50, timestep=0.02):
        """Interpolate between two waypoints"""
        for i in range(steps):
            alpha = i / (steps - 1)
            q = [(1 - alpha) * w1 + alpha * w2 for w1, w2 in zip(waypoint1, waypoint2)]
            self.write_joints(q)
            time.sleep(timestep)
    
    def get_stats(self):
        """Get communication statistics"""
        total = max(self.stats['total_reads'], 1)
        failed = self.stats['failed_reads']
        recovered = self.stats['recovered_reads']
        
        return {
            'success_rate': f"{100 * (1 - failed/total):.1f}%",
            'total_reads': total,
            'failed_reads': failed,
            'recovered_reads': recovered
        }
    
    def disconnect(self, disable_torque=False, close_port=True):
        """Disconnect from robot"""
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            # Print statistics
            print("\nCommunication Statistics:")
            stats = self.get_stats()
            for key, value in stats.items():
                print(f"  {key}: {value}")
            
            if disable_torque:
                # Optionally disable all servos
                for i in range(6):
                    self.enable_torque(i + 1, False)
                    time.sleep(0.01)
                print("All servos disabled")
            
            if close_port:
                self.serial_port.close()
                print("👋 Disconnected from robot")
            else:
                print("📌 Keeping serial connection open")
    
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