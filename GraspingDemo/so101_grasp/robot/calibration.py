"""
Robot Calibration Module

Handles SO-101 robot joint calibration and torque management.
"""

import os
import time
from pathlib import Path
from typing import Optional
from .so101_client import SO101Client


class RobotCalibrator:
    """Handles robot calibration operations."""
    
    def __init__(self, port: str, calibration_path: str = "config/calibration/robot_calibration.json"):
        """
        Initialize robot calibrator.
        
        Args:
            port: Robot serial port
            calibration_path: Path to save calibration data
        """
        self.port = port
        self.calibration_path = calibration_path
        self.client: Optional[SO101Client] = None
    
    def calibrate_robot(self, force_recalibrate: bool = False) -> bool:
        """
        Perform robot calibration.
        
        Args:
            force_recalibrate: Force recalibration even if file exists
            
        Returns:
            True if calibration successful
        """
        try:
            print("Starting robot calibration...")
            print(f"Port: {self.port}")
            
            # Create client without auto-calibration
            self.client = SO101Client(port=self.port, follower=True, force_calibration=False)
            
            if force_recalibrate or not os.path.exists(self.calibration_path):
                print("\nCalibration Process:")
                print("1. Disabling motor torque...")
                self._disable_all_torque()
                
                print("2. Please manually move the robot to neutral position")
                print("   - All joints should be at center positions")
                print("   - Robot should be in comfortable pose")
                input("   Press Enter when robot is positioned correctly...")
                
                print("3. Re-enabling torque and saving calibration...")
                self._enable_all_torque()
                
                # Perform calibration
                self.client.robot.calibrate()
                
                # Ensure directory exists
                os.makedirs(os.path.dirname(self.calibration_path), exist_ok=True)
                
                # Save calibration
                self.client.robot._save_calibration(Path(self.calibration_path))
                print(f"✅ Calibration saved to: {self.calibration_path}")
                
            else:
                # Load existing calibration
                self.client.robot._load_calibration(Path(self.calibration_path))
                print(f"✅ Loaded existing calibration from: {self.calibration_path}")
            
            # Test calibration by reading positions
            print("\nTesting calibration...")
            positions = self.client.read_joints()
            print(f"Current joint positions: {[f'{p:.3f}' for p in positions]}")
            
            return True
            
        except Exception as e:
            print(f"❌ Calibration failed: {e}")
            return False
    
    def _disable_all_torque(self):
        """Disable torque on all motors for manual positioning."""
        try:
            self.client.robot.bus.disable_torque()
            print("   ✅ All motor torque disabled - robot can be moved manually")
        except Exception as e:
            print(f"   ⚠️ Warning: Could not disable torque: {e}")
    
    def _enable_all_torque(self):
        """Re-enable torque on all motors."""
        try:
            self.client.robot.bus.enable_torque()
            time.sleep(1)  # Allow motors to stabilize
            print("   ✅ Motor torque re-enabled")
        except Exception as e:
            print(f"   ⚠️ Warning: Could not enable torque: {e}")
    
    def test_connection(self) -> bool:
        """Test robot connection."""
        try:
            self.client = SO101Client(port=self.port, follower=True, force_calibration=False)
            positions = self.client.read_joints()
            print(f"✅ Connection successful. Current positions: {[f'{p:.3f}' for p in positions]}")
            return True
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            return False
    
    def disconnect(self):
        """Safely disconnect from robot."""
        if self.client:
            self.client.disconnect()
            self.client = None