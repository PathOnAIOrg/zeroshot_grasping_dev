"""
Configuration Management Module

Handles loading and saving of system configuration files.
"""

import yaml
import json
import os
from typing import Dict, Any, Optional
from pathlib import Path


class ConfigManager:
    """Manages configuration files for the SO-101 grasping system."""
    
    def __init__(self, config_dir: str = "config"):
        """
        Initialize configuration manager.
        
        Args:
            config_dir: Directory containing configuration files
        """
        self.config_dir = Path(config_dir)
        self.config_dir.mkdir(parents=True, exist_ok=True)
        
        # Default configurations
        self._default_configs = {
            "robot": {
                "port": "/dev/tty.usbmodem5A680107891",
                "use_degrees": True,
                "joint_limits": {
                    "shoulder_pan": [-180, 180],
                    "shoulder_lift": [-90, 90],
                    "elbow_flex": [0, 180],
                    "wrist_flex": [-90, 90],
                    "wrist_roll": [-180, 180],
                    "gripper": [0, 100]
                },
                "calibration_offset": [
                    0.035926674e-01,
                    1.7880281e+00,
                    -1.5459236e+00,
                    -6.0479313e-01,
                    7.6940347e-04,
                    0.0
                ],
                "motion": {
                    "max_velocity": 1.0,
                    "max_acceleration": 2.0,
                    "interpolation_steps": 50,
                    "timestep": 0.02
                }
            },
            "camera": {
                "width": 1280,
                "height": 720,
                "fps": 30,
                "depth_scale": 0.001,
                "clipping_distance": 3.0,
                "filters": {
                    "voxel_size": 0.005,
                    "statistical_outlier": {
                        "nb_neighbors": 20,
                        "std_ratio": 2.0
                    }
                }
            },
            "grasp": {
                "prediction": {
                    "num_candidates": 50,
                    "confidence_threshold": 0.5,
                    "collision_threshold": 0.02
                },
                "execution": {
                    "approach_distance": 0.1,
                    "grasp_force": 50.0,
                    "lift_height": 0.05,
                    "safety_timeout": 10.0
                }
            }
        }
    
    def load_config(self, config_name: str) -> Dict[str, Any]:
        """
        Load configuration from file.
        
        Args:
            config_name: Name of configuration (robot, camera, grasp)
            
        Returns:
            Configuration dictionary
        """
        config_file = self.config_dir / f"{config_name}_config.yaml"
        
        if config_file.exists():
            try:
                with open(config_file, 'r') as f:
                    config = yaml.safe_load(f)
                print(f"✅ Loaded config from: {config_file}")
                return config
            except Exception as e:
                print(f"❌ Error loading config {config_file}: {e}")
                print("Using default configuration")
        else:
            print(f"⚠️  Config file not found: {config_file}")
            print("Creating default configuration")
            self.save_config(config_name, self._default_configs[config_name])
        
        return self._default_configs.get(config_name, {})
    
    def save_config(self, config_name: str, config: Dict[str, Any]):
        """
        Save configuration to file.
        
        Args:
            config_name: Name of configuration
            config: Configuration dictionary to save
        """
        config_file = self.config_dir / f"{config_name}_config.yaml"
        
        try:
            with open(config_file, 'w') as f:
                yaml.dump(config, f, default_flow_style=False, indent=2)
            print(f"✅ Saved config to: {config_file}")
        except Exception as e:
            print(f"❌ Error saving config {config_file}: {e}")
    
    def get_robot_config(self) -> Dict[str, Any]:
        """Get robot configuration."""
        return self.load_config("robot")
    
    def get_camera_config(self) -> Dict[str, Any]:
        """Get camera configuration."""
        return self.load_config("camera")
    
    def get_grasp_config(self) -> Dict[str, Any]:
        """Get grasp configuration."""
        return self.load_config("grasp")
    
    def update_robot_port(self, port: str):
        """
        Update robot port in configuration.
        
        Args:
            port: New robot port
        """
        config = self.get_robot_config()
        config["port"] = port
        self.save_config("robot", config)
        print(f"✅ Updated robot port to: {port}")
    
    def load_calibration_data(self, calibration_type: str) -> Optional[Any]:
        """
        Load calibration data.
        
        Args:
            calibration_type: Type of calibration (robot, camera, transform)
            
        Returns:
            Calibration data or None if not found
        """
        calib_dir = self.config_dir / "calibration"
        
        if calibration_type == "robot":
            calib_file = calib_dir / "robot_calibration.json"
            if calib_file.exists():
                with open(calib_file, 'r') as f:
                    return json.load(f)
        
        elif calibration_type == "transform":
            calib_file = calib_dir / "transform_matrix.npy"
            if calib_file.exists():
                import numpy as np
                return np.load(calib_file)
        
        elif calibration_type == "camera":
            calib_file = calib_dir / "camera_calibration.npz"
            if calib_file.exists():
                import numpy as np
                return np.load(calib_file)
        
        return None
    
    def save_calibration_data(self, calibration_type: str, data: Any):
        """
        Save calibration data.
        
        Args:
            calibration_type: Type of calibration
            data: Calibration data to save
        """
        calib_dir = self.config_dir / "calibration"
        calib_dir.mkdir(exist_ok=True)
        
        if calibration_type == "robot":
            calib_file = calib_dir / "robot_calibration.json"
            with open(calib_file, 'w') as f:
                json.dump(data, f, indent=2)
        
        elif calibration_type == "transform":
            calib_file = calib_dir / "transform_matrix.npy"
            import numpy as np
            np.save(calib_file, data)
        
        elif calibration_type == "camera":
            calib_file = calib_dir / "camera_calibration.npz"
            import numpy as np
            np.savez(calib_file, **data)
        
        print(f"✅ Saved {calibration_type} calibration data")
    
    def list_configs(self):
        """List all available configuration files."""
        print("Available configuration files:")
        for config_file in self.config_dir.glob("*_config.yaml"):
            print(f"  📄 {config_file.name}")
        
        calib_dir = self.config_dir / "calibration"
        if calib_dir.exists():
            print("\nCalibration files:")
            for calib_file in calib_dir.glob("*"):
                print(f"  📊 {calib_file.name}")
    
    def reset_to_defaults(self, config_name: str):
        """
        Reset configuration to defaults.
        
        Args:
            config_name: Name of configuration to reset
        """
        if config_name in self._default_configs:
            self.save_config(config_name, self._default_configs[config_name])
            print(f"✅ Reset {config_name} configuration to defaults")
        else:
            print(f"❌ Unknown configuration: {config_name}")