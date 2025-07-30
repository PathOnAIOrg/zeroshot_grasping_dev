#!/usr/bin/env python3
"""
SO-101 Robot Calibration Script

Calibrates the SO-101 robot joints and saves calibration data.
"""

import sys
import os
import argparse
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from so101_grasp.robot.calibration import RobotCalibrator


def main():
    """Main calibration function."""
    parser = argparse.ArgumentParser(description="SO-101 Robot Calibration")
    parser.add_argument("--port", default="/dev/tty.usbmodem5A680107891", 
                       help="Robot serial port")
    parser.add_argument("--force", action="store_true", 
                       help="Force recalibration even if file exists")
    parser.add_argument("--config-path", default="config/calibration/robot_calibration.json",
                       help="Path to save calibration data")
    
    args = parser.parse_args()
    
    print("SO-101 Robot Calibration")
    print("=" * 40)
    print(f"Port: {args.port}")
    print(f"Config path: {args.config_path}")
    print(f"Force recalibration: {args.force}")
    
    # Create calibrator
    calibrator = RobotCalibrator(args.port, args.config_path)
    
    try:
        # Perform calibration
        success = calibrator.calibrate_robot(args.force)
        
        if success:
            print("\n🎉 Robot calibration completed successfully!")
            print("\nNext steps:")
            print("1. Test robot movement with basic control examples")
            print("2. Proceed to camera calibration")
            print("3. Run full grasp demo")
        else:
            print("\n❌ Robot calibration failed!")
            print("\nTroubleshooting:")
            print("1. Check robot power and USB connection")
            print("2. Verify correct port setting")
            print("3. Ensure no other software is using the robot")
            print("4. Try manually moving robot to neutral position")
            
    except KeyboardInterrupt:
        print("\n\n⏹️  Calibration interrupted by user")
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
    finally:
        calibrator.disconnect()


if __name__ == "__main__":
    main()