#!/usr/bin/env python3
"""
Test checkerboard detection with RealSense camera
Helps identify the correct pattern size for your checkerboard
"""

import cv2 as cv
import numpy as np
import sys

try:
    import pyrealsense2 as rs
except ImportError:
    print("Error: pyrealsense2 not installed")
    print("Install with: pip install pyrealsense2")
    sys.exit(1)


def test_patterns():
    """Test different checkerboard patterns with RealSense."""
    
    # Patterns to test
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
    
    # Initialize RealSense
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    try:
        pipeline.start(config)
        print("✅ RealSense camera initialized")
        print("\n" + "=" * 60)
        print("CHECKERBOARD PATTERN DETECTION TEST")
        print("=" * 60)
        print("\nTesting patterns...")
        print("Show your checkerboard to the camera")
        print("Press 'q' to quit, 's' to save current frame")
        print("=" * 60 + "\n")
        
        # Allow auto-exposure to settle
        for _ in range(30):
            pipeline.wait_for_frames()
        
        detected_patterns = set()
        frame_count = 0
        
        while True:
            # Get frame
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            
            # Convert to numpy array
            frame = np.asanyarray(color_frame.get_data())
            display_frame = frame.copy()
            
            # Convert to grayscale
            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            
            # Try each pattern
            found_pattern = None
            for pattern in patterns_to_try:
                ret, corners = cv.findChessboardCorners(gray, pattern, None)
                if ret:
                    found_pattern = pattern
                    detected_patterns.add(pattern)
                    
                    # Draw the corners
                    cv.drawChessboardCorners(display_frame, pattern, corners, ret)
                    break
            
            # Display status
            if found_pattern:
                text = f"DETECTED: {found_pattern[0]}x{found_pattern[1]} corners"
                color = (0, 255, 0)
            else:
                text = "No checkerboard detected"
                color = (0, 0, 255)
            
            cv.putText(display_frame, text, (10, 30), 
                      cv.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            
            # Show all detected patterns so far
            if detected_patterns:
                y_pos = 60
                cv.putText(display_frame, "Patterns found:", (10, y_pos),
                          cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                for p in sorted(detected_patterns):
                    y_pos += 25
                    cv.putText(display_frame, f"  {p[0]}x{p[1]}", (10, y_pos),
                              cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
            
            # Display frame
            cv.imshow('Checkerboard Pattern Test', display_frame)
            
            key = cv.waitKey(1) & 0xFF
            
            if key == ord('q'):
                break
            elif key == ord('s'):
                # Save frame for debugging
                filename = f'test_frame_{frame_count}.png'
                cv.imwrite(filename, frame)
                print(f"Saved frame to {filename}")
                frame_count += 1
        
        print("\n" + "=" * 60)
        print("TEST RESULTS")
        print("=" * 60)
        
        if detected_patterns:
            print(f"\n✅ Found {len(detected_patterns)} pattern(s):")
            for pattern in sorted(detected_patterns):
                print(f"   {pattern[0]}×{pattern[1]} internal corners")
            
            # Recommend the most common size
            if len(detected_patterns) == 1:
                pattern = list(detected_patterns)[0]
                print(f"\n📌 Your checkerboard pattern is: {pattern[0]}×{pattern[1]}")
                print(f"\nUse this command for calibration:")
                print(f"   python handeye_manual_realsense.py --pattern {pattern[0]}x{pattern[1]}")
            else:
                print("\n⚠️  Multiple patterns detected. Your checkerboard might be:")
                for pattern in sorted(detected_patterns):
                    print(f"   python handeye_manual_realsense.py --pattern {pattern[0]}x{pattern[1]}")
        else:
            print("\n❌ No checkerboard pattern detected")
            print("\nPossible issues:")
            print("1. Checkerboard not fully visible")
            print("2. Poor lighting or shadows")
            print("3. Non-standard pattern size")
            print("4. Checkerboard not flat")
            
    except Exception as e:
        print(f"Error: {e}")
    finally:
        pipeline.stop()
        cv.destroyAllWindows()


if __name__ == "__main__":
    test_patterns()