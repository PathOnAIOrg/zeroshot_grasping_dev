#!/usr/bin/env python3
"""
Standalone ThinkGrasp API Test Client
This client directly calls the Flask API without requiring ROS2
"""

import requests
import json
import sys
import os
import argparse
from pathlib import Path
import shutil
import tempfile

def test_grasp_api(rgb_path, depth_path, instruction, api_host='localhost', api_port=5010):
    """Test the ThinkGrasp API with given images and instruction"""
    
    # Check if files exist
    if not os.path.exists(rgb_path):
        print(f"Error: RGB image not found: {rgb_path}")
        return False
    
    if not os.path.exists(depth_path):
        print(f"Error: Depth image not found: {depth_path}")
        return False
    
    # Create temporary directory for files
    temp_dir = tempfile.mkdtemp(prefix='thinkgrasp_test_')
    
    try:
        # Copy files to temp directory with standard names
        temp_rgb = os.path.join(temp_dir, 'rgb.png')
        temp_depth = os.path.join(temp_dir, 'depth.png')
        temp_text = os.path.join(temp_dir, 'instruction.txt')
        
        shutil.copy2(rgb_path, temp_rgb)
        shutil.copy2(depth_path, temp_depth)
        
        with open(temp_text, 'w') as f:
            f.write(instruction)
        
        # Prepare API request
        url = f"http://{api_host}:{api_port}/grasp_pose"
        
        payload = {
            'image_path': temp_rgb,
            'depth_path': temp_depth,
            'text_path': temp_text
        }
        
        print(f"Sending request to {url}")
        print(f"RGB: {rgb_path}")
        print(f"Depth: {depth_path}")
        print(f"Instruction: {instruction}")
        print("-" * 50)
        
        # Send request
        response = requests.post(url, json=payload)
        
        if response.status_code == 200:
            result = response.json()
            print("✓ Grasp detection successful!")
            print(f"Timestamp: {result.get('timestamp', 'N/A')}")
            print(f"Position (xyz): {result.get('xyz', 'N/A')}")
            print(f"Rotation matrix: {result.get('rot', 'N/A')}")
            print(f"Depth: {result.get('dep', 'N/A')}")
            print(f"Summary file: {result.get('summary_file', 'N/A')}")
            print(f"\nVisualization URL: {result.get('message', 'N/A')}")
            
            # Save results to file
            output_file = f"grasp_result_{result.get('timestamp', 'unknown')}.json"
            with open(output_file, 'w') as f:
                json.dump(result, f, indent=2)
            print(f"\nResults saved to: {output_file}")
            
            return True
        else:
            print(f"✗ Error: Request failed with status {response.status_code}")
            print(f"Response: {response.text}")
            return False
            
    except requests.exceptions.ConnectionError:
        print(f"✗ Error: Could not connect to ThinkGrasp API at {api_host}:{api_port}")
        print("Make sure the API server is running: python realarm310.py")
        return False
    except Exception as e:
        print(f"✗ Error: {str(e)}")
        return False
    finally:
        # Clean up temp directory
        if os.path.exists(temp_dir):
            shutil.rmtree(temp_dir)

def main():
    parser = argparse.ArgumentParser(description='Test ThinkGrasp API')
    parser.add_argument('rgb_image', help='Path to RGB image')
    parser.add_argument('depth_image', help='Path to depth image')
    parser.add_argument('instruction', help='Grasp instruction text')
    parser.add_argument('--host', default='localhost', help='API host (default: localhost)')
    parser.add_argument('--port', type=int, default=5010, help='API port (default: 5010)')
    
    args = parser.parse_args()
    
    # Test the API
    success = test_grasp_api(
        args.rgb_image,
        args.depth_image,
        args.instruction,
        args.host,
        args.port
    )
    
    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()