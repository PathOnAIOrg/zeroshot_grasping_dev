#!/usr/bin/env python3
"""
Simple ThinkGrasp Client - Works without ROS2 build system
This directly calls the ThinkGrasp API
"""

import sys
import os
import argparse
import requests
import json
import tempfile
import shutil
from pathlib import Path

def call_thinkgrasp_api(rgb_path, depth_path, instruction, api_host='localhost', api_port=5010):
    """Call ThinkGrasp API and return results"""
    
    # Validate input files
    if not os.path.exists(rgb_path):
        raise FileNotFoundError(f"RGB image not found: {rgb_path}")
    
    if not os.path.exists(depth_path):
        raise FileNotFoundError(f"Depth image not found: {depth_path}")
    
    # Create temporary directory for processing
    temp_dir = tempfile.mkdtemp(prefix='thinkgrasp_')
    
    try:
        # Copy files to temp directory
        temp_rgb = os.path.join(temp_dir, 'rgb.png')
        temp_depth = os.path.join(temp_dir, 'depth.png')
        temp_text = os.path.join(temp_dir, 'instruction.txt')
        
        shutil.copy2(rgb_path, temp_rgb)
        shutil.copy2(depth_path, temp_depth)
        
        with open(temp_text, 'w') as f:
            f.write(instruction)
        
        # Call API
        url = f"http://{api_host}:{api_port}/grasp_pose"
        payload = {
            'image_path': temp_rgb,
            'depth_path': temp_depth,
            'text_path': temp_text
        }
        
        print(f"🚀 Calling ThinkGrasp API at {url}")
        response = requests.post(url, json=payload, timeout=60)
        
        if response.status_code == 200:
            return response.json()
        else:
            raise Exception(f"API error ({response.status_code}): {response.text}")
            
    finally:
        # Clean up temp directory
        if os.path.exists(temp_dir):
            shutil.rmtree(temp_dir)

def format_grasp_result(result):
    """Format grasp result for display"""
    output = []
    output.append("\n" + "="*60)
    output.append("✅ GRASP DETECTION SUCCESSFUL")
    output.append("="*60)
    
    # Timestamp
    output.append(f"📅 Timestamp: {result.get('timestamp', 'N/A')}")
    
    # Position
    xyz = result.get('xyz', [])
    if xyz:
        output.append(f"\n📍 Position (XYZ):")
        output.append(f"   X: {xyz[0]:.6f}")
        output.append(f"   Y: {xyz[1]:.6f}")
        output.append(f"   Z: {xyz[2]:.6f}")
    
    # Rotation Matrix
    rot = result.get('rot', [])
    if rot:
        output.append(f"\n🔄 Rotation Matrix:")
        for i, row in enumerate(rot):
            output.append(f"   [{', '.join(f'{val:8.4f}' for val in row)}]")
    
    # Depth
    output.append(f"\n📏 Grasp Depth: {result.get('dep', 'N/A')}")
    
    # Files
    output.append(f"\n📁 Summary File: {result.get('summary_file', 'N/A')}")
    
    # Visualization
    output.append(f"\n🌐 Visualization: {result.get('message', 'N/A')}")
    output.append("="*60)
    
    return '\n'.join(output)

def main():
    parser = argparse.ArgumentParser(
        description='Simple ThinkGrasp Client - Call grasp detection API',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s /path/to/rgb.png /path/to/depth.png "grasp the red cube"
  %(prog)s --host 192.168.1.100 rgb.png depth.png "pick up the bottle"
  
Note: Make sure ThinkGrasp API server is running:
  cd /path/to/ThinkGrasp && python realarm310.py
        """
    )
    
    parser.add_argument('rgb_image', help='Path to RGB image')
    parser.add_argument('depth_image', help='Path to depth image')
    parser.add_argument('instruction', help='Grasp instruction text')
    parser.add_argument('--host', default='localhost', help='API host (default: localhost)')
    parser.add_argument('--port', type=int, default=5010, help='API port (default: 5010)')
    parser.add_argument('--json', action='store_true', help='Output raw JSON response')
    parser.add_argument('--save', help='Save results to JSON file')
    
    args = parser.parse_args()
    
    try:
        # Call API
        print(f"📷 RGB Image: {args.rgb_image}")
        print(f"🔍 Depth Image: {args.depth_image}")
        print(f"📝 Instruction: {args.instruction}")
        print(f"🔗 API: http://{args.host}:{args.port}")
        print("-" * 60)
        
        result = call_thinkgrasp_api(
            args.rgb_image,
            args.depth_image,
            args.instruction,
            args.host,
            args.port
        )
        
        # Output results
        if args.json:
            print(json.dumps(result, indent=2))
        else:
            print(format_grasp_result(result))
        
        # Save results if requested
        if args.save:
            with open(args.save, 'w') as f:
                json.dump(result, f, indent=2)
            print(f"\n💾 Results saved to: {args.save}")
        
        return 0
        
    except requests.exceptions.ConnectionError:
        print(f"\n❌ Error: Could not connect to ThinkGrasp API at {args.host}:{args.port}")
        print("   Make sure the API server is running:")
        print("   cd /path/to/ThinkGrasp && python realarm310.py")
        return 1
        
    except Exception as e:
        print(f"\n❌ Error: {str(e)}")
        return 1

if __name__ == '__main__':
    sys.exit(main())