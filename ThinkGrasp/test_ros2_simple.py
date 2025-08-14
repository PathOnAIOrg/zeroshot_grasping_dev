#!/usr/bin/env python3
"""
Simple test to verify ROS2 package build
This doesn't require service definitions, just tests basic structure
"""

import sys
import os

# Add the install path to Python path
sys.path.insert(0, '/home/pathonai/thinkgrasp_ros2_ws/install/thinkgrasp_ros2/lib/python3.12/site-packages')

try:
    # Try to import the package
    import thinkgrasp_ros2
    print("✅ Package import successful")
    
    # Check what's available
    import thinkgrasp_ros2.thinkgrasp_service_node as service
    import thinkgrasp_ros2.thinkgrasp_client_node as client
    import thinkgrasp_ros2.image_capture_node as capture
    import thinkgrasp_ros2.grasp_visualizer_node as visualizer
    
    print("✅ All node modules found")
    
    # List available functions
    print("\nAvailable nodes:")
    print("  - thinkgrasp_service_node")
    print("  - thinkgrasp_client_node")  
    print("  - image_capture_node")
    print("  - grasp_visualizer_node")
    
    print("\n✅ ROS2 package structure is valid")
    print("\nTo use with ROS2:")
    print("  1. Source the workspace: source ~/thinkgrasp_ros2_ws/install/setup.bash")
    print("  2. Run nodes: ros2 run thinkgrasp_ros2 <node_name>")
    print("\nNote: Service/message definitions need separate interface package for ROS2 Jazzy")
    
except ImportError as e:
    print(f"❌ Import error: {e}")
    sys.exit(1)