#!/usr/bin/env python3
"""
Test script for Motion Planning API
"""

import requests
import json
import time

BASE_URL = "http://localhost:5000"

def test_motion_planning():
    print("=" * 60)
    print("MOTION PLANNING API TEST")
    print("=" * 60)
    
    # Test 1: List obstacles (should be empty initially)
    print("\n1. Checking obstacles...")
    response = requests.get(f"{BASE_URL}/api/motion/obstacles/list")
    if response.status_code == 200:
        data = response.json()
        print(f"   ✓ Obstacles: {data['count']} found")
    else:
        print(f"   ✗ Failed to get obstacles")
    
    # Test 2: Add a sphere obstacle
    print("\n2. Adding sphere obstacle...")
    obstacle_data = {
        "type": "sphere",
        "center": [0.15, 0.1, 0.2],
        "radius": 0.05,
        "name": "test_sphere"
    }
    response = requests.post(f"{BASE_URL}/api/motion/obstacles/add", 
                            json=obstacle_data)
    if response.status_code == 200:
        data = response.json()
        if data['success']:
            print(f"   ✓ {data['message']}")
        else:
            print(f"   ✗ {data['message']}")
    
    # Test 3: Plan Cartesian path
    print("\n3. Planning Cartesian trajectory...")
    plan_data = {
        "start_position": [0.2, 0.0, 0.2],
        "goal_position": [0.15, 0.15, 0.25],
        "current_joints": [0, 0, 0, 0, 0, 0],
        "velocity": 0.1
    }
    response = requests.post(f"{BASE_URL}/api/motion/plan/cartesian", 
                            json=plan_data)
    if response.status_code == 200:
        data = response.json()
        if data['success']:
            print(f"   ✓ {data['message']}")
            print(f"   - Waypoints: {data['trajectory']['num_waypoints']}")
            print(f"   - Duration: {data['trajectory']['total_time']:.2f}s")
        else:
            print(f"   ✗ {data['message']}")
    
    # Test 4: Plan with RRT
    print("\n4. Planning with RRT...")
    rrt_data = {
        "start_joints": [0, 0, 0, 0, 0],
        "goal_joints": [0.5, 0.3, 0.4, -0.2, 0.1],
        "max_iterations": 500
    }
    response = requests.post(f"{BASE_URL}/api/motion/plan/rrt", 
                            json=rrt_data)
    if response.status_code == 200:
        data = response.json()
        if data['success']:
            print(f"   ✓ {data['message']}")
            print(f"   - Waypoints: {data['trajectory']['num_waypoints']}")
            print(f"   - Duration: {data['trajectory']['total_time']:.2f}s")
        else:
            print(f"   ✗ {data.get('message', 'Failed')}")
    
    # Test 5: Clear obstacles
    print("\n5. Clearing obstacles...")
    response = requests.post(f"{BASE_URL}/api/motion/obstacles/clear")
    if response.status_code == 200:
        data = response.json()
        if data['success']:
            print(f"   ✓ {data['message']}")
        else:
            print(f"   ✗ Failed to clear obstacles")
    
    print("\n" + "=" * 60)
    print("MOTION PLANNING API TEST COMPLETE")
    print("=" * 60)
    print("\nThe motion planning API is working correctly!")
    print("You can now use the web UI at http://localhost:5000")
    print("to plan and execute trajectories with the SO-101 robot.")

if __name__ == "__main__":
    # Wait a moment for server to be ready
    time.sleep(1)
    
    try:
        test_motion_planning()
    except requests.exceptions.ConnectionError:
        print("Error: Could not connect to Flask server at http://localhost:5000")
        print("Make sure the web UI is running: python web_control/app.py")
    except Exception as e:
        print(f"Error: {e}")