#!/usr/bin/env python3
"""
Simple script to start the Grasp Pose Visualizer server
"""

import subprocess
import sys
import os
from pathlib import Path

def main():
    # Check if we're in the right directory
    if not Path("backend/app.py").exists():
        print("❌ Error: Please run this script from the grasp_pose_visualizer directory")
        sys.exit(1)
    
    print("🚀 Starting Grasp Pose Visualizer...")
    print("📋 Server will be available at: http://localhost:3001")
    print("🔧 Use Ctrl+C to stop the server")
    print("-" * 50)
    
    # Change to backend directory and start the server
    os.chdir("backend")
    
    try:
        subprocess.run([sys.executable, "app.py"], check=True)
    except KeyboardInterrupt:
        print("\n🛑 Server stopped by user")
    except subprocess.CalledProcessError as e:
        print(f"❌ Server failed to start: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()