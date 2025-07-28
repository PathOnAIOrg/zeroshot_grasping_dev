#!/usr/bin/env python3
"""
Start MeshCat visualization server
"""

import meshcat

print("Starting MeshCat server...")
print("Open your browser at: http://127.0.0.1:7000/static/")
print("Press Ctrl+C to stop the server")

server = meshcat.Visualizer()
server.open()

# Keep the server running
try:
    while True:
        pass
except KeyboardInterrupt:
    print("\nStopping MeshCat server...")