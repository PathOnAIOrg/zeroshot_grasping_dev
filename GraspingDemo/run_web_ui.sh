#!/bin/bash
# SO-101 Robot Web Control Interface

echo "=========================================="
echo "SO-101 Robot Web Control"
echo "=========================================="
echo ""
echo "Features:"
echo "  • Real-time robot control"
echo "  • Point cloud visualization"
echo "  • Camera streaming"
echo "  • Trajectory recording & playback"
echo "  • Built-in reliability improvements"
echo ""

# Change to web_control directory
cd web_control

# Start the Flask application
echo "Starting web interface..."
echo "Open your browser to: http://localhost:5001"
echo ""
echo "Press Ctrl+C to stop"
echo ""

python app.py 5001