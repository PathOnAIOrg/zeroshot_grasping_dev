#!/bin/bash

# SO-101 Robot Web UI Launcher
# Simple script to start the web control interface

echo "=================================="
echo "🤖 SO-101 Robot Web Control"
echo "=================================="
echo ""

# Check if conda environment exists
if conda env list | grep -q "^sim "; then
    echo "✓ Found 'sim' conda environment"
    
    # Activate conda environment
    eval "$(conda shell.bash hook)"
    conda activate sim
    
    echo "✓ Activated conda environment"
else
    echo "⚠️  'sim' conda environment not found"
    echo "   Using system Python instead"
fi

# Navigate to web control directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Install required packages if needed
echo ""
echo "Checking dependencies..."
if ! python -c "import flask" 2>/dev/null; then
    echo "Installing Flask..."
    pip install flask
fi

# Start the web server
echo ""
echo "=================================="
echo "Starting web server..."
echo "=================================="
echo ""
echo "📱 Open your browser at:"
echo "   http://localhost:5000"
echo ""
echo "Press Ctrl+C to stop the server"
echo "=================================="
echo ""

# Run the Flask app
python app.py