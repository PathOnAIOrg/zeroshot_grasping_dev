#!/usr/bin/env python3
"""
Startup script for ThinkGrasp FastAPI server (No Ray version)
"""
import uvicorn
import argparse
from pathlib import Path

def main():
    parser = argparse.ArgumentParser(description='Start ThinkGrasp FastAPI server (No Ray)')
    parser.add_argument('--host', default='0.0.0.0', help='Host to bind to')
    parser.add_argument('--port', type=int, default=8000, help='Port to bind to')
    parser.add_argument('--reload', action='store_true', help='Enable auto-reload for development')
    args = parser.parse_args()
    
    # Ensure output directories exist
    Path('outputs/roborefit').mkdir(parents=True, exist_ok=True)
    Path('outputs/graspnet').mkdir(parents=True, exist_ok=True)
    Path('static').mkdir(parents=True, exist_ok=True)
    
    print(f"Starting ThinkGrasp server (Ray-Free) on {args.host}:{args.port}")
    print("Access the web interface at: http://localhost:8000")
    print("This version uses direct LangSAM integration without Ray")
    
    uvicorn.run(
        "realarm_fastapi_no_ray:app",
        host=args.host,
        port=args.port,
        reload=args.reload
    )

if __name__ == "__main__":
    main()