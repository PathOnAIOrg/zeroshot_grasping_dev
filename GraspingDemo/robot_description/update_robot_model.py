#!/usr/bin/env python3
"""
Helper script to update robot model paths in URDF files
and ensure all references are correct
"""

import os
import re
from pathlib import Path

def update_urdf_paths(urdf_file, base_path):
    """Update mesh file paths in URDF to use local paths"""
    
    with open(urdf_file, 'r') as f:
        content = f.read()
    
    # Replace package:// URLs with relative paths
    # Original: package://lerobot_description/meshes/so101/file.stl
    # New: ../meshes/so101/file.stl (relative to urdf folder)
    
    updated_content = re.sub(
        r'package://lerobot_description/meshes/',
        '../meshes/',
        content
    )
    
    # Save updated file
    output_file = urdf_file.replace('.xacro', '_local.xacro')
    with open(output_file, 'w') as f:
        f.write(updated_content)
    
    print(f"Updated {urdf_file} -> {output_file}")
    return output_file

def verify_mesh_files(urdf_file, base_path):
    """Verify all mesh files referenced in URDF exist"""
    
    with open(urdf_file, 'r') as f:
        content = f.read()
    
    # Find all mesh file references
    mesh_pattern = r'filename="([^"]+\.stl)"'
    meshes = re.findall(mesh_pattern, content)
    
    missing = []
    found = []
    
    for mesh in meshes:
        # Resolve path
        if mesh.startswith('../'):
            # Relative to urdf folder
            mesh_path = Path(urdf_file).parent / mesh
        else:
            mesh_path = Path(mesh)
        
        if mesh_path.exists():
            found.append(mesh)
        else:
            missing.append(mesh)
    
    print(f"\nMesh file verification:")
    print(f"  Found: {len(found)} files")
    print(f"  Missing: {len(missing)} files")
    
    if missing:
        print("\nMissing files:")
        for m in missing:
            print(f"  - {m}")
    
    return len(missing) == 0

def main():
    """Main function"""
    
    base_path = Path(__file__).parent
    urdf_path = base_path / 'urdf'
    
    print("Robot Model Path Updater")
    print("=" * 40)
    print(f"Base path: {base_path}")
    print(f"URDF path: {urdf_path}")
    
    # Process main URDF file
    main_urdf = urdf_path / 'so101_base.xacro'
    
    if main_urdf.exists():
        print(f"\nProcessing {main_urdf.name}...")
        
        # Create local version with updated paths
        local_urdf = update_urdf_paths(str(main_urdf), str(base_path))
        
        # Verify mesh files
        if verify_mesh_files(local_urdf, str(base_path)):
            print("\n✅ All mesh files found!")
        else:
            print("\n⚠️ Some mesh files are missing")
    else:
        print(f"⚠️ URDF file not found: {main_urdf}")
    
    print("\n" + "=" * 40)
    print("Configuration complete!")
    print("\nTo use the updated model:")
    print("  1. Use 'so101_base_local.xacro' for local paths")
    print("  2. Or keep using 'so101_base.xacro' with package:// resolution")

if __name__ == "__main__":
    main()