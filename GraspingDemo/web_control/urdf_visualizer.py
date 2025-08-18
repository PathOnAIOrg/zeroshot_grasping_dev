#!/usr/bin/env python3
"""
URDF Robot Model Visualizer for Plotly
Parses URDF/XACRO files and creates 3D visualization
"""

import numpy as np
import xml.etree.ElementTree as ET
import plotly.graph_objects as go
from pathlib import Path
from typing import Dict
import struct

class STLReader:
    """Simple STL file reader for binary and ASCII formats"""
    
    @staticmethod
    def read_stl(filepath):
        """
        Read STL file and return vertices and faces
        
        Returns:
            vertices: Nx3 array of vertex coordinates
            faces: Mx3 array of face indices
        """
        filepath = Path(filepath)
        if not filepath.exists():
            print(f"STL file not found: {filepath}")
            return None, None
            
        with open(filepath, 'rb') as f:
            # Check if binary or ASCII
            header = f.read(5)
            f.seek(0)
            
            if header.startswith(b'solid'):
                # Might be ASCII, but check further
                # Some binary files incorrectly start with 'solid'
                try:
                    content = f.read().decode('ascii')
                    if 'facet normal' in content:
                        return STLReader._read_ascii_stl(content)
                except:
                    pass
                # If decode fails or no facet normal, treat as binary
                f.seek(0)
                return STLReader._read_binary_stl(f)
            else:
                return STLReader._read_binary_stl(f)
    
    @staticmethod
    def _read_binary_stl(f):
        """Read binary STL file"""
        # Skip 80-byte header
        f.read(80)
        
        # Read number of triangles
        num_triangles = struct.unpack('<I', f.read(4))[0]
        
        vertices = []
        faces = []
        vertex_dict = {}  # To avoid duplicate vertices
        vertex_index = 0
        
        for i in range(num_triangles):
            # Read normal (3 floats) - we skip this
            f.read(12)
            
            # Read 3 vertices (3 floats each)
            face_indices = []
            for j in range(3):
                vertex = struct.unpack('<fff', f.read(12))
                vertex_key = tuple(vertex)
                
                if vertex_key not in vertex_dict:
                    vertex_dict[vertex_key] = vertex_index
                    vertices.append(vertex)
                    vertex_index += 1
                
                face_indices.append(vertex_dict[vertex_key])
            
            faces.append(face_indices)
            
            # Skip attribute byte count
            f.read(2)
        
        return np.array(vertices), np.array(faces)
    
    @staticmethod
    def _read_ascii_stl(content):
        """Read ASCII STL file"""
        vertices = []
        faces = []
        vertex_dict = {}
        vertex_index = 0
        
        lines = content.strip().split('\n')
        i = 0
        
        while i < len(lines):
            line = lines[i].strip()
            
            if line.startswith('facet normal'):
                # Start of a face
                face_indices = []
                i += 1  # Skip 'outer loop'
                
                for j in range(3):
                    i += 1
                    vertex_line = lines[i].strip()
                    if vertex_line.startswith('vertex'):
                        coords = vertex_line.split()[1:4]
                        vertex = tuple(float(c) for c in coords)
                        
                        if vertex not in vertex_dict:
                            vertex_dict[vertex] = vertex_index
                            vertices.append(vertex)
                            vertex_index += 1
                        
                        face_indices.append(vertex_dict[vertex])
                
                if len(face_indices) == 3:
                    faces.append(face_indices)
                
                i += 2  # Skip 'endloop' and 'endfacet'
            
            i += 1
        
        return np.array(vertices), np.array(faces)


class URDFVisualizer:
    """Parse and visualize URDF robot models"""
    
    def __init__(self, urdf_path: str = None):
        """
        Initialize URDF visualizer
        
        Args:
            urdf_path: Path to URDF or XACRO file
        """
        if urdf_path is None:
            # Default path to SO101 URDF (now in local folder)
            urdf_path = "/home/pathonai/Documents/Github/opensource_dev/GraspingDemo/robot_description/urdf/so101_base.xacro"
        
        self.urdf_path = Path(urdf_path)
        self.links = {}
        self.joints = {}
        self.joint_positions = {}  # Current joint angles
        self.mesh_cache = {}  # Cache loaded meshes
        
    def parse_urdf(self):
        """Parse URDF/XACRO file to extract robot structure"""
        if not self.urdf_path.exists():
            print(f"URDF file not found: {self.urdf_path}")
            return False
            
        try:
            tree = ET.parse(self.urdf_path)
            root = tree.getroot()
            
            # Parse links
            for link in root.findall('.//link'):
                link_name = link.get('name')
                self.links[link_name] = {
                    'name': link_name,
                    'visual': [],
                    'collision': [],
                    'inertial': None
                }
                
                # Parse visual elements
                for visual in link.findall('visual'):
                    visual_data = self._parse_visual(visual)
                    if visual_data:
                        self.links[link_name]['visual'].append(visual_data)
            
            # Parse joints (skip duplicates and joints without parent/child)
            for joint in root.findall('.//joint'):
                joint_name = joint.get('name')
                joint_type = joint.get('type')
                
                parent = joint.find('parent')
                child = joint.find('child')
                origin = joint.find('origin')
                
                # Skip if joint already exists or has no parent/child
                if joint_name in self.joints:
                    continue
                if parent is None or child is None:
                    continue
                    
                parent_link = parent.get('link')
                child_link = child.get('link')
                
                # Skip if parent or child link is invalid
                if not parent_link or not child_link:
                    continue
                
                self.joints[joint_name] = {
                    'name': joint_name,
                    'type': joint_type,
                    'parent': parent_link,
                    'child': child_link,
                    'origin': self._parse_origin(origin) if origin is not None else {'xyz': [0, 0, 0], 'rpy': [0, 0, 0]}
                }
                
                # Initialize joint position
                self.joint_positions[joint_name] = 0.0
                
            # Successfully parsed
            return True
            
        except Exception as e:
            print(f"Error parsing URDF: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _parse_visual(self, visual_elem):
        """Parse visual element from URDF"""
        visual_data = {}
        
        # Get origin
        origin = visual_elem.find('origin')
        if origin is not None:
            visual_data['origin'] = self._parse_origin(origin)
        else:
            visual_data['origin'] = {'xyz': [0, 0, 0], 'rpy': [0, 0, 0]}
        
        # Get geometry
        geometry = visual_elem.find('geometry')
        if geometry is not None:
            # Check for basic shapes
            box = geometry.find('box')
            cylinder = geometry.find('cylinder')
            sphere = geometry.find('sphere')
            mesh = geometry.find('mesh')
            
            if box is not None:
                size = box.get('size', '0.1 0.1 0.1').split()
                visual_data['geometry'] = {
                    'type': 'box',
                    'size': [float(s) for s in size]
                }
            elif cylinder is not None:
                visual_data['geometry'] = {
                    'type': 'cylinder',
                    'radius': float(cylinder.get('radius', 0.05)),
                    'length': float(cylinder.get('length', 0.1))
                }
            elif sphere is not None:
                visual_data['geometry'] = {
                    'type': 'sphere',
                    'radius': float(sphere.get('radius', 0.05))
                }
            elif mesh is not None:
                visual_data['geometry'] = {
                    'type': 'mesh',
                    'filename': mesh.get('filename', '')
                }
        
        # Get material
        material = visual_elem.find('material')
        if material is not None:
            color = material.find('color')
            if color is not None:
                rgba = color.get('rgba', '0.5 0.5 0.5 1').split()
                visual_data['color'] = [float(c) for c in rgba]
            else:
                visual_data['color'] = [0.7, 0.7, 0.7, 1.0]
        else:
            visual_data['color'] = [0.7, 0.7, 0.7, 1.0]
            
        return visual_data
    
    def _parse_origin(self, origin_elem):
        """Parse origin element to get xyz and rpy"""
        xyz = origin_elem.get('xyz', '0 0 0').split()
        rpy = origin_elem.get('rpy', '0 0 0').split()
        
        return {
            'xyz': [float(x) for x in xyz],
            'rpy': [float(r) for r in rpy]
        }
    
    def _create_transform_matrix(self, xyz, rpy):
        """Create 4x4 transformation matrix from xyz and rpy"""
        # Translation
        T = np.eye(4)
        T[:3, 3] = xyz
        
        # Rotation (Roll-Pitch-Yaw)
        roll, pitch, yaw = rpy
        
        # Roll (X-axis)
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])
        
        # Pitch (Y-axis)
        Ry = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])
        
        # Yaw (Z-axis)
        Rz = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])
        
        # Combined rotation
        R = Rz @ Ry @ Rx
        T[:3, :3] = R
        
        return T
    
    def get_link_transforms(self):
        """Calculate global transform for each link based on joint positions"""
        transforms = {}
        
        # Start with base/world frame
        transforms['world'] = np.eye(4)
        transforms['base'] = np.eye(4)
        
        # Process joints to get link transforms
        processed = set(['world', 'base'])
        
        while len(processed) < len(self.links):
            for joint_name, joint in self.joints.items():
                parent = joint['parent']
                child = joint['child']
                
                if parent in processed and child not in processed:
                    # Get parent transform
                    parent_transform = transforms.get(parent, np.eye(4))
                    
                    # Get joint transform
                    origin = joint['origin']
                    joint_transform = self._create_transform_matrix(origin['xyz'], origin['rpy'])
                    
                    # Apply joint angle if it's a revolute joint
                    if joint['type'] == 'revolute' or joint['type'] == 'continuous':
                        angle = self.joint_positions.get(joint_name, 0)
                        # Assume rotation around Z-axis (simplified)
                        rotation = np.eye(4)
                        rotation[:3, :3] = np.array([
                            [np.cos(angle), -np.sin(angle), 0],
                            [np.sin(angle), np.cos(angle), 0],
                            [0, 0, 1]
                        ])
                        joint_transform = joint_transform @ rotation
                    
                    # Calculate child transform
                    transforms[child] = parent_transform @ joint_transform
                    processed.add(child)
        
        return transforms
    
    def create_plotly_traces(self, joint_angles: Dict[str, float] = None, 
                            robot_position: np.ndarray = None,
                            robot_rotation: np.ndarray = None,
                            scale: float = 100.0,
                            simplified: bool = True):
        """
        Create Plotly traces for the robot model
        
        Args:
            joint_angles: Dictionary of joint angles in radians
            robot_position: Robot base position [x, y, z] in meters
            robot_rotation: Robot base rotation [roll, pitch, yaw] in degrees
            scale: Scale factor to convert to display units (100 for cm)
        
        Returns:
            List of Plotly trace objects
        """
        traces = []
        
        # Update joint positions if provided
        if joint_angles:
            self.joint_positions.update(joint_angles)
        
        # Get transforms for all links
        transforms = self.get_link_transforms()
        
        # Create base transform with position and rotation
        base_transform = np.eye(4)
        
        # Apply rotation if provided (convert degrees to radians)
        if robot_rotation is not None:
            roll_rad = np.deg2rad(robot_rotation[0]) if len(robot_rotation) > 0 else 0
            pitch_rad = np.deg2rad(robot_rotation[1]) if len(robot_rotation) > 1 else 0
            yaw_rad = np.deg2rad(robot_rotation[2]) if len(robot_rotation) > 2 else 0
            
            # Create rotation matrix (ZYX Euler order)
            Rx = np.array([
                [1, 0, 0],
                [0, np.cos(roll_rad), -np.sin(roll_rad)],
                [0, np.sin(roll_rad), np.cos(roll_rad)]
            ])
            
            Ry = np.array([
                [np.cos(pitch_rad), 0, np.sin(pitch_rad)],
                [0, 1, 0],
                [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]
            ])
            
            Rz = np.array([
                [np.cos(yaw_rad), -np.sin(yaw_rad), 0],
                [np.sin(yaw_rad), np.cos(yaw_rad), 0],
                [0, 0, 1]
            ])
            
            # Combined rotation
            base_transform[:3, :3] = Rz @ Ry @ Rx
        
        # Apply position if provided
        if robot_position is not None:
            base_transform[:3, 3] = robot_position
        
        # Apply base transform to all links
        for link_name in transforms:
            transforms[link_name] = base_transform @ transforms[link_name]
        
        # If simplified mode, just show joint positions and connections
        if simplified:
            # Collect all joint positions
            joint_positions_x = []
            joint_positions_y = []
            joint_positions_z = []
            joint_names = []
            
            for link_name in ['base', 'shoulder', 'upper_arm', 'lower_arm', 'wrist', 'gripper', 'jaw']:
                if link_name in transforms:
                    pos = transforms[link_name][:3, 3] * scale
                    joint_positions_x.append(pos[0])
                    joint_positions_y.append(pos[1])
                    joint_positions_z.append(pos[2])
                    joint_names.append(link_name)
            
            # Add robot joint markers
            if joint_positions_x:
                traces.append(go.Scatter3d(
                    x=joint_positions_x,
                    y=joint_positions_y,
                    z=joint_positions_z,
                    mode='markers+text',
                    marker=dict(
                        size=10,
                        color='orange',
                        symbol='circle',
                        line=dict(color='darkred', width=2)
                    ),
                    text=joint_names,
                    textposition='top center',
                    name='Robot Joints',
                    hovertemplate='%{text}<br>X: %{x:.1f}<br>Y: %{y:.1f}<br>Z: %{z:.1f}'
                ))
        else:
            # Create detailed traces for each link
            for link_name, link_data in self.links.items():
                if link_name not in transforms:
                    continue
                    
                transform = transforms[link_name]
                
                # Add a larger marker at the link position for visibility
                pos = transform[:3, 3] * scale
                traces.append(go.Scatter3d(
                    x=[pos[0]], y=[pos[1]], z=[pos[2]],
                    mode='markers+text',
                    marker=dict(
                        size=8,
                        color='orange',
                        symbol='circle',
                        line=dict(color='darkred', width=1)
                    ),
                    text=[link_name],
                    textposition='top center',
                    name=f'{link_name}',
                    hovertemplate=f'{link_name}<br>X: %{{x:.1f}}<br>Y: %{{y:.1f}}<br>Z: %{{z:.1f}}',
                    showlegend=False
                ))
            
                # Create simplified geometry for each visual element
                for visual in link_data['visual']:
                    geometry = visual.get('geometry', {})
                    origin = visual['origin']
                    color = visual.get('color', [0.7, 0.7, 0.7, 1.0])
                    
                    # Apply visual origin transform
                    visual_transform = self._create_transform_matrix(origin['xyz'], origin['rpy'])
                    full_transform = transform @ visual_transform
                
                    # Create geometry based on type
                    if geometry.get('type') == 'box':
                        trace = self._create_box_trace(
                            geometry['size'], full_transform, color, link_name, scale
                        )
                        traces.append(trace)
                    elif geometry.get('type') == 'cylinder':
                        trace = self._create_cylinder_trace(
                            geometry['radius'], geometry['length'], 
                            full_transform, color, link_name, scale
                        )
                        traces.append(trace)
                    elif geometry.get('type') == 'sphere':
                        trace = self._create_sphere_trace(
                            geometry['radius'], full_transform, color, link_name, scale
                        )
                        traces.append(trace)
                    elif geometry.get('type') == 'mesh':
                        # Load and display mesh
                        trace = self._create_mesh_trace(
                            geometry['filename'], full_transform, color, link_name, scale
                        )
                        traces.append(trace)
                    else:
                        # For unknown geometry, create a simple marker
                        pos = full_transform[:3, 3] * scale
                        traces.append(go.Scatter3d(
                            x=[pos[0]], y=[pos[1]], z=[pos[2]],
                            mode='markers',
                            marker=dict(
                                size=5,
                                color=f'rgba({int(color[0]*255)},{int(color[1]*255)},{int(color[2]*255)},{color[3]})'
                            ),
                            name=f'{link_name}',
                            hovertemplate=f'{link_name}<br>X: %{{x:.1f}}<br>Y: %{{y:.1f}}<br>Z: %{{z:.1f}}'
                        ))
        
        # Add joint connections
        joint_lines_x = []
        joint_lines_y = []
        joint_lines_z = []
        
        for joint_name, joint in self.joints.items():
            parent = joint['parent']
            child = joint['child']
            
            if parent in transforms and child in transforms:
                parent_pos = transforms[parent][:3, 3] * scale
                child_pos = transforms[child][:3, 3] * scale
                
                joint_lines_x.extend([parent_pos[0], child_pos[0], None])
                joint_lines_y.extend([parent_pos[1], child_pos[1], None])
                joint_lines_z.extend([parent_pos[2], child_pos[2], None])
        
        # Add joint connection lines
        if joint_lines_x:  # Only add if there are connections
            traces.append(go.Scatter3d(
                x=joint_lines_x,
                y=joint_lines_y,
                z=joint_lines_z,
                mode='lines',
                line=dict(color='red', width=5),
                name='Robot Structure',
                hoverinfo='name'
            ))
        
        return traces
    
    def _create_box_trace(self, size, transform, color, name, scale):
        """Create a box trace"""
        # Create box vertices
        sx, sy, sz = [s * scale for s in size]
        vertices = np.array([
            [-sx/2, -sy/2, -sz/2, 1],
            [sx/2, -sy/2, -sz/2, 1],
            [sx/2, sy/2, -sz/2, 1],
            [-sx/2, sy/2, -sz/2, 1],
            [-sx/2, -sy/2, sz/2, 1],
            [sx/2, -sy/2, sz/2, 1],
            [sx/2, sy/2, sz/2, 1],
            [-sx/2, sy/2, sz/2, 1]
        ]).T
        
        # Transform vertices
        transformed = (transform @ vertices)[:3, :]
        transformed *= scale
        
        # Create box edges
        edges_x = []
        edges_y = []
        edges_z = []
        
        # Define box edges
        edge_pairs = [
            (0,1), (1,2), (2,3), (3,0),  # Bottom face
            (4,5), (5,6), (6,7), (7,4),  # Top face
            (0,4), (1,5), (2,6), (3,7)   # Vertical edges
        ]
        
        for i, j in edge_pairs:
            edges_x.extend([transformed[0,i], transformed[0,j], None])
            edges_y.extend([transformed[1,i], transformed[1,j], None])
            edges_z.extend([transformed[2,i], transformed[2,j], None])
        
        return go.Scatter3d(
            x=edges_x, y=edges_y, z=edges_z,
            mode='lines',
            line=dict(
                color=f'rgba({int(color[0]*255)},{int(color[1]*255)},{int(color[2]*255)},{color[3]})',
                width=2
            ),
            name=name,
            hoverinfo='name'
        )
    
    def _create_cylinder_trace(self, radius, length, transform, color, name, scale):
        """Create a simplified cylinder trace"""
        # Create circles at top and bottom
        theta = np.linspace(0, 2*np.pi, 20)
        
        # Bottom circle
        x_bottom = radius * np.cos(theta) * scale
        y_bottom = radius * np.sin(theta) * scale
        z_bottom = np.full_like(theta, -length/2) * scale
        
        # Top circle
        x_top = radius * np.cos(theta) * scale
        y_top = radius * np.sin(theta) * scale
        z_top = np.full_like(theta, length/2) * scale
        
        # Combine points
        points = np.vstack([
            np.concatenate([x_bottom, x_top]),
            np.concatenate([y_bottom, y_top]),
            np.concatenate([z_bottom, z_top]),
            np.ones(len(theta) * 2)
        ])
        
        # Transform
        transformed = (transform @ points)[:3, :] * scale
        
        n = len(theta)
        
        # Create cylinder lines
        lines_x = []
        lines_y = []
        lines_z = []
        
        # Bottom circle
        for i in range(n):
            j = (i + 1) % n
            lines_x.extend([transformed[0,i], transformed[0,j], None])
            lines_y.extend([transformed[1,i], transformed[1,j], None])
            lines_z.extend([transformed[2,i], transformed[2,j], None])
        
        # Top circle
        for i in range(n):
            j = (i + 1) % n
            lines_x.extend([transformed[0,n+i], transformed[0,n+j], None])
            lines_y.extend([transformed[1,n+i], transformed[1,n+j], None])
            lines_z.extend([transformed[2,n+i], transformed[2,n+j], None])
        
        # Vertical lines
        for i in range(0, n, 5):  # Every 5th line for clarity
            lines_x.extend([transformed[0,i], transformed[0,n+i], None])
            lines_y.extend([transformed[1,i], transformed[1,n+i], None])
            lines_z.extend([transformed[2,i], transformed[2,n+i], None])
        
        return go.Scatter3d(
            x=lines_x, y=lines_y, z=lines_z,
            mode='lines',
            line=dict(
                color=f'rgba({int(color[0]*255)},{int(color[1]*255)},{int(color[2]*255)},{color[3]})',
                width=2
            ),
            name=name,
            hoverinfo='name'
        )
    
    def _load_mesh(self, filename):
        """Load mesh from file (with caching)"""
        if filename in self.mesh_cache:
            return self.mesh_cache[filename]
        
        # Resolve package:// URLs to local path
        if filename.startswith('package://'):
            # Replace package:// with local robot_description path
            filename = filename.replace('package://lerobot_description', 
                                      '/home/pathonai/Documents/Github/opensource_dev/GraspingDemo/robot_description')
        
        filepath = Path(filename)
        if not filepath.exists():
            print(f"Mesh file not found: {filepath}")
            return None
        
        # Load STL file
        vertices, faces = STLReader.read_stl(filepath)
        
        if vertices is not None and faces is not None:
            self.mesh_cache[filename] = (vertices, faces)
            return vertices, faces
        
        return None
    
    def _create_mesh_trace(self, filename, transform, color, name, scale):
        """Create a mesh trace from STL file"""
        mesh_data = self._load_mesh(filename)
        
        if mesh_data is None:
            # Return a placeholder marker if mesh can't be loaded
            pos = transform[:3, 3] * scale
            return go.Scatter3d(
                x=[pos[0]], y=[pos[1]], z=[pos[2]],
                mode='markers',
                marker=dict(size=5, color='gray'),
                name=f'{name} (mesh)',
                hoverinfo='name'
            )
        
        vertices, faces = mesh_data
        
        # Transform vertices
        vertices_homo = np.column_stack([vertices, np.ones(len(vertices))])
        transformed_vertices = (transform @ vertices_homo.T).T[:, :3] * scale
        
        # Create mesh3d trace for better visualization
        return go.Mesh3d(
            x=transformed_vertices[:, 0],
            y=transformed_vertices[:, 1],
            z=transformed_vertices[:, 2],
            i=faces[:, 0],
            j=faces[:, 1],
            k=faces[:, 2],
            color=f'rgba({int(color[0]*255)},{int(color[1]*255)},{int(color[2]*255)},{color[3]*0.7})',
            opacity=0.7,
            name=f'{name}',
            hoverinfo='name',
            showscale=False
        )
    
    def _create_sphere_trace(self, radius, transform, color, name, scale):
        """Create a simplified sphere trace"""
        # Create sphere using latitude/longitude lines
        u = np.linspace(0, 2 * np.pi, 20)
        v = np.linspace(0, np.pi, 10)
        
        x = radius * np.outer(np.cos(u), np.sin(v)) * scale
        y = radius * np.outer(np.sin(u), np.sin(v)) * scale
        z = radius * np.outer(np.ones(np.size(u)), np.cos(v)) * scale
        
        # Transform center position
        pos = transform[:3, 3] * scale
        
        return go.Scatter3d(
            x=[pos[0]], y=[pos[1]], z=[pos[2]],
            mode='markers',
            marker=dict(
                size=radius * scale * 20,  # Approximate size
                color=f'rgba({int(color[0]*255)},{int(color[1]*255)},{int(color[2]*255)},{color[3]})',
                symbol='circle'
            ),
            name=name,
            hovertemplate=f'{name}<br>X: %{{x:.1f}}<br>Y: %{{y:.1f}}<br>Z: %{{z:.1f}}'
        )


def add_urdf_to_plotly(fig=None, urdf_path=None, joint_angles=None, robot_position=None):
    """
    Convenience function to add URDF robot model to existing Plotly figure
    
    Args:
        fig: Existing Plotly figure (if None, creates new one)
        urdf_path: Path to URDF file
        joint_angles: Dictionary of joint angles
        robot_position: Robot base position [x, y, z] in meters
    
    Returns:
        Updated Plotly figure
    """
    visualizer = URDFVisualizer(urdf_path)
    
    if visualizer.parse_urdf():
        traces = visualizer.create_plotly_traces(
            joint_angles=joint_angles,
            robot_position=robot_position,
            scale=100.0  # Convert to cm for display
        )
        
        if fig is None:
            fig = go.Figure()
        
        for trace in traces:
            fig.add_trace(trace)
    
    return fig