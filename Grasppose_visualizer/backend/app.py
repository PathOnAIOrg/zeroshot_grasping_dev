from flask import Flask, request, jsonify, send_from_directory
from flask_cors import CORS
import numpy as np
import cv2
import json
import os
from pathlib import Path
import base64
from io import BytesIO
from PIL import Image
import open3d as o3d
from point_cloud_utils import PointCloudGenerator
from unified_transform import UnifiedTransformer
from reconstruct_pointcloud import PointCloudReconstructor

app = Flask(__name__, static_folder='../frontend', template_folder='../frontend')
CORS(app)

# Configuration
UPLOAD_FOLDER = Path("../uploads")
UPLOAD_FOLDER.mkdir(exist_ok=True)

# Initialize point cloud generator, transformer, and reconstructor
pc_generator = PointCloudGenerator()
transformer = UnifiedTransformer()
reconstructor = PointCloudReconstructor()

@app.route('/')
def index():
    return send_from_directory('../frontend', 'index.html')

@app.route('/static/<path:filename>')
def serve_static(filename):
    return send_from_directory('../static', filename)

@app.route('/api/upload_rgbd', methods=['POST'])
def upload_rgbd():
    """Generate point cloud from RGB-D images"""
    try:
        # Get RGB and depth images from request
        rgb_data = request.files.get('rgb_image')
        depth_data = request.files.get('depth_image')
        
        # Get camera parameters from form data (optional)
        camera_params = {}
        if 'width' in request.form:
            camera_params['width'] = int(request.form['width'])
        if 'height' in request.form:
            camera_params['height'] = int(request.form['height'])
        if 'fx' in request.form:
            camera_params['fx'] = float(request.form['fx'])
        if 'fy' in request.form:
            camera_params['fy'] = float(request.form['fy'])
        if 'cx' in request.form:
            camera_params['cx'] = float(request.form['cx'])
        if 'cy' in request.form:
            camera_params['cy'] = float(request.form['cy'])
        if 'scale' in request.form:
            camera_params['scale'] = float(request.form['scale'])
        
        if not rgb_data or not depth_data:
            return jsonify({'error': 'Both RGB and depth images are required'}), 400
        
        # Save uploaded files
        rgb_path = UPLOAD_FOLDER / 'rgb_image.jpg'
        depth_path = UPLOAD_FOLDER / 'depth_image.png'
        
        rgb_data.save(rgb_path)
        depth_data.save(depth_path)
        
        # Load images
        rgb_image = cv2.imread(str(rgb_path))
        rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
        depth_image = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
        
        # Update camera parameters if provided
        if camera_params:
            for key, value in camera_params.items():
                if hasattr(pc_generator, key):
                    setattr(pc_generator, key, value)
        
        # Generate point cloud
        point_cloud = pc_generator.generate_point_cloud(rgb_image, depth_image)
        
        return jsonify({
            'success': True,
            'point_cloud': point_cloud,
            'message': f'Generated point cloud with {len(point_cloud)} points'
        })
        
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/api/upload_poses', methods=['POST'])
def upload_poses():
    """Accept grasp poses JSON for visualization"""
    try:
        data = request.get_json()
        
        if not data:
            return jsonify({'error': 'No JSON data provided'}), 400
        
        # Validate grasp poses format
        grasp_poses = data.get('grasp_poses', [])
        point_cloud = data.get('point_cloud', [])
        
        if not grasp_poses:
            return jsonify({'error': 'No grasp poses provided'}), 400
        
        # Validate each pose
        for i, pose in enumerate(grasp_poses):
            if not all(key in pose for key in ['dep', 'rot', 'xyz']):
                return jsonify({'error': f'Invalid pose format at index {i}. Required keys: dep, rot, xyz'}), 400
            
            if not isinstance(pose['rot'], list) or len(pose['rot']) != 3:
                return jsonify({'error': f'Invalid rotation matrix at index {i}. Must be 3x3 matrix'}), 400
            
            if not isinstance(pose['xyz'], list) or len(pose['xyz']) != 3:
                return jsonify({'error': f'Invalid position at index {i}. Must be 3D vector'}), 400
        
        # Transform grasp poses using unified transformer
        # 这确保grasp poses和点云使用相同的坐标转换
        transformed_grasp_poses = transformer.transform_grasp_poses_only(grasp_poses)
        
        return jsonify({
            'success': True,
            'grasp_poses': transformed_grasp_poses,
            'point_cloud': point_cloud,
            'message': f'Received {len(grasp_poses)} grasp poses for visualization'
        })
        
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/api/sample_data')
def sample_data():
    """Generate sample data for testing"""
    try:
        # Get camera parameters from query string (optional)
        camera_params = {}
        for param in ['width', 'height', 'fx', 'fy', 'cx', 'cy', 'scale']:
            if param in request.args:
                if param in ['width', 'height']:
                    camera_params[param] = int(request.args[param])
                else:
                    camera_params[param] = float(request.args[param])
        
        # Update camera parameters if provided
        if camera_params:
            for key, value in camera_params.items():
                if hasattr(pc_generator, key):
                    setattr(pc_generator, key, value)
        
        # Generate sample point cloud (cube)
        point_cloud = pc_generator.create_sample_point_cloud('cube', 0.5, 200)
        
        # Generate sample grasp poses with proper rotation matrices
        sample_poses = []
        
        # Camera to Three.js transformation
        transform = np.array([
            [1, 0, 0],
            [0, -1, 0],
            [0, 0, -1]
        ])
        
        for i in range(5):
            # Create proper rotation matrix in camera coordinates
            angles = np.random.uniform(-np.pi/4, np.pi/4, 3)
            from scipy.spatial.transform import Rotation as R
            rotation_matrix = R.from_euler('xyz', angles).as_matrix()
            
            # Position in camera coordinates
            xyz_camera = np.array([
                float(np.random.uniform(-0.3, 0.3)),
                float(np.random.uniform(-0.3, 0.3)),
                float(np.random.uniform(0.5, 1.0))  # Z forward in camera coords
            ])
            
            # Transform to Three.js coordinates
            xyz_threejs = transform @ xyz_camera
            rot_threejs = transform @ rotation_matrix @ transform.T
            
            pose = {
                "dep": float(np.random.uniform(0.01, 0.05)),
                "rot": rot_threejs.tolist(),
                "xyz": xyz_threejs.tolist()
            }
            sample_poses.append(pose)
        
        return jsonify({
            'success': True,
            'point_cloud': point_cloud,
            'grasp_poses': sample_poses,
            'message': f'Sample data generated: {len(point_cloud)} points, {len(sample_poses)} poses'
        })
        
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/api/test_thinkgrasp_compatibility', methods=['GET'])
def test_thinkgrasp_compatibility():
    """Test ThinkGrasp compatibility with current camera parameters"""
    try:
        # Create test data similar to ThinkGrasp
        depth_image = np.full((pc_generator.height, pc_generator.width), 800, dtype=np.uint16)
        
        # Add test objects at different locations
        test_locations = [
            (200, 300, 500),  # (u, v, depth_mm)
            (400, 500, 600),
            (100, 200, 700),
        ]
        
        for u, v, depth_mm in test_locations:
            if v < pc_generator.height and u < pc_generator.width:
                depth_image[v-20:v+20, u-20:u+20] = depth_mm
        
        # Generate point cloud using ThinkGrasp method
        cloud = pc_generator.create_point_cloud_from_depth_image(depth_image, organized=False)
        
        # Create sample grasp poses at test locations (GraspNet format)
        grasp_poses = []
        
        # Camera to Three.js transformation
        transform = np.array([
            [1, 0, 0],
            [0, -1, 0],
            [0, 0, -1]
        ])
        
        for i, (u, v, depth_mm) in enumerate(test_locations):
            depth_m = depth_mm / 1000.0
            x = (u - pc_generator.cx) * depth_m / pc_generator.fx
            y = (v - pc_generator.cy) * depth_m / pc_generator.fy  # Camera: Y=down
            z = depth_m
            
            # Position in camera coordinates
            xyz_camera = np.array([float(x), float(y), float(z)])
            
            # Transform to Three.js coordinates
            xyz_threejs = transform @ xyz_camera
            
            # Identity rotation in camera coords, transformed to Three.js
            rot_camera = np.eye(3)
            rot_threejs = transform @ rot_camera @ transform.T
            
            pose = {
                "dep": 0.02,
                "rot": rot_threejs.tolist(),
                "xyz": xyz_threejs.tolist()
            }
            grasp_poses.append(pose)
        
        # Convert point cloud to our format
        point_cloud_data = []
        valid_points = cloud[~np.isnan(cloud).any(axis=1)]  # Remove NaN points
        valid_points = valid_points[valid_points[:, 2] > 0]  # Remove zero depth
        
        # Sample points for performance
        if len(valid_points) > 10000:
            indices = np.random.choice(len(valid_points), 10000, replace=False)
            valid_points = valid_points[indices]
        
        for point in valid_points:
            # Transform coordinates from GraspNet system to Three.js system
            transformed_point = [point[0], -point[1], -point[2]]
            point_cloud_data.append({
                'position': transformed_point,
                'color': [0.5, 0.7, 0.9]
            })
        
        return jsonify({
            'success': True,
            'point_cloud': point_cloud_data,
            'grasp_poses': grasp_poses,
            'camera_params': {
                'width': pc_generator.width,
                'height': pc_generator.height,
                'fx': pc_generator.fx,
                'fy': pc_generator.fy,
                'cx': pc_generator.cx,
                'cy': pc_generator.cy,
                'scale': pc_generator.depth_scale
            },
            'message': f'ThinkGrasp compatibility test: {len(point_cloud_data)} points, {len(grasp_poses)} poses'
        })
        
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/api/reconstruct_and_visualize', methods=['POST'])
def reconstruct_and_visualize():
    """从深度图、RGB图像和grasp poses重建点云并准备可视化"""
    try:
        # 获取上传的文件
        depth_file = request.files.get('depth_image')
        rgb_file = request.files.get('rgb_image')
        
        if not depth_file or not rgb_file:
            return jsonify({'error': 'Both depth and RGB images are required'}), 400
        
        # 获取grasp poses (JSON)
        grasp_data = request.form.get('grasp_poses')
        if not grasp_data:
            return jsonify({'error': 'Grasp poses are required'}), 400
        
        try:
            grasp_poses = json.loads(grasp_data)
            if isinstance(grasp_poses, dict):
                grasp_poses = grasp_poses.get('grasp_poses', [])
        except json.JSONDecodeError:
            return jsonify({'error': 'Invalid JSON format for grasp poses'}), 400
        
        # 获取相机参数（可选）
        camera_params = {}
        for param in ['width', 'height', 'fx', 'fy', 'cx', 'cy', 'depth_scale']:
            if param in request.form:
                if param in ['width', 'height']:
                    camera_params[param] = int(request.form[param])
                else:
                    camera_params[param] = float(request.form[param])
        
        # 如果提供了相机参数，更新reconstructor
        if camera_params:
            reconstructor.camera_params.update(camera_params)
        
        # 保存上传的文件
        depth_path = UPLOAD_FOLDER / 'temp_depth.png'
        rgb_path = UPLOAD_FOLDER / 'temp_rgb.png'
        
        depth_file.save(depth_path)
        rgb_file.save(rgb_path)
        
        # 读取图像
        depth_image = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
        rgb_image = cv2.imread(str(rgb_path))
        rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
        
        print(f"Depth shape: {depth_image.shape}, RGB shape: {rgb_image.shape}")
        print(f"Number of grasp poses: {len(grasp_poses)}")
        
        # 重建点云
        pcd = reconstructor.depth_to_pointcloud(depth_image, rgb_image)
        points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors) if pcd.has_colors() else np.full((len(points), 3), [0.5, 0.7, 0.9])
        
        print(f"Reconstructed {len(points)} points")
        
        # 应用坐标转换（相机坐标系 -> Three.js坐标系）
        transform = np.array([
            [1, 0, 0],
            [0, -1, 0],
            [0, 0, -1]
        ])
        
        # 转换点云
        transformed_points = (transform @ points.T).T
        
        # 创建前端格式的点云数据
        point_cloud_data = []
        # 限制点数以提高性能
        step = max(1, len(transformed_points) // 50000)
        for i in range(0, len(transformed_points), step):
            color = colors[i] if i < len(colors) else [0.5, 0.7, 0.9]
            point_cloud_data.append({
                'position': transformed_points[i].tolist(),
                'color': color.tolist()
            })
        
        # 转换grasp poses
        transformed_grasp_poses = transformer.transform_grasp_poses_only(grasp_poses)
        
        # 保存重建的文件（可选）
        output_dir = UPLOAD_FOLDER / 'reconstructed'
        output_dir.mkdir(exist_ok=True)
        
        # 保存PLY文件
        o3d.io.write_point_cloud(str(output_dir / 'reconstructed.ply'), pcd)
        
        # 保存grasp array
        grasp_array = reconstructor.convert_grasps_to_array(grasp_poses)
        np.save(str(output_dir / 'grasps.npy'), grasp_array)
        
        return jsonify({
            'success': True,
            'point_cloud': point_cloud_data,
            'grasp_poses': transformed_grasp_poses,
            'message': f'Reconstructed {len(point_cloud_data)} points with {len(transformed_grasp_poses)} grasp poses',
            'stats': {
                'original_points': len(points),
                'downsampled_points': len(point_cloud_data),
                'grasp_count': len(transformed_grasp_poses)
            }
        })
        
    except Exception as e:
        import traceback
        traceback.print_exc()
        return jsonify({'error': str(e)}), 500

@app.route('/api/upload_ply_npy', methods=['POST'])
def upload_ply_npy():
    """Process PLY point cloud and NPY grasp poses"""
    try:
        # Get uploaded files
        ply_data = request.files.get('ply_file')
        npy_data = request.files.get('npy_file')
        
        if not ply_data or not npy_data:
            return jsonify({'error': 'Both PLY and NPY files are required'}), 400
        
        # Save uploaded files
        ply_path = UPLOAD_FOLDER / 'point_cloud.ply'
        npy_path = UPLOAD_FOLDER / 'grasp_poses.npy'
        
        ply_data.save(ply_path)
        npy_data.save(npy_path)
        
        # Use simple transformation
        from simple_transform import process_ply_npy_simple
        
        try:
            # Process with simple transformation
            point_cloud_data, grasp_poses = process_ply_npy_simple(ply_path, npy_path)
            
        except Exception as e:
            return jsonify({'error': f'Failed to process files: {str(e)}'}), 400
        
        return jsonify({
            'success': True,
            'point_cloud': point_cloud_data,
            'grasp_poses': grasp_poses,
            'message': f'Loaded {len(point_cloud_data)} points and {len(grasp_poses)} grasp poses from PLY+NPY files'
        })
        
    except Exception as e:
        return jsonify({'error': str(e)}), 500

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=3001)