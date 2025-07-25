"""
Unified transformation for point cloud and grasp poses
确保grasp poses相对于点云的位置关系保持不变
"""

import numpy as np
import open3d as o3d

class UnifiedTransformer:
    """统一的坐标转换器，确保点云和grasp poses的相对关系保持不变"""
    
    def __init__(self):
        # 从相机坐标系到Three.js坐标系的转换
        # Camera: X=right, Y=down, Z=forward
        # Three.js: X=right, Y=up, Z=backward
        self.transform = np.array([
            [1, 0, 0],
            [0, -1, 0],
            [0, 0, -1]
        ])
    
    def transform_point_cloud_and_grasps(self, ply_path, npy_path):
        """
        同时转换点云和grasp poses，保持它们的相对关系
        
        Returns:
            point_cloud_data: 转换后的点云数据
            grasp_poses: 转换后的grasp poses
        """
        # 1. 加载原始数据（相机坐标系）
        pcd = o3d.io.read_point_cloud(str(ply_path))
        points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors) if pcd.has_colors() else np.full((len(points), 3), [0.5, 0.7, 0.9])
        
        grasp_array = np.load(str(npy_path))
        
        print(f"Original point cloud bounds (camera coords):")
        print(f"  X: [{points[:, 0].min():.3f}, {points[:, 0].max():.3f}]")
        print(f"  Y: [{points[:, 1].min():.3f}, {points[:, 1].max():.3f}]")
        print(f"  Z: [{points[:, 2].min():.3f}, {points[:, 2].max():.3f}]")
        
        # 2. 转换点云到Three.js坐标系
        transformed_points = (self.transform @ points.T).T
        
        # 3. 创建点云数据格式
        point_cloud_data = []
        for i, point in enumerate(transformed_points):
            color = colors[i] if i < len(colors) else [0.5, 0.7, 0.9]
            point_cloud_data.append({
                'position': point.tolist(),
                'color': color.tolist()
            })
        
        # 4. 转换grasp poses
        grasp_poses = []
        for i, grasp in enumerate(grasp_array):
            if len(grasp) >= 17:
                # 提取grasp参数
                score = float(grasp[0])
                width = float(grasp[1])
                depth = float(grasp[3])
                rot_matrix = grasp[4:13].reshape(3, 3)
                translation = grasp[13:16]
                
                # 检查grasp是否在点云范围内（原始坐标）
                in_bounds = (
                    points[:, 0].min() <= translation[0] <= points[:, 0].max() and
                    points[:, 1].min() <= translation[1] <= points[:, 1].max() and
                    points[:, 2].min() <= translation[2] <= points[:, 2].max()
                )
                
                if i < 5:  # 打印前5个grasp的信息
                    print(f"\nGrasp {i+1}:")
                    print(f"  Original position: {translation}")
                    print(f"  Within bounds: {in_bounds}")
                
                # 应用相同的转换
                trans_threejs = self.transform @ translation
                rot_threejs = self.transform @ rot_matrix @ self.transform.T
                
                if i < 5:
                    print(f"  Transformed position: {trans_threejs}")
                
                pose = {
                    "dep": depth,
                    "width": width,
                    "score": score,
                    "rot": rot_threejs.tolist(),
                    "xyz": trans_threejs.tolist()
                }
                grasp_poses.append(pose)
        
        print(f"\nTransformed {len(point_cloud_data)} points and {len(grasp_poses)} grasps")
        print("Both using the same coordinate transformation")
        
        return point_cloud_data, grasp_poses
    
    def transform_grasp_poses_only(self, grasp_poses_json):
        """
        仅转换grasp poses（用于JSON输入）
        假设grasp poses是在相机坐标系中定义的
        """
        transformed_poses = []
        
        for pose in grasp_poses_json:
            # 转换位置
            xyz = np.array(pose['xyz'])
            xyz_transformed = self.transform @ xyz
            
            # 转换旋转矩阵
            rot = np.array(pose['rot'])
            rot_transformed = self.transform @ rot @ self.transform.T
            
            transformed_pose = {
                "dep": pose.get('dep', 0.02),
                "width": pose.get('width', 0.08),
                "score": pose.get('score', 0.5),
                "rot": rot_transformed.tolist(),
                "xyz": xyz_transformed.tolist()
            }
            transformed_poses.append(transformed_pose)
        
        return transformed_poses
    
    def verify_alignment(self, points, grasps):
        """验证转换后的grasp poses是否仍然在点云附近"""
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd_tree = o3d.geometry.KDTreeFlann(pcd)
        
        distances = []
        for grasp in grasps[:10]:  # 检查前10个grasps
            translation = grasp['xyz']
            [k, idx, _] = pcd_tree.search_knn_vector_3d(translation, 1)
            if k > 0:
                closest_point = points[idx[0]]
                distance = np.linalg.norm(np.array(translation) - closest_point)
                distances.append(distance)
        
        if distances:
            print(f"\nAlignment verification:")
            print(f"Average distance to closest point: {np.mean(distances):.3f}m")
            print(f"Max distance: {np.max(distances):.3f}m")
            
            if np.mean(distances) > 0.1:
                print("WARNING: Grasps seem far from point cloud!")
            else:
                print("✓ Grasps are properly aligned with point cloud")
        
        return distances