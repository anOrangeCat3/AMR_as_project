import numpy as np
import open3d as o3d

def show_source_point_cloud():
    pcd1 = o3d.io.read_point_cloud('bunny1.ply') # change to your file path
    pcd2 = o3d.io.read_point_cloud('bunny2.ply') # change to your file path
    point_cloud1 = np.array(pcd1.points)
    point_cloud2 = np.array(pcd2.points)

    axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10, origin=[0, 0, 0])
    
    pcd1 = o3d.geometry.PointCloud()
    pcd1.points = o3d.utility.Vector3dVector(point_cloud1)
    pcd2 = o3d.geometry.PointCloud()
    pcd2.points = o3d.utility.Vector3dVector(point_cloud2)

    pcd1.paint_uniform_color([1, 0, 0])  # Red for reference cloud
    pcd2.paint_uniform_color([0, 0, 1])  # Blue for original cloud

    o3d.visualization.draw_geometries([pcd1, pcd2, axis_pcd])

def show_pca_result():
    pcd1 = o3d.io.read_point_cloud('bunny1.ply') # change to your file path
    pcd2 = o3d.io.read_point_cloud('bunny2.ply') # change to your file path
    
    source_points = np.array(pcd2.points) # source point cloud
    # compute the mean of the source point cloud
    source_mean = np.mean(source_points, axis=0)
    source_centered = source_points - source_mean
    source_cov = np.cov(source_centered.T)
    source_eigenvalues, source_eigenvectors = np.linalg.eigh(source_cov)

    target_points = np.array(pcd1.points) # target point cloud
    # compute the mean of the target point cloud
    target_mean = np.mean(target_points, axis=0)
    target_centered = target_points - target_mean
    target_cov = np.cov(target_centered.T)
    target_eigenvalues, target_eigenvectors = np.linalg.eigh(target_cov)
    # print(source_eigenvectors)
    # print(target_eigenvectors)

    # Construct Rotation Matrix
    R = target_eigenvectors @ source_eigenvectors.T
    
    # Construct Translation Vector
    t = target_mean - R @ source_mean
    
    # Construct Transformation Matrix
    transformation = np.eye(4)
    transformation[:3, :3] = R
    transformation[:3, 3] = t
    
    source_points_homogeneous = np.hstack((source_points, np.ones((source_points.shape[0], 1))))
    # 应用变换矩阵
    source_points_transformed_homogeneous = (transformation @ source_points_homogeneous.T).T
    # 转换回3D坐标（去掉第4维）
    source_points_transformed = source_points_transformed_homogeneous[:, :3]
    
    # Visualization
    axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10, origin=[0, 0, 0])
    
    pcd1 = o3d.geometry.PointCloud()
    pcd1.points = o3d.utility.Vector3dVector(source_points)
    pcd2 = o3d.geometry.PointCloud()
    pcd2.points = o3d.utility.Vector3dVector(target_points)
    pcd2_transformed = o3d.geometry.PointCloud()
    pcd2_transformed.points = o3d.utility.Vector3dVector(source_points_transformed)

    pcd1.paint_uniform_color([1, 0, 0])  # Red for reference cloud
    pcd2.paint_uniform_color([0, 1, 0])  # Green for original cloud
    pcd2_transformed.paint_uniform_color([0, 0, 1])  # Blue for transformed cloud
    
    pcd1.paint_uniform_color([1, 0, 0])  # Red for reference cloud
    pcd2.paint_uniform_color([0, 1, 0])  # Green for original cloud
    pcd2_transformed.paint_uniform_color([0, 0, 1])  # Blue for transformed cloud

    o3d.visualization.draw_geometries([pcd1, pcd2, pcd2_transformed, axis_pcd])
    

if __name__ == '__main__':
    show_source_point_cloud()
    # show_pca_result()