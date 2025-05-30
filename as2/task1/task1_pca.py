import numpy as np
import open3d as o3d
import datetime

def icp_core(point_cloud1, point_cloud2):
    """
    Solve transformation from point_cloud2 to point_cloud1, T1_2
    :param point_cloud1: numpy array, size = n x 3, n is num of point
    :param point_cloud2: numpy array, size = n x 3, n is num of point
    :return: transformation matrix T, size = 4x4
    
    Note: point cloud should be in same size. Point with same index are corresponding points.
          For example, point_cloud1[i] and point_cloud2[i] are a pair of cooresponding points.
    
    """
    assert point_cloud1.shape == point_cloud2.shape, 'point cloud size not match'
    
    T1_2 = np.eye(4)
    # TODO: Finish icp based on SVD, you can refer the lecture slides. Please leave comments and explainations for each step.
    # 步骤1: 计算两个点云的质心
    centroid1 = np.mean(point_cloud1, axis=0)
    centroid2 = np.mean(point_cloud2, axis=0)

    # 步骤2: 通过减去各自的质心，将两个点云中心化
    centered_cloud1 = point_cloud1 - centroid1
    centered_cloud2 = point_cloud2 - centroid2

    # 步骤3: 计算互协方差矩阵H
    H = np.zeros((3, 3))
    for i in range(point_cloud1.shape[0]):
        H += np.outer(centered_cloud2[i], centered_cloud1[i])

    # 步骤4: 对互协方差矩阵进行奇异值分解(SVD)
    U, S, Vt = np.linalg.svd(H)

    # 步骤5: 计算旋转矩阵R
    # 检查是否存在反射情况(行列式为-1)
    det_UV = np.linalg.det(U @ Vt)
    correction = np.eye(3)
    if det_UV < 0:
        correction[2, 2] = -1  # 反射z坐标
    
    R = U @ correction @ Vt
    
    # 步骤6: 计算平移向量t
    t = centroid1 - R @ centroid2
    
    # 步骤7: 构建4x4变换矩阵
    T1_2[:3, :3] = R
    T1_2[:3, 3] = t

    # 步骤8: 返回变换矩阵
    return T1_2


def pca_alignment(source_points, target_points):
    """使用PCA进行点云对齐"""
    # 1. 计算源点云的PCA
    # source_points = np.asarray(source.points)
    source_mean = np.mean(source_points, axis=0)
    source_centered = source_points - source_mean
    source_cov = np.cov(source_centered.T)
    source_eigenvalues, source_eigenvectors = np.linalg.eigh(source_cov)
    
    # 特征值按升序排列，需要反转以获得降序
    source_eigenvalues = source_eigenvalues[::-1]
    source_eigenvectors = source_eigenvectors[:, ::-1]
    
    # 2. 计算目标点云的PCA
    # target_points = np.asarray(target.points)
    target_mean = np.mean(target_points, axis=0)
    target_centered = target_points - target_mean
    target_cov = np.cov(target_centered.T)
    target_eigenvalues, target_eigenvectors = np.linalg.eigh(target_cov)
    
    # 特征值按升序排列，需要反转以获得降序
    target_eigenvalues = target_eigenvalues[::-1]
    target_eigenvectors = target_eigenvectors[:, ::-1]
    
    # 3. 构建旋转矩阵
    R = target_eigenvectors @ source_eigenvectors.T
    
    # 确保R是正交矩阵且行列式为1（避免反射）
    if np.linalg.det(R) < 0:
        # 如果行列式为负，翻转最后一个特征向量
        source_eigenvectors[:, 2] = -source_eigenvectors[:, 2]
        R = target_eigenvectors @ source_eigenvectors.T
    
    # 4. 构建平移向量
    t = target_mean - R @ source_mean
    
    # 5. 构建变换矩阵
    transformation = np.eye(4)
    transformation[:3, :3] = R
    transformation[:3, 3] = t
    
    return transformation

def solve_icp_with_known_correspondence(point_cloud1, point_cloud2):
    # Solve for transformation matrix
    T1_2 = icp_core(point_cloud1, point_cloud2)
    print('------------ transformation matrix T1_2 ------------')
    print(T1_2)

    # TODO: calculate transformed point_cloud2 based on T1_2 solved above
    # 将点云2转换为齐次坐标（添加第4维为1）
    point_cloud2_homogeneous = np.hstack((point_cloud2, np.ones((point_cloud2.shape[0], 1))))
    # 应用变换矩阵
    point_cloud2_transformed_homogeneous = (T1_2 @ point_cloud2_homogeneous.T).T
    # 转换回3D坐标（去掉第4维）
    point_cloud2_transformed = point_cloud2_transformed_homogeneous[:, :3]

    # Visualization
    mean_distance = mean_dist(point_cloud2_transformed, point_cloud1)
    print('mean_error= ' + str(mean_distance))

    axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10, origin=[0, 0, 0])
    
    pcd1 = o3d.geometry.PointCloud()
    pcd1.points = o3d.utility.Vector3dVector(point_cloud1)
    pcd2 = o3d.geometry.PointCloud()
    pcd2.points = o3d.utility.Vector3dVector(point_cloud2)
    pcd2_transformed = o3d.geometry.PointCloud()
    pcd2_transformed.points = o3d.utility.Vector3dVector(point_cloud2_transformed)
    
    pcd1.paint_uniform_color([1, 0, 0])  # Red for reference cloud
    pcd2.paint_uniform_color([0, 1, 0])  # Green for original cloud
    pcd2_transformed.paint_uniform_color([0, 0, 1])  # Blue for transformed cloud
    
    o3d.visualization.draw_geometries([pcd1, pcd2, pcd2_transformed, axis_pcd])


def solve_icp_without_known_correspondence(point_cloud1, point_cloud2, n_iter, threshold):
    point_cloud2_temp = point_cloud2.copy()
    T_1_2accumulated = np.eye(4)

    axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10, origin=[0, 0, 0])
    pcd1 = o3d.geometry.PointCloud()
    pcd1.points = o3d.utility.Vector3dVector(point_cloud1)
    pcd1.paint_uniform_color([0, 0, 1])
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(axis_pcd)
    vis.add_geometry(pcd1)
    
    total_time_cost = 0

    # 在icp之前先进行pca对齐
    # 不变的为source, 为point_cloud1
    T_pca = pca_alignment(source_points=point_cloud2_temp, target_points=point_cloud1)
    # 将当前点云转换为齐次坐标
    point_cloud2_homogeneous = np.hstack((point_cloud2_temp, np.ones((point_cloud2_temp.shape[0], 1))))
    # 应用当前迭代的变换
    point_cloud2_transformed_homogeneous = (T_pca @ point_cloud2_homogeneous.T).T
    # 转换回3D坐标
    point_cloud2_temp = point_cloud2_transformed_homogeneous[:, :3]
    # point_cloud2_temp = (T_pca @ np.hstack((point_cloud2_temp, np.ones((point_cloud2_temp.shape[0], 1))).T).T)[:, :3]
    T_1_2accumulated = T_1_2accumulated @T_pca
    pca_error=mean_dist(point_cloud1, point_cloud2_temp)
    print('pca_error= ' + str(pca_error))

    for i in range(n_iter):
        start_time = datetime.datetime.now()
        
        # TODO: Try to estimate correspondence of points between 2 point clouds, 
        #       and reindex point_cloud2 based on your estimated correspondence
        # 为每个点找到最近邻点
        point_cloud2_reordered = np.zeros_like(point_cloud2_temp)
        for j in range(point_cloud2_temp.shape[0]):
            # 计算当前点与点云1中所有点的距离
            distances = np.linalg.norm(point_cloud1 - point_cloud2_temp[j], axis=1)
            # 找到最近点的索引
            nearest_point_idx = np.argmin(distances)
            # 重排序点云2，使其与点云1对应
            point_cloud2_reordered[j] = point_cloud2_temp[nearest_point_idx]
        # Solve ICP for current iteration
            
        # Solve ICP for current iteration
        T1_2_cur = icp_core(point_cloud1, point_cloud2_reordered)
        
        end_time = datetime.datetime.now()
        time_difference = (end_time - start_time).total_seconds()
        total_time_cost += time_difference
        
        # TODO: Update accumulated transformation
        T_1_2accumulated = T1_2_cur @ T_1_2accumulated
        
        print('-----------------------------------------')
        print('iteration = ' + str(i+1))
        print('time cost = ' + str(time_difference) + 's')
        print('total time cost = ' + str(total_time_cost) + 's')        
        print('T1_2_cur = ')
        print(T1_2_cur)
        print('accumulated T = ')
        print(T_1_2accumulated)
        
        # TODO: Update point cloud2 using transform from current iteration
        # point_cloud2_temp = (T_1_2accumulated @ np.hstack((point_cloud2_temp, np.ones((point_cloud2_temp.shape[0], 1))).T).T)[:, :3]
        # 将当前点云转换为齐次坐标
        point_cloud2_homogeneous = np.hstack((point_cloud2_temp, np.ones((point_cloud2_temp.shape[0], 1))))
        # 应用当前迭代的变换
        point_cloud2_transformed_homogeneous = (T1_2_cur @ point_cloud2_homogeneous.T).T
        # 转换回3D坐标
        point_cloud2_temp = point_cloud2_transformed_homogeneous[:, :3]

        mean_distance = mean_dist(point_cloud1, point_cloud2_temp)
        print('mean_error= ' + str(mean_distance))

        # Update visualization
        pcd2_transed = o3d.geometry.PointCloud()
        pcd2_transed.points = o3d.utility.Vector3dVector(point_cloud2_temp)
        pcd2_transed.paint_uniform_color([1, 0, 0])
        vis.add_geometry(pcd2_transed)
        vis.poll_events()
        vis.update_renderer()
        vis.remove_geometry(pcd2_transed)

        if mean_distance < 0.00001 or mean_distance < threshold:
            print('------- fully converged! -------')
            break
        
        if i == n_iter - 1:
            print('------- reach iteration limit -------')

    print('time cost: ' + str(total_time_cost) + ' s')
    
    vis.destroy_window()
    
    # Final visualization
    pcd2_final = o3d.geometry.PointCloud()
    pcd2_final.points = o3d.utility.Vector3dVector(point_cloud2_temp)
    pcd2_final.paint_uniform_color([1, 0, 0])
    o3d.visualization.draw_geometries([axis_pcd, pcd1, pcd2_final])

def mean_dist(point_cloud1, point_cloud2):
    dis_array = []
    for i in range(point_cloud1.shape[0]):
        dif = point_cloud1[i] - point_cloud2[i]
        dis = np.linalg.norm(dif)
        dis_array.append(dis)
        
    return np.mean(np.array(dis_array))

def main():
    print('start hw program')
    pcd1 = o3d.io.read_point_cloud('bunny1.ply') # change to your file path
    pcd2 = o3d.io.read_point_cloud('bunny2.ply') # change to your file path
    points1 = np.array(pcd1.points)
    points2 = np.array(pcd2.points)

    # uncomment the lines following task 1 or 2 to run the corresponding task
    # task 1:
    # solve_icp_with_known_correspondence(points1, points2)
    # task 2:
    solve_icp_without_known_correspondence(points1, points2, n_iter=30, threshold=0.1)

if __name__ == '__main__':
    main()