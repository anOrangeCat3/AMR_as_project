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
    
    # Step 1: Calculate centroids of both point clouds
    centroid1 = np.mean(point_cloud1, axis=0)
    centroid2 = np.mean(point_cloud2, axis=0)

    # Step 2: Center the point clouds by subtracting their respective centroids
    centered_cloud1 = point_cloud1 - centroid1
    centered_cloud2 = point_cloud2 - centroid2

    # Step 3: Compute cross-covariance matrix H
    H = np.zeros((3, 3))
    for i in range(point_cloud1.shape[0]):
        H += np.outer(centered_cloud2[i], centered_cloud1[i])

    # Step 4: Perform Singular Value Decomposition (SVD) on the cross-covariance matrix
    U, S, Vt = np.linalg.svd(H)

    # Step 5: Calculate rotation matrix R
    # Check for reflection case (determinant = -1)
    det_UV = np.linalg.det(U @ Vt)
    correction = np.eye(3)
    if det_UV < 0:
        correction[2, 2] = -1  # reflect z coordinate
    
    R = U @ correction @ Vt
    
    # Step 6: Calculate translation vector t
    t = centroid1 - R @ centroid2
    
    # Step 7: Build 4x4 transformation matrix
    T1_2[:3, :3] = R
    T1_2[:3, 3] = t

    # Step 8: Return transformation matrix
    return T1_2

def solve_icp_with_known_correspondence(point_cloud1, point_cloud2):
    # Solve for transformation matrix
    T1_2 = icp_core(point_cloud1, point_cloud2)
    print('------------ transformation matrix T1_2 ------------')
    print(T1_2)

    # TODO: calculate transformed point_cloud2 based on T1_2 solved above
    # Transform point cloud 2 to homogeneous coordinates (add 4th dimension as 1)
    point_cloud2_homogeneous = np.hstack((point_cloud2, np.ones((point_cloud2.shape[0], 1))))
    # Apply transformation matrix
    point_cloud2_transformed_homogeneous = (T1_2 @ point_cloud2_homogeneous.T).T
    # Convert back to 3D coordinates (remove 4th dimension)
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

    # viz
    axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10, origin=[0, 0, 0])
    pcd1 = o3d.geometry.PointCloud()
    pcd1.points = o3d.utility.Vector3dVector(point_cloud1)
    pcd1.paint_uniform_color([0, 0, 1])
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(axis_pcd)
    vis.add_geometry(pcd1)
    
    total_time_cost = 0

    for i in range(n_iter):
        start_time = datetime.datetime.now()
        
        # TODO: Try to estimate correspondence of points between 2 point clouds, 
        #       and reindex point_cloud2 based on your estimated correspondence
        # Find nearest neighbor for each point
        point_cloud2_reordered = np.zeros_like(point_cloud2_temp)
        for j in range(point_cloud2_temp.shape[0]):
            # Calculate distance between current point and all points in point cloud 1
            distances = np.linalg.norm(point_cloud1 - point_cloud2_temp[j], axis=1)
            # Find index of nearest point
            nearest_point_idx = np.argmin(distances)
            # Rearrange point cloud 2 to match point cloud 1
            point_cloud2_reordered[j] = point_cloud2_temp[nearest_point_idx]
        # Solve ICP for current iteration
        T1_2_cur = icp_core(point_cloud1, point_cloud2_reordered)
        
        end_time = datetime.datetime.now()
        time_difference = (end_time - start_time).total_seconds()
        total_time_cost += time_difference
        
        # TODO: Update accumulated transformation
        # Update accumulated transformation matrix: new transformation on the right
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
        # Transform point cloud 2 to homogeneous coordinates (add 4th dimension as 1)
        point_cloud2_homogeneous = np.hstack((point_cloud2_temp, np.ones((point_cloud2_temp.shape[0], 1))))
        # Apply current iteration's transformation
        point_cloud2_transformed_homogeneous = (T1_2_cur @ point_cloud2_homogeneous.T).T
        # Convert back to 3D coordinates
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