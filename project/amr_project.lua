include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",

  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = true,
  
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,  -- 启用点云数据

  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 0.5,
  fixed_frame_pose_sampling_ratio = 0.5,
  imu_sampling_ratio = 0.5,
  landmarks_sampling_ratio = 0.5,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- 1. 激光测距范围设置：设置激光数据的最小和最大测距范围
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 50.0
TRAJECTORY_BUILDER_2D.min_z = 0.2

-- TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.02

-- 2. 缺失数据时默认射线长度：用于处理无返回数据的情况
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.0

-- 3. IMU数据设置：如果有IMU数据，启用IMU数据融合
TRAJECTORY_BUILDER_2D.use_imu_data = true

-- 运动滤波更灵敏
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.)
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.3

-- -- Scan Matcher（激光匹配器）参数调优，匹配精度
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 12
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.2
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(10.0) 

POSE_GRAPH.optimize_every_n_nodes = 80 --2倍的num_range_data以上
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.max_constraint_distance = 25
POSE_GRAPH.constraint_builder.min_score = 2.
POSE_GRAPH.constraint_builder.global_localization_min_score = 2.



return options