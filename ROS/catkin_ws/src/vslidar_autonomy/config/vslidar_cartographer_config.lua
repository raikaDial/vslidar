-- Google Cartographer configuration for the VS-LIDAR system.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
	map_builder = MAP_BUILDER,
	trajectory_builder = TRAJECTORY_BUILDER,
	map_frame = "map",
	tracking_frame = "imu_link",
	published_frame = "odom",
	odom_frame = "odom",
	provide_odom_frame = false,
	publish_frame_projected_to_2d = false,
	use_odometry = true,
	use_nav_sat = false,
	use_landmarks = false,
	num_laser_scans = 0,
	num_multi_echo_laser_scans = 0,
	num_subdivisions_per_laser_scan = 1,
	num_point_clouds = 1,
	lookup_transform_timeout_sec = 0.2,
	submap_publish_period_sec = 0.3,
	pose_publish_period_sec = 5e-3,
	trajectory_publish_period_sec = 30e-3,
	rangefinder_sampling_ratio = 1.0,
	odometry_sampling_ratio = 1.0,
	fixed_frame_pose_sampling_ratio = 1.0,
	imu_sampling_ratio = 1.0,
	landmarks_sampling_ratio = 1.0
}

MAP_BUILDER.use_trajectory_builder_3d = true

MAX_3D_LASER_RANGE = 6.0
TRAJECTORY_BUILDER_3D.min_range = 0.1
TRAJECTORY_BUILDER_3D.max_range = MAX_3D_LASER_RANGE
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_range = MAX_3D_LASER_RANGE
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points = 500
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_range = MAX_3D_LASER_RANGE
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 250
TRAJECTORY_BUILDER_3D.submaps.high_resolution_max_range = MAX_3D_LASER_RANGE

-- TRAJECTORY_BUILDER_3D.motion_filter.max_time_seconds = 0.25
-- TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = math.rad(0.1)
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.035
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.2

TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_0 = 10.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_1 = 15.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 4.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 1e3
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.only_optimize_yaw = true

POSE_GRAPH.constraint_builder.sampling_ratio = 0.2
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
POSE_GRAPH.constraint_builder.min_score = 0.48
POSE_GRAPH.constraint_builder.log_matches = true
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.min_rotational_score = 0.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 2.

POSE_GRAPH.optimization_problem.huber_scale = 1e1
POSE_GRAPH.optimization_problem.acceleration_weight = 1e-1
POSE_GRAPH.optimization_problem.rotation_weight = 1e3

return options
