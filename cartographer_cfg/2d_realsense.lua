-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "camera_depth_frame",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1., 
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

--[[
-- ** TURTLEBOT ** 
MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.use_imu_data = false -- default true

TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 8.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7
]]

-- ** revo lds ** 
MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.5
TRAJECTORY_BUILDER_2D.max_range = 3.5
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
--TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
--TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
--TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1
--TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1) -- default math.rad(1.0) (turtlebot3)

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60 --submap size, default 90
--TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.55 -- default 0.55
--TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.49 -- default 0.49
--TRAJECTORY_BUILDER_2D.submaps.grid_type =  "PROBABILITY_GRID" --default "PROBABILITY_GRID",
--TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 100 -- default 200
--TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.015 -- deafult 0.025
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 2e2 --default 10.   (jackal)
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 4e2 -- default 40.   (jackal)

--POSE_GRAPH.optimization_problem.huber_scale = 1e2 -- default 1e1 (jackal)
POSE_GRAPH.optimize_every_n_nodes = 50 -- default 90
--POSE_GRAPH.constraint_builder.min_score = 0.25 -- default 0.55 (turtlebot3 0.65)
--POSE_GRAPH.constraint_builder.global_localization_min_score = 0.3 -- default 0.6 (turtlebot3 0.7)

--POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5 --default 1e5
--POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5 --default 1e5
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e1 --default 1e5 
--POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5 --default 1e5
--POSE_GRAPH.global_sampling_ratio = 0.01 -- default 0.003
--POSE_GRAPH.global_constraint_search_after_n_seconds = 2. -- default 10.


return options

