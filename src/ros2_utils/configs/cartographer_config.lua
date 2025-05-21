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

-- Main options table
options = {
  map_builder = MAP_BUILDER,  -- Maps are built using the MAP_BUILDER configuration (defined in map_builder.lua).
  trajectory_builder = TRAJECTORY_BUILDER,  -- Trajectories are built using the TRAJECTORY_BUILDER configuration (defined in trajectory_builder.lua).
  
  -- Coordinate frames for the system
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_nav_sat = false,
  use_landmarks = false,
  publish_tracked_pose = true,
  
  -- Enable odometry
  use_odometry = true,  -- Enable the use of odometry for SLAM.

  -- LIDAR settings (Hokuyo)
  num_laser_scans = 1,  -- Number of laser scans (we are using 1 scan from Hokuyo LIDAR).

  -- Other sensor settings
  use_nav_sat = false,  -- Disable GPS (set to true for outdoor navigation if needed).
  use_landmarks = false,  -- Disable the use of landmarks.

  -- Configuration for processing the scan data
  num_multi_echo_laser_scans = 0,  -- Number of multi-echo LIDAR scans.
  num_subdivisions_per_laser_scan = 1,  -- Number of subdivisions for each laser scan.
  num_point_clouds = 0,  -- Number of point clouds (we are using laser scans, not point clouds).

  -- Timeout and period settings
  lookup_transform_timeout_sec = 0.2,  -- Timeout for lookup transformations.
  submap_publish_period_sec = 0.3,  -- Period for publishing submaps.
  pose_publish_period_sec = 5e-3,  -- Period for publishing poses.
  trajectory_publish_period_sec = 30e-3,  -- Period for publishing trajectory data.
  
  -- Sampling ratio settings (for downsampling the sensor data)
  rangefinder_sampling_ratio = 1.,  -- Sampling ratio for the rangefinder (no down-sampling).
  odometry_sampling_ratio = 1.,  -- Sampling ratio for odometry data.
  fixed_frame_pose_sampling_ratio = 1.,  -- Sampling ratio for fixed-frame pose.
  imu_sampling_ratio = 1.,  -- Sampling ratio for IMU data.
  landmarks_sampling_ratio = 1.,  -- Sampling ratio for landmarks.
}

TRAJECTORY_BUILDER.pure_localization_trimmer = {
    max_submaps_to_keep = 3,
}

-- Pose Graph Configuration
POSE_GRAPH.constraint_builder.sampling_ratio = 0.1  -- Controls how often constraints are built between trajectories.
POSE_GRAPH.global_sampling_ratio = .0  -- Controls how often global constraints are sampled (set to 0 for no global constraints).
--POSE_GRAPH.optimize_every_n_nodes = 0  -- If needed, uncomment and set a value to optimize the graph periodically.
POSE_GRAPH.optimize_every_n_nodes = 10

-- Map Builder Configuration
MAP_BUILDER.num_background_threads = 10  -- Number of background threads for map building.
MAP_BUILDER.use_trajectory_builder_2d = true  -- Use the 2D trajectory builder for 2D SLAM.
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 10  -- Number of range data points to accumulate before processing.
TRAJECTORY_BUILDER_2D.use_imu_data = false  -- Disable IMU data for 2D SLAM.


return options
