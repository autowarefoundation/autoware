// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "static_centerline_generator/centerline_source/optimization_trajectory_based_centerline.hpp"

#include "motion_utils/resample/resample.hpp"
#include "motion_utils/trajectory/conversion.hpp"
#include "obstacle_avoidance_planner/node.hpp"
#include "path_smoother/elastic_band_smoother.hpp"
#include "static_centerline_generator/static_centerline_generator_node.hpp"
#include "static_centerline_generator/utils.hpp"
#include "tier4_autoware_utils/ros/parameter.hpp"

namespace static_centerline_generator
{
namespace
{
rclcpp::NodeOptions create_node_options()
{
  return rclcpp::NodeOptions{};
}

Path convert_to_path(const PathWithLaneId & path_with_lane_id)
{
  Path path;
  path.header = path_with_lane_id.header;
  path.left_bound = path_with_lane_id.left_bound;
  path.right_bound = path_with_lane_id.right_bound;
  for (const auto & point : path_with_lane_id.points) {
    path.points.push_back(point.point);
  }

  return path;
}

Trajectory convert_to_trajectory(const Path & path)
{
  Trajectory traj;
  for (const auto & point : path.points) {
    TrajectoryPoint traj_point;
    traj_point.pose = point.pose;
    traj_point.longitudinal_velocity_mps = point.longitudinal_velocity_mps;
    traj_point.lateral_velocity_mps = point.lateral_velocity_mps;
    traj_point.heading_rate_rps = point.heading_rate_rps;

    traj.points.push_back(traj_point);
  }
  return traj;
}
}  // namespace

OptimizationTrajectoryBasedCenterline::OptimizationTrajectoryBasedCenterline(rclcpp::Node & node)
{
  pub_raw_path_with_lane_id_ =
    node.create_publisher<PathWithLaneId>("input_centerline", utils::create_transient_local_qos());
  pub_raw_path_ =
    node.create_publisher<Path>("debug/raw_centerline", utils::create_transient_local_qos());
}

std::vector<TrajectoryPoint>
OptimizationTrajectoryBasedCenterline::generate_centerline_with_optimization(
  rclcpp::Node & node, const RouteHandler & route_handler,
  const std::vector<lanelet::Id> & route_lane_ids)
{
  const auto route_lanelets = utils::get_lanelets_from_ids(route_handler, route_lane_ids);

  // optimize centerline inside the lane
  const auto start_center_pose = utils::get_center_pose(route_handler, route_lane_ids.front());

  // get ego nearest search parameters and resample interval in behavior_path_planner
  const double ego_nearest_dist_threshold =
    tier4_autoware_utils::getOrDeclareParameter<double>(node, "ego_nearest_dist_threshold");
  const double ego_nearest_yaw_threshold =
    tier4_autoware_utils::getOrDeclareParameter<double>(node, "ego_nearest_yaw_threshold");
  const double behavior_path_interval =
    tier4_autoware_utils::getOrDeclareParameter<double>(node, "output_path_interval");
  const double behavior_vel_interval =
    tier4_autoware_utils::getOrDeclareParameter<double>(node, "behavior_output_path_interval");

  // extract path with lane id from lanelets
  const auto raw_path_with_lane_id = [&]() {
    const auto non_resampled_path_with_lane_id = utils::get_path_with_lane_id(
      route_handler, route_lanelets, start_center_pose, ego_nearest_dist_threshold,
      ego_nearest_yaw_threshold);
    return motion_utils::resamplePath(non_resampled_path_with_lane_id, behavior_path_interval);
  }();
  pub_raw_path_with_lane_id_->publish(raw_path_with_lane_id);
  RCLCPP_INFO(node.get_logger(), "Calculated raw path with lane id and published.");

  // convert path with lane id to path
  const auto raw_path = [&]() {
    const auto non_resampled_path = convert_to_path(raw_path_with_lane_id);
    return motion_utils::resamplePath(non_resampled_path, behavior_vel_interval);
  }();
  pub_raw_path_->publish(raw_path);
  RCLCPP_INFO(node.get_logger(), "Converted to path and published.");

  // smooth trajectory and road collision avoidance
  const auto optimized_traj_points = optimize_trajectory(raw_path);
  RCLCPP_INFO(
    node.get_logger(),
    "Smoothed trajectory and made it collision free with the road and published.");

  return optimized_traj_points;
}

std::vector<TrajectoryPoint> OptimizationTrajectoryBasedCenterline::optimize_trajectory(
  const Path & raw_path) const
{
  // convert to trajectory points
  const auto raw_traj_points = [&]() {
    const auto raw_traj = convert_to_trajectory(raw_path);
    return motion_utils::convertToTrajectoryPointArray(raw_traj);
  }();

  // create an instance of elastic band and model predictive trajectory.
  const auto eb_path_smoother_ptr =
    path_smoother::ElasticBandSmoother(create_node_options()).getElasticBandSmoother();
  const auto mpt_optimizer_ptr =
    obstacle_avoidance_planner::ObstacleAvoidancePlanner(create_node_options()).getMPTOptimizer();

  // NOTE: The optimization is executed every valid_optimized_traj_points_num points.
  constexpr int valid_optimized_traj_points_num = 10;
  const int traj_segment_num = raw_traj_points.size() / valid_optimized_traj_points_num;

  // NOTE: num_initial_optimization exists to make the both optimizations stable since they may use
  // warm start.
  constexpr int num_initial_optimization = 2;

  std::vector<TrajectoryPoint> whole_optimized_traj_points;
  for (int virtual_ego_pose_idx = -num_initial_optimization;
       virtual_ego_pose_idx < traj_segment_num; ++virtual_ego_pose_idx) {
    // calculate virtual ego pose for the optimization
    constexpr int virtual_ego_pose_offset_idx = 1;
    const auto virtual_ego_pose =
      raw_traj_points
        .at(
          valid_optimized_traj_points_num * std::max(virtual_ego_pose_idx, 0) +
          virtual_ego_pose_offset_idx)
        .pose;

    // smooth trajectory by elastic band in the path_smoother package
    const auto smoothed_traj_points =
      eb_path_smoother_ptr->smoothTrajectory(raw_traj_points, virtual_ego_pose);

    // road collision avoidance by model predictive trajectory in the obstacle_avoidance_planner
    // package
    const obstacle_avoidance_planner::PlannerData planner_data{
      raw_path.header, smoothed_traj_points, raw_path.left_bound, raw_path.right_bound,
      virtual_ego_pose};
    const auto optimized_traj_points = mpt_optimizer_ptr->optimizeTrajectory(planner_data);

    // connect the previously and currently optimized trajectory points
    for (size_t j = 0; j < whole_optimized_traj_points.size(); ++j) {
      const double dist = tier4_autoware_utils::calcDistance2d(
        whole_optimized_traj_points.at(j), optimized_traj_points.front());
      if (dist < 0.5) {
        const std::vector<TrajectoryPoint> extracted_whole_optimized_traj_points{
          whole_optimized_traj_points.begin(),
          whole_optimized_traj_points.begin() + std::max(j, 1UL) - 1};
        whole_optimized_traj_points = extracted_whole_optimized_traj_points;
        break;
      }
    }
    for (size_t j = 0; j < optimized_traj_points.size(); ++j) {
      whole_optimized_traj_points.push_back(optimized_traj_points.at(j));
    }
  }

  return whole_optimized_traj_points;
}
}  // namespace static_centerline_generator
