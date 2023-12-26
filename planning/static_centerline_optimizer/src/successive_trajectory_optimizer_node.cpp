// Copyright 2022 Tier IV, Inc.
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

#include "static_centerline_optimizer/successive_trajectory_optimizer_node.hpp"

#include "motion_utils/resample/resample.hpp"
#include "motion_utils/trajectory/conversion.hpp"
#include "motion_utils/trajectory/trajectory.hpp"
#include "obstacle_avoidance_planner/utils/trajectory_utils.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace static_centerline_optimizer
{
SuccessiveTrajectoryOptimizer::SuccessiveTrajectoryOptimizer(
  const rclcpp::NodeOptions & node_options)
: ObstacleAvoidancePlanner(node_options)
{
  // subscriber
  centerline_sub_ = create_subscription<Path>(
    "debug/raw_centerline", rclcpp::QoS{1}.transient_local(),
    std::bind(&SuccessiveTrajectoryOptimizer::on_centerline, this, std::placeholders::_1));

  // update parameters for replan_checker to execute optimization every cycle
  std::vector<rclcpp::Parameter> parameters;
  parameters.push_back(rclcpp::Parameter("replan.max_path_shape_around_ego_lat_dist", 100.0));
  parameters.push_back(rclcpp::Parameter("replan.max_ego_moving_dist", 100.0));
  parameters.push_back(rclcpp::Parameter("replan.max_goal_moving_dist", 100.0));
  parameters.push_back(rclcpp::Parameter("replan.max_delta_time_sec", 0.0));
  onParam(parameters);
}

Trajectory SuccessiveTrajectoryOptimizer::on_centerline(const Path & path)
{
  if (path.points.size() < 2) {
    RCLCPP_WARN(get_logger(), "Input path size is less than 2.");
    return Trajectory{};
  }

  // parameters for input path sampling
  const double resample_interval = 2.0;
  const double valid_optimized_path_length = 30.0;
  const double path_length = motion_utils::calcArcLength(path.points);
  const size_t path_segment_num = static_cast<size_t>(path_length / valid_optimized_path_length);

  const auto resampled_path = motion_utils::resamplePath(path, resample_interval);  // TODO(murooka)
  const auto resampled_traj_points =
    obstacle_avoidance_planner::trajectory_utils::convertToTrajectoryPoints(resampled_path.points);

  const size_t initial_index = 3;
  std::vector<TrajectoryPoint> whole_optimized_traj_points;

  for (size_t i = 0; i < path_segment_num; ++i) {
    // calculate initial pose to start optimization
    const auto initial_pose =
      resampled_path.points.at(initial_index + valid_optimized_path_length / resample_interval * i)
        .pose;

    // create planner data
    obstacle_avoidance_planner::PlannerData planner_data;
    planner_data.traj_points = resampled_traj_points;
    planner_data.left_bound = path.left_bound;
    planner_data.right_bound = path.right_bound;
    planner_data.ego_pose = initial_pose;

    const auto optimized_traj_points = optimizeTrajectory(planner_data);

    for (size_t j = 0; j < whole_optimized_traj_points.size(); ++j) {
      const double dist = tier4_autoware_utils::calcDistance2d(
        whole_optimized_traj_points.at(j), optimized_traj_points.front());
      if (dist < 0.5) {
        const std::vector<TrajectoryPoint> extracted_whole_optimized_traj_points{
          whole_optimized_traj_points.begin(), whole_optimized_traj_points.begin() + j - 1};
        whole_optimized_traj_points = extracted_whole_optimized_traj_points;
      }
    }

    for (size_t j = 0; j < optimized_traj_points.size(); ++j) {
      whole_optimized_traj_points.push_back(optimized_traj_points.at(j));
    }
  }

  // resample
  auto output_traj_msg = motion_utils::resampleTrajectory(
    motion_utils::convertToTrajectory(whole_optimized_traj_points), 1.0);
  output_traj_msg.header = path.header;

  return output_traj_msg;
}
}  // namespace static_centerline_optimizer

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(static_centerline_optimizer::SuccessiveTrajectoryOptimizer)
