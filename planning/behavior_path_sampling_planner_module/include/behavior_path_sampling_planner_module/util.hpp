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

#ifndef BEHAVIOR_PATH_SAMPLING_PLANNER_MODULE__UTIL_HPP_
#define BEHAVIOR_PATH_SAMPLING_PLANNER_MODULE__UTIL_HPP_
#include "sampler_common/structures.hpp"
#include "sampler_common/transform/spline_transform.hpp"

#include <tier4_autoware_utils/geometry/boost_geometry.hpp>

#include <boost/geometry/index/rtree.hpp>

#include <any>
#include <functional>
#include <numeric>
#include <optional>
#include <vector>
namespace behavior_path_planner
{

using geometry_msgs::msg::Pose;

struct SoftConstraintsInputs
{
  Pose goal_pose;
  Pose ego_pose;
  lanelet::ArcCoordinates ego_arc;
  lanelet::ArcCoordinates goal_arc;
  lanelet::ConstLanelets closest_lanelets_to_goal;
  behavior_path_planner::PlanResult reference_path;
  behavior_path_planner::PlanResult prev_module_path;
  std::optional<sampler_common::Path> prev_path;
  lanelet::ConstLanelets current_lanes;
};

using SoftConstraintsFunctionVector = std::vector<std::function<double(
  sampler_common::Path &, const sampler_common::Constraints &, const SoftConstraintsInputs &)>>;

using HardConstraintsFunctionVector = std::vector<std::function<bool(
  sampler_common::Path &, const sampler_common::Constraints &, const MultiPoint2d &)>>;

inline std::vector<double> evaluateSoftConstraints(
  sampler_common::Path & path, const sampler_common::Constraints & constraints,
  const SoftConstraintsFunctionVector & soft_constraints_functions,
  const SoftConstraintsInputs & input_data)
{
  std::vector<double> constraints_results;
  for (const auto & f : soft_constraints_functions) {
    const auto cost = f(path, constraints, input_data);
    constraints_results.push_back(cost);
  }
  if (constraints.soft.weights.size() != constraints_results.size()) {
    path.cost = std::accumulate(constraints_results.begin(), constraints_results.end(), 0.0);
    return constraints_results;
  }

  path.cost = std::inner_product(
    constraints_results.begin(), constraints_results.end(), constraints.soft.weights.begin(), 0.0);
  return constraints_results;
}

inline std::vector<bool> evaluateHardConstraints(
  sampler_common::Path & path, const sampler_common::Constraints & constraints,
  const MultiPoint2d & footprint, const HardConstraintsFunctionVector & hard_constraints)
{
  std::vector<bool> constraints_results;
  bool constraints_passed = true;
  for (const auto & f : hard_constraints) {
    const bool constraint_check = f(path, constraints, footprint);
    constraints_passed &= constraint_check;
    constraints_results.push_back(constraint_check);
  }

  path.constraints_satisfied = constraints_passed;
  return constraints_results;
}

inline sampler_common::State getInitialState(
  const geometry_msgs::msg::Pose & pose,
  const sampler_common::transform::Spline2D & reference_spline)
{
  sampler_common::State initial_state;
  Point2d initial_state_pose{pose.position.x, pose.position.y};
  const auto rpy = tier4_autoware_utils::getRPY(pose.orientation);
  initial_state.pose = initial_state_pose;
  initial_state.frenet = reference_spline.frenet({pose.position.x, pose.position.y});
  initial_state.heading = rpy.z;
  return initial_state;
}

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_SAMPLING_PLANNER_MODULE__UTIL_HPP_
