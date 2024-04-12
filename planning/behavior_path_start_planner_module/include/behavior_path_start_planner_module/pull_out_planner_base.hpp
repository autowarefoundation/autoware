// Copyright 2022 TIER IV, Inc.
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

#ifndef BEHAVIOR_PATH_START_PLANNER_MODULE__PULL_OUT_PLANNER_BASE_HPP_
#define BEHAVIOR_PATH_START_PLANNER_MODULE__PULL_OUT_PLANNER_BASE_HPP_

#include "behavior_path_planner_common/data_manager.hpp"
#include "behavior_path_planner_common/utils/path_safety_checker/objects_filtering.hpp"
#include "behavior_path_start_planner_module/data_structs.hpp"
#include "behavior_path_start_planner_module/pull_out_path.hpp"
#include "behavior_path_start_planner_module/util.hpp"

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <memory>

namespace behavior_path_planner
{
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Pose;
using tier4_autoware_utils::LinearRing2d;

enum class PlannerType {
  NONE = 0,
  SHIFT = 1,
  GEOMETRIC = 2,
  STOP = 3,
  FREESPACE = 4,
};

class PullOutPlannerBase
{
public:
  explicit PullOutPlannerBase(rclcpp::Node & node, const StartPlannerParameters & parameters)
  {
    vehicle_info_ = vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo();
    vehicle_footprint_ = vehicle_info_.createFootprint();
    parameters_ = parameters;
  }
  virtual ~PullOutPlannerBase() = default;

  void setPlannerData(const std::shared_ptr<const PlannerData> & planner_data)
  {
    planner_data_ = planner_data;
  }

  void setCollisionCheckMargin(const double collision_check_margin)
  {
    collision_check_margin_ = collision_check_margin;
  };
  virtual PlannerType getPlannerType() const = 0;
  virtual std::optional<PullOutPath> plan(const Pose & start_pose, const Pose & goal_pose) = 0;

protected:
  bool isPullOutPathCollided(
    behavior_path_planner::PullOutPath & pull_out_path,
    double collision_check_distance_from_end) const
  {
    // check for collisions
    const auto & dynamic_objects = planner_data_->dynamic_object;
    const auto pull_out_lanes = start_planner_utils::getPullOutLanes(
      planner_data_,
      planner_data_->parameters.backward_path_length + parameters_.max_back_distance);
    const auto & vehicle_footprint = vehicle_info_.createFootprint();
    // extract stop objects in pull out lane for collision check
    const auto stop_objects = utils::path_safety_checker::filterObjectsByVelocity(
      *dynamic_objects, parameters_.th_moving_object_velocity);
    auto [pull_out_lane_stop_objects, others] =
      utils::path_safety_checker::separateObjectsByLanelets(
        stop_objects, pull_out_lanes, utils::path_safety_checker::isPolygonOverlapLanelet);
    utils::path_safety_checker::filterObjectsByClass(
      pull_out_lane_stop_objects, parameters_.object_types_to_check_for_path_generation);

    const auto collision_check_section_path =
      behavior_path_planner::start_planner_utils::extractCollisionCheckSection(
        pull_out_path, collision_check_distance_from_end);
    if (!collision_check_section_path) return true;

    return utils::checkCollisionBetweenPathFootprintsAndObjects(
      vehicle_footprint_, collision_check_section_path.value(), pull_out_lane_stop_objects,
      collision_check_margin_);
  };
  std::shared_ptr<const PlannerData> planner_data_;
  vehicle_info_util::VehicleInfo vehicle_info_;
  LinearRing2d vehicle_footprint_;
  StartPlannerParameters parameters_;
  double collision_check_margin_;
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_START_PLANNER_MODULE__PULL_OUT_PLANNER_BASE_HPP_
