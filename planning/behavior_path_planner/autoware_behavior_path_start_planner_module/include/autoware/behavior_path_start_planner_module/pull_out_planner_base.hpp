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

#ifndef AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__PULL_OUT_PLANNER_BASE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__PULL_OUT_PLANNER_BASE_HPP_

#include "autoware/behavior_path_planner_common/data_manager.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/objects_filtering.hpp"
#include "autoware/behavior_path_start_planner_module/data_structs.hpp"
#include "autoware/behavior_path_start_planner_module/pull_out_path.hpp"
#include "autoware/behavior_path_start_planner_module/util.hpp"
#include "autoware/universe_utils/system/time_keeper.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{
using autoware::universe_utils::LinearRing2d;
using geometry_msgs::msg::Pose;
using tier4_planning_msgs::msg::PathWithLaneId;

class PullOutPlannerBase
{
public:
  explicit PullOutPlannerBase(
    rclcpp::Node & node, const StartPlannerParameters & parameters,
    std::shared_ptr<universe_utils::TimeKeeper> time_keeper)
  : time_keeper_(time_keeper)
  {
    vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo();
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
  virtual std::optional<PullOutPath> plan(
    const Pose & start_pose, const Pose & goal_pose, PlannerDebugData & planner_debug_data) = 0;

protected:
  bool isPullOutPathCollided(
    autoware::behavior_path_planner::PullOutPath & pull_out_path,
    double collision_check_distance_from_end) const;

  std::shared_ptr<const PlannerData> planner_data_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
  LinearRing2d vehicle_footprint_;
  StartPlannerParameters parameters_;
  double collision_check_margin_;

  mutable std::shared_ptr<universe_utils::TimeKeeper> time_keeper_;
};
}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__PULL_OUT_PLANNER_BASE_HPP_
