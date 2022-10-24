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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OVER__PULL_OVER_PLANNER_BASE_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OVER__PULL_OVER_PLANNER_BASE_HPP_

#include "behavior_path_planner/data_manager.hpp"
#include "behavior_path_planner/parameters.hpp"
#include "behavior_path_planner/scene_module/pull_over/pull_over_parameters.hpp"
#include "behavior_path_planner/util/create_vehicle_footprint.hpp"

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <boost/optional.hpp>

#include <memory>
#include <vector>

using autoware_auto_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Pose;
using tier4_autoware_utils::LinearRing2d;

namespace behavior_path_planner
{
enum class PullOverPlannerType {
  NONE = 0,
  SHIFT,
  ARC_FORWARD,
  ARC_BACKWARD,
};

struct PullOverPath
{
  PathWithLaneId path{};
  std::vector<PathWithLaneId> partial_paths{};
  Pose start_pose{};
  Pose end_pose{};
};

class PullOverPlannerBase
{
public:
  PullOverPlannerBase(rclcpp::Node & node, const PullOverParameters & parameters)
  {
    vehicle_info_ = vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo();
    vehicle_footprint_ = createVehicleFootprint(vehicle_info_);
    parameters_ = parameters;
  }
  virtual ~PullOverPlannerBase() = default;

  void setPlannerData(const std::shared_ptr<const PlannerData> planner_data)
  {
    planner_data_ = planner_data;
  }

  virtual PullOverPlannerType getPlannerType() const = 0;
  virtual boost::optional<PullOverPath> plan(const Pose & goal_pose) = 0;

protected:
  std::shared_ptr<const PlannerData> planner_data_;
  vehicle_info_util::VehicleInfo vehicle_info_;
  LinearRing2d vehicle_footprint_;
  PullOverParameters parameters_;
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OVER__PULL_OVER_PLANNER_BASE_HPP_
