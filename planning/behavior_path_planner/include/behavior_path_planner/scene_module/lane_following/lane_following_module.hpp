// Copyright 2021 Tier IV, Inc.
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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_FOLLOWING__LANE_FOLLOWING_MODULE_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_FOLLOWING__LANE_FOLLOWING_MODULE_HPP_

#include "behavior_path_planner/scene_module/scene_module_interface.hpp"

#include <lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

#include <memory>
#include <string>

namespace behavior_path_planner
{
using autoware_auto_planning_msgs::msg::PathWithLaneId;

struct LaneFollowingParameters
{
  bool expand_drivable_area;
  double right_bound_offset;
  double left_bound_offset;
  double lane_change_prepare_duration;
};

class LaneFollowingModule : public SceneModuleInterface
{
public:
  LaneFollowingModule(
    const std::string & name, rclcpp::Node & node, const LaneFollowingParameters & parameters);

  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  BT::NodeStatus updateState() override;
  BehaviorModuleOutput plan() override;
  PathWithLaneId planCandidate() const override;
  void onEntry() override;
  void onExit() override;

  void setParameters(const LaneFollowingParameters & parameters);

private:
  LaneFollowingParameters parameters_;

  PathWithLaneId getReferencePath() const;
  void initParam();
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_FOLLOWING__LANE_FOLLOWING_MODULE_HPP_
