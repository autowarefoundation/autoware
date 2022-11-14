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

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

#include <memory>
#include <string>

namespace behavior_path_planner
{
using autoware_auto_planning_msgs::msg::PathWithLaneId;

struct LaneFollowingParameters
{
  double lane_change_prepare_duration;
  // drivable area expansion
  double drivable_area_right_bound_offset;
  double drivable_area_left_bound_offset;
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
  CandidateOutput planCandidate() const override;
  void onEntry() override;
  void onExit() override;

  void setParameters(const LaneFollowingParameters & parameters);
  void acceptVisitor(
    [[maybe_unused]] const std::shared_ptr<SceneModuleVisitor> & visitor) const override
  {
  }

private:
  LaneFollowingParameters parameters_;

  PathWithLaneId getReferencePath() const;
  void initParam();
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_FOLLOWING__LANE_FOLLOWING_MODULE_HPP_
