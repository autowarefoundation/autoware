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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__SCENE_MODULE_VISITOR_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__SCENE_MODULE_VISITOR_HPP_

#include "tier4_planning_msgs/msg/avoidance_debug_msg.hpp"
#include "tier4_planning_msgs/msg/avoidance_debug_msg_array.hpp"
#include "tier4_planning_msgs/msg/detail/avoidance_debug_msg_array__struct.hpp"
#include "tier4_planning_msgs/msg/detail/lane_change_debug_msg_array__struct.hpp"
#include "tier4_planning_msgs/msg/lane_change_debug_msg.hpp"
#include "tier4_planning_msgs/msg/lane_change_debug_msg_array.hpp"

#include <memory>
namespace behavior_path_planner
{
// Forward Declaration
class AvoidanceModule;
class LaneChangeModule;
class LaneFollowingModule;
class PullOutModule;
class PullOverModule;
class SideShiftModule;

using tier4_planning_msgs::msg::AvoidanceDebugMsg;
using tier4_planning_msgs::msg::AvoidanceDebugMsgArray;
using tier4_planning_msgs::msg::LaneChangeDebugMsg;
using tier4_planning_msgs::msg::LaneChangeDebugMsgArray;

class SceneModuleVisitor
{
public:
  void visitLaneChangeModule(const LaneChangeModule * module) const;
  void visitAvoidanceModule(const AvoidanceModule * module) const;

  std::shared_ptr<AvoidanceDebugMsgArray> getAvoidanceModuleDebugMsg() const;
  std::shared_ptr<LaneChangeDebugMsgArray> getLaneChangeModuleDebugMsg() const;

protected:
  mutable std::shared_ptr<LaneChangeDebugMsgArray> lane_change_visitor_;
  mutable std::shared_ptr<AvoidanceDebugMsgArray> avoidance_visitor_;
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__SCENE_MODULE_VISITOR_HPP_
