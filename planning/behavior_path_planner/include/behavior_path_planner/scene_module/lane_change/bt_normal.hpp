// Copyright 2023 TIER IV, Inc.
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
#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__BT_NORMAL_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__BT_NORMAL_HPP_

#include "behavior_path_planner/scene_module/lane_change/normal.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_path_planner
{
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using marker_utils::CollisionCheckDebugMap;
using route_handler::Direction;
using tier4_planning_msgs::msg::LaneChangeDebugMsg;
using tier4_planning_msgs::msg::LaneChangeDebugMsgArray;

class NormalLaneChangeBT : public NormalLaneChange
{
public:
  NormalLaneChangeBT(
    const std::shared_ptr<LaneChangeParameters> & parameters, LaneChangeModuleType type,
    Direction direction);

  ~NormalLaneChangeBT() override = default;

  PathWithLaneId getReferencePath() const override;

protected:
  lanelet::ConstLanelets getCurrentLanes() const override;

  int getNumToPreferredLane(const lanelet::ConstLanelet & lane) const override;

  PathWithLaneId getPrepareSegment(
    const lanelet::ConstLanelets & current_lanes, const double arc_length_from_current,
    const double backward_path_length, const double prepare_length,
    const double prepare_velocity) const override;

  std::vector<DrivableLanes> getDrivableLanes() const override;
};
}  // namespace behavior_path_planner
#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__BT_NORMAL_HPP_
