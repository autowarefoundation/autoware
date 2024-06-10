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

#ifndef SCENE_HPP_
#define SCENE_HPP_

#include "autoware_behavior_path_lane_change_module/scene.hpp"
#include "autoware_behavior_path_static_obstacle_avoidance_module/helper.hpp"
#include "data_structs.hpp"

#include <memory>

namespace autoware::behavior_path_planner
{
using ::behavior_path_planner::DebugData;
using AvoidanceDebugData = DebugData;
using ::behavior_path_planner::AvoidancePlanningData;
using ::behavior_path_planner::LaneChangeParameters;
using ::behavior_path_planner::NormalLaneChange;
using ::behavior_path_planner::ObjectData;
using ::behavior_path_planner::ObjectDataArray;
using ::behavior_path_planner::PredictedObject;
using ::behavior_path_planner::helper::static_obstacle_avoidance::AvoidanceHelper;

class AvoidanceByLaneChange : public NormalLaneChange
{
public:
  AvoidanceByLaneChange(
    const std::shared_ptr<LaneChangeParameters> & parameters,
    std::shared_ptr<AvoidanceByLCParameters> avoidance_by_lane_change_parameters);

  bool specialRequiredCheck() const override;

  bool specialExpiredCheck() const override;

  void updateSpecialData() override;

private:
  std::shared_ptr<AvoidanceByLCParameters> avoidance_parameters_;

  AvoidancePlanningData calcAvoidancePlanningData(AvoidanceDebugData & debug) const;
  AvoidancePlanningData avoidance_data_;
  mutable AvoidanceDebugData avoidance_debug_data_;

  ObjectDataArray registered_objects_;
  mutable ObjectDataArray stopped_objects_;
  std::shared_ptr<AvoidanceHelper> avoidance_helper_;

  std::optional<ObjectData> createObjectData(
    const AvoidancePlanningData & data, const PredictedObject & object) const;

  void fillAvoidanceTargetObjects(AvoidancePlanningData & data, AvoidanceDebugData & debug) const;

  double calcMinAvoidanceLength(const ObjectData & nearest_object) const;
  double calcMinimumLaneChangeLength() const;
  double calcLateralOffset() const;
};
}  // namespace autoware::behavior_path_planner

#endif  // SCENE_HPP_
