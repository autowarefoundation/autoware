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

#ifndef BEHAVIOR_PATH_AVOIDANCE_BY_LANE_CHANGE_MODULE__SCENE_HPP_
#define BEHAVIOR_PATH_AVOIDANCE_BY_LANE_CHANGE_MODULE__SCENE_HPP_

#include "behavior_path_avoidance_by_lane_change_module/data_structs.hpp"
#include "behavior_path_avoidance_module/helper.hpp"
#include "behavior_path_lane_change_module/scene.hpp"

#include <memory>

namespace behavior_path_planner
{
using AvoidanceDebugData = DebugData;
using helper::avoidance::AvoidanceHelper;

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

  ObjectData createObjectData(
    const AvoidancePlanningData & data, const PredictedObject & object) const;

  void fillAvoidanceTargetObjects(AvoidancePlanningData & data, AvoidanceDebugData & debug) const;
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_AVOIDANCE_BY_LANE_CHANGE_MODULE__SCENE_HPP_
