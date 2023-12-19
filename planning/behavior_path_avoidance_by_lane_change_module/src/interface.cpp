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

#include "behavior_path_avoidance_by_lane_change_module/interface.hpp"

#include "behavior_path_planner_common/interface/scene_module_interface.hpp"
#include "behavior_path_planner_common/interface/scene_module_visitor.hpp"
#include "behavior_path_planner_common/marker_utils/utils.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_path_planner
{
AvoidanceByLaneChangeInterface::AvoidanceByLaneChangeInterface(
  const std::string & name, rclcpp::Node & node,
  const std::shared_ptr<LaneChangeParameters> & parameters,
  const std::shared_ptr<AvoidanceByLCParameters> & avoidance_by_lane_change_parameters,
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
    objects_of_interest_marker_interface_ptr_map)
: LaneChangeInterface{
    name,
    node,
    parameters,
    rtc_interface_ptr_map,
    objects_of_interest_marker_interface_ptr_map,
    std::make_unique<AvoidanceByLaneChange>(parameters, avoidance_by_lane_change_parameters)}
{
}

bool AvoidanceByLaneChangeInterface::isExecutionRequested() const
{
  return module_type_->isLaneChangeRequired() && module_type_->specialRequiredCheck();
}
void AvoidanceByLaneChangeInterface::processOnEntry()
{
  waitApproval();
}

void AvoidanceByLaneChangeInterface::updateRTCStatus(
  const double start_distance, const double finish_distance)
{
  const auto direction = std::invoke([&]() -> std::string {
    const auto dir = module_type_->getDirection();
    return (dir == Direction::LEFT) ? "left" : "right";
  });

  rtc_interface_ptr_map_.at(direction)->updateCooperateStatus(
    uuid_map_.at(direction), isExecutionReady(), start_distance, finish_distance, clock_->now());
}
}  // namespace behavior_path_planner
