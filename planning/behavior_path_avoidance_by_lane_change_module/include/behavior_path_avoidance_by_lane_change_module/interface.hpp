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

#ifndef BEHAVIOR_PATH_AVOIDANCE_BY_LANE_CHANGE_MODULE__INTERFACE_HPP_
#define BEHAVIOR_PATH_AVOIDANCE_BY_LANE_CHANGE_MODULE__INTERFACE_HPP_

#include "behavior_path_avoidance_by_lane_change_module/data_structs.hpp"
#include "behavior_path_avoidance_by_lane_change_module/scene.hpp"
#include "behavior_path_lane_change_module/interface.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <unordered_map>

namespace behavior_path_planner
{
class AvoidanceByLaneChangeInterface : public LaneChangeInterface
{
public:
  AvoidanceByLaneChangeInterface(
    const std::string & name, rclcpp::Node & node,
    const std::shared_ptr<LaneChangeParameters> & parameters,
    const std::shared_ptr<AvoidanceByLCParameters> & avoidance_by_lane_change_parameters,
    const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
    std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
      objects_of_interest_marker_interface_ptr_map);

  bool isExecutionRequested() const override;

  void processOnEntry() override;

protected:
  void updateRTCStatus(const double start_distance, const double finish_distance) override;
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_AVOIDANCE_BY_LANE_CHANGE_MODULE__INTERFACE_HPP_
