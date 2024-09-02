// Copyright 2021 TIER IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_SIDE_SHIFT_MODULE__SCENE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_SIDE_SHIFT_MODULE__SCENE_HPP_

#include "autoware/behavior_path_planner_common/interface/scene_module_interface.hpp"
#include "autoware/behavior_path_planner_common/utils/path_shifter/path_shifter.hpp"
#include "autoware/behavior_path_side_shift_module/data_structs.hpp"

#include <rclcpp/rclcpp.hpp>

#include <tier4_planning_msgs/msg/lateral_offset.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{
using geometry_msgs::msg::Pose;
using nav_msgs::msg::OccupancyGrid;
using tier4_planning_msgs::msg::LateralOffset;
using tier4_planning_msgs::msg::PathWithLaneId;

class SideShiftModule : public SceneModuleInterface
{
public:
  SideShiftModule(
    const std::string & name, rclcpp::Node & node,
    const std::shared_ptr<SideShiftParameters> & parameters,
    const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
    std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
      objects_of_interest_marker_interface_ptr_map,
    std::shared_ptr<SteeringFactorInterface> & steering_factor_interface_ptr);

  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  bool isReadyForNextRequest(
    const double & min_request_time_sec, bool override_requests = false) const noexcept;
  void updateData() override;
  BehaviorModuleOutput plan() override;
  BehaviorModuleOutput planWaitingApproval() override;
  CandidateOutput planCandidate() const override;
  void processOnEntry() override;
  void processOnExit() override;

  void setParameters(const std::shared_ptr<SideShiftParameters> & parameters);

  void updateModuleParams(const std::any & parameters) override
  {
    parameters_ = std::any_cast<std::shared_ptr<SideShiftParameters>>(parameters);
  }

  void acceptVisitor(
    [[maybe_unused]] const std::shared_ptr<SceneModuleVisitor> & visitor) const override
  {
  }

private:
  bool canTransitSuccessState() override;

  bool canTransitFailureState() override { return false; }

  void initVariables();

  // non-const methods
  BehaviorModuleOutput adjustDrivableArea(const ShiftedPath & path) const;

  ShiftLine calcShiftLine() const;

  void replaceShiftLine();

  // const methods
  void publishPath(const PathWithLaneId & path) const;

  double getClosestShiftLength() const;

  // member
  PathWithLaneId refined_path_{};
  PathWithLaneId reference_path_{};
  lanelet::ConstLanelets current_lanelets_;
  std::shared_ptr<SideShiftParameters> parameters_;

  // Requested lateral offset to shift the reference path.
  double requested_lateral_offset_{0.0};

  // Inserted lateral offset to shift the reference path.
  double inserted_lateral_offset_{0.0};

  // Inserted shift lines in the path
  ShiftLine inserted_shift_line_;

  // Shift status
  SideShiftStatus shift_status_;

  // Flag to check lateral offset change is requested
  bool lateral_offset_change_request_{false};

  // Triggered when offset is changed, released when start pose is refound.
  bool start_pose_reset_request_{false};

  PathShifter path_shifter_;

  ShiftedPath prev_output_;
  ShiftLine prev_shift_line_;

  PathWithLaneId extendBackwardLength(const PathWithLaneId & original_path) const;

  mutable rclcpp::Time last_requested_shift_change_time_{clock_->now()};

  rclcpp::Time latest_lateral_offset_stamp_;

  // debug
  mutable SideShiftDebugData debug_data_;
  void setDebugMarkersVisualization() const;
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_SIDE_SHIFT_MODULE__SCENE_HPP_
