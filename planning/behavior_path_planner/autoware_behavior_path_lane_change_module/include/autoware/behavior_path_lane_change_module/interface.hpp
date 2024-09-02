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

#ifndef AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__INTERFACE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__INTERFACE_HPP_

#include "autoware/behavior_path_lane_change_module/scene.hpp"
#include "autoware/behavior_path_lane_change_module/utils/base_class.hpp"
#include "autoware/behavior_path_lane_change_module/utils/data_structs.hpp"
#include "autoware/behavior_path_lane_change_module/utils/path.hpp"
#include "autoware/behavior_path_planner_common/interface/scene_module_interface.hpp"
#include "autoware/behavior_path_planner_common/turn_signal_decider.hpp"
#include "autoware/behavior_path_planner_common/utils/path_shifter/path_shifter.hpp"

#include <autoware/universe_utils/system/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <tf2/utils.h>

#include <memory>
#include <string>
#include <unordered_map>

namespace autoware::behavior_path_planner
{
using autoware::objects_of_interest_marker_interface::ColorName;
using autoware::objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using tier4_planning_msgs::msg::PathWithLaneId;

class LaneChangeInterface : public SceneModuleInterface
{
public:
  LaneChangeInterface(
    const std::string & name, rclcpp::Node & node, std::shared_ptr<LaneChangeParameters> parameters,
    const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
    std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
      objects_of_interest_marker_interface_ptr_map,
    std::shared_ptr<SteeringFactorInterface> & steering_factor_interface_ptr,
    std::unique_ptr<LaneChangeBase> && module_type);

  LaneChangeInterface(const LaneChangeInterface &) = delete;
  LaneChangeInterface(LaneChangeInterface &&) = delete;
  LaneChangeInterface & operator=(const LaneChangeInterface &) = delete;
  LaneChangeInterface & operator=(LaneChangeInterface &&) = delete;
  ~LaneChangeInterface() override = default;

  void processOnExit() override;

  bool isExecutionRequested() const override;

  bool isExecutionReady() const override;

  bool isCurrentRouteLaneletToBeReset() const override
  {
    return getCurrentStatus() == ModuleStatus::SUCCESS;
  }

  void updateData() override;

  void postProcess() override;

  BehaviorModuleOutput plan() override;

  BehaviorModuleOutput planWaitingApproval() override;

  CandidateOutput planCandidate() const override;

  void acceptVisitor(
    [[maybe_unused]] const std::shared_ptr<SceneModuleVisitor> & visitor) const override
  {
  }

  void updateModuleParams(const std::any & parameters) override;

  void setData(const std::shared_ptr<const PlannerData> & data) override;

  MarkerArray getModuleVirtualWall() override;

protected:
  std::shared_ptr<LaneChangeParameters> parameters_;

  std::unique_ptr<LaneChangeBase> module_type_;

  PathSafetyStatus post_process_safety_status_;

  bool canTransitSuccessState() override;

  bool canTransitFailureState() override;

  ModuleStatus setInitState() const override { return ModuleStatus::WAITING_APPROVAL; };

  void updateRTCStatus(
    const double start_distance, const double finish_distance, const bool safe,
    const uint8_t & state)
  {
    universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);
    for (const auto & [module_name, ptr] : rtc_interface_ptr_map_) {
      if (ptr) {
        ptr->updateCooperateStatus(
          uuid_map_.at(module_name), safe, state, start_distance, finish_distance, clock_->now());
      }
    }
  }

  void updateDebugMarker() const;

  void updateSteeringFactorPtr(const BehaviorModuleOutput & output);

  void updateSteeringFactorPtr(
    const CandidateOutput & output, const LaneChangePath & selected_path) const;

  mutable MarkerArray virtual_wall_marker_;

  std::unique_ptr<PathWithLaneId> prev_approved_path_;

  void clearAbortApproval() { is_abort_path_approved_ = false; }

  bool is_abort_path_approved_{false};

  bool is_abort_approval_requested_{false};
};
}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__INTERFACE_HPP_
