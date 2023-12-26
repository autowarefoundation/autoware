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

#ifndef BEHAVIOR_PATH_LANE_CHANGE_MODULE__INTERFACE_HPP_
#define BEHAVIOR_PATH_LANE_CHANGE_MODULE__INTERFACE_HPP_

#include "behavior_path_lane_change_module/scene.hpp"
#include "behavior_path_lane_change_module/utils/base_class.hpp"
#include "behavior_path_lane_change_module/utils/data_structs.hpp"
#include "behavior_path_lane_change_module/utils/path.hpp"
#include "behavior_path_planner_common/interface/scene_module_interface.hpp"
#include "behavior_path_planner_common/turn_signal_decider.hpp"
#include "behavior_path_planner_common/utils/path_shifter/path_shifter.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <tf2/utils.h>

#include <memory>
#include <string>
#include <unordered_map>

namespace behavior_path_planner
{
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using objects_of_interest_marker_interface::ColorName;
using objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface;

class LaneChangeInterface : public SceneModuleInterface
{
public:
  LaneChangeInterface(
    const std::string & name, rclcpp::Node & node, std::shared_ptr<LaneChangeParameters> parameters,
    const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
    std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
      objects_of_interest_marker_interface_ptr_map,
    std::unique_ptr<LaneChangeBase> && module_type);

  LaneChangeInterface(const LaneChangeInterface &) = delete;
  LaneChangeInterface(LaneChangeInterface &&) = delete;
  LaneChangeInterface & operator=(const LaneChangeInterface &) = delete;
  LaneChangeInterface & operator=(LaneChangeInterface &&) = delete;
  ~LaneChangeInterface() override = default;

  void processOnEntry() override;

  void processOnExit() override;

  bool isExecutionRequested() const override;

  bool isExecutionReady() const override;

  bool isRootLaneletToBeUpdated() const override
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

  TurnSignalInfo getCurrentTurnSignalInfo(
    const PathWithLaneId & path, const TurnSignalInfo & original_turn_signal_info);

  // TODO(someone): remove this, and use base class function
  [[deprecated]] BehaviorModuleOutput run() override
  {
    updateData();

    if (!isWaitingApproval()) {
      return plan();
    }

    // module is waiting approval. Check it.
    if (isActivated()) {
      RCLCPP_DEBUG(getLogger(), "Was waiting approval, and now approved. Do plan().");
      return plan();
    } else {
      RCLCPP_DEBUG(getLogger(), "keep waiting approval... Do planCandidate().");
      return planWaitingApproval();
    }
  }

protected:
  std::shared_ptr<LaneChangeParameters> parameters_;

  std::unique_ptr<LaneChangeBase> module_type_;

  PathSafetyStatus post_process_safety_status_;

  bool canTransitSuccessState() override;

  bool canTransitFailureState() override;

  bool canTransitIdleToRunningState() override;

  void setObjectDebugVisualization() const;

  void updateSteeringFactorPtr(const BehaviorModuleOutput & output);

  void updateSteeringFactorPtr(
    const CandidateOutput & output, const LaneChangePath & selected_path) const;

  mutable MarkerArray virtual_wall_marker_;

  std::unique_ptr<PathWithLaneId> prev_approved_path_;

  void clearAbortApproval() { is_abort_path_approved_ = false; }

  bool is_abort_path_approved_{false};

  bool is_abort_approval_requested_{false};
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_LANE_CHANGE_MODULE__INTERFACE_HPP_
