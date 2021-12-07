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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__LANE_CHANGE_MODULE_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__LANE_CHANGE_MODULE_HPP_

#include "behavior_path_planner/scene_module/lane_change/lane_change_path.hpp"
#include "behavior_path_planner/scene_module/scene_module_interface.hpp"
#include "behavior_path_planner/utilities.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>

#include <tf2/utils.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_path_planner
{
using autoware_auto_planning_msgs::msg::PathWithLaneId;

struct LaneChangeParameters
{
  double min_stop_distance;
  double stop_time;
  double hysteresis_buffer_distance;
  double lane_change_prepare_duration;
  double lane_changing_duration;
  double lane_change_finish_judge_buffer;
  double minimum_lane_change_velocity;
  double prediction_duration;
  double prediction_time_resolution;
  double static_obstacle_velocity_thresh;
  double maximum_deceleration;
  int lane_change_sampling_num;
  double abort_lane_change_velocity_thresh;
  double abort_lane_change_angle_thresh;
  double abort_lane_change_distance_thresh;
  bool enable_abort_lane_change;
  bool enable_collision_check_at_prepare_phase;
  bool use_predicted_path_outside_lanelet;
  bool use_all_predicted_path;
  bool enable_blocked_by_obstacle;
  double lane_change_search_distance;
};

struct LaneChangeStatus
{
  PathWithLaneId lane_follow_path;
  LaneChangePath lane_change_path;
  lanelet::ConstLanelets current_lanes;
  lanelet::ConstLanelets lane_change_lanes;
  std::vector<uint64_t> lane_follow_lane_ids;
  std::vector<uint64_t> lane_change_lane_ids;
  bool is_safe;
  double start_distance;
};

class LaneChangeModule : public SceneModuleInterface
{
public:
  LaneChangeModule(
    const std::string & name, rclcpp::Node & node, const LaneChangeParameters & parameters);

  BehaviorModuleOutput run() override;

  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  BT::NodeStatus updateState() override;
  BehaviorModuleOutput plan() override;
  BehaviorModuleOutput planWaitingApproval() override;
  PathWithLaneId planCandidate() const override;
  void onEntry() override;
  void onExit() override;

  void setParameters(const LaneChangeParameters & parameters);

private:
  LaneChangeParameters parameters_;
  LaneChangeStatus status_;
  PathShifter path_shifter_;

  double lane_change_lane_length_{200.0};
  double check_distance_{100.0};

  PathWithLaneId getReferencePath() const;
  lanelet::ConstLanelets getCurrentLanes() const;
  lanelet::ConstLanelets getLaneChangeLanes(
    const lanelet::ConstLanelets & current_lanes, const double lane_change_lane_length) const;
  std::pair<bool, bool> getSafePath(
    const lanelet::ConstLanelets & lane_change_lanes, const double check_distance,
    LaneChangePath & safe_path) const;

  void updateLaneChangeStatus();

  bool isSafe() const;
  bool isLaneBlocked(const lanelet::ConstLanelets & lanes) const;
  bool isNearEndOfLane() const;
  bool isCurrentSpeedLow() const;
  bool isAbortConditionSatisfied() const;
  bool hasFinishedLaneChange() const;
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__LANE_CHANGE_MODULE_HPP_
