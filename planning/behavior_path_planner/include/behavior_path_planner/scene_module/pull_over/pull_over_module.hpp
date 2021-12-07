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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OVER__PULL_OVER_MODULE_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OVER__PULL_OVER_MODULE_HPP_

#include "behavior_path_planner/path_shifter/path_shifter.hpp"
#include "behavior_path_planner/scene_module/pull_over/pull_over_path.hpp"
#include "behavior_path_planner/scene_module/scene_module_interface.hpp"
#include "behavior_path_planner/utilities.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>

#include <tf2/utils.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_path_planner
{
using autoware_auto_vehicle_msgs::msg::HazardLightsCommand;
struct PullOverParameters
{
  double min_stop_distance;
  double stop_time;
  double hysteresis_buffer_distance;
  double pull_over_prepare_duration;
  double pull_over_duration;
  double pull_over_finish_judge_buffer;
  double minimum_pull_over_velocity;
  double prediction_duration;
  double prediction_time_resolution;
  double static_obstacle_velocity_thresh;
  double maximum_deceleration;
  int pull_over_sampling_num;
  bool enable_collision_check_at_prepare_phase;
  bool use_predicted_path_outside_lanelet;
  bool use_all_predicted_path;
  bool enable_blocked_by_obstacle;
  double pull_over_search_distance;
  double after_pull_over_straight_distance;
  double before_pull_over_straight_distance;
  double margin_from_boundary;
  double maximum_lateral_jerk;
  double minimum_lateral_jerk;
  double deceleration_interval;
  double hazard_on_threshold_dis;
  double hazard_on_threshold_vel;
};

struct PullOverStatus
{
  PathWithLaneId lane_follow_path;
  PullOverPath pull_over_path;
  lanelet::ConstLanelets current_lanes;
  lanelet::ConstLanelets pull_over_lanes;
  std::vector<uint64_t> lane_follow_lane_ids;
  std::vector<uint64_t> pull_over_lane_ids;
  bool is_safe;
  double start_distance;
};

class PullOverModule : public SceneModuleInterface
{
public:
  PullOverModule(
    const std::string & name, rclcpp::Node & node, const PullOverParameters & parameters);

  BehaviorModuleOutput run() override;

  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  BT::NodeStatus updateState() override;
  BehaviorModuleOutput plan() override;
  BehaviorModuleOutput planWaitingApproval() override;
  PathWithLaneId planCandidate() const override;
  void onEntry() override;
  void onExit() override;

  void setParameters(const PullOverParameters & parameters);

private:
  PullOverParameters parameters_;
  PullOverStatus status_;

  double pull_over_lane_length_ = 200.0;
  double check_distance_ = 100.0;

  PathWithLaneId getReferencePath() const;
  lanelet::ConstLanelets getCurrentLanes() const;
  lanelet::ConstLanelets getPullOverLanes(const lanelet::ConstLanelets & current_lanes) const;
  std::pair<bool, bool> getSafePath(
    const lanelet::ConstLanelets & pull_over_lanes, const double check_distance,
    PullOverPath & safe_path) const;
  TurnSignalInfo getTurnSignalAndDistance(const PathWithLaneId & path) const;

  // turn signal
  std::pair<HazardLightsCommand, double> getHazard(
    const lanelet::ConstLanelets & target_lanes, const Pose & current_pose, const Pose & goal_pose,
    const double & velocity, const double & hazard_on_threshold_dis,
    const double & hazard_on_threshold_vel, const double & base_link2front) const;

  void updatePullOverStatus();
  bool isInLane(
    const lanelet::ConstLanelet & candidate_lanelet,
    const autoware_utils::LinearRing2d & vehicle_footprint) const;
  bool isLongEnough(const lanelet::ConstLanelets & lanelets) const;
  bool isSafe() const;
  bool isLaneBlocked(const lanelet::ConstLanelets & lanes) const;
  bool isNearEndOfLane() const;
  bool isCurrentSpeedLow() const;
  bool hasFinishedPullOver() const;
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OVER__PULL_OVER_MODULE_HPP_
