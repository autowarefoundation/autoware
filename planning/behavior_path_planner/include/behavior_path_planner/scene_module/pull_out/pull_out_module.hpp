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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OUT__PULL_OUT_MODULE_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OUT__PULL_OUT_MODULE_HPP_

#include "behavior_path_planner/scene_module/pull_out/geometric_pull_out.hpp"
#include "behavior_path_planner/scene_module/pull_out/pull_out_parameters.hpp"
#include "behavior_path_planner/scene_module/pull_out/pull_out_path.hpp"
#include "behavior_path_planner/scene_module/pull_out/shift_pull_out.hpp"
#include "behavior_path_planner/scene_module/scene_module_interface.hpp"
#include "behavior_path_planner/scene_module/utils/geometric_parallel_parking.hpp"
#include "behavior_path_planner/scene_module/utils/path_shifter.hpp"

#include <lane_departure_checker/lane_departure_checker.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <vehicle_info_util/vehicle_info.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

#include <tf2/utils.h>

#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_path_planner
{
using geometry_msgs::msg::PoseArray;
using lane_departure_checker::LaneDepartureChecker;

struct PullOutStatus
{
  PullOutPath pull_out_path;
  size_t current_path_idx = 0;
  PlannerType planner_type = PlannerType::NONE;
  PathWithLaneId backward_path;
  lanelet::ConstLanelets current_lanes;
  lanelet::ConstLanelets pull_out_lanes;
  std::vector<DrivableLanes> lanes;
  std::vector<uint64_t> lane_follow_lane_ids;
  std::vector<uint64_t> pull_out_lane_ids;
  bool is_safe = false;
  bool back_finished = false;
  Pose pull_out_start_pose;
};

class PullOutModule : public SceneModuleInterface
{
public:
  PullOutModule(
    const std::string & name, rclcpp::Node & node, const PullOutParameters & parameters);

  BehaviorModuleOutput run() override;

  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  BT::NodeStatus updateState() override;
  BehaviorModuleOutput plan() override;
  BehaviorModuleOutput planWaitingApproval() override;
  CandidateOutput planCandidate() const override;
  void onEntry() override;
  void onExit() override;

  void setParameters(const PullOutParameters & parameters) { parameters_ = parameters; }
  void resetStatus();

  void acceptVisitor(
    [[maybe_unused]] const std::shared_ptr<SceneModuleVisitor> & visitor) const override
  {
  }

private:
  PullOutParameters parameters_;
  vehicle_info_util::VehicleInfo vehicle_info_;

  std::vector<std::shared_ptr<PullOutPlannerBase>> pull_out_planners_;
  PullOutStatus status_;

  std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> odometry_buffer_;

  std::unique_ptr<rclcpp::Time> last_route_received_time_;
  std::unique_ptr<rclcpp::Time> last_pull_out_start_update_time_;
  std::unique_ptr<Pose> last_approved_pose_;

  std::shared_ptr<PullOutPlannerBase> getCurrentPlanner() const;
  PathWithLaneId getFullPath() const;
  ParallelParkingParameters getGeometricPullOutParameters() const;
  std::vector<Pose> searchBackedPoses();

  std::shared_ptr<LaneDepartureChecker> lane_departure_checker_;

  // turn signal
  TurnSignalInfo calcTurnSignalInfo() const;

  void incrementPathIndex();
  PathWithLaneId getCurrentPath() const;
  void planWithPriorityOnEfficientPath(
    const std::vector<Pose> & start_pose_candidates, const Pose & goal_pose);
  void planWithPriorityOnShortBackDistance(
    const std::vector<Pose> & start_pose_candidates, const Pose & goal_pose);
  void updatePullOutStatus();
  static bool isOverlappedWithLane(
    const lanelet::ConstLanelet & candidate_lanelet,
    const tier4_autoware_utils::LinearRing2d & vehicle_footprint);
  bool hasFinishedPullOut() const;
  void checkBackFinished();
  bool isStopped();
  bool hasFinishedCurrentPath();

  void setDebugData() const;
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OUT__PULL_OUT_MODULE_HPP_
