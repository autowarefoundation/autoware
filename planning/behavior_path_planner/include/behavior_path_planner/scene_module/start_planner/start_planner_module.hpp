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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__START_PLANNER__START_PLANNER_MODULE_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__START_PLANNER__START_PLANNER_MODULE_HPP_

#include "behavior_path_planner/scene_module/scene_module_interface.hpp"
#include "behavior_path_planner/utils/geometric_parallel_parking/geometric_parallel_parking.hpp"
#include "behavior_path_planner/utils/path_shifter/path_shifter.hpp"
#include "behavior_path_planner/utils/start_planner/geometric_pull_out.hpp"
#include "behavior_path_planner/utils/start_planner/pull_out_path.hpp"
#include "behavior_path_planner/utils/start_planner/shift_pull_out.hpp"
#include "behavior_path_planner/utils/start_planner/start_planner_parameters.hpp"

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
#include <unordered_map>
#include <utility>
#include <vector>

namespace behavior_path_planner
{
using geometry_msgs::msg::PoseArray;
using lane_departure_checker::LaneDepartureChecker;

struct PullOutStatus
{
  PullOutPath pull_out_path{};
  size_t current_path_idx{0};
  PlannerType planner_type{PlannerType::NONE};
  PathWithLaneId backward_path{};
  lanelet::ConstLanelets pull_out_lanes{};
  bool is_safe{false};
  bool back_finished{false};
  Pose pull_out_start_pose{};

  PullOutStatus() {}
};

class StartPlannerModule : public SceneModuleInterface
{
public:
  StartPlannerModule(
    const std::string & name, rclcpp::Node & node,
    const std::shared_ptr<StartPlannerParameters> & parameters,
    const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map);

  void updateModuleParams(const std::any & parameters) override
  {
    parameters_ = std::any_cast<std::shared_ptr<StartPlannerParameters>>(parameters);
  }

  // TODO(someone): remove this, and use base class function
  [[deprecated]] BehaviorModuleOutput run() override;

  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  // TODO(someone): remove this, and use base class function
  [[deprecated]] ModuleStatus updateState() override;
  BehaviorModuleOutput plan() override;
  BehaviorModuleOutput planWaitingApproval() override;
  CandidateOutput planCandidate() const override;

  void processOnExit() override;

  void setParameters(const std::shared_ptr<StartPlannerParameters> & parameters)
  {
    parameters_ = parameters;
  }
  void resetStatus();

  void acceptVisitor(
    [[maybe_unused]] const std::shared_ptr<SceneModuleVisitor> & visitor) const override
  {
  }

  // set is_simultaneously_executable_ as false when backward driving.
  // keep initial value to return it after finishing backward driving.
  bool initial_value_simultaneously_executable_as_approved_module_;
  bool initial_value_simultaneously_executable_as_candidate_module_;
  void setInitialIsSimultaneousExecutableAsApprovedModule(const bool is_simultaneously_executable)
  {
    initial_value_simultaneously_executable_as_approved_module_ = is_simultaneously_executable;
  };
  void setInitialIsSimultaneousExecutableAsCandidateModule(const bool is_simultaneously_executable)
  {
    initial_value_simultaneously_executable_as_candidate_module_ = is_simultaneously_executable;
  };

private:
  bool canTransitSuccessState() override { return false; }

  bool canTransitFailureState() override { return false; }

  bool canTransitIdleToRunningState() override { return false; }

  std::shared_ptr<StartPlannerParameters> parameters_;
  vehicle_info_util::VehicleInfo vehicle_info_;

  std::vector<std::shared_ptr<PullOutPlannerBase>> start_planners_;
  PullOutStatus status_;

  std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> odometry_buffer_;

  std::unique_ptr<rclcpp::Time> last_route_received_time_;
  std::unique_ptr<rclcpp::Time> last_pull_out_start_update_time_;
  std::unique_ptr<Pose> last_approved_pose_;

  std::shared_ptr<PullOutPlannerBase> getCurrentPlanner() const;
  PathWithLaneId getFullPath() const;
  std::vector<Pose> searchPullOutStartPoses();

  std::shared_ptr<LaneDepartureChecker> lane_departure_checker_;

  // turn signal
  TurnSignalInfo calcTurnSignalInfo() const;

  void incrementPathIndex();
  PathWithLaneId getCurrentPath() const;
  void planWithPriority(
    const std::vector<Pose> & start_pose_candidates, const Pose & goal_pose,
    const std::string search_priority);
  PathWithLaneId generateStopPath() const;
  lanelet::ConstLanelets getPathRoadLanes(const PathWithLaneId & path) const;
  std::vector<DrivableLanes> generateDrivableLanes(const PathWithLaneId & path) const;
  void updatePullOutStatus();
  static bool isOverlappedWithLane(
    const lanelet::ConstLanelet & candidate_lanelet,
    const tier4_autoware_utils::LinearRing2d & vehicle_footprint);
  bool hasFinishedPullOut() const;
  void checkBackFinished();
  bool isStopped();
  bool hasFinishedCurrentPath();

  // check if the goal is located behind the ego in the same route segment.
  bool IsGoalBehindOfEgoInSameRouteSegment() const;

  // generate BehaviorPathOutput with stopping path and update status
  BehaviorModuleOutput generateStopOutput();

  void setDebugData() const;
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__START_PLANNER__START_PLANNER_MODULE_HPP_
