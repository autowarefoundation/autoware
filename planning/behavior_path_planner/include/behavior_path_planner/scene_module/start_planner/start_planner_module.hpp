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
#include "behavior_path_planner/utils/path_safety_checker/path_safety_checker_parameters.hpp"
#include "behavior_path_planner/utils/path_shifter/path_shifter.hpp"
#include "behavior_path_planner/utils/start_goal_planner_common/common_module_data.hpp"
#include "behavior_path_planner/utils/start_planner/freespace_pull_out.hpp"
#include "behavior_path_planner/utils/start_planner/geometric_pull_out.hpp"
#include "behavior_path_planner/utils/start_planner/pull_out_path.hpp"
#include "behavior_path_planner/utils/start_planner/shift_pull_out.hpp"
#include "behavior_path_planner/utils/start_planner/start_planner_parameters.hpp"
#include "behavior_path_planner/utils/utils.hpp"

#include <lane_departure_checker/lane_departure_checker.hpp>
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
using behavior_path_planner::utils::path_safety_checker::EgoPredictedPathParams;
using behavior_path_planner::utils::path_safety_checker::ObjectsFilteringParams;
using behavior_path_planner::utils::path_safety_checker::PoseWithVelocityStamped;
using behavior_path_planner::utils::path_safety_checker::SafetyCheckParams;
using behavior_path_planner::utils::path_safety_checker::TargetObjectsOnLane;
using geometry_msgs::msg::PoseArray;
using lane_departure_checker::LaneDepartureChecker;

struct PullOutStatus
{
  PullOutPath pull_out_path{};
  size_t current_path_idx{0};
  PlannerType planner_type{PlannerType::NONE};
  PathWithLaneId backward_path{};
  lanelet::ConstLanelets pull_out_lanes{};
  bool is_safe_static_objects{false};   // current path is safe against static objects
  bool is_safe_dynamic_objects{false};  // current path is safe against dynamic objects
  bool back_finished{false};  // if backward driving is not required, this is also set to true
                              // todo: rename to clear variable name.
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
  void processOnEntry() override;
  void processOnExit() override;

  void setParameters(const std::shared_ptr<StartPlannerParameters> & parameters)
  {
    parameters_ = parameters;
    if (parameters->safety_check_params.enable_safety_check) {
      ego_predicted_path_params_ =
        std::make_shared<EgoPredictedPathParams>(parameters_->ego_predicted_path_params);
      objects_filtering_params_ =
        std::make_shared<ObjectsFilteringParams>(parameters_->objects_filtering_params);
      safety_check_params_ = std::make_shared<SafetyCheckParams>(parameters_->safety_check_params);
    }
  }
  void resetStatus();

  void acceptVisitor(
    [[maybe_unused]] const std::shared_ptr<SceneModuleVisitor> & visitor) const override
  {
  }

  // Condition to disable simultaneous execution
  bool isBackFinished() const { return status_.back_finished; }
  bool isFreespacePlanning() const { return status_.planner_type == PlannerType::FREESPACE; }

private:
  bool canTransitSuccessState() override { return false; }

  bool canTransitFailureState() override { return false; }

  bool canTransitIdleToRunningState() override { return false; }

  void initializeSafetyCheckParameters();

  std::shared_ptr<StartPlannerParameters> parameters_;
  mutable std::shared_ptr<EgoPredictedPathParams> ego_predicted_path_params_;
  mutable std::shared_ptr<ObjectsFilteringParams> objects_filtering_params_;
  mutable std::shared_ptr<SafetyCheckParams> safety_check_params_;
  vehicle_info_util::VehicleInfo vehicle_info_;

  std::vector<std::shared_ptr<PullOutPlannerBase>> start_planners_;
  PullOutStatus status_;
  mutable StartGoalPlannerData start_planner_data_;

  std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> odometry_buffer_;

  std::unique_ptr<rclcpp::Time> last_route_received_time_;
  std::unique_ptr<rclcpp::Time> last_pull_out_start_update_time_;

  // generate freespace pull out paths in a separate thread
  std::unique_ptr<PullOutPlannerBase> freespace_planner_;
  rclcpp::TimerBase::SharedPtr freespace_planner_timer_;
  rclcpp::CallbackGroup::SharedPtr freespace_planner_timer_cb_group_;
  // TODO(kosuke55)
  // Currently, we only do lock when updating a member of status_.
  // However, we need to ensure that the value does not change when referring to it.
  std::mutex mutex_;

  PathWithLaneId getFullPath() const;
  PathWithLaneId calcStartPoseCandidatesBackwardPath() const;
  std::vector<Pose> searchPullOutStartPoses(const PathWithLaneId & start_pose_candidates) const;

  std::shared_ptr<LaneDepartureChecker> lane_departure_checker_;

  // turn signal
  TurnSignalInfo calcTurnSignalInfo() const;

  void incrementPathIndex();
  PathWithLaneId getCurrentPath() const;
  void planWithPriority(
    const std::vector<Pose> & start_pose_candidates, const Pose & refined_start_pose,
    const Pose & goal_pose, const std::string search_priority);
  PathWithLaneId generateStopPath() const;
  lanelet::ConstLanelets getPathRoadLanes(const PathWithLaneId & path) const;
  std::vector<DrivableLanes> generateDrivableLanes(const PathWithLaneId & path) const;
  void updatePullOutStatus();
  void updateStatusAfterBackwardDriving();
  static bool isOverlappedWithLane(
    const lanelet::ConstLanelet & candidate_lanelet,
    const tier4_autoware_utils::LinearRing2d & vehicle_footprint);
  bool hasFinishedPullOut() const;
  bool isBackwardDrivingComplete() const;
  bool isStopped();
  bool isStuck();
  bool hasFinishedCurrentPath();
  void updateSafetyCheckTargetObjectsData(
    const PredictedObjects & filtered_objects, const TargetObjectsOnLane & target_objects_on_lane,
    const std::vector<PoseWithVelocityStamped> & ego_predicted_path) const;
  bool isSafePath() const;
  void setDrivableAreaInfo(BehaviorModuleOutput & output) const;

  // check if the goal is located behind the ego in the same route segment.
  bool IsGoalBehindOfEgoInSameRouteSegment() const;

  // generate BehaviorPathOutput with stopping path and update status
  BehaviorModuleOutput generateStopOutput();

  SafetyCheckParams createSafetyCheckParams() const;
  // freespace planner
  void onFreespacePlannerTimer();
  bool planFreespacePath();

  void setDebugData() const;
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__START_PLANNER__START_PLANNER_MODULE_HPP_
