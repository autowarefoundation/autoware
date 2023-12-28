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

#ifndef BEHAVIOR_PATH_START_PLANNER_MODULE__START_PLANNER_MODULE_HPP_
#define BEHAVIOR_PATH_START_PLANNER_MODULE__START_PLANNER_MODULE_HPP_

#include "behavior_path_planner_common/interface/scene_module_interface.hpp"
#include "behavior_path_planner_common/utils/parking_departure/common_module_data.hpp"
#include "behavior_path_planner_common/utils/parking_departure/geometric_parallel_parking.hpp"
#include "behavior_path_planner_common/utils/path_safety_checker/path_safety_checker_parameters.hpp"
#include "behavior_path_planner_common/utils/path_shifter/path_shifter.hpp"
#include "behavior_path_planner_common/utils/utils.hpp"
#include "behavior_path_start_planner_module/freespace_pull_out.hpp"
#include "behavior_path_start_planner_module/geometric_pull_out.hpp"
#include "behavior_path_start_planner_module/pull_out_path.hpp"
#include "behavior_path_start_planner_module/shift_pull_out.hpp"
#include "behavior_path_start_planner_module/start_planner_parameters.hpp"

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
using PriorityOrder = std::vector<std::pair<size_t, std::shared_ptr<PullOutPlannerBase>>>;

struct PullOutStatus
{
  PullOutPath pull_out_path{};
  size_t current_path_idx{0};
  PlannerType planner_type{PlannerType::NONE};
  PathWithLaneId backward_path{};
  bool found_pull_out_path{false};      // current path is safe against static objects
  bool is_safe_dynamic_objects{false};  // current path is safe against dynamic objects
  bool driving_forward{false};          // if ego is driving on backward path, this is set to false
  bool backward_driving_complete{
    false};  // after backward driving is complete, this is set to true (warning: this is set to
             // false at next cycle after backward driving is complete)
  Pose pull_out_start_pose{};
  bool prev_is_safe_dynamic_objects{false};
  std::shared_ptr<PathWithLaneId> prev_stop_path_after_approval{nullptr};
  std::optional<Pose> stop_pose{std::nullopt};

  PullOutStatus() {}
};

class StartPlannerModule : public SceneModuleInterface
{
public:
  StartPlannerModule(
    const std::string & name, rclcpp::Node & node,
    const std::shared_ptr<StartPlannerParameters> & parameters,
    const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
    std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
      objects_of_interest_marker_interface_ptr_map);

  void updateModuleParams(const std::any & parameters) override
  {
    parameters_ = std::any_cast<std::shared_ptr<StartPlannerParameters>>(parameters);
  }

  // TODO(someone): remove this, and use base class function
  [[deprecated]] BehaviorModuleOutput run() override;

  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  BehaviorModuleOutput plan() override;
  BehaviorModuleOutput planWaitingApproval() override;
  CandidateOutput planCandidate() const override;
  void processOnEntry() override;
  void processOnExit() override;
  void updateData() override;

  void updateEgoPredictedPathParams(
    std::shared_ptr<EgoPredictedPathParams> & ego_predicted_path_params,
    const std::shared_ptr<StartPlannerParameters> & start_planner_params);

  void updateSafetyCheckParams(
    std::shared_ptr<SafetyCheckParams> & safety_check_params,
    const std::shared_ptr<StartPlannerParameters> & start_planner_params);

  void updateObjectsFilteringParams(
    std::shared_ptr<ObjectsFilteringParams> & objects_filtering_params,
    const std::shared_ptr<StartPlannerParameters> & start_planner_params);

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
  bool isDrivingForward() const { return status_.driving_forward; }
  bool isFreespacePlanning() const { return status_.planner_type == PlannerType::FREESPACE; }

private:
  bool canTransitSuccessState() override;

  bool canTransitFailureState() override { return false; }

  bool canTransitIdleToRunningState() override;

  /**
   * @brief init member variables.
   */
  void initVariables();

  void initializeSafetyCheckParameters();

  bool requiresDynamicObjectsCollisionDetection() const;

  /**
   * @brief Check if there are no moving objects around within a certain radius.
   *
   * This function filters the dynamic objects within a certain radius and then filters them by
   * their velocity. If there are no moving objects around, it returns true. Otherwise, it returns
   * false.
   *
   * @return True if there are no moving objects around. False otherwise.
   */
  bool noMovingObjectsAround() const;
  bool receivedNewRoute() const;

  bool isModuleRunning() const;
  bool isCurrentPoseOnMiddleOfTheRoad() const;
  bool isCloseToOriginalStartPose() const;
  bool hasArrivedAtGoal() const;
  bool isMoving() const;

  PriorityOrder determinePriorityOrder(
    const std::string & search_priority, const size_t start_pose_candidates_num);
  bool findPullOutPath(
    const Pose & start_pose_candidate, const std::shared_ptr<PullOutPlannerBase> & planner,
    const Pose & refined_start_pose, const Pose & goal_pose);

  PathWithLaneId extractCollisionCheckPath(const PullOutPath & path);
  void updateStatusWithCurrentPath(
    const behavior_path_planner::PullOutPath & path, const Pose & start_pose,
    const behavior_path_planner::PlannerType & planner_type);
  void updateStatusWithNextPath(
    const behavior_path_planner::PullOutPath & path, const Pose & start_pose,
    const behavior_path_planner::PlannerType & planner_type);
  void updateStatusIfNoSafePathFound();

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
  PathWithLaneId calcBackwardPathFromStartPose() const;
  std::vector<Pose> searchPullOutStartPoseCandidates(
    const PathWithLaneId & back_path_from_start_pose) const;

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
  PredictedObjects filterStopObjectsInPullOutLanes(
    const lanelet::ConstLanelets & pull_out_lanes, const geometry_msgs::msg::Point & current_pose,
    const double velocity_threshold, const double object_check_backward_distance,
    const double object_check_forward_distance) const;
  bool hasFinishedPullOut() const;
  bool hasFinishedBackwardDriving() const;
  bool hasCollisionWithDynamicObjects() const;
  bool isStopped();
  bool isStuck();
  bool hasFinishedCurrentPath();
  void updateSafetyCheckTargetObjectsData(
    const PredictedObjects & filtered_objects, const TargetObjectsOnLane & target_objects_on_lane,
    const std::vector<PoseWithVelocityStamped> & ego_predicted_path) const;
  bool isSafePath() const;
  void setDrivableAreaInfo(BehaviorModuleOutput & output) const;

  // check if the goal is located behind the ego in the same route segment.
  bool isGoalBehindOfEgoInSameRouteSegment() const;

  // generate BehaviorPathOutput with stopping path and update status
  BehaviorModuleOutput generateStopOutput();

  SafetyCheckParams createSafetyCheckParams() const;
  // freespace planner
  void onFreespacePlannerTimer();
  bool planFreespacePath();

  void setDebugData() const;
  void logPullOutStatus(rclcpp::Logger::Level log_level = rclcpp::Logger::Level::Info) const;
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_START_PLANNER_MODULE__START_PLANNER_MODULE_HPP_
