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

#ifndef AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__START_PLANNER_MODULE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__START_PLANNER_MODULE_HPP_

#include "autoware/behavior_path_planner_common/interface/scene_module_interface.hpp"
#include "autoware/behavior_path_planner_common/utils/parking_departure/common_module_data.hpp"
#include "autoware/behavior_path_planner_common/utils/parking_departure/geometric_parallel_parking.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/path_safety_checker_parameters.hpp"
#include "autoware/behavior_path_planner_common/utils/path_shifter/path_shifter.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"
#include "autoware/behavior_path_start_planner_module/data_structs.hpp"
#include "autoware/behavior_path_start_planner_module/freespace_pull_out.hpp"
#include "autoware/behavior_path_start_planner_module/geometric_pull_out.hpp"
#include "autoware/behavior_path_start_planner_module/pull_out_path.hpp"
#include "autoware/behavior_path_start_planner_module/shift_pull_out.hpp"

#include <autoware/lane_departure_checker/lane_departure_checker.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <lanelet2_core/Forward.h>
#include <tf2/utils.h>

#include <atomic>
#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{
using autoware::behavior_path_planner::utils::path_safety_checker::EgoPredictedPathParams;
using autoware::behavior_path_planner::utils::path_safety_checker::ObjectsFilteringParams;
using autoware::behavior_path_planner::utils::path_safety_checker::PoseWithVelocityStamped;
using autoware::behavior_path_planner::utils::path_safety_checker::SafetyCheckParams;
using autoware::behavior_path_planner::utils::path_safety_checker::TargetObjectsOnLane;
using autoware::lane_departure_checker::LaneDepartureChecker;
using geometry_msgs::msg::PoseArray;
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
  //! record the first time when ego started forward-driving (maybe after backward driving
  //! completion) in AUTONOMOUS operation mode
  std::optional<rclcpp::Time> first_engaged_and_driving_forward_time{std::nullopt};
  // record if the ego has departed from the start point
  bool has_departed{false};

  PullOutStatus() = default;
};

class StartPlannerModule : public SceneModuleInterface
{
public:
  StartPlannerModule(
    const std::string & name, rclcpp::Node & node,
    const std::shared_ptr<StartPlannerParameters> & parameters,
    const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
    std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
      objects_of_interest_marker_interface_ptr_map,
    std::shared_ptr<SteeringFactorInterface> & steering_factor_interface_ptr);

  ~StartPlannerModule() override
  {
    if (freespace_planner_timer_) {
      freespace_planner_timer_->cancel();
    }

    while (is_freespace_planner_cb_running_.load()) {
      RCLCPP_INFO_THROTTLE(
        getLogger(), *clock_, 1000, "Waiting for freespace planner callback to finish...");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    RCLCPP_INFO_THROTTLE(getLogger(), *clock_, 1000, "freespace planner callback finished");
  }

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
  struct StartPlannerData
  {
    StartPlannerParameters parameters;
    PlannerData planner_data;
    ModuleStatus current_status;
    PullOutStatus main_thread_pull_out_status;
    bool is_stopped;

    StartPlannerData clone() const;
    void update(
      const StartPlannerParameters & parameters, const PlannerData & planner_data,
      const ModuleStatus & current_status, const PullOutStatus & pull_out_status,
      const bool is_stopped);
  };
  std::optional<PullOutStatus> freespace_thread_status_{std::nullopt};
  std::optional<StartPlannerData> start_planner_data_{std::nullopt};
  std::mutex start_planner_data_mutex_;

  // Flag class for managing whether a certain callback is running in multi-threading
  class ScopedFlag
  {
  public:
    explicit ScopedFlag(std::atomic<bool> & flag) : flag_(flag) { flag_.store(true); }

    ~ScopedFlag() { flag_.store(false); }

  private:
    std::atomic<bool> & flag_;
  };

  bool canTransitSuccessState() override;

  bool canTransitFailureState() override { return false; }

  /**
   * @brief init member variables.
   */
  void initVariables();

  void initializeSafetyCheckParameters();

  bool requiresDynamicObjectsCollisionDetection() const;

  uint16_t getSteeringFactorDirection(
    const autoware::behavior_path_planner::BehaviorModuleOutput & output) const
  {
    switch (output.turn_signal_info.turn_signal.command) {
      case TurnIndicatorsCommand::ENABLE_LEFT:
        return SteeringFactor::LEFT;

      case TurnIndicatorsCommand::ENABLE_RIGHT:
        return SteeringFactor::RIGHT;

      default:
        return SteeringFactor::STRAIGHT;
    }
  };

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

  /**
   * @brief Check if the ego vehicle is preventing the rear vehicle from passing through.
   *
   * This function just call isPreventingRearVehicleFromPassingThrough(const Pose & ego_pose) with
   * two poses. If rear vehicle is obstructed by ego vehicle at either of the two poses, it returns
   * true.
   *
   * @return true if the ego vehicle is preventing the rear vehicle from passing through with the
   * current pose or the pose if it stops.
   */
  bool isPreventingRearVehicleFromPassingThrough() const;

  /**
    * @brief Check if the ego vehicle is preventing the rear vehicle from passing through.
    *
    * This function measures the distance to the lane boundary from the current pose and the pose if
it stops, and determines whether there is enough space for the rear vehicle to pass through. If
    * it is obstructing at either of the two poses, it returns true.
    *
    * @return true if the ego vehicle is preventing the rear vehicle from passing through with given
ego pose.
    */
  bool isPreventingRearVehicleFromPassingThrough(const Pose & ego_pose) const;

  bool isCloseToOriginalStartPose() const;
  bool hasArrivedAtGoal() const;
  bool isMoving() const;

  PriorityOrder determinePriorityOrder(
    const std::string & search_priority, const size_t start_pose_candidates_num);
  bool findPullOutPath(
    const Pose & start_pose_candidate, const std::shared_ptr<PullOutPlannerBase> & planner,
    const Pose & refined_start_pose, const Pose & goal_pose, const double collision_check_margin,
    std::vector<PlannerDebugData> & debug_data_vector);

  PathWithLaneId extractCollisionCheckSection(
    const PullOutPath & path, const autoware::behavior_path_planner::PlannerType & planner_type);
  void updateStatusWithCurrentPath(
    const autoware::behavior_path_planner::PullOutPath & path, const Pose & start_pose,
    const autoware::behavior_path_planner::PlannerType & planner_type);
  void updateStatusWithNextPath(
    const autoware::behavior_path_planner::PullOutPath & path, const Pose & start_pose,
    const autoware::behavior_path_planner::PlannerType & planner_type);
  void updateStatusIfNoSafePathFound();

  std::shared_ptr<StartPlannerParameters> parameters_;
  mutable std::shared_ptr<EgoPredictedPathParams> ego_predicted_path_params_;
  mutable std::shared_ptr<ObjectsFilteringParams> objects_filtering_params_;
  mutable std::shared_ptr<SafetyCheckParams> safety_check_params_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;

  std::vector<std::shared_ptr<PullOutPlannerBase>> start_planners_;
  PullOutStatus status_;
  mutable StartPlannerDebugData debug_data_;

  // Keeps track of lanelets that should be ignored when calculating the turnSignalInfo for this
  // module's output. If the ego vehicle is in this lanelet, the calculation is skipped.
  std::optional<lanelet::Id> ignore_signal_{std::nullopt};

  std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> odometry_buffer_;

  std::unique_ptr<rclcpp::Time> last_route_received_time_;
  std::unique_ptr<rclcpp::Time> last_pull_out_start_update_time_;

  // generate freespace pull out paths in a separate thread
  std::unique_ptr<PullOutPlannerBase> freespace_planner_;
  rclcpp::TimerBase::SharedPtr freespace_planner_timer_;
  rclcpp::CallbackGroup::SharedPtr freespace_planner_timer_cb_group_;
  std::atomic<bool> is_freespace_planner_cb_running_;

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
  TurnSignalInfo calcTurnSignalInfo();

  void incrementPathIndex();
  PathWithLaneId getCurrentPath() const;
  void planWithPriority(
    const std::vector<Pose> & start_pose_candidates, const Pose & refined_start_pose,
    const Pose & goal_pose, const std::string & search_priority);
  PathWithLaneId generateStopPath() const;
  lanelet::ConstLanelets getPathRoadLanes(const PathWithLaneId & path) const;
  std::vector<DrivableLanes> generateDrivableLanes(const PathWithLaneId & path) const;
  void updatePullOutStatus();
  void updateStatusAfterBackwardDriving();
  PredictedObjects filterStopObjectsInPullOutLanes(
    const lanelet::ConstLanelets & pull_out_lanes, const geometry_msgs::msg::Point & current_point,
    const double velocity_threshold, const double object_check_forward_distance,
    const double object_check_backward_distance) const;
  bool needToPrepareBlinkerBeforeStartDrivingForward() const;
  bool hasReachedFreespaceEnd() const;
  bool hasReachedPullOutEnd() const;
  bool hasFinishedBackwardDriving() const;
  bool hasCollisionWithDynamicObjects() const;
  bool isStopped();
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
  std::optional<PullOutStatus> planFreespacePath(
    const StartPlannerParameters & parameters,
    const std::shared_ptr<const PlannerData> & planner_data, const PullOutStatus & pull_out_status);

  void setDebugData();
  void logPullOutStatus(rclcpp::Logger::Level log_level = rclcpp::Logger::Level::Info) const;
};
}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__START_PLANNER_MODULE_HPP_
