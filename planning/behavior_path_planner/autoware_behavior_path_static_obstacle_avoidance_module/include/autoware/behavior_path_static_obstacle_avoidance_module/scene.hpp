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

#ifndef AUTOWARE__BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__SCENE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__SCENE_HPP_

#include "autoware/behavior_path_planner_common/interface/scene_module_interface.hpp"
#include "autoware/behavior_path_planner_common/interface/scene_module_visitor.hpp"
#include "autoware/behavior_path_static_obstacle_avoidance_module/data_structs.hpp"
#include "autoware/behavior_path_static_obstacle_avoidance_module/helper.hpp"
#include "autoware/behavior_path_static_obstacle_avoidance_module/shift_line_generator.hpp"
#include "autoware/behavior_path_static_obstacle_avoidance_module/type_alias.hpp"

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>

#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{

using helper::static_obstacle_avoidance::AvoidanceHelper;
using tier4_planning_msgs::msg::AvoidanceDebugMsg;

class StaticObstacleAvoidanceModule : public SceneModuleInterface
{
public:
  StaticObstacleAvoidanceModule(
    const std::string & name, rclcpp::Node & node, std::shared_ptr<AvoidanceParameters> parameters,
    const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
    std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
      objects_of_interest_marker_interface_ptr_map);

  CandidateOutput planCandidate() const override;
  BehaviorModuleOutput plan() override;
  BehaviorModuleOutput planWaitingApproval() override;
  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  void processOnEntry() override;
  void processOnExit() override;
  void updateData() override;
  void acceptVisitor(const std::shared_ptr<SceneModuleVisitor> & visitor) const override;

  void updateModuleParams(const std::any & parameters) override
  {
    parameters_ = std::any_cast<std::shared_ptr<AvoidanceParameters>>(parameters);
  }
  std::shared_ptr<AvoidanceDebugMsgArray> get_debug_msg_array() const;

private:
  /**
   * @brief return the result whether the module can stop path generation process.
   * @param avoidance data.
   * @return it will return AvoidanceState::RUNNING when there are obstacles ego should avoid.
   *         it will return AvoidanceState::CANCEL when all obstacles have gone.
   *         it will return AvoidanceState::SUCCEEDED when the ego avoid all obstacles.
   */
  AvoidanceState getCurrentModuleState(const AvoidancePlanningData & data) const;

  bool canTransitSuccessState() override;

  bool canTransitFailureState() override { return false; }

  /**
   * @brief update RTC status for candidate shift line.
   * @param candidate path.
   */
  void updateCandidateRTCStatus(const CandidateOutput & candidate)
  {
    if (candidate.lateral_shift > 0.0) {
      rtc_interface_ptr_map_.at("left")->updateCooperateStatus(
        uuid_map_.at("left"), isExecutionReady(), State::WAITING_FOR_EXECUTION,
        candidate.start_distance_to_path_change, candidate.finish_distance_to_path_change,
        clock_->now());
      candidate_uuid_ = uuid_map_.at("left");
      return;
    }
    if (candidate.lateral_shift < 0.0) {
      rtc_interface_ptr_map_.at("right")->updateCooperateStatus(
        uuid_map_.at("right"), isExecutionReady(), State::WAITING_FOR_EXECUTION,
        candidate.start_distance_to_path_change, candidate.finish_distance_to_path_change,
        clock_->now());
      candidate_uuid_ = uuid_map_.at("right");
      return;
    }

    RCLCPP_WARN_STREAM(
      getLogger(),
      "Direction is UNKNOWN, start_distance = " << candidate.start_distance_to_path_change);
  }

  /**
   * @brief update RTC status for approved shift line.
   * @param approved avoidance path.
   */
  void updateRegisteredRTCStatus(const PathWithLaneId & path)
  {
    const auto ego_idx = planner_data_->findEgoIndex(path.points);

    for (const auto & left_shift : left_shift_array_) {
      const double start_distance = autoware::motion_utils::calcSignedArcLength(
        path.points, ego_idx, left_shift.start_pose.position);
      const double finish_distance = start_distance + left_shift.relative_longitudinal;
      rtc_interface_ptr_map_.at("left")->updateCooperateStatus(
        left_shift.uuid, true, State::RUNNING, start_distance, finish_distance, clock_->now());
      if (finish_distance > -1.0e-03) {
        steering_factor_interface_ptr_->updateSteeringFactor(
          {left_shift.start_pose, left_shift.finish_pose}, {start_distance, finish_distance},
          PlanningBehavior::AVOIDANCE, SteeringFactor::LEFT, SteeringFactor::TURNING, "");
      }
    }

    for (const auto & right_shift : right_shift_array_) {
      const double start_distance = autoware::motion_utils::calcSignedArcLength(
        path.points, ego_idx, right_shift.start_pose.position);
      const double finish_distance = start_distance + right_shift.relative_longitudinal;
      rtc_interface_ptr_map_.at("right")->updateCooperateStatus(
        right_shift.uuid, true, State::RUNNING, start_distance, finish_distance, clock_->now());
      if (finish_distance > -1.0e-03) {
        steering_factor_interface_ptr_->updateSteeringFactor(
          {right_shift.start_pose, right_shift.finish_pose}, {start_distance, finish_distance},
          PlanningBehavior::AVOIDANCE, SteeringFactor::RIGHT, SteeringFactor::TURNING, "");
      }
    }
  }

  /**
   * @brief remove RTC status for candidate path.
   */
  void removeCandidateRTCStatus()
  {
    if (rtc_interface_ptr_map_.at("left")->isRegistered(candidate_uuid_)) {
      rtc_interface_ptr_map_.at("left")->updateCooperateStatus(
        candidate_uuid_, true, State::FAILED, std::numeric_limits<double>::lowest(),
        std::numeric_limits<double>::lowest(), clock_->now());
    }

    if (rtc_interface_ptr_map_.at("right")->isRegistered(candidate_uuid_)) {
      rtc_interface_ptr_map_.at("right")->updateCooperateStatus(
        candidate_uuid_, true, State::FAILED, std::numeric_limits<double>::lowest(),
        std::numeric_limits<double>::lowest(), clock_->now());
    }
  }

  /**
   * @brief remove RTC status for left approved path.
   */
  void removePreviousRTCStatusLeft()
  {
    if (rtc_interface_ptr_map_.at("left")->isRegistered(uuid_map_.at("left"))) {
      rtc_interface_ptr_map_.at("left")->removeCooperateStatus(uuid_map_.at("left"));
    }
  }

  /**
   * @brief remove RTC status for right approved path.
   */
  void removePreviousRTCStatusRight()
  {
    if (rtc_interface_ptr_map_.at("right")->isRegistered(uuid_map_.at("right"))) {
      rtc_interface_ptr_map_.at("right")->removeCooperateStatus(uuid_map_.at("right"));
    }
  }

  // initializer

  /**
   * @brief init member variables.
   */
  void initVariables();

  /**
   * @brief init RTC status.
   */
  void initRTCStatus();

  /**
   * @brief update RTC status.
   */
  void updateRTCData();

  // ego behavior update

  /**
   * @brief insert stop/decel point in output path.
   * @param avoidance data.
   * @param target path.
   */
  void updateEgoBehavior(const AvoidancePlanningData & data, ShiftedPath & path);

  /**
   * @brief insert stop point in output path.
   * @param flag. if it is true, the ego decelerates within accel/jerk constraints.
   * @param target path.
   */
  void insertWaitPoint(const bool use_constraints_for_decel, ShiftedPath & shifted_path) const;

  /**
   * @brief insert stop point to yield. (stop in the lane if possible, even if the shift has
   * initiated.)
   * @param flag. if it is true, the ego decelerates within accel/jerk constraints.
   * @param target path.
   */
  void insertStopPoint(const bool use_constraints_for_decel, ShiftedPath & shifted_path) const;

  /**
   * @brief insert stop point in return path to original lane.
   * @param flag. if it is true, the ego decelerates within accel/jerk constraints.
   * @param target path.
   */
  void insertReturnDeadLine(const bool use_constraints_for_decel, ShiftedPath & shifted_path) const;

  /**
   * @brief insert stop point in output path.
   * @param target path.
   */
  void insertPrepareVelocity(ShiftedPath & shifted_path) const;

  /**
   * @brief insert max velocity in output path to limit acceleration.
   * @param target path.
   */
  void insertAvoidanceVelocity(ShiftedPath & shifted_path) const;

  /**
   * @brief calculate stop distance based on object's overhang.
   * @param stop distance.
   */
  double calcDistanceToStopLine(const ObjectData & object) const;

  // avoidance data preparation

  /**
   * @brief update main avoidance data for avoidance path generation based on latest planner data.
   */
  void fillFundamentalData(AvoidancePlanningData & data, DebugData & debug);

  /**
   * @brief fill additional data so that the module judges target objects.
   * @return object that has additional data.
   */
  ObjectData createObjectData(
    const AvoidancePlanningData & data, const PredictedObject & object) const;

  /**
   * @brief fill additional data so that the module judges target objects.
   * @param avoidance data.
   * @param debug data.
   */
  void fillAvoidanceTargetObjects(AvoidancePlanningData & data, DebugData & debug) const;

  /**
   * @brief fill candidate shift lines.
   * @param avoidance data.
   * @param debug data.
   * @details in this function, following two shift line arrays are generated.
   * - unapproved raw shift lines.
   * - unapproved new shift lines.
   * and check whether the new shift lines are safe or not.
   */
  void fillShiftLine(AvoidancePlanningData & data, DebugData & debug) const;

  /**
   * @brief fill ego status based on the candidate path safety check result.
   * @param avoidance data.
   * @param debug data.
   */
  void fillEgoStatus(AvoidancePlanningData & data, DebugData & debug) const;

  /**
   * @brief fill debug data.
   * @param avoidance data.
   * @param debug data.
   */
  void fillDebugData(const AvoidancePlanningData & data, DebugData & debug) const;

  /**
   * @brief check whether the ego can transit yield maneuver.
   * @param avoidance data.
   */
  bool canYieldManeuver(const AvoidancePlanningData & data) const;

  /**
   * @brief add new shift line to path shifter if the RTC status is activated.
   * @param new shift lines.
   */
  void updatePathShifter(const AvoidLineArray & point);

  /**
   * @brief add new shift line to path shifter.
   * @param new shift lines.
   */
  void addNewShiftLines(PathShifter & path_shifter, const AvoidLineArray & shift_lines) const;

  /**
   * @brief once generate avoidance path from new shift lines, and calculate lateral offset between
   * ego and the path.
   * @param new shift lines.
   * @param path shifter.
   * @return result. if there is huge gap between the ego position and candidate path, return false.
   */
  bool isValidShiftLine(const AvoidLineArray & shift_lines, const PathShifter & shifter) const;

  // generate output data

  /**
   * @brief fill debug markers.
   */
  void updateDebugMarker(
    const AvoidancePlanningData & data, const PathShifter & shifter, const DebugData & debug) const;

  /**
   * @brief fill information markers that are shown in Rviz by default.
   */
  void updateInfoMarker(const AvoidancePlanningData & data) const;

  /**
   * @brief fill debug msg that are published as a topic.
   */
  void updateAvoidanceDebugData(std::vector<AvoidanceDebugMsg> & avoidance_debug_msg_array) const;

  // safety check

  /**
   * @brief check avoidance path safety for surround moving objects.
   * @param avoidance path.
   * @param debug data.
   * @return result.
   */
  bool isSafePath(ShiftedPath & shifted_path, DebugData & debug) const;

  // post process

  /**
   * @brief extend backward length so that path shift inserts behind shift lines.
   * @param current output path.
   * @return extended path.
   */
  PathWithLaneId extendBackwardLength(const PathWithLaneId & original_path) const;

  /**
   * @brief reset registered shift lines.
   * @details reset only when the base offset is zero. Otherwise, sudden steering will be caused;
   */
  void removeRegisteredShiftLines(const uint8_t state)
  {
    constexpr double threshold = 0.1;
    if (std::abs(path_shifter_.getBaseOffset()) > threshold) {
      RCLCPP_INFO(getLogger(), "base offset is not zero. can't reset registered shift lines.");
      return;
    }

    unlockNewModuleLaunch();

    for (const auto & left_shift : left_shift_array_) {
      if (rtc_interface_ptr_map_.at("left")->isRegistered(left_shift.uuid)) {
        rtc_interface_ptr_map_.at("left")->updateCooperateStatus(
          left_shift.uuid, true, state, std::numeric_limits<double>::lowest(),
          std::numeric_limits<double>::lowest(), clock_->now());
      }
    }

    for (const auto & right_shift : right_shift_array_) {
      if (rtc_interface_ptr_map_.at("right")->isRegistered(right_shift.uuid)) {
        rtc_interface_ptr_map_.at("right")->updateCooperateStatus(
          right_shift.uuid, true, state, std::numeric_limits<double>::lowest(),
          std::numeric_limits<double>::lowest(), clock_->now());
      }
    }

    if (!path_shifter_.getShiftLines().empty()) {
      left_shift_array_.clear();
      right_shift_array_.clear();
    }

    generator_.reset();
    path_shifter_.setShiftLines(ShiftLineArray{});
  }

  /**
   * @brief remove behind shift lines.
   * @param path shifter.
   */
  void postProcess() override
  {
    const size_t idx = planner_data_->findEgoIndex(path_shifter_.getReferencePath().points);
    path_shifter_.removeBehindShiftLineAndSetBaseOffset(idx);
  }

  struct RegisteredShiftLine
  {
    UUID uuid;
    Pose start_pose;
    Pose finish_pose;
    double relative_longitudinal{0.0};
  };

  using RegisteredShiftLineArray = std::vector<RegisteredShiftLine>;

  bool arrived_path_end_{false};

  bool safe_{true};

  std::optional<UUID> ignore_signal_{std::nullopt};

  std::shared_ptr<AvoidanceHelper> helper_;

  std::shared_ptr<AvoidanceParameters> parameters_;

  utils::static_obstacle_avoidance::ShiftLineGenerator generator_;

  AvoidancePlanningData avoid_data_;

  PathShifter path_shifter_;

  RegisteredShiftLineArray left_shift_array_;

  RegisteredShiftLineArray right_shift_array_;

  UUID candidate_uuid_;

  ObjectDataArray clip_objects_;

  // TODO(Satoshi OTA) create detected object manager.
  ObjectDataArray registered_objects_;

  // TODO(Satoshi OTA) remove this variable.
  mutable ObjectDataArray ego_stopped_objects_;

  // TODO(Satoshi OTA) remove this variable.
  mutable ObjectDataArray stopped_objects_;

  mutable size_t safe_count_{0};

  mutable DebugData debug_data_;

  mutable std::shared_ptr<AvoidanceDebugMsgArray> debug_msg_ptr_;

  mutable std::vector<AvoidanceDebugMsg> debug_avoidance_initializer_for_shift_line_;

  mutable rclcpp::Time debug_avoidance_initializer_for_shift_line_time_;
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__SCENE_HPP_
