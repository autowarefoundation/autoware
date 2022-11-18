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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__AVOIDANCE__AVOIDANCE_MODULE_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__AVOIDANCE__AVOIDANCE_MODULE_HPP_

#include "behavior_path_planner/scene_module/avoidance/avoidance_module_data.hpp"
#include "behavior_path_planner/scene_module/scene_module_interface.hpp"
#include "behavior_path_planner/scene_module/scene_module_visitor.hpp"
#include "behavior_path_planner/scene_module/utils/path_shifter.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <tier4_planning_msgs/msg/avoidance_debug_msg.hpp>
#include <tier4_planning_msgs/msg/avoidance_debug_msg_array.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_path_planner
{
using tier4_planning_msgs::msg::AvoidanceDebugMsg;
class AvoidanceModule : public SceneModuleInterface
{
public:
  AvoidanceModule(
    const std::string & name, rclcpp::Node & node, std::shared_ptr<AvoidanceParameters> parameters);

  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  BT::NodeStatus updateState() override;
  BehaviorModuleOutput plan() override;
  CandidateOutput planCandidate() const override;
  BehaviorModuleOutput planWaitingApproval() override;
  void onEntry() override;
  void onExit() override;
  void updateData() override;
  void acceptVisitor(const std::shared_ptr<SceneModuleVisitor> & visitor) const override;

  void publishRTCStatus() override
  {
    rtc_interface_left_.publishCooperateStatus(clock_->now());
    rtc_interface_right_.publishCooperateStatus(clock_->now());
  }

  bool isActivated() override
  {
    if (rtc_interface_left_.isRegistered(uuid_left_)) {
      return rtc_interface_left_.isActivated(uuid_left_);
    }
    if (rtc_interface_right_.isRegistered(uuid_right_)) {
      return rtc_interface_right_.isActivated(uuid_right_);
    }
    return false;
  }

  void lockRTCCommand() override
  {
    rtc_interface_left_.lockCommandUpdate();
    rtc_interface_right_.lockCommandUpdate();
  }

  void unlockRTCCommand() override
  {
    rtc_interface_left_.unlockCommandUpdate();
    rtc_interface_right_.unlockCommandUpdate();
  }
  std::shared_ptr<AvoidanceDebugMsgArray> get_debug_msg_array() const;

private:
  struct RegisteredShiftLine
  {
    UUID uuid;
    Pose start_pose;
    Pose finish_pose;
  };
  using RegisteredShiftLineArray = std::vector<RegisteredShiftLine>;

  std::shared_ptr<AvoidanceParameters> parameters_;

  AvoidancePlanningData avoidance_data_;

  PathShifter path_shifter_;

  RTCInterface rtc_interface_left_;
  RTCInterface rtc_interface_right_;

  RegisteredShiftLineArray left_shift_array_;
  RegisteredShiftLineArray right_shift_array_;
  UUID candidate_uuid_;
  UUID uuid_left_;
  UUID uuid_right_;

  void updateCandidateRTCStatus(const CandidateOutput & candidate)
  {
    if (candidate.lateral_shift > 0.0) {
      rtc_interface_left_.updateCooperateStatus(
        uuid_left_, isExecutionReady(), candidate.start_distance_to_path_change,
        candidate.finish_distance_to_path_change, clock_->now());
      candidate_uuid_ = uuid_left_;
      return;
    }
    if (candidate.lateral_shift < 0.0) {
      rtc_interface_right_.updateCooperateStatus(
        uuid_right_, isExecutionReady(), candidate.start_distance_to_path_change,
        candidate.finish_distance_to_path_change, clock_->now());
      candidate_uuid_ = uuid_right_;
      return;
    }

    RCLCPP_WARN_STREAM(
      getLogger(),
      "Direction is UNKNOWN, start_distance = " << candidate.start_distance_to_path_change);
  }

  void updateRegisteredRTCStatus(const PathWithLaneId & path)
  {
    const Point ego_position = planner_data_->self_pose->pose.position;

    for (const auto & left_shift : left_shift_array_) {
      const double start_distance = motion_utils::calcSignedArcLength(
        path.points, ego_position, left_shift.start_pose.position);
      const double finish_distance = motion_utils::calcSignedArcLength(
        path.points, ego_position, left_shift.finish_pose.position);
      rtc_interface_left_.updateCooperateStatus(
        left_shift.uuid, true, start_distance, finish_distance, clock_->now());
      if (finish_distance > -1.0e-03) {
        steering_factor_interface_ptr_->updateSteeringFactor(
          {left_shift.start_pose, left_shift.finish_pose}, {start_distance, finish_distance},
          SteeringFactor::AVOIDANCE_PATH_CHANGE, SteeringFactor::LEFT, SteeringFactor::TURNING, "");
      }
    }

    for (const auto & right_shift : right_shift_array_) {
      const double start_distance = motion_utils::calcSignedArcLength(
        path.points, ego_position, right_shift.start_pose.position);
      const double finish_distance = motion_utils::calcSignedArcLength(
        path.points, ego_position, right_shift.finish_pose.position);
      rtc_interface_right_.updateCooperateStatus(
        right_shift.uuid, true, start_distance, finish_distance, clock_->now());
      if (finish_distance > -1.0e-03) {
        steering_factor_interface_ptr_->updateSteeringFactor(
          {right_shift.start_pose, right_shift.finish_pose}, {start_distance, finish_distance},
          SteeringFactor::AVOIDANCE_PATH_CHANGE, SteeringFactor::RIGHT, SteeringFactor::TURNING,
          "");
      }
    }
  }

  void removeRTCStatus() override
  {
    rtc_interface_left_.clearCooperateStatus();
    rtc_interface_right_.clearCooperateStatus();
  }

  void removeCandidateRTCStatus()
  {
    if (rtc_interface_left_.isRegistered(candidate_uuid_)) {
      rtc_interface_left_.removeCooperateStatus(candidate_uuid_);
    } else if (rtc_interface_right_.isRegistered(candidate_uuid_)) {
      rtc_interface_right_.removeCooperateStatus(candidate_uuid_);
    }
  }

  void removePreviousRTCStatusLeft()
  {
    if (rtc_interface_left_.isRegistered(uuid_left_)) {
      rtc_interface_left_.removeCooperateStatus(uuid_left_);
    }
  }

  void removePreviousRTCStatusRight()
  {
    if (rtc_interface_right_.isRegistered(uuid_right_)) {
      rtc_interface_right_.removeCooperateStatus(uuid_right_);
    }
  }

  /**
   * object pre-process
   */
  void fillAvoidanceTargetObjects(AvoidancePlanningData & data, DebugData & debug) const;
  void fillObjectEnvelopePolygon(const Pose & closest_pose, ObjectData & object_data) const;
  void fillObjectMovingTime(ObjectData & object_data) const;
  void compensateDetectionLost(
    ObjectDataArray & target_objects, ObjectDataArray & other_objects) const;

  // data used in previous planning
  ShiftedPath prev_output_;
  ShiftedPath prev_linear_shift_path_;  // used for shift point check
  PathWithLaneId prev_reference_;

  // for raw_shift_line registration
  AvoidLineArray registered_raw_shift_lines_;
  AvoidLineArray current_raw_shift_lines_;
  void registerRawShiftLines(const AvoidLineArray & future_registered);
  void updateRegisteredRawShiftLines();

  // -- for state management --
  bool isAvoidancePlanRunning() const;

  // -- for pre-processing --
  void initVariables();
  AvoidancePlanningData calcAvoidancePlanningData(DebugData & debug) const;

  ObjectDataArray registered_objects_;
  void updateRegisteredObject(const ObjectDataArray & objects);

  // -- for shift point generation --
  AvoidLineArray calcShiftLines(AvoidLineArray & current_raw_shift_lines, DebugData & debug) const;

  // shift point generation: generator
  double getShiftLength(
    const ObjectData & object, const bool & is_object_on_right, const double & avoid_margin) const;
  AvoidLineArray calcRawShiftLinesFromObjects(const ObjectDataArray & objects) const;
  double getRightShiftBound() const;
  double getLeftShiftBound() const;

  // shift point generation: combiner
  AvoidLineArray combineRawShiftLinesWithUniqueCheck(
    const AvoidLineArray & base_lines, const AvoidLineArray & added_lines) const;

  // shift point generation: merger
  AvoidLineArray mergeShiftLines(const AvoidLineArray & raw_shift_lines, DebugData & debug) const;
  void generateTotalShiftLine(
    const AvoidLineArray & avoid_points, ShiftLineData & shift_line_data) const;
  AvoidLineArray extractShiftLinesFromLine(ShiftLineData & shift_line_data) const;
  std::vector<size_t> calcParentIds(
    const AvoidLineArray & parent_candidates, const AvoidLine & child) const;

  // shift point generation: trimmers
  AvoidLineArray trimShiftLine(const AvoidLineArray & shift_lines, DebugData & debug) const;
  void quantizeShiftLine(AvoidLineArray & shift_lines, const double interval) const;
  void trimSmallShiftLine(AvoidLineArray & shift_lines, const double shift_diff_thres) const;
  void trimSimilarGradShiftLine(AvoidLineArray & shift_lines, const double threshold) const;
  void trimMomentaryReturn(AvoidLineArray & shift_lines) const;
  void trimTooSharpShift(AvoidLineArray & shift_lines) const;
  void trimSharpReturn(AvoidLineArray & shift_lines) const;

  // shift point generation: return-shift generator
  void addReturnShiftLineFromEgo(
    AvoidLineArray & sl_candidates, AvoidLineArray & current_raw_shift_lines) const;

  // -- for shift point operations --
  void alignShiftLinesOrder(
    AvoidLineArray & shift_lines, const bool recalculate_start_length = true) const;
  AvoidLineArray fillAdditionalInfo(const AvoidLineArray & shift_lines) const;
  AvoidLine fillAdditionalInfo(const AvoidLine & shift_line) const;
  void fillAdditionalInfoFromPoint(AvoidLineArray & shift_lines) const;
  void fillAdditionalInfoFromLongitudinal(AvoidLineArray & shift_lines) const;

  // -- for new shift point approval --
  boost::optional<AvoidLineArray> findNewShiftLine(
    const AvoidLineArray & shift_lines, const PathShifter & shifter) const;
  void addShiftLineIfApproved(const AvoidLineArray & point);
  void addNewShiftLines(PathShifter & path_shifter, const AvoidLineArray & shift_lines) const;

  // -- path generation --
  ShiftedPath generateAvoidancePath(PathShifter & shifter) const;
  void generateExtendedDrivableArea(ShiftedPath * shifted_path) const;

  // -- velocity planning --
  std::shared_ptr<double> ego_velocity_starting_avoidance_ptr_;
  void modifyPathVelocityToPreventAccelerationOnAvoidance(ShiftedPath & shifted_path);

  // clean up shifter
  void postProcess(PathShifter & shifter) const;

  // turn signal
  TurnSignalInfo calcTurnSignalInfo(const ShiftedPath & path) const;

  // intersection (old)
  boost::optional<AvoidLine> calcIntersectionShiftLine(const AvoidancePlanningData & data) const;

  bool isTargetObjectType(const PredictedObject & object) const;

  // debug
  mutable DebugData debug_data_;
  mutable std::shared_ptr<AvoidanceDebugMsgArray> debug_msg_ptr_;
  void setDebugData(
    const AvoidancePlanningData & data, const PathShifter & shifter, const DebugData & debug) const;
  void updateAvoidanceDebugData(std::vector<AvoidanceDebugMsg> & avoidance_debug_msg_array) const;
  mutable std::vector<AvoidanceDebugMsg> debug_avoidance_initializer_for_shift_line_;
  mutable rclcpp::Time debug_avoidance_initializer_for_shift_line_time_;
  // =====================================
  // ========= helper functions ==========
  // =====================================

  PathWithLaneId calcCenterLinePath(
    const std::shared_ptr<const PlannerData> & planner_data, const PoseStamped & pose) const;

  // TODO(Horibe): think later.
  // for unique ID
  mutable uint64_t original_unique_id = 0;  // TODO(Horibe) remove mutable
  uint64_t getOriginalShiftLineUniqueId() const { return original_unique_id++; }

  double getNominalAvoidanceDistance(const double shift_length) const;
  double getNominalPrepareDistance() const;
  double getNominalAvoidanceEgoSpeed() const;

  double getSharpAvoidanceDistance(const double shift_length) const;
  double getSharpAvoidanceEgoSpeed() const;

  double getEgoSpeed() const;
  Point getEgoPosition() const;
  PoseStamped getEgoPose() const;
  PoseStamped getUnshiftedEgoPose(const ShiftedPath & prev_path) const;
  double getCurrentBaseShift() const { return path_shifter_.getBaseOffset(); }
  double getCurrentShift() const;
  double getCurrentLinearShift() const;

  /**
   * avoidance module misc data
   */
  mutable ObjectDataArray stopped_objects_;
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__AVOIDANCE__AVOIDANCE_MODULE_HPP_
