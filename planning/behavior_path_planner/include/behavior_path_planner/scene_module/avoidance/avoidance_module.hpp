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
#include "behavior_path_planner/scene_module/utils/path_shifter.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <tier4_planning_msgs/msg/avoidance_debug_factor.hpp>
#include <tier4_planning_msgs/msg/avoidance_debug_msg.hpp>
#include <tier4_planning_msgs/msg/avoidance_debug_msg_array.hpp>

#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace behavior_path_planner
{
class AvoidanceModule : public SceneModuleInterface
{
public:
  AvoidanceModule(
    const std::string & name, rclcpp::Node & node, const AvoidanceParameters & parameters);

  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  BT::NodeStatus updateState() override;
  BehaviorModuleOutput plan() override;
  CandidateOutput planCandidate() const override;
  BehaviorModuleOutput planWaitingApproval() override;
  void onEntry() override;
  void onExit() override;
  void updateData() override;

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

  void setParameters(const AvoidanceParameters & parameters);

private:
  AvoidanceParameters parameters_;

  AvoidancePlanningData avoidance_data_;

  PathShifter path_shifter_;

  RTCInterface rtc_interface_left_;
  RTCInterface rtc_interface_right_;
  UUID uuid_left_;
  UUID uuid_right_;

  void updateRTCStatus(const CandidateOutput & candidate)
  {
    if (candidate.lateral_shift > 0.0) {
      rtc_interface_left_.updateCooperateStatus(
        uuid_left_, isExecutionReady(), candidate.distance_to_path_change, clock_->now());
      return;
    }
    if (candidate.lateral_shift < 0.0) {
      rtc_interface_right_.updateCooperateStatus(
        uuid_right_, isExecutionReady(), candidate.distance_to_path_change, clock_->now());
      return;
    }

    RCLCPP_WARN_STREAM(
      getLogger(), "Direction is UNKNOWN, distance = " << candidate.distance_to_path_change);
  }

  void removeRTCStatus() override
  {
    rtc_interface_left_.clearCooperateStatus();
    rtc_interface_right_.clearCooperateStatus();
  }

  // data used in previous planning
  ShiftedPath prev_output_;
  ShiftedPath prev_linear_shift_path_;  // used for shift point check
  PathWithLaneId prev_reference_;

  // for raw_shift_point registration
  AvoidPointArray registered_raw_shift_points_;
  AvoidPointArray current_raw_shift_points_;
  void registerRawShiftPoints(const AvoidPointArray & future_registered);
  void updateRegisteredRawShiftPoints();

  // -- for state management --
  bool isAvoidancePlanRunning() const;

  // -- for pre-processing --
  void initVariables();
  AvoidancePlanningData calcAvoidancePlanningData(DebugData & debug) const;
  ObjectDataArray calcAvoidanceTargetObjects(
    const lanelet::ConstLanelets & lanelets, const PathWithLaneId & reference_path,
    DebugData & debug) const;

  ObjectDataArray registered_objects_;
  void updateRegisteredObject(const ObjectDataArray & objects);
  void CompensateDetectionLost(ObjectDataArray & objects) const;

  // -- for shift point generation --
  AvoidPointArray calcShiftPoints(
    AvoidPointArray & current_raw_shift_points, DebugData & debug) const;

  // shift point generation: generator
  double getShiftLength(
    const ObjectData & object, const bool & is_object_on_right, const double & avoid_margin) const;
  AvoidPointArray calcRawShiftPointsFromObjects(const ObjectDataArray & objects) const;
  double getRightShiftBound() const;
  double getLeftShiftBound() const;

  // shift point generation: combiner
  AvoidPointArray combineRawShiftPointsWithUniqueCheck(
    const AvoidPointArray & base_points, const AvoidPointArray & added_points) const;

  // shift point generation: merger
  AvoidPointArray mergeShiftPoints(
    const AvoidPointArray & raw_shift_points, DebugData & debug) const;
  void generateTotalShiftLine(
    const AvoidPointArray & avoid_points, ShiftLineData & shift_line_data) const;
  AvoidPointArray extractShiftPointsFromLine(ShiftLineData & shift_line_data) const;
  std::vector<size_t> calcParentIds(
    const AvoidPointArray & parent_candidates, const AvoidPoint & child) const;

  // shift point generation: trimmers
  AvoidPointArray trimShiftPoint(const AvoidPointArray & shift_points, DebugData & debug) const;
  void quantizeShiftPoint(AvoidPointArray & shift_points, const double interval) const;
  void trimSmallShiftPoint(AvoidPointArray & shift_points, const double shift_diff_thres) const;
  void trimSimilarGradShiftPoint(AvoidPointArray & shift_points, const double threshold) const;
  void trimMomentaryReturn(AvoidPointArray & shift_points) const;
  void trimTooSharpShift(AvoidPointArray & shift_points) const;
  void trimSharpReturn(AvoidPointArray & shift_points) const;

  // shift point generation: return-shift generator
  void addReturnShiftPointFromEgo(
    AvoidPointArray & sp_candidates, AvoidPointArray & current_raw_shift_points) const;

  // -- for shift point operations --
  void alignShiftPointsOrder(
    AvoidPointArray & shift_points, const bool recalc_start_length = true) const;
  AvoidPointArray fillAdditionalInfo(const AvoidPointArray & shift_points) const;
  AvoidPoint fillAdditionalInfo(const AvoidPoint & shift_point) const;
  void fillAdditionalInfoFromPoint(AvoidPointArray & shift_points) const;
  void fillAdditionalInfoFromLongitudinal(AvoidPointArray & shift_points) const;

  // -- for new shift point approval --
  boost::optional<AvoidPointArray> findNewShiftPoint(
    const AvoidPointArray & shift_points, const PathShifter & shifter) const;
  void addShiftPointIfApproved(const AvoidPointArray & point);
  void addNewShiftPoints(PathShifter & path_shifter, const AvoidPointArray & shift_points) const;

  // -- path generation --
  ShiftedPath generateAvoidancePath(PathShifter & shifter) const;
  void generateExtendedDrivableArea(ShiftedPath * shifted_path) const;

  // -- velocity planning --
  void modifyPathVelocityToPreventAccelerationOnAvoidance(ShiftedPath & shifted_path) const;

  // clean up shifter
  void postProcess(PathShifter & shifter) const;

  // turn signal
  TurnSignalInfo calcTurnSignalInfo(const ShiftedPath & path) const;

  // intersection (old)
  boost::optional<AvoidPoint> calcIntersectionShiftPoint(const AvoidancePlanningData & data) const;

  bool isTargetObjectType(const PredictedObject & object) const;

  // debug
  mutable DebugData debug_data_;
  void setDebugData(const PathShifter & shifter, const DebugData & debug);
  void updateAvoidanceDebugData(std::vector<AvoidanceDebugMsg> & avoidance_debug_msg_array) const;
  mutable std::vector<AvoidanceDebugMsg> debug_avoidance_initializer_for_shift_point_;
  mutable rclcpp::Time debug_avoidance_initializer_for_shift_point_time_;
  // =====================================
  // ========= helper functions ==========
  // =====================================

  PathWithLaneId calcCenterLinePath(
    const std::shared_ptr<const PlannerData> & planner_data, const PoseStamped & pose) const;

  void clipPathLength(PathWithLaneId & path) const;

  // TODO(Horibe): think later.
  // for unique ID
  mutable uint64_t original_unique_id = 0;  // TODO(Horibe) remove mutable
  uint64_t getOriginalShiftPointUniqueId() const { return original_unique_id++; }

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
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__AVOIDANCE__AVOIDANCE_MODULE_HPP_
