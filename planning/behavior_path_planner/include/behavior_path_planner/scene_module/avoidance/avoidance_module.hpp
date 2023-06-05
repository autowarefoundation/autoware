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

#include "behavior_path_planner/scene_module/scene_module_interface.hpp"
#include "behavior_path_planner/scene_module/scene_module_visitor.hpp"
#include "behavior_path_planner/utils/avoidance/avoidance_module_data.hpp"
#include "behavior_path_planner/utils/avoidance/helper.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <tier4_planning_msgs/msg/avoidance_debug_msg.hpp>
#include <tier4_planning_msgs/msg/avoidance_debug_msg_array.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace behavior_path_planner
{

using motion_utils::calcSignedArcLength;
using motion_utils::findNearestIndex;

using tier4_planning_msgs::msg::AvoidanceDebugMsg;

class AvoidanceModule : public SceneModuleInterface
{
public:
#ifdef USE_OLD_ARCHITECTURE
  AvoidanceModule(
    const std::string & name, rclcpp::Node & node, std::shared_ptr<AvoidanceParameters> parameters);
#else
  AvoidanceModule(
    const std::string & name, rclcpp::Node & node, std::shared_ptr<AvoidanceParameters> parameters,
    const std::unordered_map<std::string, std::shared_ptr<RTCInterface> > & rtc_interface_ptr_map);
#endif

  ModuleStatus updateState() override;
  ModuleStatus getNodeStatusWhileWaitingApproval() const override { return ModuleStatus::SUCCESS; }
  BehaviorModuleOutput plan() override;
  CandidateOutput planCandidate() const override;
  BehaviorModuleOutput planWaitingApproval() override;
  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  void processOnEntry() override;
  void processOnExit() override;
  void updateData() override;
  void acceptVisitor(const std::shared_ptr<SceneModuleVisitor> & visitor) const override;

#ifndef USE_OLD_ARCHITECTURE
  void updateModuleParams(const std::shared_ptr<AvoidanceParameters> & parameters)
  {
    parameters_ = parameters;
  }
#endif
  std::shared_ptr<AvoidanceDebugMsgArray> get_debug_msg_array() const;

private:
  /**
   * @brief update RTC status for candidate shift line.
   * @param candidate path.
   */
  void updateCandidateRTCStatus(const CandidateOutput & candidate)
  {
    if (candidate.lateral_shift > 0.0) {
      rtc_interface_ptr_map_.at("left")->updateCooperateStatus(
        uuid_map_.at("left"), isExecutionReady(), candidate.start_distance_to_path_change,
        candidate.finish_distance_to_path_change, clock_->now());
      candidate_uuid_ = uuid_map_.at("left");
      return;
    }
    if (candidate.lateral_shift < 0.0) {
      rtc_interface_ptr_map_.at("right")->updateCooperateStatus(
        uuid_map_.at("right"), isExecutionReady(), candidate.start_distance_to_path_change,
        candidate.finish_distance_to_path_change, clock_->now());
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
    const Point ego_position = planner_data_->self_odometry->pose.pose.position;

    for (const auto & left_shift : left_shift_array_) {
      const double start_distance =
        calcSignedArcLength(path.points, ego_position, left_shift.start_pose.position);
      const double finish_distance =
        calcSignedArcLength(path.points, ego_position, left_shift.finish_pose.position);
      rtc_interface_ptr_map_.at("left")->updateCooperateStatus(
        left_shift.uuid, true, start_distance, finish_distance, clock_->now());
      if (finish_distance > -1.0e-03) {
        steering_factor_interface_ptr_->updateSteeringFactor(
          {left_shift.start_pose, left_shift.finish_pose}, {start_distance, finish_distance},
          SteeringFactor::AVOIDANCE_PATH_CHANGE, SteeringFactor::LEFT, SteeringFactor::TURNING, "");
      }
    }

    for (const auto & right_shift : right_shift_array_) {
      const double start_distance =
        calcSignedArcLength(path.points, ego_position, right_shift.start_pose.position);
      const double finish_distance =
        calcSignedArcLength(path.points, ego_position, right_shift.finish_pose.position);
      rtc_interface_ptr_map_.at("right")->updateCooperateStatus(
        right_shift.uuid, true, start_distance, finish_distance, clock_->now());
      if (finish_distance > -1.0e-03) {
        steering_factor_interface_ptr_->updateSteeringFactor(
          {right_shift.start_pose, right_shift.finish_pose}, {start_distance, finish_distance},
          SteeringFactor::AVOIDANCE_PATH_CHANGE, SteeringFactor::RIGHT, SteeringFactor::TURNING,
          "");
      }
    }
  }

  /**
   * @brief remove RTC status for candidate path.
   */
  void removeCandidateRTCStatus()
  {
    if (rtc_interface_ptr_map_.at("left")->isRegistered(candidate_uuid_)) {
      rtc_interface_ptr_map_.at("left")->removeCooperateStatus(candidate_uuid_);
    } else if (rtc_interface_ptr_map_.at("right")->isRegistered(candidate_uuid_)) {
      rtc_interface_ptr_map_.at("right")->removeCooperateStatus(candidate_uuid_);
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

  // ego state check

  /**
   * @brief update ego status based on avoidance path and surround condition.
   * @param ego status. (NOT_AVOID, AVOID, YIELD, AVOID_EXECUTE, AVOID_PATH_NOT_READY)
   */
  AvoidanceState updateEgoState(const AvoidancePlanningData & data) const;

  /**
   * @brief check whether the ego is shifted based on shift line.
   * @return result.
   */
  bool isAvoidanceManeuverRunning();

  /**
   * @brief check whether the ego is in avoidance maneuver based on shift line and target object
   * existence.
   * @return result.
   */
  bool isAvoidancePlanRunning() const;

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
   * @brief insert stop point in output path.
   * @param flag. if it is true, the ego decelerates within accel/jerk constraints.
   * @param target path.
   */
  void insertPrepareVelocity(const bool avoidable, ShiftedPath & shifted_path) const;

  /**
   * @brief insert decel point in output path in order to yield. the ego decelerates within
   * accel/jerk constraints.
   * @param target path.
   */
  void insertYieldVelocity(ShiftedPath & shifted_path) const;

  // avoidance data preparation

  /**
   * @brief update main avoidance data for avoidance path generation based on latest planner data.
   * @return avoidance data.
   */
  AvoidancePlanningData calcAvoidancePlanningData(DebugData & debug) const;

  /**
   * @brief fill additional data so that the module judges target objects.
   * @return object that has additional data.
   */
  ObjectData createObjectData(
    const AvoidancePlanningData & data, const PredictedObject & object) const;

  /**
   * @brief get objects that are driving on adjacent lanes.
   * @param left or right lanelets.
   */
  ObjectDataArray getAdjacentLaneObjects(const lanelet::ConstLanelets & adjacent_lanes) const;

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
   * @details in this function, three shift line sets are generated.
   * - unapproved raw shift lines.
   * - unapproved new shift lines.
   * - safe new shift lines. (avoidance path is generated from this shift line set.)
   */
  void fillShiftLine(AvoidancePlanningData & data, DebugData & debug) const;

  /**
   * @brief fill debug data.
   * @param avoidance data.
   * @param debug data.
   */
  void fillDebugData(const AvoidancePlanningData & data, DebugData & debug) const;

  /**
   * @brief update registered shift lines.
   * @param current shift lines.
   */
  void registerRawShiftLines(const AvoidLineArray & future_registered);

  /**
   * @brief update path index of the registered objects. remove old objects whose end point is
   * behind ego pose.
   */
  void updateRegisteredRawShiftLines();

  // shift line generation

  /**
   * @brief fill index and longitudinal.
   * @param target shift line.
   * @return processed shift line.
   */
  AvoidLine fillAdditionalInfo(const AvoidLine & shift_line) const;

  /**
   * @brief fill index and longitudinal.
   * @param target shift lines.
   * @return processed shift lines.
   */
  AvoidLineArray fillAdditionalInfo(const AvoidLineArray & shift_lines) const;

  /**
   * @brief Calculate the shift points (start/end point, shift length) from the object lateral
   * and longitudinal positions in the Frenet coordinate. The jerk limit is also considered here.
   * @param avoidance data.
   * @param debug data.
   * @return processed shift lines.
   */
  AvoidLineArray calcRawShiftLinesFromObjects(
    AvoidancePlanningData & data, DebugData & debug) const;

  /**
   * @brief clean up raw shift lines.
   * @param target shift lines.
   * @param debug data.
   * @return processed shift lines.
   * @details process flow:
   * 1. combine raw shirt lines and previous registered shift lines.
   * 2. add return shift line.
   * 3. merge raw shirt lines.
   * 4. trim unnecessary shift lines.
   */
  AvoidLineArray applyPreProcessToRawShiftLines(
    AvoidLineArray & current_raw_shift_points, DebugData & debug) const;

  /*
   * @brief Combine points A into B. If shift_line of A which has same object_id and
   * similar shape is already in B, it will not be added into B.
   * @param original shift lines.
   * @param new shift lines.
   * @return processed shift lines.
   */
  AvoidLineArray combineRawShiftLinesWithUniqueCheck(
    const AvoidLineArray & base_lines, const AvoidLineArray & added_lines) const;

  /*
   * @brief merge negative & positive shift lines.
   * @param original shift lines.
   * @param debug data.
   * @return processed shift lines.
   */
  AvoidLineArray mergeShiftLines(const AvoidLineArray & raw_shift_lines, DebugData & debug) const;

  /*
   * @brief extract shift lines from total shift lines based on their gradient.
   * @param shift length data.
   * @return extracted shift lines.
   */
  AvoidLineArray extractShiftLinesFromLine(ShiftLineData & shift_line_data) const;

  /*
   * @brief remove unnecessary avoid points
   * @param original shift lines.
   * @param debug data.
   * @return processed shift lines.
   * @details
   * - Combine avoid points that have almost same gradient
   * - Quantize the shift length to reduce the shift point noise
   * - Change the shift length to the previous one if the deviation is small.
   * - Remove unnecessary return shift (back to the center line).
   */
  AvoidLineArray trimShiftLine(const AvoidLineArray & shift_lines, DebugData & debug) const;

  /*
   * @brief extract new shift lines based on current shifted path. the module makes a RTC request
   * for those new shift lines.
   * @param candidate shift lines.
   * @return new shift lines.
   */
  AvoidLineArray findNewShiftLine(const AvoidLineArray & shift_lines) const;

  /*
   * @brief calculate parent ids.
   * @param parent shift lines.
   * @param child shift lines.
   * @return parent ids.
   */
  std::vector<size_t> calcParentIds(
    const AvoidLineArray & parent_candidates, const AvoidLine & child) const;

  /*
   * @brief add return shift line from ego position.
   * @param shift lines which the return shift is added.
   * @param current shift lines.
   * Pick up the last shift point, which is the most farthest from ego, from the current candidate
   * avoidance points and registered points in the shifter. If the last shift length of the point is
   * non-zero, add a return-shift to center line from the point. If there is no shift point in
   * candidate avoidance points nor registered points, and base_shift > 0, add a return-shift to
   * center line from ego.
   */
  void addReturnShiftLineFromEgo(
    AvoidLineArray & sl_candidates, AvoidLineArray & current_raw_shift_lines) const;

  /*
   * @brief generate total shift line. total shift line has shift length and gradient array.
   * @param raw shift lines.
   * @param total shift line.
   */
  void generateTotalShiftLine(
    const AvoidLineArray & avoid_points, ShiftLineData & shift_line_data) const;

  /*
   * @brief quantize shift line length.
   * @param target shift lines.
   * @param threshold. shift length is quantized by this value. (if it is 0.3[m], the output shift
   * length is 0.0, 0.3, 0.6...)
   */
  void quantizeShiftLine(AvoidLineArray & shift_lines, const double threshold) const;

  /*
   * @brief trim shift line whose relative longitudinal distance is less than threshold.
   * @param target shift lines.
   * @param threshold.
   */
  void trimSmallShiftLine(AvoidLineArray & shift_lines, const double threshold) const;

  /*
   * @brief merge multiple shift lines whose relative gradient is less than threshold.
   * @param target shift lines.
   * @param threshold.
   */
  void trimSimilarGradShiftLine(AvoidLineArray & shift_lines, const double threshold) const;

  /**
   * @brief remove short "return to center" shift point. ¯¯\_/¯¯　-> ¯¯¯¯¯¯
   * @param input shift lines.
   * @details
   * Is the shift point for "return to center"?
   *  - no : Do not trim anything.
   *  - yes: Is it short distance enough to be removed?
   *     - no : Do not trim anything.
   *     - yes: Remove the "return" shift point.
   *            Recalculate longitudinal distance and modify the shift point.
   */
  void trimMomentaryReturn(AvoidLineArray & shift_lines) const;

  /*
   * @brief trim invalid shift lines whose gradient it too large to follow.
   * @param target shift lines.
   */
  void trimTooSharpShift(AvoidLineArray & shift_lines) const;

  /*
   * @brief trim invalid shift lines whose gradient it too large to follow.
   * @param target shift lines.
   */
  void trimSharpReturn(AvoidLineArray & shift_lines, const double threshold) const;

  /*
   * @brief sort shift line order based on their end longitudinal distance.
   * @param target shift lines.
   * @param re-calculate shift line start length from previous one's. (optional)
   */
  void alignShiftLinesOrder(
    AvoidLineArray & shift_lines, const bool recalculate_start_length = true) const;

  /**
   * @brief fill index and longitudinal.
   * @param target shift line.
   */
  void fillAdditionalInfoFromPoint(AvoidLineArray & shift_lines) const;

  /**
   * @brief fill index and pose.
   * @param target shift line.
   */
  void fillAdditionalInfoFromLongitudinal(AvoidLineArray & shift_lines) const;

  /**
   * @brief add new shift line to path shifter if the RTC status is activated.
   * @param new shift lines.
   */
  void addShiftLineIfApproved(const AvoidLineArray & point);

  /**
   * @brief add new shift line to path shifter.
   * @param new shift lines.
   */
  void addNewShiftLines(PathShifter & path_shifter, const AvoidLineArray & shift_lines) const;

  /**
   * @brief validate shift lines.
   * @param new shift lines.
   * @param path shifter.
   * @return result. if there is huge gap between the ego position and candidate path, return false.
   */
  bool isValidShiftLine(const AvoidLineArray & shift_lines, const PathShifter & shifter) const;

  // generate output data

  /**
   * @brief generate avoidance path from path shifter.
   * @return avoidance path.
   */
  ShiftedPath generateAvoidancePath(PathShifter & shifter) const;

  /**
   * @brief calculate turn signal infomation.
   * @param avoidance path.
   * @return turn signal command.
   */
  TurnSignalInfo calcTurnSignalInfo(const ShiftedPath & path) const;

  // TODO(murooka) judge when and which way to extend drivable area. current implementation is keep
  // extending during avoidance module
  // TODO(murooka) freespace during turning in intersection where there is no neighbor lanes
  // NOTE: Assume that there is no situation where there is an object in the middle lane of more
  // than two lanes since which way to avoid is not obvious
  void generateExtendedDrivableArea(BehaviorModuleOutput & output) const;

  /**
   * @brief insert slow down point to prevent acceleration in avoidance maneuver.
   * @param avoidance path.
   */
  void modifyPathVelocityToPreventAccelerationOnAvoidance(ShiftedPath & shifted_path);

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
   * @brief get shift side adjacent lanes.
   * @param path shifter.
   * @param forward distance to get.
   * @param backward distance to get.
   * @return adjacent lanes.
   */
  lanelet::ConstLanelets getAdjacentLane(
    const PathShifter & path_shifter, const double forward_distance,
    const double backward_distance) const;

  /**
   * @brief calculate lateral margin from avoidance velocity for safety check.
   * @param target velocity.
   */
  double getLateralMarginFromVelocity(const double velocity) const;

  /**
   * @brief calculate rss longitudinal distance for safety check.
   * @param ego velocity.
   * @param object velocity.
   * @param option for rss calculation.
   * @return rss longitudinal distance.
   */
  double getRSSLongitudinalDistance(
    const double v_ego, const double v_obj, const bool is_front_object) const;

  /**
   * @brief check avoidance path safety for surround moving objects.
   * @param path shifter.
   * @param avoidance path.
   * @param debug data.
   * @return result.
   */
  bool isSafePath(
    const PathShifter & path_shifter, ShiftedPath & shifted_path, DebugData & debug) const;

  /**
   * @brief check avoidance path safety for surround moving objects.
   * @param avoidance path.
   * @param shift side lanes.
   * @param debug data.
   * @return result.
   */
  bool isSafePath(
    const PathWithLaneId & path, const lanelet::ConstLanelets & check_lanes,
    DebugData & debug) const;

  /**
   * @brief check predicted points safety.
   * @param predicted points.
   * @param future time.
   * @param object data.
   * @param margin data for debug.
   * @return result.
   */
  bool isEnoughMargin(
    const PathPointWithLaneId & p_ego, const double t, const ObjectData & object,
    MarginData & margin_data) const;

  // post process

  /**
   * @brief extend backward length so that path shift inserts behind shift lines.
   * @param current output path.
   * @return extended path.
   */
  PathWithLaneId extendBackwardLength(const PathWithLaneId & original_path) const;

  /**
   * @brief reset registered shift lines.
   * @param path shifter.
   */
  void removeAllRegisteredShiftPoints(PathShifter & path_shifter)
  {
    current_raw_shift_lines_.clear();
    registered_raw_shift_lines_.clear();
    path_shifter.setShiftLines(ShiftLineArray{});
  }

  /**
   * @brief remove behind shift lines.
   * @param path shifter.
   */
  void postProcess(PathShifter & path_shifter) const
  {
    const size_t nearest_idx = planner_data_->findEgoIndex(path_shifter.getReferencePath().points);
    path_shifter.removeBehindShiftLineAndSetBaseOffset(nearest_idx);
  }

  // misc functions

  double getCurrentBaseShift() const { return path_shifter_.getBaseOffset(); }

  // TODO(Horibe): think later.
  // for unique ID
  mutable uint64_t original_unique_id = 0;  // TODO(Horibe) remove mutable
  uint64_t getOriginalShiftLineUniqueId() const { return original_unique_id++; }

  struct RegisteredShiftLine
  {
    UUID uuid;
    Pose start_pose;
    Pose finish_pose;
  };

  using RegisteredShiftLineArray = std::vector<RegisteredShiftLine>;

  bool is_avoidance_maneuver_starts;

  std::shared_ptr<AvoidanceParameters> parameters_;

  std::shared_ptr<double> ego_velocity_starting_avoidance_ptr_;

  helper::avoidance::AvoidanceHelper helper_;

  AvoidancePlanningData avoidance_data_;

  PathShifter path_shifter_;

  RegisteredShiftLineArray left_shift_array_;

  RegisteredShiftLineArray right_shift_array_;

  AvoidLineArray registered_raw_shift_lines_;

  AvoidLineArray current_raw_shift_lines_;

  UUID candidate_uuid_;

  ObjectDataArray registered_objects_;

  mutable ObjectDataArray stopped_objects_;

  mutable DebugData debug_data_;

  mutable std::shared_ptr<AvoidanceDebugMsgArray> debug_msg_ptr_;

  mutable std::vector<AvoidanceDebugMsg> debug_avoidance_initializer_for_shift_line_;

  mutable rclcpp::Time debug_avoidance_initializer_for_shift_line_time_;
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__AVOIDANCE__AVOIDANCE_MODULE_HPP_
