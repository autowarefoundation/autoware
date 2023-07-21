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

#include "behavior_path_planner/scene_module/avoidance/avoidance_module.hpp"

#include "behavior_path_planner/marker_util/avoidance/debug.hpp"
#include "behavior_path_planner/scene_module/scene_module_visitor.hpp"
#include "behavior_path_planner/utils/avoidance/utils.hpp"
#include "behavior_path_planner/utils/path_utils.hpp"
#include "behavior_path_planner/utils/utils.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <tier4_planning_msgs/msg/avoidance_debug_factor.hpp>
#include <tier4_planning_msgs/msg/avoidance_debug_msg.hpp>
#include <tier4_planning_msgs/msg/avoidance_debug_msg_array.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <vector>

// set as macro so that calling function name will be printed.
// debug print is heavy. turn on only when debugging.
#define DEBUG_PRINT(...) \
  RCLCPP_DEBUG_EXPRESSION(getLogger(), parameters_->print_debug_info, __VA_ARGS__)
#define printShiftLines(p, msg) DEBUG_PRINT("[%s] %s", msg, toStrInfo(p).c_str())

namespace behavior_path_planner
{

using motion_utils::calcLongitudinalOffsetPose;
using motion_utils::calcSignedArcLength;
using motion_utils::findNearestIndex;
using motion_utils::findNearestSegmentIndex;
using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::calcInterpolatedPose;
using tier4_autoware_utils::calcLateralDeviation;
using tier4_autoware_utils::calcLongitudinalDeviation;
using tier4_autoware_utils::calcYawDeviation;
using tier4_autoware_utils::getPoint;
using tier4_autoware_utils::getPose;
using tier4_autoware_utils::toHexString;
using tier4_planning_msgs::msg::AvoidanceDebugFactor;

namespace
{
bool isEndPointsConnected(
  const lanelet::ConstLanelet & left_lane, const lanelet::ConstLanelet & right_lane)
{
  const auto & left_back_point_2d = right_lane.leftBound2d().back().basicPoint();
  const auto & right_back_point_2d = left_lane.rightBound2d().back().basicPoint();

  constexpr double epsilon = 1e-5;
  return (right_back_point_2d - left_back_point_2d).norm() < epsilon;
}

template <typename T>
void pushUniqueVector(T & base_vector, const T & additional_vector)
{
  base_vector.insert(base_vector.end(), additional_vector.begin(), additional_vector.end());
}

bool isDrivingSameLane(
  const lanelet::ConstLanelets & previous_lanes, const lanelet::ConstLanelets & current_lanes)
{
  std::multiset<lanelet::Id> prev_ids{};
  std::multiset<lanelet::Id> curr_ids{};
  std::multiset<lanelet::Id> same_ids{};

  std::for_each(
    previous_lanes.begin(), previous_lanes.end(), [&](const auto & l) { prev_ids.insert(l.id()); });
  std::for_each(
    current_lanes.begin(), current_lanes.end(), [&](const auto & l) { curr_ids.insert(l.id()); });

  std::set_intersection(
    prev_ids.begin(), prev_ids.end(), curr_ids.begin(), curr_ids.end(),
    std::inserter(same_ids, same_ids.end()));

  return !same_ids.empty();
}
}  // namespace

AvoidanceModule::AvoidanceModule(
  const std::string & name, rclcpp::Node & node, std::shared_ptr<AvoidanceParameters> parameters,
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map)
: SceneModuleInterface{name, node, rtc_interface_ptr_map},
  parameters_{parameters},
  helper_{parameters}
{
}

bool AvoidanceModule::isExecutionRequested() const
{
  DEBUG_PRINT("AVOIDANCE isExecutionRequested");

  if (current_state_ == ModuleStatus::RUNNING) {
    return true;
  }

  // Check ego is in preferred lane
  updateInfoMarker(avoidance_data_);
  updateDebugMarker(avoidance_data_, path_shifter_, debug_data_);

  // there is object that should be avoid. return true.
  if (!!avoidance_data_.stop_target_object) {
    return true;
  }

  if (avoidance_data_.unapproved_new_sl.empty()) {
    return false;
  }

  return !avoidance_data_.target_objects.empty();
}

bool AvoidanceModule::isExecutionReady() const
{
  DEBUG_PRINT("AVOIDANCE isExecutionReady");
  return avoidance_data_.safe;
}

ModuleStatus AvoidanceModule::updateState()
{
  const auto & data = avoidance_data_;
  const auto is_plan_running = isAvoidancePlanRunning();
  const bool has_avoidance_target = !data.target_objects.empty();

  if (!isDrivingSameLane(helper_.getPreviousDrivingLanes(), data.current_lanelets)) {
    RCLCPP_WARN_THROTTLE(getLogger(), *clock_, 500, "previous module lane is updated.");
    return ModuleStatus::SUCCESS;
  }

  const auto idx = planner_data_->findEgoIndex(data.reference_path.points);
  if (idx == data.reference_path.points.size() - 1) {
    arrived_path_end_ = true;
  }

  constexpr double THRESHOLD = 1.0;
  if (
    calcDistance2d(getEgoPose(), getPose(data.reference_path.points.back())) > THRESHOLD &&
    arrived_path_end_) {
    RCLCPP_WARN_THROTTLE(getLogger(), *clock_, 500, "reach path end point. exit avoidance module.");
    return ModuleStatus::SUCCESS;
  }

  DEBUG_PRINT(
    "is_plan_running = %d, has_avoidance_target = %d", is_plan_running, has_avoidance_target);

  if (!is_plan_running && !has_avoidance_target) {
    return ModuleStatus::SUCCESS;
  }

  if (
    !has_avoidance_target && parameters_->enable_update_path_when_object_is_gone &&
    !isAvoidanceManeuverRunning()) {
    // if dynamic objects are removed on path, change current state to reset path
    return ModuleStatus::SUCCESS;
  }

  helper_.setPreviousDrivingLanes(data.current_lanelets);

  if (is_plan_running || current_state_ == ModuleStatus::RUNNING) {
    return ModuleStatus::RUNNING;
  }
  return ModuleStatus::IDLE;
}

bool AvoidanceModule::isAvoidancePlanRunning() const
{
  constexpr double AVOIDING_SHIFT_THR = 0.1;
  const bool has_base_offset = std::abs(path_shifter_.getBaseOffset()) > AVOIDING_SHIFT_THR;
  const bool has_shift_point = (path_shifter_.getShiftLinesSize() > 0);
  return has_base_offset || has_shift_point;
}
bool AvoidanceModule::isAvoidanceManeuverRunning()
{
  const auto path_idx = avoidance_data_.ego_closest_path_index;

  for (const auto & al : registered_raw_shift_lines_) {
    if (path_idx > al.start_idx || is_avoidance_maneuver_starts) {
      is_avoidance_maneuver_starts = true;
      return true;
    }
  }
  return false;
}

AvoidancePlanningData AvoidanceModule::calcAvoidancePlanningData(DebugData & debug) const
{
  AvoidancePlanningData data;

  // reference pose
  const auto reference_pose =
    utils::getUnshiftedEgoPose(getEgoPose(), helper_.getPreviousSplineShiftPath());
  data.reference_pose = reference_pose;

  // special for avoidance: take behind distance upt ot shift-start-point if it exist.
  const auto longest_dist_to_shift_line = [&]() {
    double max_dist = 0.0;
    for (const auto & pnt : path_shifter_.getShiftLines()) {
      max_dist = std::max(max_dist, calcDistance2d(getEgoPose(), pnt.start));
    }
    for (const auto & sl : registered_raw_shift_lines_) {
      max_dist = std::max(max_dist, calcDistance2d(getEgoPose(), sl.start));
    }
    return max_dist;
  }();

  // center line path (output of this function must have size > 1)
  const auto center_path = utils::calcCenterLinePath(
    planner_data_, reference_pose, longest_dist_to_shift_line,
    *getPreviousModuleOutput().reference_path);

  debug.center_line = center_path;
  if (center_path.points.size() < 2) {
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 5000, "calcCenterLinePath() must return path which size > 1");
    return data;
  }

  // reference path
  data.reference_path_rough = extendBackwardLength(*getPreviousModuleOutput().path);

  data.reference_path = utils::resamplePathWithSpline(
    data.reference_path_rough, parameters_->resample_interval_for_planning);

  if (data.reference_path.points.size() < 2) {
    // if the resampled path has only 1 point, use original path.
    data.reference_path = center_path;
  }

  const size_t nearest_segment_index =
    findNearestSegmentIndex(data.reference_path.points, data.reference_pose.position);
  data.ego_closest_path_index =
    std::min(nearest_segment_index + 1, data.reference_path.points.size() - 1);

  // arclength from ego pose (used in many functions)
  data.arclength_from_ego = utils::calcPathArcLengthArray(
    data.reference_path, 0, data.reference_path.points.size(),
    calcSignedArcLength(data.reference_path.points, getEgoPosition(), 0));

  // lanelet info
  data.current_lanelets = utils::avoidance::getCurrentLanesFromPath(
    *getPreviousModuleOutput().reference_path, planner_data_);

  // keep avoidance state
  data.state = avoidance_data_.state;

  // target objects for avoidance
  fillAvoidanceTargetObjects(data, debug);

  DEBUG_PRINT("target object size = %lu", data.target_objects.size());

  return data;
}

void AvoidanceModule::fillAvoidanceTargetObjects(
  AvoidancePlanningData & data, DebugData & debug) const
{
  using utils::avoidance::fillObjectStoppableJudge;
  using utils::avoidance::filterTargetObjects;
  using utils::avoidance::getTargetLanelets;

  // Separate dynamic objects based on whether they are inside or outside of the expanded lanelets.
  const auto expanded_lanelets = getTargetLanelets(
    planner_data_, data.current_lanelets, parameters_->detection_area_left_expand_dist,
    parameters_->detection_area_right_expand_dist * (-1.0));

  const auto [object_within_target_lane, object_outside_target_lane] =
    utils::separateObjectsByLanelets(*planner_data_->dynamic_object, expanded_lanelets);

  for (const auto & object : object_outside_target_lane.objects) {
    ObjectData other_object;
    other_object.object = object;
    other_object.reason = "OutOfTargetArea";
    data.other_objects.push_back(other_object);
  }

  ObjectDataArray objects;
  for (const auto & object : object_within_target_lane.objects) {
    objects.push_back(createObjectData(data, object));
  }

  // Filter out the objects to determine the ones to be avoided.
  filterTargetObjects(objects, data, debug, planner_data_, parameters_);

  // Calculate the distance needed to safely decelerate the ego vehicle to a stop line.
  const auto feasible_stop_distance = helper_.getFeasibleDecelDistance(0.0, false);
  std::for_each(data.target_objects.begin(), data.target_objects.end(), [&, this](auto & o) {
    o.to_stop_line = calcDistanceToStopLine(o);
    fillObjectStoppableJudge(o, registered_objects_, feasible_stop_distance, parameters_);
  });

  // debug
  {
    debug.current_lanelets = std::make_shared<lanelet::ConstLanelets>(data.current_lanelets);
    debug.expanded_lanelets = std::make_shared<lanelet::ConstLanelets>(expanded_lanelets);

    std::vector<AvoidanceDebugMsg> debug_info_array;
    const auto append = [&](const auto & o) {
      AvoidanceDebugMsg debug_info;
      debug_info.object_id = toHexString(o.object.object_id);
      debug_info.longitudinal_distance = o.longitudinal;
      debug_info.lateral_distance_from_centerline = o.lateral;
      debug_info.allow_avoidance = o.reason == "";
      debug_info.failed_reason = o.reason;
      debug_info_array.push_back(debug_info);
    };

    std::for_each(objects.begin(), objects.end(), [&](const auto & o) { append(o); });

    updateAvoidanceDebugData(debug_info_array);
  }
}

ObjectData AvoidanceModule::createObjectData(
  const AvoidancePlanningData & data, const PredictedObject & object) const
{
  using boost::geometry::return_centroid;

  const auto & path_points = data.reference_path.points;
  const auto & object_pose = object.kinematics.initial_pose_with_covariance.pose;
  const auto object_closest_index = findNearestIndex(path_points, object_pose.position);
  const auto object_closest_pose = path_points.at(object_closest_index).point.pose;
  const auto object_type = utils::getHighestProbLabel(object.classification);
  const auto object_parameter = parameters_->object_parameters.at(object_type);

  ObjectData object_data{};

  object_data.object = object;

  const auto lower = parameters_->lower_distance_for_polygon_expansion;
  const auto upper = parameters_->upper_distance_for_polygon_expansion;
  const auto clamp =
    std::clamp(calcDistance2d(getEgoPose(), object_pose) - lower, 0.0, upper) / upper;
  object_data.distance_factor = object_parameter.max_expand_ratio * clamp + 1.0;

  // Calc envelop polygon.
  utils::avoidance::fillObjectEnvelopePolygon(
    object_data, registered_objects_, object_closest_pose, parameters_);

  // calc object centroid.
  object_data.centroid = return_centroid<Point2d>(object_data.envelope_poly);

  // Calc moving time.
  utils::avoidance::fillObjectMovingTime(object_data, stopped_objects_, parameters_);

  // Calc lateral deviation from path to target object.
  object_data.lateral = calcLateralDeviation(object_closest_pose, object_pose.position);

  // Find the footprint point closest to the path, set to object_data.overhang_distance.
  object_data.overhang_dist = utils::avoidance::calcEnvelopeOverhangDistance(
    object_data, object_closest_pose, object_data.overhang_pose.position);

  // Check whether the the ego should avoid the object.
  const auto & vehicle_width = planner_data_->parameters.vehicle_width;
  utils::avoidance::fillAvoidanceNecessity(
    object_data, registered_objects_, vehicle_width, parameters_);

  return object_data;
}

bool AvoidanceModule::canYieldManeuver(const AvoidancePlanningData & data) const
{
  // transit yield maneuver only when the avoidance maneuver is not initiated.
  if (data.avoiding_now) {
    return false;
  }

  if (!data.stop_target_object) {
    return true;
  }

  constexpr double TH_STOP_SPEED = 0.01;
  constexpr double TH_STOP_POSITION = 0.5;

  const auto stopped_for_the_object =
    getEgoSpeed() < TH_STOP_SPEED && std::abs(data.to_stop_line) < TH_STOP_POSITION;

  const auto id = data.stop_target_object.get().object.object_id;
  const auto same_id_obj = std::find_if(
    ego_stopped_objects_.begin(), ego_stopped_objects_.end(),
    [&id](const auto & o) { return o.object.object_id == id; });

  // if the ego already started avoiding for the object, never transit yield maneuver again.
  if (same_id_obj != ego_stopped_objects_.end()) {
    return stopped_for_the_object;
  }

  // registered objects whom the ego stopped for at the moment of stopping.
  if (stopped_for_the_object) {
    ego_stopped_objects_.push_back(data.stop_target_object.get());
  }

  return true;
}

void AvoidanceModule::fillShiftLine(AvoidancePlanningData & data, DebugData & debug) const
{
  constexpr double AVOIDING_SHIFT_THR = 0.1;
  data.avoiding_now = std::abs(helper_.getEgoShift()) > AVOIDING_SHIFT_THR;

  auto path_shifter = path_shifter_;

  /**
   * STEP 1
   * Create raw shift points from target object. The lateral margin between the ego and the target
   * object varies depending on the relative speed between the ego and the target object.
   */
  data.unapproved_raw_sl = calcRawShiftLinesFromObjects(data, debug);

  /**
   * STEP 2
   * Modify the raw shift points. (Merging, Trimming)
   */
  const auto processed_raw_sp = applyPreProcessToRawShiftLines(data.unapproved_raw_sl, debug);

  /**
   * STEP 3
   * Find new shift point
   */
  const auto new_sp = findNewShiftLine(processed_raw_sp);
  if (isValidShiftLine(new_sp, path_shifter)) {
    data.unapproved_new_sl = new_sp;
  }

  const auto found_new_sl = data.unapproved_new_sl.size() > 0;
  const auto registered = path_shifter.getShiftLines().size() > 0;
  data.found_avoidance_path = found_new_sl || registered;

  /**
   * STEP 4
   * If there are new shift points, these shift points are registered in path_shifter.
   */
  if (!data.unapproved_new_sl.empty()) {
    addNewShiftLines(path_shifter, data.unapproved_new_sl);
  }

  /**
   * STEP 5
   * Generate avoidance path.
   */
  ShiftedPath spline_shift_path = utils::avoidance::toShiftedPath(data.reference_path);
  const auto success_spline_path_generation =
    path_shifter.generate(&spline_shift_path, true, SHIFT_TYPE::SPLINE);
  data.candidate_path = success_spline_path_generation
                          ? spline_shift_path
                          : utils::avoidance::toShiftedPath(data.reference_path);

  /**
   * STEP 6
   * Check avoidance path safety. For each target objects and the objects in adjacent lanes,
   * check that there is a certain amount of margin in the lateral and longitudinal direction.
   */
  data.safe = isSafePath(path_shifter, data.candidate_path, debug);
}

void AvoidanceModule::fillEgoStatus(
  AvoidancePlanningData & data, [[maybe_unused]] DebugData & debug) const
{
  /**
   * Find the nearest object that should be avoid. When the ego follows reference path,
   * if the both of following two conditions are satisfied, the module surely avoid the object.
   * Condition1: there is risk to collide with object without avoidance.
   * Condition2: there is enough space to avoid.
   * In TOO_LARGE_JERK condition, it is possible to avoid object by deceleration even if the flag
   * is_avoidable is FALSE. So, the module inserts stop point for such a object.
   */
  for (const auto & o : data.target_objects) {
    const auto enough_space = o.is_avoidable || o.reason == AvoidanceDebugFactor::TOO_LARGE_JERK;
    if (o.avoid_required && enough_space) {
      data.avoid_required = true;
      data.stop_target_object = o;
      data.to_stop_line = o.to_stop_line;
      break;
    }
  }

  const auto can_yield_maneuver = canYieldManeuver(data);

  const size_t ego_seg_idx =
    planner_data_->findEgoSegmentIndex(helper_.getPreviousSplineShiftPath().path.points);
  const auto offset = std::abs(motion_utils::calcLateralOffset(
    helper_.getPreviousSplineShiftPath().path.points, getEgoPosition(), ego_seg_idx));

  // don't output new candidate path if the offset between the ego and previous output path is
  // larger than threshold.
  // TODO(Satoshi OTA): remove this workaround
  if (offset > parameters_->safety_check_ego_offset) {
    data.safe_new_sl.clear();
    data.candidate_path = helper_.getPreviousSplineShiftPath();
    RCLCPP_WARN_THROTTLE(getLogger(), *clock_, 500, "unsafe. canceling candidate path...");
    return;
  }

  /**
   * If the avoidance path is safe, use unapproved_new_sl for avoidance path generation.
   */
  if (data.safe) {
    data.yield_required = false;
    data.safe_new_sl = data.unapproved_new_sl;
    return;
  }

  /**
   * If the yield maneuver is disabled, use unapproved_new_sl for avoidance path generation even if
   * the shift line is unsafe.
   */
  if (!parameters_->enable_yield_maneuver) {
    data.yield_required = false;
    data.safe_new_sl = data.unapproved_new_sl;
    return;
  }

  /**
   * TODO(Satoshi OTA) Think yield maneuver in the middle of avoidance.
   * Even if it is determined that a yield is necessary, the yield maneuver is not executed
   * if the avoidance has already been initiated.
   */
  if (!can_yield_maneuver) {
    data.safe = true;  // overwrite safety judge.
    data.yield_required = false;
    data.safe_new_sl = data.unapproved_new_sl;
    RCLCPP_WARN_THROTTLE(getLogger(), *clock_, 500, "unsafe. but could not transit yield status.");
    return;
  }

  /**
   * Transit yield maneuver. Clear shift lines and output yield path.
   */
  {
    data.yield_required = true;
    data.safe_new_sl = data.unapproved_new_sl;
  }

  /**
   * Even if data.avoid_required is false, the module cancels registered shift point when the
   * approved avoidance path is not safe.
   */
  const auto approved_path_exist = !path_shifter_.getShiftLines().empty();
  if (approved_path_exist) {
    RCLCPP_WARN_THROTTLE(getLogger(), *clock_, 5000, "unsafe. canceling approved path...");
    return;
  }

  /**
   * If the avoidance path is not safe in situation where the ego should avoid object, the ego
   * stops in front of the front object with the necessary distance to avoid the object.
   */
  {
    RCLCPP_WARN_THROTTLE(getLogger(), *clock_, 5000, "unsafe. transit yield maneuver...");
  }
}

void AvoidanceModule::fillDebugData(const AvoidancePlanningData & data, DebugData & debug) const
{
  debug.output_shift = data.candidate_path.shift_length;
  debug.current_raw_shift = data.unapproved_raw_sl;
  debug.new_shift_lines = data.unapproved_new_sl;

  if (!data.stop_target_object) {
    return;
  }

  if (data.avoiding_now) {
    return;
  }

  if (data.unapproved_new_sl.empty()) {
    return;
  }

  const auto o_front = data.stop_target_object.get();
  const auto object_type = utils::getHighestProbLabel(o_front.object.classification);
  const auto object_parameter = parameters_->object_parameters.at(object_type);
  const auto & base_link2front = planner_data_->parameters.base_link2front;
  const auto & vehicle_width = planner_data_->parameters.vehicle_width;

  const auto max_avoid_margin = object_parameter.safety_buffer_lateral * o_front.distance_factor +
                                object_parameter.avoid_margin_lateral + 0.5 * vehicle_width;

  const auto variable = helper_.getSharpAvoidanceDistance(
    helper_.getShiftLength(o_front, utils::avoidance::isOnRight(o_front), max_avoid_margin));
  const auto constant = helper_.getNominalPrepareDistance() +
                        object_parameter.safety_buffer_longitudinal + base_link2front;
  const auto total_avoid_distance = variable + constant;

  dead_pose_ = calcLongitudinalOffsetPose(
    data.reference_path.points, getEgoPosition(), o_front.longitudinal - total_avoid_distance);

  if (!dead_pose_) {
    dead_pose_ = getPose(data.reference_path.points.front());
  }
}

AvoidanceState AvoidanceModule::updateEgoState(const AvoidancePlanningData & data) const
{
  if (data.yield_required) {
    return AvoidanceState::YIELD;
  }

  if (!data.avoid_required) {
    return AvoidanceState::NOT_AVOID;
  }

  if (!data.found_avoidance_path) {
    return AvoidanceState::AVOID_PATH_NOT_READY;
  }

  if (isWaitingApproval() && path_shifter_.getShiftLines().empty()) {
    return AvoidanceState::AVOID_PATH_READY;
  }

  return AvoidanceState::AVOID_EXECUTE;
}

void AvoidanceModule::updateEgoBehavior(const AvoidancePlanningData & data, ShiftedPath & path)
{
  if (parameters_->disable_path_update) {
    return;
  }

  insertPrepareVelocity(path);

  switch (data.state) {
    case AvoidanceState::NOT_AVOID: {
      break;
    }
    case AvoidanceState::YIELD: {
      insertYieldVelocity(path);
      insertWaitPoint(parameters_->use_constraints_for_decel, path);
      removeRegisteredShiftLines();
      break;
    }
    case AvoidanceState::AVOID_PATH_NOT_READY: {
      insertWaitPoint(parameters_->use_constraints_for_decel, path);
      break;
    }
    case AvoidanceState::AVOID_PATH_READY: {
      insertWaitPoint(parameters_->use_constraints_for_decel, path);
      break;
    }
    case AvoidanceState::AVOID_EXECUTE: {
      insertStopPoint(parameters_->use_constraints_for_decel, path);
      break;
    }
    default:
      throw std::domain_error("invalid behavior");
  }

  setStopReason(StopReason::AVOIDANCE, path.path);
}

void AvoidanceModule::updateRegisteredRawShiftLines()
{
  const auto & data = avoidance_data_;

  utils::avoidance::fillAdditionalInfoFromPoint(data, registered_raw_shift_lines_);

  AvoidLineArray avoid_lines;

  const auto has_large_offset = [this](const auto & s) {
    constexpr double THRESHOLD = 0.1;
    const auto ego_shift_length = helper_.getEgoLinearShift();

    const auto start_to_ego_longitudinal = -1.0 * s.start_longitudinal;

    if (start_to_ego_longitudinal < 0.0) {
      return false;
    }

    const auto reg_shift_length =
      s.getGradient() * start_to_ego_longitudinal + s.start_shift_length;

    return std::abs(ego_shift_length - reg_shift_length) > THRESHOLD;
  };

  const auto ego_idx = data.ego_closest_path_index;

  for (const auto & s : registered_raw_shift_lines_) {
    // invalid
    if (s.end_idx < ego_idx) {
      continue;
    }

    // invalid
    if (has_large_offset(s)) {
      continue;
    }

    // valid
    avoid_lines.push_back(s);
  }

  DEBUG_PRINT(
    "ego_closest_path_index = %lu, registered_raw_shift_lines_ size: %lu -> %lu",
    data.ego_closest_path_index, registered_raw_shift_lines_.size(), avoid_lines.size());

  printShiftLines(registered_raw_shift_lines_, "registered_raw_shift_lines_ (before)");
  printShiftLines(avoid_lines, "registered_raw_shift_lines_ (after)");

  registered_raw_shift_lines_ = avoid_lines;
  debug_data_.registered_raw_shift = registered_raw_shift_lines_;
}

AvoidLineArray AvoidanceModule::applyPreProcessToRawShiftLines(
  AvoidLineArray & raw_shift_lines, DebugData & debug) const
{
  /**
   * Use all registered points. For the current points, if the similar one of the current
   * points are already registered, will not use it.
   * TODO(Horibe): enrich this logic to be able to consider the removal of the registered
   *               shift, because it cannot handle the case like "we don't have to avoid
   *               the object anymore".
   */
  raw_shift_lines = utils::avoidance::combineRawShiftLinesWithUniqueCheck(
    registered_raw_shift_lines_, raw_shift_lines);

  printShiftLines(raw_shift_lines, "raw_shift_lines");
  printShiftLines(registered_raw_shift_lines_, "registered_raw_shift_lines");

  /*
   * Add return-to-center shift point from the last shift point, if needed.
   * If there is no shift points, set return-to center shift from ego.
   */
  // TODO(Horibe) Here, the return point is calculated considering the prepare distance,
  // but there is an issue that sometimes this prepare distance is erased by the trimSimilarGrad,
  // and it suddenly tries to return from ego. Then steer rotates aggressively.
  // It is temporally solved by changing the threshold of trimSimilarGrad, but it needs to be
  // fixed in a proper way.
  // Maybe after merge, all shift points before the prepare distance can be deleted.
  addReturnShiftLineFromEgo(raw_shift_lines);

  /*
   * Add gap filled shift lines so that merged shift lines connect smoothly.
   */
  fillShiftLineGap(raw_shift_lines);
  debug.gap_filled = raw_shift_lines;

  /**
   * On each path point, compute shift length with considering the raw shift points.
   * Then create a merged shift points by finding the change point of the gradient of shifting.
   *  - take maximum shift length if there is duplicate shift point
   *  - take sum if there are shifts for opposite direction (right and left)
   *  - shift length is interpolated linearly.
   * Note: Because this function just foolishly extracts points, it includes
   *       insignificant small (useless) shift points, which should be removed in post-process.
   */
  auto merged_shift_lines = mergeShiftLines(raw_shift_lines, debug);
  debug.merged = merged_shift_lines;

  /*
   * Remove unnecessary shift points
   *  - Quantize the shift length to reduce the shift point noise
   *  - Change the shift length to the previous one if the deviation is small.
   *  - Combine shift points that have almost same gradient
   *  - Remove unnecessary return shift (back to the center line).
   */
  auto shift_lines = trimShiftLine(merged_shift_lines, debug);
  DEBUG_PRINT("final shift point size = %lu", shift_lines.size());

  return shift_lines;
}

void AvoidanceModule::registerRawShiftLines(const AvoidLineArray & future)
{
  if (future.empty()) {
    RCLCPP_ERROR(getLogger(), "future is empty! return.");
    return;
  }

  const auto old_size = registered_raw_shift_lines_.size();

  auto future_with_info = future;
  utils::avoidance::fillAdditionalInfoFromPoint(avoidance_data_, future_with_info);
  printShiftLines(future_with_info, "future_with_info");
  printShiftLines(registered_raw_shift_lines_, "registered_raw_shift_lines_");
  printShiftLines(current_raw_shift_lines_, "current_raw_shift_lines_");

  // sort by longitudinal
  std::sort(future_with_info.begin(), future_with_info.end(), [](auto a, auto b) {
    return a.end_longitudinal < b.end_longitudinal;
  });

  // calc relative lateral length
  future_with_info.front().start_shift_length = getCurrentBaseShift();
  for (size_t i = 1; i < future_with_info.size(); ++i) {
    future_with_info.at(i).start_shift_length = future_with_info.at(i - 1).end_shift_length;
  }

  const auto is_registered = [this](const auto id) {
    const auto & r = registered_raw_shift_lines_;
    return std::any_of(r.begin(), r.end(), [id](const auto & s) { return s.id == id; });
  };

  const auto same_id_shift_line = [this](const auto id) {
    const auto & r = current_raw_shift_lines_;
    const auto itr = std::find_if(r.begin(), r.end(), [id](const auto & s) { return s.id == id; });
    if (itr != r.end()) {
      return *itr;
    }
    throw std::logic_error("not found same id current raw shift line.");
  };

  for (const auto & s : future_with_info) {
    if (s.parent_ids.empty()) {
      RCLCPP_ERROR(getLogger(), "avoid line for path_shifter must have parent_id.");
    }

    for (const auto id : s.parent_ids) {
      if (is_registered(id)) {
        continue;
      }

      registered_raw_shift_lines_.push_back(same_id_shift_line(id));
    }
  }

  DEBUG_PRINT("registered object size: %lu -> %lu", old_size, registered_raw_shift_lines_.size());
}

AvoidLineArray AvoidanceModule::calcRawShiftLinesFromObjects(
  AvoidancePlanningData & data, [[maybe_unused]] DebugData & debug) const
{
  // To be consistent with changes in the ego position, the current shift length is considered.
  const auto current_ego_shift = helper_.getEgoShift();
  const auto & base_link2front = planner_data_->parameters.base_link2front;
  const auto & base_link2rear = planner_data_->parameters.base_link2rear;

  // Calculate feasible shift length
  const auto get_shift_length =
    [&](auto & object, const auto & desire_shift_length) -> boost::optional<double> {
    // use each object param
    const auto object_type = utils::getHighestProbLabel(object.object.classification);
    const auto object_parameter = parameters_->object_parameters.at(object_type);
    const auto is_object_on_right = utils::avoidance::isOnRight(object);

    // use absolute dist for return-to-center, relative dist from current for avoiding.
    const auto avoiding_shift = desire_shift_length - current_ego_shift;
    const auto nominal_avoid_distance = helper_.getMaxAvoidanceDistance(avoiding_shift);

    // ego already has enough positive shift.
    const auto has_enough_positive_shift = avoiding_shift < -1e-3 && desire_shift_length > 1e-3;
    if (is_object_on_right && has_enough_positive_shift) {
      return desire_shift_length;
    }

    // ego already has enough negative shift.
    const auto has_enough_negative_shift = avoiding_shift > 1e-3 && desire_shift_length < -1e-3;
    if (!is_object_on_right && has_enough_negative_shift) {
      return desire_shift_length;
    }

    // calculate remaining distance.
    const auto prepare_distance = helper_.getNominalPrepareDistance();
    const auto constant =
      object_parameter.safety_buffer_longitudinal + base_link2front + prepare_distance;
    const auto has_enough_distance = object.longitudinal > constant + nominal_avoid_distance;
    const auto remaining_distance = object.longitudinal - constant;

    // the avoidance path is already approved
    const auto & object_pos = object.object.kinematics.initial_pose_with_covariance.pose.position;
    const auto is_approved = (helper_.getShift(object_pos) > 0.0 && is_object_on_right) ||
                             (helper_.getShift(object_pos) < 0.0 && !is_object_on_right);
    if (is_approved) {
      return desire_shift_length;
    }

    // prepare distance is not enough. unavoidable.
    if (remaining_distance < 1e-3) {
      object.reason = AvoidanceDebugFactor::REMAINING_DISTANCE_LESS_THAN_ZERO;
      return boost::none;
    }

    // nominal case. avoidable.
    if (has_enough_distance) {
      return desire_shift_length;
    }

    // calculate lateral jerk.
    const auto required_jerk = PathShifter::calcJerkFromLatLonDistance(
      avoiding_shift, remaining_distance, helper_.getAvoidanceEgoSpeed());

    // relax lateral jerk limit. avoidable.
    if (required_jerk < helper_.getLateralMaxJerkLimit()) {
      return desire_shift_length;
    }

    // avoidance distance is not enough. unavoidable.
    if (!parameters_->use_constraints_for_decel) {
      object.reason = AvoidanceDebugFactor::TOO_LARGE_JERK;
      return boost::none;
    }

    // output avoidance path under lateral jerk constraints.
    const auto feasible_shift_length = PathShifter::calcLateralDistFromJerk(
      remaining_distance, helper_.getLateralMaxJerkLimit(), helper_.getAvoidanceEgoSpeed());

    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 1000,
      "original shift length is not feasible. generate avoidance path under the constraints. "
      "[original: (%.2f) actual: (%.2f)]",
      std::abs(avoiding_shift), feasible_shift_length);

    return desire_shift_length > 0.0 ? feasible_shift_length + current_ego_shift
                                     : -1.0 * feasible_shift_length + current_ego_shift;
  };

  const auto is_forward_object = [](const auto & object) { return object.longitudinal > 0.0; };

  const auto is_valid_shift_line = [](const auto & s) {
    return s.start_longitudinal > 0.0 && s.start_longitudinal < s.end_longitudinal;
  };

  AvoidLineArray avoid_lines;
  for (auto & o : data.target_objects) {
    if (!o.avoid_margin) {
      o.reason = AvoidanceDebugFactor::INSUFFICIENT_LATERAL_MARGIN;
      if (o.avoid_required && is_forward_object(o)) {
        break;
      } else {
        continue;
      }
    }

    const auto is_object_on_right = utils::avoidance::isOnRight(o);
    const auto desire_shift_length =
      helper_.getShiftLength(o, is_object_on_right, o.avoid_margin.get());
    if (utils::avoidance::isSameDirectionShift(is_object_on_right, desire_shift_length)) {
      o.reason = AvoidanceDebugFactor::SAME_DIRECTION_SHIFT;
      if (o.avoid_required && is_forward_object(o)) {
        break;
      } else {
        continue;
      }
    }

    // use each object param
    const auto object_type = utils::getHighestProbLabel(o.object.classification);
    const auto object_parameter = parameters_->object_parameters.at(object_type);
    const auto feasible_shift_length = get_shift_length(o, desire_shift_length);

    if (!feasible_shift_length) {
      if (o.avoid_required && is_forward_object(o)) {
        break;
      } else {
        continue;
      }
    }

    // use absolute dist for return-to-center, relative dist from current for avoiding.
    const auto feasible_avoid_distance =
      helper_.getMaxAvoidanceDistance(feasible_shift_length.get() - current_ego_shift);
    const auto feasible_return_distance =
      helper_.getMaxAvoidanceDistance(feasible_shift_length.get());

    AvoidLine al_avoid;
    {
      const auto offset = object_parameter.safety_buffer_longitudinal + base_link2front;
      const auto path_front_to_ego =
        avoidance_data_.arclength_from_ego.at(avoidance_data_.ego_closest_path_index);

      al_avoid.start_longitudinal =
        std::max(o.longitudinal - offset - feasible_avoid_distance, 1e-3);
      al_avoid.start_idx = utils::avoidance::findPathIndexFromArclength(
        avoidance_data_.arclength_from_ego, al_avoid.start_longitudinal + path_front_to_ego);
      al_avoid.start = avoidance_data_.reference_path.points.at(al_avoid.start_idx).point.pose;
      al_avoid.start_shift_length = helper_.getLinearShift(al_avoid.start.position);

      al_avoid.end_shift_length = feasible_shift_length.get();
      al_avoid.end_longitudinal = o.longitudinal - offset;
      al_avoid.id = getOriginalShiftLineUniqueId();
      al_avoid.object = o;

      if (is_valid_shift_line(al_avoid)) {
        avoid_lines.push_back(al_avoid);
      }
    }

    AvoidLine al_return;
    {
      const auto offset = object_parameter.safety_buffer_longitudinal + base_link2rear + o.length;
      // The end_margin also has the purpose of preventing the return path from NOT being
      // triggered at the end point.
      const auto end_margin = 1.0;
      const auto return_remaining_distance =
        std::max(data.arclength_from_ego.back() - o.longitudinal - offset - end_margin, 0.0);

      al_return.start_shift_length = feasible_shift_length.get();
      al_return.end_shift_length = 0.0;
      al_return.start_longitudinal = o.longitudinal + offset;
      al_return.end_longitudinal =
        o.longitudinal + offset + std::min(feasible_return_distance, return_remaining_distance);
      al_return.id = getOriginalShiftLineUniqueId();
      al_return.object = o;

      if (is_valid_shift_line(al_return)) {
        avoid_lines.push_back(al_return);
      }
    }

    o.is_avoidable = true;
  }

  // debug
  {
    std::vector<AvoidanceDebugMsg> debug_info_array;
    const auto append = [&](const auto & o) {
      AvoidanceDebugMsg debug_info;
      debug_info.object_id = toHexString(o.object.object_id);
      debug_info.longitudinal_distance = o.longitudinal;
      debug_info.lateral_distance_from_centerline = o.lateral;
      debug_info.allow_avoidance = o.reason == "";
      debug_info.failed_reason = o.reason;
      debug_info_array.push_back(debug_info);
    };

    for (const auto & o : data.target_objects) {
      append(o);
    }

    debug_avoidance_initializer_for_shift_line_.clear();
    debug_avoidance_initializer_for_shift_line_ = std::move(debug_info_array);
    debug_avoidance_initializer_for_shift_line_time_ = clock_->now();
  }

  utils::avoidance::fillAdditionalInfoFromLongitudinal(data, avoid_lines);

  return avoid_lines;
}

void AvoidanceModule::generateTotalShiftLine(
  const AvoidLineArray & avoid_lines, ShiftLineData & shift_line_data) const
{
  const auto & path = avoidance_data_.reference_path;
  const auto & arcs = avoidance_data_.arclength_from_ego;
  const auto N = path.points.size();

  auto & sl = shift_line_data;

  sl.shift_line = std::vector<double>(N, 0.0);
  sl.shift_line_grad = std::vector<double>(N, 0.0);

  sl.pos_shift_line = std::vector<double>(N, 0.0);
  sl.neg_shift_line = std::vector<double>(N, 0.0);

  sl.pos_shift_line_grad = std::vector<double>(N, 0.0);
  sl.neg_shift_line_grad = std::vector<double>(N, 0.0);

  // debug
  sl.shift_line_history = std::vector<std::vector<double>>(avoid_lines.size(), sl.shift_line);

  // take minmax for same directional shift length
  for (size_t j = 0; j < avoid_lines.size(); ++j) {
    const auto & al = avoid_lines.at(j);
    for (size_t i = 0; i < N; ++i) {
      // calc current interpolated shift
      const auto i_shift = utils::avoidance::lerpShiftLengthOnArc(arcs.at(i), al);

      // update maximum shift for positive direction
      if (i_shift > sl.pos_shift_line.at(i)) {
        sl.pos_shift_line.at(i) = i_shift;
        sl.pos_shift_line_grad.at(i) = al.getGradient();
      }

      // update minumum shift for negative direction
      if (i_shift < sl.neg_shift_line.at(i)) {
        sl.neg_shift_line.at(i) = i_shift;
        sl.neg_shift_line_grad.at(i) = al.getGradient();
      }

      // store for debug print
      sl.shift_line_history.at(j).at(i) = i_shift;
    }
  }

  // Merge shift length of opposite directions.
  for (size_t i = 0; i < N; ++i) {
    sl.shift_line.at(i) = sl.pos_shift_line.at(i) + sl.neg_shift_line.at(i);
    sl.shift_line_grad.at(i) = sl.pos_shift_line_grad.at(i) + sl.neg_shift_line_grad.at(i);
  }

  // overwrite shift with current_ego_shift until ego pose.
  const auto current_shift = helper_.getEgoLinearShift();
  for (size_t i = 0; i <= avoidance_data_.ego_closest_path_index; ++i) {
    sl.shift_line.at(i) = current_shift;
    sl.shift_line_grad.at(i) = 0.0;
  }

  // If the shift point does not have an associated object,
  // use previous value.
  for (size_t i = 1; i < N; ++i) {
    bool has_object = false;
    for (const auto & al : avoid_lines) {
      if (al.start_idx <= i && i <= al.end_idx) {
        has_object = true;
        break;
      }
    }
    if (!has_object) {
      sl.shift_line.at(i) = sl.shift_line.at(i - 1);
    }
  }

  if (avoid_lines.empty()) {
    sl.shift_line_history.push_back(sl.shift_line);
    return;
  }

  const auto grad_first_shift_line = (avoid_lines.front().start_shift_length - current_shift) /
                                     avoid_lines.front().start_longitudinal;

  for (size_t i = avoidance_data_.ego_closest_path_index; i <= avoid_lines.front().start_idx; ++i) {
    sl.shift_line.at(i) = helper_.getLinearShift(getPoint(path.points.at(i)));
    sl.shift_line_grad.at(i) = grad_first_shift_line;
  }

  sl.shift_line_history.push_back(sl.shift_line);
}

AvoidLineArray AvoidanceModule::extractShiftLinesFromLine(ShiftLineData & shift_line_data) const
{
  using utils::avoidance::setEndData;
  using utils::avoidance::setStartData;

  const auto & path = avoidance_data_.reference_path;
  const auto & arcs = avoidance_data_.arclength_from_ego;
  const auto N = path.points.size();

  auto & sl = shift_line_data;

  const auto backward_grad = [&](const size_t i) {
    if (i == 0) {
      return sl.shift_line_grad.at(i);
    }
    const double ds = arcs.at(i) - arcs.at(i - 1);
    if (ds < 1.0e-5) {
      return sl.shift_line_grad.at(i);
    }  // use theoretical value when ds is too small.
    return (sl.shift_line.at(i) - sl.shift_line.at(i - 1)) / ds;
  };

  const auto forward_grad = [&](const size_t i) {
    if (i == arcs.size() - 1) {
      return sl.shift_line_grad.at(i);
    }
    const double ds = arcs.at(i + 1) - arcs.at(i);
    if (ds < 1.0e-5) {
      return sl.shift_line_grad.at(i);
    }  // use theoretical value when ds is too small.
    return (sl.shift_line.at(i + 1) - sl.shift_line.at(i)) / ds;
  };

  // calculate forward and backward gradient of the shift length.
  // This will be used for grad-change-point check.
  sl.forward_grad = std::vector<double>(N, 0.0);
  sl.backward_grad = std::vector<double>(N, 0.0);
  for (size_t i = 0; i < N - 1; ++i) {
    sl.forward_grad.at(i) = forward_grad(i);
    sl.backward_grad.at(i) = backward_grad(i);
  }

  AvoidLineArray merged_avoid_lines;
  AvoidLine al{};
  bool found_first_start = false;
  constexpr auto CREATE_SHIFT_GRAD_THR = 0.001;
  constexpr auto IS_ALREADY_SHIFTING_THR = 0.001;
  for (size_t i = avoidance_data_.ego_closest_path_index; i < N - 1; ++i) {
    const auto & p = path.points.at(i).point.pose;
    const auto shift = sl.shift_line.at(i);

    // If the vehicle is already on the avoidance (checked by the first point has shift),
    // set a start point at the first path point.
    if (!found_first_start && std::abs(shift) > IS_ALREADY_SHIFTING_THR) {
      setStartData(al, 0.0, p, i, arcs.at(i));  // start length is overwritten later.
      found_first_start = true;
      DEBUG_PRINT("shift (= %f) is not zero at i = %lu. set start shift here.", shift, i);
    }

    // find the point where the gradient of the shift is changed
    const bool set_shift_line_flag =
      std::abs(sl.forward_grad.at(i) - sl.backward_grad.at(i)) > CREATE_SHIFT_GRAD_THR;

    if (!set_shift_line_flag) {
      continue;
    }

    if (!found_first_start) {
      setStartData(al, 0.0, p, i, arcs.at(i));  // start length is overwritten later.
      found_first_start = true;
      DEBUG_PRINT("grad change detected. start at i = %lu", i);
    } else {
      setEndData(al, shift, p, i, arcs.at(i));
      al.id = getOriginalShiftLineUniqueId();
      merged_avoid_lines.push_back(al);
      setStartData(al, 0.0, p, i, arcs.at(i));  // start length is overwritten later.
      DEBUG_PRINT("end and start point found at i = %lu", i);
    }
  }

  if (std::abs(backward_grad(N - 1)) > CREATE_SHIFT_GRAD_THR) {
    const auto & p = path.points.at(N - 1).point.pose;
    const auto shift = sl.shift_line.at(N - 1);
    setEndData(al, shift, p, N - 1, arcs.at(N - 1));
    al.id = getOriginalShiftLineUniqueId();
    merged_avoid_lines.push_back(al);
  }

  return merged_avoid_lines;
}

void AvoidanceModule::fillShiftLineGap(AvoidLineArray & shift_lines) const
{
  using utils::avoidance::setEndData;

  if (shift_lines.empty()) {
    return;
  }

  const auto & data = avoidance_data_;

  helper_.alignShiftLinesOrder(shift_lines, false);

  const auto fill_gap = [&shift_lines, this](const auto & front_line, const auto & back_line) {
    const auto has_gap = back_line.start_longitudinal - front_line.end_longitudinal > 0.0;
    if (!has_gap) {
      return;
    }

    AvoidLine new_line{};
    new_line.start_shift_length = front_line.end_shift_length;
    new_line.start_longitudinal = front_line.end_longitudinal;
    new_line.end_shift_length = back_line.start_shift_length;
    new_line.end_longitudinal = back_line.start_longitudinal;
    new_line.id = getOriginalShiftLineUniqueId();

    shift_lines.push_back(new_line);
  };

  // fill gap between ego and nearest shift line.
  {
    AvoidLine ego_line{};
    setEndData(
      ego_line, helper_.getEgoLinearShift(), data.reference_pose, data.ego_closest_path_index, 0.0);

    fill_gap(ego_line, shift_lines.front());
  }

  // fill gap among shift lines.
  for (size_t i = 0; i < shift_lines.size() - 1; ++i) {
    fill_gap(shift_lines.at(i), shift_lines.at(i + 1));
  }

  utils::avoidance::fillAdditionalInfoFromLongitudinal(data, shift_lines);

  helper_.alignShiftLinesOrder(shift_lines, false);
}

AvoidLineArray AvoidanceModule::mergeShiftLines(
  const AvoidLineArray & raw_shift_lines, DebugData & debug) const
{
  // Generate shift line by merging raw_shift_lines.
  ShiftLineData shift_line_data;
  generateTotalShiftLine(raw_shift_lines, shift_line_data);

  // Re-generate shift points by detecting gradient-change point of the shift line.
  auto merged_shift_lines = extractShiftLinesFromLine(shift_line_data);

  // set parent id
  for (auto & al : merged_shift_lines) {
    al.parent_ids = utils::avoidance::calcParentIds(raw_shift_lines, al);
  }

  // sort by distance from ego.
  helper_.alignShiftLinesOrder(merged_shift_lines);

  // debug visualize
  {
    debug.pos_shift = shift_line_data.pos_shift_line;
    debug.neg_shift = shift_line_data.neg_shift_line;
    debug.total_shift = shift_line_data.shift_line;
    debug.pos_shift_grad = shift_line_data.pos_shift_line_grad;
    debug.neg_shift_grad = shift_line_data.neg_shift_line_grad;
    debug.total_forward_grad = shift_line_data.forward_grad;
    debug.total_backward_grad = shift_line_data.backward_grad;
  }

  // debug print
  {
    const auto & arc = avoidance_data_.arclength_from_ego;
    const auto & closest = avoidance_data_.ego_closest_path_index;
    const auto & sl = shift_line_data.shift_line;
    const auto & sg = shift_line_data.shift_line_grad;
    const auto & fg = shift_line_data.forward_grad;
    const auto & bg = shift_line_data.backward_grad;
    using std::setw;
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3);
    ss << "\n[idx, arc, shift (for each shift points, filtered | total), grad (ideal, bwd, fwd)]: "
          "closest = "
       << closest << ", raw_shift_lines size = " << raw_shift_lines.size() << std::endl;
    for (size_t i = 0; i < arc.size(); ++i) {
      ss << "i = " << i << " | arc: " << arc.at(i) << " | shift: (";
      for (const auto & p : shift_line_data.shift_line_history) {
        ss << setw(5) << p.at(i) << ", ";
      }
      ss << "| total: " << setw(5) << sl.at(i) << ") | grad: (" << sg.at(i) << ", " << fg.at(i)
         << ", " << bg.at(i) << ")" << std::endl;
    }
    DEBUG_PRINT("%s", ss.str().c_str());
  }

  printShiftLines(merged_shift_lines, "merged_shift_lines");

  return merged_shift_lines;
}

AvoidLineArray AvoidanceModule::trimShiftLine(
  const AvoidLineArray & shift_lines, DebugData & debug) const
{
  if (shift_lines.empty()) {
    return shift_lines;
  }

  AvoidLineArray sl_array_trimmed = shift_lines;

  // sort shift points from front to back.
  helper_.alignShiftLinesOrder(sl_array_trimmed);

  // - Change the shift length to the previous one if the deviation is small.
  {
    constexpr double SHIFT_DIFF_THRES = 1.0;
    trimSmallShiftLine(sl_array_trimmed, SHIFT_DIFF_THRES);
  }

  // - Combine avoid points that have almost same gradient.
  // this is to remove the noise.
  {
    const auto THRESHOLD = parameters_->same_grad_filter_1_threshold;
    trimSimilarGradShiftLine(sl_array_trimmed, THRESHOLD);
    debug.trim_similar_grad_shift = sl_array_trimmed;
    printShiftLines(sl_array_trimmed, "after trim_similar_grad_shift");
  }

  // - Quantize the shift length to reduce the shift point noise
  // This is to remove the noise coming from detection accuracy, interpolation, resampling, etc.
  {
    const auto THRESHOLD = parameters_->quantize_filter_threshold;
    quantizeShiftLine(sl_array_trimmed, THRESHOLD);
    printShiftLines(sl_array_trimmed, "after sl_array_trimmed");
    debug.quantized = sl_array_trimmed;
  }

  // - Change the shift length to the previous one if the deviation is small.
  {
    constexpr double SHIFT_DIFF_THRES = 1.0;
    trimSmallShiftLine(sl_array_trimmed, SHIFT_DIFF_THRES);
    debug.trim_small_shift = sl_array_trimmed;
    printShiftLines(sl_array_trimmed, "after trim_small_shift");
  }

  // - Combine avoid points that have almost same gradient (again)
  {
    const auto THRESHOLD = parameters_->same_grad_filter_2_threshold;
    trimSimilarGradShiftLine(sl_array_trimmed, THRESHOLD);
    debug.trim_similar_grad_shift_second = sl_array_trimmed;
    printShiftLines(sl_array_trimmed, "after trim_similar_grad_shift_second");
  }

  // - trimTooSharpShift
  // Check if it is not too sharp for the return-to-center shift point.
  // If the shift is sharp, it is combined with the next shift point until it gets non-sharp.
  {
    const auto THRESHOLD = parameters_->sharp_shift_filter_threshold;
    trimSharpReturn(sl_array_trimmed, THRESHOLD);
    debug.trim_too_sharp_shift = sl_array_trimmed;
    printShiftLines(sl_array_trimmed, "after trimSharpReturn");
  }

  // - Combine avoid points that have almost same gradient (again)
  {
    const auto THRESHOLD = parameters_->same_grad_filter_3_threshold;
    trimSimilarGradShiftLine(sl_array_trimmed, THRESHOLD);
    debug.trim_similar_grad_shift_third = sl_array_trimmed;
    printShiftLines(sl_array_trimmed, "after trim_similar_grad_shift_second");
  }

  return sl_array_trimmed;
}

void AvoidanceModule::quantizeShiftLine(AvoidLineArray & shift_lines, const double threshold) const
{
  if (threshold < 1.0e-5) {
    return;  // no need to process
  }

  for (auto & sl : shift_lines) {
    sl.end_shift_length = std::round(sl.end_shift_length / threshold) * threshold;
  }

  helper_.alignShiftLinesOrder(shift_lines);
}

void AvoidanceModule::trimSmallShiftLine(AvoidLineArray & shift_lines, const double threshold) const
{
  if (shift_lines.empty()) {
    return;
  }

  AvoidLineArray input = shift_lines;
  shift_lines.clear();

  for (const auto & s : input) {
    if (s.getRelativeLongitudinal() < threshold) {
      continue;
    }

    shift_lines.push_back(s);
  }
}

void AvoidanceModule::trimSimilarGradShiftLine(
  AvoidLineArray & avoid_lines, const double threshold) const
{
  if (avoid_lines.empty()) {
    return;
  }

  AvoidLineArray input = avoid_lines;
  avoid_lines.clear();
  avoid_lines.push_back(input.front());  // Take the first one anyway (think later)

  AvoidLine base_line = input.front();

  AvoidLineArray combine_buffer;
  combine_buffer.push_back(input.front());

  constexpr auto SHIFT_THR = 1e-3;
  const auto is_negative_shift = [&](const auto & s) {
    return s.getRelativeLength() < -1.0 * SHIFT_THR;
  };

  const auto is_positive_shift = [&](const auto & s) { return s.getRelativeLength() > SHIFT_THR; };

  for (size_t i = 1; i < input.size(); ++i) {
    AvoidLine combine{};

    utils::avoidance::setStartData(
      combine, base_line.start_shift_length, base_line.start, base_line.start_idx,
      base_line.start_longitudinal);
    utils::avoidance::setEndData(
      combine, input.at(i).end_shift_length, input.at(i).end, input.at(i).end_idx,
      input.at(i).end_longitudinal);

    combine_buffer.push_back(input.at(i));

    const auto violates = [&]() {
      if (is_negative_shift(input.at(i)) && is_positive_shift(base_line)) {
        return true;
      }

      if (is_negative_shift(base_line) && is_positive_shift(input.at(i))) {
        return true;
      }

      const auto lon_combine = combine.getRelativeLongitudinal();
      const auto base_length = base_line.getGradient() * lon_combine;
      return std::abs(combine.getRelativeLength() - base_length) > threshold;
    }();

    if (violates) {
      avoid_lines.push_back(input.at(i));
      base_line = input.at(i);
      combine_buffer.clear();
      combine_buffer.push_back(input.at(i));
    } else {
      avoid_lines.back() = combine;
    }
  }

  helper_.alignShiftLinesOrder(avoid_lines);

  DEBUG_PRINT("size %lu -> %lu", input.size(), avoid_lines.size());
}

void AvoidanceModule::trimSharpReturn(AvoidLineArray & shift_lines, const double threshold) const
{
  AvoidLineArray shift_lines_orig = shift_lines;
  shift_lines.clear();

  const auto isZero = [](double v) { return std::abs(v) < 0.01; };

  // check if the shift point is positive (avoiding) shift
  const auto isPositive = [&](const auto & sl) {
    constexpr auto POSITIVE_SHIFT_THR = 0.1;
    return std::abs(sl.end_shift_length) - std::abs(sl.start_shift_length) > POSITIVE_SHIFT_THR;
  };

  // check if the shift point is negative (returning) shift
  const auto isNegative = [&](const auto & sl) {
    constexpr auto NEGATIVE_SHIFT_THR = -0.1;
    return std::abs(sl.end_shift_length) - std::abs(sl.start_shift_length) < NEGATIVE_SHIFT_THR;
  };

  // combine two shift points. Be careful the order of "now" and "next".
  const auto combineShiftLine = [this](const auto & sl_next, const auto & sl_now) {
    auto sl_modified = sl_now;
    utils::avoidance::setEndData(
      sl_modified, sl_next.end_shift_length, sl_next.end, sl_next.end_idx,
      sl_next.end_longitudinal);
    sl_modified.parent_ids =
      utils::avoidance::concatParentIds(sl_modified.parent_ids, sl_now.parent_ids);
    return sl_modified;
  };

  // Check if the merged shift has a conflict with the original shifts.
  const auto hasViolation = [&threshold](const auto & combined, const auto & combined_src) {
    for (const auto & sl : combined_src) {
      const auto combined_shift =
        utils::avoidance::lerpShiftLengthOnArc(sl.end_longitudinal, combined);
      if (sl.end_shift_length < -0.01 && combined_shift > sl.end_shift_length + threshold) {
        return true;
      }
      if (sl.end_shift_length > 0.01 && combined_shift < sl.end_shift_length - threshold) {
        return true;
      }
    }
    return false;
  };

  // check for all shift points
  for (size_t i = 0; i < shift_lines_orig.size(); ++i) {
    auto sl_now = shift_lines_orig.at(i);
    sl_now.start_shift_length =
      shift_lines.empty() ? helper_.getEgoLinearShift() : shift_lines.back().end_shift_length;

    if (sl_now.end_shift_length * sl_now.start_shift_length < -0.01) {
      DEBUG_PRINT("i = %lu, This is avoid shift for opposite direction. take this one", i);
      continue;
    }

    // Do nothing for non-reduce shift point
    if (!isNegative(sl_now)) {
      shift_lines.push_back(sl_now);
      DEBUG_PRINT(
        "i = %lu, positive shift. take this one. sl_now.length * sl_now.start_length = %f", i,
        sl_now.end_shift_length * sl_now.start_shift_length);
      continue;
    }

    // The last point is out of target of this function.
    if (i == shift_lines_orig.size() - 1) {
      shift_lines.push_back(sl_now);
      DEBUG_PRINT("i = %lu, last shift. take this one.", i);
      continue;
    }

    // -----------------------------------------------------------------------
    // ------------ From here, the shift point is "negative" -----------------
    // -----------------------------------------------------------------------

    // if next shift is negative, combine them. loop until combined shift line
    // exceeds merged shift point.
    DEBUG_PRINT("i = %lu, found negative dist. search.", i);
    {
      auto sl_combined = sl_now;
      auto sl_combined_prev = sl_combined;
      AvoidLineArray sl_combined_array{sl_now};
      size_t j = i + 1;
      for (; i < shift_lines_orig.size(); ++j) {
        const auto sl_combined = combineShiftLine(shift_lines_orig.at(j), sl_now);

        {
          std::stringstream ss;
          ss << "i = " << i << ", j = " << j << ": sl_combined = " << toStrInfo(sl_combined);
          DEBUG_PRINT("%s", ss.str().c_str());
        }

        // it gets positive. Finish merging.
        if (isPositive(sl_combined)) {
          shift_lines.push_back(sl_combined);
          DEBUG_PRINT("reach positive.");
          break;
        }

        // Still negative, but it violates the original shift points.
        // Finish with the previous merge result.
        if (hasViolation(sl_combined, sl_combined_array)) {
          shift_lines.push_back(sl_combined_prev);
          DEBUG_PRINT("violation found.");
          --j;
          break;
        }

        // Still negative, but it has an enough long distance. Finish merging.
        const auto nominal_distance =
          helper_.getMaxAvoidanceDistance(sl_combined.getRelativeLength());
        const auto long_distance =
          isZero(sl_combined.end_shift_length) ? nominal_distance : nominal_distance * 5.0;
        if (sl_combined.getRelativeLongitudinal() > long_distance) {
          shift_lines.push_back(sl_combined);
          DEBUG_PRINT("still negative, but long enough. Threshold = %f", long_distance);
          break;
        }

        // It reaches the last point. Still the shift is sharp, but merge with the current result.
        if (j == shift_lines_orig.size() - 1) {
          shift_lines.push_back(sl_combined);
          DEBUG_PRINT("reach end point.");
          break;
        }

        // Still negative shift, and the distance is not enough. Search next.
        sl_combined_prev = sl_combined;
        sl_combined_array.push_back(shift_lines_orig.at(j));
      }
      i = j;
      continue;
    }
  }

  helper_.alignShiftLinesOrder(shift_lines);

  DEBUG_PRINT("trimSharpReturn: size %lu -> %lu", shift_lines_orig.size(), shift_lines.size());
}

void AvoidanceModule::addReturnShiftLineFromEgo(AvoidLineArray & sl_candidates) const
{
  constexpr double ep = 1.0e-3;
  const auto & data = avoidance_data_;
  const bool has_candidate_point = !sl_candidates.empty();
  const bool has_registered_point = !path_shifter_.getShiftLines().empty();

  // If the return-to-center shift points are already registered, do nothing.
  if (!has_registered_point && std::fabs(getCurrentBaseShift()) < ep) {
    DEBUG_PRINT("No shift points, not base offset. Do not have to add return-shift.");
    return;
  }

  constexpr double RETURN_SHIFT_THRESHOLD = 0.1;
  DEBUG_PRINT("registered last shift = %f", path_shifter_.getLastShiftLength());
  if (std::abs(path_shifter_.getLastShiftLength()) < RETURN_SHIFT_THRESHOLD) {
    DEBUG_PRINT("Return shift is already registered. do nothing.");
    return;
  }

  // From here, the return-to-center is not registered. But perhaps the candidate is
  // already generated.

  // If it has a shift point, add return shift from the existing last shift point.
  // If not, add return shift from ego point. (prepare distance is considered for both.)
  ShiftLine last_sl;  // the return-shift will be generated after the last shift point.
  {
    // avoidance points: Yes, shift points: No -> select last avoidance point.
    if (has_candidate_point && !has_registered_point) {
      helper_.alignShiftLinesOrder(sl_candidates, false);
      last_sl = sl_candidates.back();
    }

    // avoidance points: No, shift points: Yes -> select last shift point.
    if (!has_candidate_point && has_registered_point) {
      last_sl = utils::avoidance::fillAdditionalInfo(
        data, AvoidLine{path_shifter_.getLastShiftLine().get()});
    }

    // avoidance points: Yes, shift points: Yes -> select the last one from both.
    if (has_candidate_point && has_registered_point) {
      helper_.alignShiftLinesOrder(sl_candidates, false);
      const auto & al = sl_candidates.back();
      const auto & sl = utils::avoidance::fillAdditionalInfo(
        data, AvoidLine{path_shifter_.getLastShiftLine().get()});
      last_sl = (sl.end_longitudinal > al.end_longitudinal) ? sl : al;
    }

    // avoidance points: No, shift points: No -> set the ego position to the last shift point
    // so that the return-shift will be generated from ego position.
    if (!has_candidate_point && !has_registered_point) {
      last_sl.end_idx = avoidance_data_.ego_closest_path_index;
      last_sl.end = avoidance_data_.reference_path.points.at(last_sl.end_idx).point.pose;
      last_sl.end_shift_length = getCurrentBaseShift();
    }
  }
  printShiftLines(ShiftLineArray{last_sl}, "last shift point");

  // There already is a shift point candidates to go back to center line, but it could be too sharp
  // due to detection noise or timing.
  // Here the return-shift from ego is added for the in case.
  if (std::fabs(last_sl.end_shift_length) < RETURN_SHIFT_THRESHOLD) {
    const auto current_base_shift = helper_.getEgoShift();
    if (std::abs(current_base_shift) < ep) {
      DEBUG_PRINT("last shift almost is zero, and current base_shift is zero. do nothing.");
      return;
    }

    // Is there a shift point in the opposite direction of the current_base_shift?
    //   No  -> we can overwrite the return shift, because the other shift points that decrease
    //          the shift length are for return-shift.
    //   Yes -> we can NOT overwrite, because it might be not a return-shift, but a avoiding
    //          shift to the opposite direction which can not be overwritten by the return-shift.
    for (const auto & sl : sl_candidates) {
      if (
        (current_base_shift > 0.0 && sl.end_shift_length < -ep) ||
        (current_base_shift < 0.0 && sl.end_shift_length > ep)) {
        DEBUG_PRINT(
          "try to put overwrite return shift, but there is shift for opposite direction. Skip "
          "adding return shift.");
        return;
      }
    }

    // If return shift already exists in candidate or registered shift lines, skip adding return
    // shift.
    if (has_candidate_point || has_registered_point) {
      return;
    }

    // set the return-shift from ego.
    DEBUG_PRINT(
      "return shift already exists, but they are all candidates. Add return shift for overwrite.");
    last_sl.end_idx = avoidance_data_.ego_closest_path_index;
    last_sl.end = avoidance_data_.reference_path.points.at(last_sl.end_idx).point.pose;
    last_sl.end_shift_length = current_base_shift;
  }

  const auto & arclength_from_ego = avoidance_data_.arclength_from_ego;

  const auto nominal_prepare_distance = helper_.getNominalPrepareDistance();
  const auto nominal_avoid_distance = helper_.getMaxAvoidanceDistance(last_sl.end_shift_length);

  if (arclength_from_ego.empty()) {
    return;
  }

  const auto remaining_distance = arclength_from_ego.back();

  // If the avoidance point has already been set, the return shift must be set after the point.
  const auto last_sl_distance = avoidance_data_.arclength_from_ego.at(last_sl.end_idx);

  // check if there is enough distance for return.
  if (last_sl_distance + 1.0 > remaining_distance) {  // tmp: add some small number (+1.0)
    DEBUG_PRINT("No enough distance for return.");
    return;
  }

  // If the remaining distance is not enough, the return shift needs to be shrunk.
  // (or another option is just to ignore the return-shift.)
  // But we do not want to change the last shift point, so we will shrink the distance after
  // the last shift point.
  //
  //  The line "===" is fixed, "---" is scaled.
  //
  // [Before Scaling]
  //  ego              last_sl_end             prepare_end            path_end    avoid_end
  // ==o====================o----------------------o----------------------o------------o
  //   |            prepare_dist                   |          avoid_dist               |
  //
  // [After Scaling]
  // ==o====================o------------------o--------------------------o
  //   |        prepare_dist_scaled            |    avoid_dist_scaled     |
  //
  const double variable_prepare_distance =
    std::max(nominal_prepare_distance - last_sl_distance, 0.0);

  double prepare_distance_scaled = std::max(nominal_prepare_distance, last_sl_distance);
  double avoid_distance_scaled = nominal_avoid_distance;
  if (remaining_distance < prepare_distance_scaled + avoid_distance_scaled) {
    const auto scale = (remaining_distance - last_sl_distance) /
                       std::max(nominal_avoid_distance + variable_prepare_distance, 0.1);
    prepare_distance_scaled = last_sl_distance + scale * nominal_prepare_distance;
    avoid_distance_scaled *= scale;
    DEBUG_PRINT(
      "last_sl_distance = %f, nominal_prepare_distance = %f, nominal_avoid_distance = %f, "
      "remaining_distance = %f, variable_prepare_distance = %f, scale = %f, "
      "prepare_distance_scaled = %f,avoid_distance_scaled = %f",
      last_sl_distance, nominal_prepare_distance, nominal_avoid_distance, remaining_distance,
      variable_prepare_distance, scale, prepare_distance_scaled, avoid_distance_scaled);
  } else {
    DEBUG_PRINT("there is enough distance. Use nominal for prepare & avoidance.");
  }

  // shift point for prepare distance: from last shift to return-start point.
  if (nominal_prepare_distance > last_sl_distance) {
    AvoidLine al;
    al.id = getOriginalShiftLineUniqueId();
    al.start_idx = last_sl.end_idx;
    al.start = last_sl.end;
    al.start_longitudinal = arclength_from_ego.at(al.start_idx);
    al.end_idx =
      utils::avoidance::findPathIndexFromArclength(arclength_from_ego, prepare_distance_scaled);
    al.end = avoidance_data_.reference_path.points.at(al.end_idx).point.pose;
    al.end_longitudinal = arclength_from_ego.at(al.end_idx);
    al.end_shift_length = last_sl.end_shift_length;
    al.start_shift_length = last_sl.end_shift_length;
    sl_candidates.push_back(al);
    printShiftLines(AvoidLineArray{al}, "prepare for return");
    debug_data_.extra_return_shift.push_back(al);
  }

  // shift point for return to center line
  {
    AvoidLine al;
    al.id = getOriginalShiftLineUniqueId();
    al.start_idx =
      utils::avoidance::findPathIndexFromArclength(arclength_from_ego, prepare_distance_scaled);
    al.start = avoidance_data_.reference_path.points.at(al.start_idx).point.pose;
    al.start_longitudinal = arclength_from_ego.at(al.start_idx);
    al.end_idx = utils::avoidance::findPathIndexFromArclength(
      arclength_from_ego, prepare_distance_scaled + avoid_distance_scaled);
    al.end = avoidance_data_.reference_path.points.at(al.end_idx).point.pose;
    al.end_longitudinal = arclength_from_ego.at(al.end_idx);
    al.end_shift_length = 0.0;
    al.start_shift_length = last_sl.end_shift_length;
    sl_candidates.push_back(al);
    printShiftLines(AvoidLineArray{al}, "return point");
    debug_data_.extra_return_shift.push_back(al);
  }

  DEBUG_PRINT("Return Shift is added.");
}

bool AvoidanceModule::isSafePath(
  const PathShifter & path_shifter, ShiftedPath & shifted_path, DebugData & debug) const
{
  const auto & p = parameters_;

  if (!p->enable_safety_check) {
    return true;  // if safety check is disabled, it always return safe.
  }

  const auto & forward_check_distance = p->object_check_forward_distance;
  const auto & backward_check_distance = p->safety_check_backward_distance;
  const auto check_lanes =
    getAdjacentLane(path_shifter, forward_check_distance, backward_check_distance);

  auto path_with_current_velocity = shifted_path.path;

  const size_t ego_idx = planner_data_->findEgoIndex(path_with_current_velocity.points);
  utils::clipPathLength(path_with_current_velocity, ego_idx, forward_check_distance, 0.0);

  constexpr double MIN_EGO_VEL_IN_PREDICTION = 1.38;  // 5km/h
  for (auto & p : path_with_current_velocity.points) {
    p.point.longitudinal_velocity_mps = std::max(getEgoSpeed(), MIN_EGO_VEL_IN_PREDICTION);
  }

  {
    debug_data_.path_with_planned_velocity = path_with_current_velocity;
  }

  return isSafePath(path_with_current_velocity, check_lanes, debug);
}

bool AvoidanceModule::isSafePath(
  const PathWithLaneId & path, const lanelet::ConstLanelets & check_lanes, DebugData & debug) const
{
  if (path.points.empty()) {
    return true;
  }

  const auto path_with_time = [&path]() {
    std::vector<std::pair<PathPointWithLaneId, double>> ret{};

    float travel_time = 0.0;
    ret.emplace_back(path.points.front(), travel_time);

    for (size_t i = 1; i < path.points.size(); ++i) {
      const auto & p1 = path.points.at(i - 1);
      const auto & p2 = path.points.at(i);

      const auto v = std::max(p1.point.longitudinal_velocity_mps, float{1.0});
      const auto ds = calcDistance2d(p1, p2);

      travel_time += ds / v;

      ret.emplace_back(p2, travel_time);
    }

    return ret;
  }();

  const auto move_objects = getAdjacentLaneObjects(check_lanes);

  {
    debug.unsafe_objects.clear();
    debug.margin_data_array.clear();
    debug.exist_adjacent_objects = !move_objects.empty();
  }

  bool is_safe = true;
  for (const auto & p : path_with_time) {
    MarginData margin_data{};
    margin_data.pose = getPose(p.first);

    if (p.second > parameters_->safety_check_time_horizon) {
      break;
    }

    for (const auto & o : move_objects) {
      const auto is_enough_margin = isEnoughMargin(p.first, p.second, o, margin_data);

      if (!is_enough_margin) {
        debug.unsafe_objects.push_back(o);
      }

      is_safe = is_safe && is_enough_margin;
    }

    debug.margin_data_array.push_back(margin_data);
  }

  return is_safe;
}

bool AvoidanceModule::isEnoughMargin(
  const PathPointWithLaneId & p_ego, const double t, const ObjectData & object,
  MarginData & margin_data) const
{
  const auto & common_param = planner_data_->parameters;
  const auto & vehicle_width = common_param.vehicle_width;
  const auto & base_link2front = common_param.base_link2front;
  const auto & base_link2rear = common_param.base_link2rear;

  const auto p_ref = [this, &p_ego]() {
    const auto idx = findNearestIndex(avoidance_data_.reference_path.points, getPoint(p_ego));
    return getPose(avoidance_data_.reference_path.points.at(idx));
  }();

  const auto & v_ego = p_ego.point.longitudinal_velocity_mps;
  const auto & v_ego_lon = utils::avoidance::getLongitudinalVelocity(p_ref, getPose(p_ego), v_ego);
  const auto & v_obj = object.object.kinematics.initial_twist_with_covariance.twist.linear.x;

  if (!utils::avoidance::isTargetObjectType(object.object, parameters_)) {
    return true;
  }

  // |           centerline
  // |               ^ x
  // |  +-------+    |
  // |  |       |    |
  // |  |       | D1 |     D2      D4
  // |  |  obj  |<-->|<---------->|<->|
  // |  |       | D3 |        +-------+
  // |  |       |<----------->|       |
  // |  +-------+    |        |       |
  // |               |        |  ego  |
  // |               |        |       |
  // |               |        |       |
  // |               |        +-------+
  // |        y <----+
  // D1: overhang_dist (signed value)
  // D2: shift_length (signed value)
  // D3: lateral_distance (should be larger than margin that's calculated from relative velocity.)
  // D4: vehicle_width (unsigned value)

  const auto reliable_path = std::max_element(
    object.object.kinematics.predicted_paths.begin(),
    object.object.kinematics.predicted_paths.end(),
    [](const PredictedPath & a, const PredictedPath & b) { return a.confidence < b.confidence; });

  if (reliable_path == object.object.kinematics.predicted_paths.end()) {
    return true;
  }

  const auto p_obj = [&t, &reliable_path]() {
    boost::optional<Pose> ret{boost::none};

    const auto dt = rclcpp::Duration(reliable_path->time_step).seconds();
    const auto idx = static_cast<size_t>(std::floor(t / dt));
    const auto res = t - dt * idx;

    if (idx > reliable_path->path.size() - 2) {
      return ret;
    }

    const auto & p_src = reliable_path->path.at(idx);
    const auto & p_dst = reliable_path->path.at(idx + 1);
    ret = calcInterpolatedPose(p_src, p_dst, res / dt);
    return ret;
  }();

  if (!p_obj) {
    return true;
  }

  const auto v_obj_lon = utils::avoidance::getLongitudinalVelocity(p_ref, p_obj.get(), v_obj);

  double hysteresis_factor = 1.0;
  if (avoidance_data_.state == AvoidanceState::YIELD) {
    hysteresis_factor = parameters_->safety_check_hysteresis_factor;
  }

  const auto shift_length = calcLateralDeviation(p_ref, getPoint(p_ego));
  const auto lateral_distance = std::abs(object.overhang_dist - shift_length) - 0.5 * vehicle_width;
  const auto lateral_margin = getLateralMarginFromVelocity(std::abs(v_ego_lon - v_obj_lon));

  if (lateral_distance > lateral_margin * hysteresis_factor) {
    return true;
  }

  const auto lon_deviation = calcLongitudinalDeviation(getPose(p_ego), p_obj.get().position);
  const auto is_front_object = lon_deviation > 0.0;
  const auto longitudinal_margin =
    getRSSLongitudinalDistance(v_ego_lon, v_obj_lon, is_front_object);
  const auto vehicle_offset = is_front_object ? base_link2front : base_link2rear;
  const auto longitudinal_distance =
    std::abs(lon_deviation) - vehicle_offset - 0.5 * object.object.shape.dimensions.x;

  {
    margin_data.pose.orientation = p_ref.orientation;
    margin_data.enough_lateral_margin = false;
    margin_data.longitudinal_distance =
      std::min(margin_data.longitudinal_distance, longitudinal_distance);
    margin_data.longitudinal_margin =
      std::max(margin_data.longitudinal_margin, longitudinal_margin);
    margin_data.vehicle_width = vehicle_width;
    margin_data.base_link2front = base_link2front;
    margin_data.base_link2rear = base_link2rear;
  }

  if (longitudinal_distance > longitudinal_margin * hysteresis_factor) {
    return true;
  }

  return false;
}

double AvoidanceModule::getLateralMarginFromVelocity(const double velocity) const
{
  const auto & p = parameters_;

  if (p->col_size < 2 || p->col_size * 2 != p->target_velocity_matrix.size()) {
    throw std::logic_error("invalid matrix col size.");
  }

  if (velocity < p->target_velocity_matrix.front()) {
    return p->target_velocity_matrix.at(p->col_size);
  }

  if (velocity > p->target_velocity_matrix.at(p->col_size - 1)) {
    return p->target_velocity_matrix.back();
  }

  for (size_t i = 1; i < p->col_size; ++i) {
    if (velocity < p->target_velocity_matrix.at(i)) {
      const auto v1 = p->target_velocity_matrix.at(i - 1);
      const auto v2 = p->target_velocity_matrix.at(i);
      const auto m1 = p->target_velocity_matrix.at(i - 1 + p->col_size);
      const auto m2 = p->target_velocity_matrix.at(i + p->col_size);

      const auto v_clamp = std::clamp(velocity, v1, v2);
      return m1 + (m2 - m1) * (v_clamp - v1) / (v2 - v1);
    }
  }

  return p->target_velocity_matrix.back();
}

double AvoidanceModule::getRSSLongitudinalDistance(
  const double v_ego, const double v_obj, const bool is_front_object) const
{
  const auto & accel_for_rss = parameters_->safety_check_accel_for_rss;
  const auto & idling_time = parameters_->safety_check_idling_time;

  const auto opposite_lane_vehicle = v_obj < 0.0;

  /**
   * object and ego already pass each other.
   * =======================================
   *                          Ego-->
   * ---------------------------------------
   *       <--Obj
   * =======================================
   */
  if (!is_front_object && opposite_lane_vehicle) {
    return 0.0;
  }

  /**
   * object drive opposite direction.
   * =======================================
   *       Ego-->
   * ---------------------------------------
   *                          <--Obj
   * =======================================
   */
  if (is_front_object && opposite_lane_vehicle) {
    return v_ego * idling_time + 0.5 * accel_for_rss * std::pow(idling_time, 2.0) +
           std::pow(v_ego + accel_for_rss * idling_time, 2.0) / (2.0 * accel_for_rss) +
           std::abs(v_obj) * idling_time + 0.5 * accel_for_rss * std::pow(idling_time, 2.0) +
           std::pow(v_obj + accel_for_rss * idling_time, 2.0) / (2.0 * accel_for_rss);
  }

  /**
   * object is in front of ego, and drive same direction.
   * =======================================
   *       Ego-->
   * ---------------------------------------
   *                          Obj-->
   * =======================================
   */
  if (is_front_object && !opposite_lane_vehicle) {
    return v_ego * idling_time + 0.5 * accel_for_rss * std::pow(idling_time, 2.0) +
           std::pow(v_ego + accel_for_rss * idling_time, 2.0) / (2.0 * accel_for_rss) -
           std::pow(v_obj, 2.0) / (2.0 * accel_for_rss);
  }

  /**
   * object is behind ego, and drive same direction.
   * =======================================
   *                          Ego-->
   * ---------------------------------------
   *       Obj-->
   * =======================================
   */
  if (!is_front_object && !opposite_lane_vehicle) {
    return v_obj * idling_time + 0.5 * accel_for_rss * std::pow(idling_time, 2.0) +
           std::pow(v_obj + accel_for_rss * idling_time, 2.0) / (2.0 * accel_for_rss) -
           std::pow(v_ego, 2.0) / (2.0 * accel_for_rss);
  }

  return 0.0;
}

lanelet::ConstLanelets AvoidanceModule::getAdjacentLane(
  const PathShifter & path_shifter, const double forward_distance,
  const double backward_distance) const
{
  const auto & rh = planner_data_->route_handler;

  bool has_left_shift = false;
  bool has_right_shift = false;

  for (const auto & sp : path_shifter.getShiftLines()) {
    if (sp.end_shift_length > 0.01) {
      has_left_shift = true;
      continue;
    }

    if (sp.end_shift_length < -0.01) {
      has_right_shift = true;
      continue;
    }
  }

  lanelet::ConstLanelet current_lane;
  if (!rh->getClosestLaneletWithinRoute(getEgoPose(), &current_lane)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("behavior_path_planner").get_child("avoidance"),
      "failed to find closest lanelet within route!!!");
    return {};  // TODO(Satoshi Ota)
  }

  const auto ego_succeeding_lanes =
    rh->getLaneletSequence(current_lane, getEgoPose(), backward_distance, forward_distance);

  lanelet::ConstLanelets check_lanes{};
  for (const auto & lane : ego_succeeding_lanes) {
    const auto opt_left_lane = rh->getLeftLanelet(lane);
    if (has_left_shift && opt_left_lane) {
      check_lanes.push_back(opt_left_lane.get());
    }

    const auto opt_right_lane = rh->getRightLanelet(lane);
    if (has_right_shift && opt_right_lane) {
      check_lanes.push_back(opt_right_lane.get());
    }

    const auto right_opposite_lanes = rh->getRightOppositeLanelets(lane);
    if (has_right_shift && !right_opposite_lanes.empty()) {
      check_lanes.push_back(right_opposite_lanes.front());
    }
  }

  return check_lanes;
}

ObjectDataArray AvoidanceModule::getAdjacentLaneObjects(
  const lanelet::ConstLanelets & adjacent_lanes) const
{
  ObjectDataArray objects;
  for (const auto & o : avoidance_data_.other_objects) {
    if (utils::avoidance::isCentroidWithinLanelets(o.object, adjacent_lanes)) {
      objects.push_back(o);
    }
  }

  return objects;
}

void AvoidanceModule::generateExtendedDrivableArea(BehaviorModuleOutput & output) const
{
  const auto has_same_lane =
    [](const lanelet::ConstLanelets lanes, const lanelet::ConstLanelet & lane) {
      if (lanes.empty()) return false;
      const auto has_same = [&](const auto & ll) { return ll.id() == lane.id(); };
      return std::find_if(lanes.begin(), lanes.end(), has_same) != lanes.end();
    };

  const auto & route_handler = planner_data_->route_handler;
  const auto & current_lanes = avoidance_data_.current_lanelets;
  const auto & enable_opposite = parameters_->use_opposite_lane;
  std::vector<DrivableLanes> drivable_lanes;

  for (const auto & current_lane : current_lanes) {
    DrivableLanes current_drivable_lanes;
    current_drivable_lanes.left_lane = current_lane;
    current_drivable_lanes.right_lane = current_lane;

    if (!parameters_->use_adjacent_lane) {
      drivable_lanes.push_back(current_drivable_lanes);
      continue;
    }

    // 1. get left/right side lanes
    const auto update_left_lanelets = [&](const lanelet::ConstLanelet & target_lane) {
      const auto all_left_lanelets =
        route_handler->getAllLeftSharedLinestringLanelets(target_lane, enable_opposite, true);
      if (!all_left_lanelets.empty()) {
        current_drivable_lanes.left_lane = all_left_lanelets.back();  // leftmost lanelet
        pushUniqueVector(
          current_drivable_lanes.middle_lanes,
          lanelet::ConstLanelets(all_left_lanelets.begin(), all_left_lanelets.end() - 1));
      }
    };
    const auto update_right_lanelets = [&](const lanelet::ConstLanelet & target_lane) {
      const auto all_right_lanelets =
        route_handler->getAllRightSharedLinestringLanelets(target_lane, enable_opposite, true);
      if (!all_right_lanelets.empty()) {
        current_drivable_lanes.right_lane = all_right_lanelets.back();  // rightmost lanelet
        pushUniqueVector(
          current_drivable_lanes.middle_lanes,
          lanelet::ConstLanelets(all_right_lanelets.begin(), all_right_lanelets.end() - 1));
      }
    };

    update_left_lanelets(current_lane);
    update_right_lanelets(current_lane);

    // 2.1 when there are multiple lanes whose previous lanelet is the same
    const auto get_next_lanes_from_same_previous_lane =
      [&route_handler](const lanelet::ConstLanelet & lane) {
        // get previous lane, and return false if previous lane does not exist
        lanelet::ConstLanelets prev_lanes;
        if (!route_handler->getPreviousLaneletsWithinRoute(lane, &prev_lanes)) {
          return lanelet::ConstLanelets{};
        }

        lanelet::ConstLanelets next_lanes;
        for (const auto & prev_lane : prev_lanes) {
          const auto next_lanes_from_prev = route_handler->getNextLanelets(prev_lane);
          pushUniqueVector(next_lanes, next_lanes_from_prev);
        }
        return next_lanes;
      };

    const auto next_lanes_for_right =
      get_next_lanes_from_same_previous_lane(current_drivable_lanes.right_lane);
    const auto next_lanes_for_left =
      get_next_lanes_from_same_previous_lane(current_drivable_lanes.left_lane);

    // 2.2 look for neighbor lane recursively, where end line of the lane is connected to end line
    // of the original lane
    const auto update_drivable_lanes =
      [&](const lanelet::ConstLanelets & next_lanes, const bool is_left) {
        for (const auto & next_lane : next_lanes) {
          const auto & edge_lane =
            is_left ? current_drivable_lanes.left_lane : current_drivable_lanes.right_lane;
          if (next_lane.id() == edge_lane.id()) {
            continue;
          }

          const auto & left_lane = is_left ? next_lane : edge_lane;
          const auto & right_lane = is_left ? edge_lane : next_lane;
          if (!isEndPointsConnected(left_lane, right_lane)) {
            continue;
          }

          if (is_left) {
            current_drivable_lanes.left_lane = next_lane;
          } else {
            current_drivable_lanes.right_lane = next_lane;
          }

          if (!has_same_lane(current_drivable_lanes.middle_lanes, edge_lane)) {
            if (is_left) {
              if (current_drivable_lanes.right_lane.id() != edge_lane.id()) {
                current_drivable_lanes.middle_lanes.push_back(edge_lane);
              }
            } else {
              if (current_drivable_lanes.left_lane.id() != edge_lane.id()) {
                current_drivable_lanes.middle_lanes.push_back(edge_lane);
              }
            }
          }

          return true;
        }
        return false;
      };

    const auto expand_drivable_area_recursively =
      [&](const lanelet::ConstLanelets & next_lanes, const bool is_left) {
        // NOTE: set max search num to avoid infinity loop for drivable area expansion
        constexpr size_t max_recursive_search_num = 3;
        for (size_t i = 0; i < max_recursive_search_num; ++i) {
          const bool is_update_kept = update_drivable_lanes(next_lanes, is_left);
          if (!is_update_kept) {
            break;
          }
          if (i == max_recursive_search_num - 1) {
            RCLCPP_ERROR(
              rclcpp::get_logger("behavior_path_planner").get_child("avoidance"),
              "Drivable area expansion reaches max iteration.");
          }
        }
      };
    expand_drivable_area_recursively(next_lanes_for_right, false);
    expand_drivable_area_recursively(next_lanes_for_left, true);

    // 3. update again for new left/right lanes
    update_left_lanelets(current_drivable_lanes.left_lane);
    update_right_lanelets(current_drivable_lanes.right_lane);

    // 4. compensate that current_lane is in either of left_lane, right_lane or middle_lanes.
    if (
      current_drivable_lanes.left_lane.id() != current_lane.id() &&
      current_drivable_lanes.right_lane.id() != current_lane.id()) {
      current_drivable_lanes.middle_lanes.push_back(current_lane);
    }

    drivable_lanes.push_back(current_drivable_lanes);
  }

  {  // for new architecture
    DrivableAreaInfo current_drivable_area_info;
    // generate drivable lanes
    current_drivable_area_info.drivable_lanes = drivable_lanes;
    // generate obstacle polygons
    current_drivable_area_info.obstacles =
      utils::avoidance::generateObstaclePolygonsForDrivableArea(
        avoidance_data_.target_objects, parameters_, planner_data_->parameters.vehicle_width / 2.0);
    // expand hatched road markings
    current_drivable_area_info.enable_expanding_hatched_road_markings =
      parameters_->use_hatched_road_markings;
    // expand intersection areas
    current_drivable_area_info.enable_expanding_intersection_areas =
      parameters_->use_intersection_areas;

    output.drivable_area_info = utils::combineDrivableAreaInfo(
      current_drivable_area_info, getPreviousModuleOutput().drivable_area_info);
  }
}

PathWithLaneId AvoidanceModule::extendBackwardLength(const PathWithLaneId & original_path) const
{
  // special for avoidance: take behind distance upt ot shift-start-point if it exist.
  const auto longest_dist_to_shift_point = [&]() {
    double max_dist = 0.0;
    for (const auto & pnt : path_shifter_.getShiftLines()) {
      max_dist = std::max(max_dist, calcDistance2d(getEgoPose(), pnt.start));
    }
    for (const auto & sp : registered_raw_shift_lines_) {
      max_dist = std::max(max_dist, calcDistance2d(getEgoPose(), sp.start));
    }
    return max_dist;
  }();

  const auto extra_margin = 10.0;  // Since distance does not consider arclength, but just line.
  const auto backward_length = std::max(
    planner_data_->parameters.backward_path_length, longest_dist_to_shift_point + extra_margin);
  const auto previous_path = helper_.getPreviousReferencePath();

  const size_t orig_ego_idx = planner_data_->findEgoIndex(original_path.points);
  const size_t prev_ego_idx =
    findNearestSegmentIndex(previous_path.points, getPoint(original_path.points.at(orig_ego_idx)));

  size_t clip_idx = 0;
  for (size_t i = 0; i < prev_ego_idx; ++i) {
    if (backward_length > calcSignedArcLength(previous_path.points, clip_idx, prev_ego_idx)) {
      break;
    }
    clip_idx = i;
  }

  PathWithLaneId extended_path{};
  {
    extended_path.points.insert(
      extended_path.points.end(), previous_path.points.begin() + clip_idx,
      previous_path.points.begin() + prev_ego_idx);
  }

  // overwrite backward path velocity by latest one.
  std::for_each(extended_path.points.begin(), extended_path.points.end(), [&](auto & p) {
    p.point.longitudinal_velocity_mps =
      original_path.points.at(orig_ego_idx).point.longitudinal_velocity_mps;
  });

  {
    extended_path.points.insert(
      extended_path.points.end(), original_path.points.begin() + orig_ego_idx,
      original_path.points.end());
  }

  return extended_path;
}

BehaviorModuleOutput AvoidanceModule::plan()
{
  const auto & data = avoidance_data_;

  resetPathCandidate();
  resetPathReference();

  /**
   * Has new shift point?
   *   Yes -> Is it approved?
   *       Yes -> add the shift point.
   *       No  -> set approval_handler to WAIT_APPROVAL state.
   *   No -> waiting approval?
   *       Yes -> clear WAIT_APPROVAL state.
   *       No  -> do nothing.
   */
  if (!data.safe_new_sl.empty()) {
    debug_data_.new_shift_lines = data.safe_new_sl;
    DEBUG_PRINT("new_shift_lines size = %lu", data.safe_new_sl.size());
    printShiftLines(data.safe_new_sl, "new_shift_lines");

    const auto sl = helper_.getMainShiftLine(data.safe_new_sl);
    if (helper_.getRelativeShiftToPath(sl) > 0.0) {
      removePreviousRTCStatusRight();
    } else if (helper_.getRelativeShiftToPath(sl) < 0.0) {
      removePreviousRTCStatusLeft();
    } else {
      RCLCPP_WARN_STREAM(getLogger(), "Direction is UNKNOWN");
    }
    if (!parameters_->disable_path_update) {
      addShiftLineIfApproved(data.safe_new_sl);
    }
  } else if (isWaitingApproval()) {
    clearWaitingApproval();
    removeCandidateRTCStatus();
  }

  // generate path with shift points that have been inserted.
  ShiftedPath linear_shift_path = utils::avoidance::toShiftedPath(data.reference_path);
  ShiftedPath spline_shift_path = utils::avoidance::toShiftedPath(data.reference_path);
  const auto success_spline_path_generation =
    path_shifter_.generate(&spline_shift_path, true, SHIFT_TYPE::SPLINE);
  const auto success_linear_path_generation =
    path_shifter_.generate(&linear_shift_path, true, SHIFT_TYPE::LINEAR);

  // set previous data
  if (success_spline_path_generation && success_linear_path_generation) {
    helper_.setPreviousLinearShiftPath(linear_shift_path);
    helper_.setPreviousSplineShiftPath(spline_shift_path);
    helper_.setPreviousReferencePath(data.reference_path);
  } else {
    spline_shift_path = helper_.getPreviousSplineShiftPath();
  }

  // post processing
  {
    postProcess();  // remove old shift points
  }

  BehaviorModuleOutput output;

  // turn signal info
  {
    const auto original_signal = getPreviousModuleOutput().turn_signal_info;
    const auto new_signal = calcTurnSignalInfo(spline_shift_path);
    const auto current_seg_idx = planner_data_->findEgoSegmentIndex(spline_shift_path.path.points);
    output.turn_signal_info = planner_data_->turn_signal_decider.use_prior_turn_signal(
      spline_shift_path.path, getEgoPose(), current_seg_idx, original_signal, new_signal,
      planner_data_->parameters.ego_nearest_dist_threshold,
      planner_data_->parameters.ego_nearest_yaw_threshold);
  }

  // sparse resampling for computational cost
  {
    spline_shift_path.path = utils::resamplePathWithSpline(
      spline_shift_path.path, parameters_->resample_interval_for_output);
  }

  avoidance_data_.state = updateEgoState(data);

  // update output data
  {
    updateEgoBehavior(data, spline_shift_path);
    updateInfoMarker(avoidance_data_);
    updateDebugMarker(avoidance_data_, path_shifter_, debug_data_);
  }

  output.path = std::make_shared<PathWithLaneId>(spline_shift_path.path);
  output.reference_path = getPreviousModuleOutput().reference_path;
  path_reference_ = getPreviousModuleOutput().reference_path;

  const size_t ego_idx = planner_data_->findEgoIndex(output.path->points);
  utils::clipPathLength(*output.path, ego_idx, planner_data_->parameters);

  // Drivable area generation.
  generateExtendedDrivableArea(output);

  updateRegisteredRTCStatus(spline_shift_path.path);

  return output;
}

CandidateOutput AvoidanceModule::planCandidate() const
{
  const auto & data = avoidance_data_;

  CandidateOutput output;

  auto shifted_path = data.candidate_path;

  if (!data.safe_new_sl.empty()) {  // clip from shift start index for visualize
    utils::clipPathLength(
      shifted_path.path, data.safe_new_sl.front().start_idx, std::numeric_limits<double>::max(),
      0.0);

    const auto sl = helper_.getMainShiftLine(data.safe_new_sl);
    const auto sl_front = data.safe_new_sl.front();
    const auto sl_back = data.safe_new_sl.back();

    output.lateral_shift = helper_.getRelativeShiftToPath(sl);
    output.start_distance_to_path_change = sl_front.start_longitudinal;
    output.finish_distance_to_path_change = sl_back.end_longitudinal;

    const uint16_t steering_factor_direction = std::invoke([&output]() {
      if (output.lateral_shift > 0.0) {
        return SteeringFactor::LEFT;
      }
      return SteeringFactor::RIGHT;
    });
    steering_factor_interface_ptr_->updateSteeringFactor(
      {sl_front.start, sl_back.end},
      {output.start_distance_to_path_change, output.finish_distance_to_path_change},
      SteeringFactor::AVOIDANCE_PATH_CHANGE, steering_factor_direction, SteeringFactor::APPROACHING,
      "");
  }

  const size_t ego_idx = planner_data_->findEgoIndex(shifted_path.path.points);
  utils::clipPathLength(shifted_path.path, ego_idx, planner_data_->parameters);

  output.path_candidate = shifted_path.path;

  return output;
}

BehaviorModuleOutput AvoidanceModule::planWaitingApproval()
{
  const auto & data = avoidance_data_;

  // we can execute the plan() since it handles the approval appropriately.
  BehaviorModuleOutput out = plan();

  if (path_shifter_.getShiftLines().empty()) {
    out.turn_signal_info = getPreviousModuleOutput().turn_signal_info;
  }

  const auto all_unavoidable = std::all_of(
    data.target_objects.begin(), data.target_objects.end(),
    [](const auto & o) { return !o.is_avoidable; });

  const auto candidate = planCandidate();
  if (!data.safe_new_sl.empty()) {
    updateCandidateRTCStatus(candidate);
    waitApproval();
  } else if (path_shifter_.getShiftLines().empty()) {
    waitApproval();
  } else if (all_unavoidable) {
    waitApproval();
  } else {
    clearWaitingApproval();
    removeCandidateRTCStatus();
  }

  path_candidate_ = std::make_shared<PathWithLaneId>(candidate.path_candidate);
  path_reference_ = getPreviousModuleOutput().reference_path;

  return out;
}

void AvoidanceModule::addShiftLineIfApproved(const AvoidLineArray & shift_lines)
{
  if (isActivated()) {
    DEBUG_PRINT("We want to add this shift point, and approved. ADD SHIFT POINT!");
    const size_t prev_size = path_shifter_.getShiftLinesSize();
    addNewShiftLines(path_shifter_, shift_lines);

    current_raw_shift_lines_ = avoidance_data_.unapproved_raw_sl;

    // register original points for consistency
    registerRawShiftLines(shift_lines);

    const auto sl = helper_.getMainShiftLine(shift_lines);
    const auto sl_front = shift_lines.front();
    const auto sl_back = shift_lines.back();

    if (helper_.getRelativeShiftToPath(sl) > 0.0) {
      left_shift_array_.push_back({uuid_map_.at("left"), sl_front.start, sl_back.end});
    } else if (helper_.getRelativeShiftToPath(sl) < 0.0) {
      right_shift_array_.push_back({uuid_map_.at("right"), sl_front.start, sl_back.end});
    }

    uuid_map_.at("left") = generateUUID();
    uuid_map_.at("right") = generateUUID();
    candidate_uuid_ = generateUUID();

    lockNewModuleLaunch();

    DEBUG_PRINT("shift_line size: %lu -> %lu", prev_size, path_shifter_.getShiftLinesSize());
  } else {
    DEBUG_PRINT("We want to add this shift point, but NOT approved. waiting...");
    waitApproval();
  }
}

/**
 * set new shift points. remove old shift points if it has a conflict.
 */
void AvoidanceModule::addNewShiftLines(
  PathShifter & path_shifter, const AvoidLineArray & new_shift_lines) const
{
  ShiftLineArray future = utils::avoidance::toShiftLineArray(new_shift_lines);

  size_t min_start_idx = std::numeric_limits<size_t>::max();
  for (const auto & sl : new_shift_lines) {
    min_start_idx = std::min(min_start_idx, sl.start_idx);
  }

  const auto current_shift_lines = path_shifter.getShiftLines();
  const auto front_new_shift_line = new_shift_lines.front();
  const auto new_shift_length = front_new_shift_line.end_shift_length;
  const auto new_shift_end_idx = front_new_shift_line.end_idx;

  DEBUG_PRINT("min_start_idx = %lu", min_start_idx);

  // Remove shift points that starts later than the new_shift_line from path_shifter.
  //
  // Why? Because shifter sorts by start position and applies shift points, so if there is a
  // shift point that starts after the one you are going to put in, new one will be affected
  // by the old one.
  //
  // Is it ok? This is a situation where the vehicle was originally going to avoid at the farther
  // point, but decided to avoid it at a closer point. In this case, it is reasonable to cancel the
  // farther avoidance.
  for (const auto & sl : current_shift_lines) {
    if (sl.start_idx >= min_start_idx) {
      DEBUG_PRINT(
        "sl.start_idx = %lu, this sl starts after new proposal. remove this one.", sl.start_idx);
      continue;
    }

    if (sl.end_idx >= new_shift_end_idx) {
      if (
        sl.end_shift_length > -1e-3 && new_shift_length > -1e-3 &&
        sl.end_shift_length < new_shift_length) {
        continue;
      }

      if (
        sl.end_shift_length < 1e-3 && new_shift_length < 1e-3 &&
        sl.end_shift_length > new_shift_length) {
        continue;
      }
    }

    DEBUG_PRINT("sl.start_idx = %lu, no conflict. keep this one.", sl.start_idx);
    future.push_back(sl);
  }

  const double road_velocity =
    avoidance_data_.reference_path.points.at(front_new_shift_line.start_idx)
      .point.longitudinal_velocity_mps;
  const double shift_time = PathShifter::calcShiftTimeFromJerk(
    front_new_shift_line.getRelativeLength(), helper_.getLateralMaxJerkLimit(),
    helper_.getLateralMaxAccelLimit());
  const double longitudinal_acc =
    std::clamp(road_velocity / shift_time, 0.0, parameters_->max_acceleration);

  path_shifter.setShiftLines(future);
  path_shifter.setVelocity(getEgoSpeed());
  path_shifter.setLongitudinalAcceleration(longitudinal_acc);
  path_shifter.setLateralAccelerationLimit(helper_.getLateralMaxAccelLimit());
}

AvoidLineArray AvoidanceModule::findNewShiftLine(const AvoidLineArray & candidates) const
{
  if (candidates.empty()) {
    return {};
  }

  // add small shift lines.
  const auto add_straight_shift =
    [&, this](auto & subsequent, bool has_large_shift, const size_t start_idx) {
      for (size_t i = start_idx; i < candidates.size(); ++i) {
        if (
          std::abs(candidates.at(i).getRelativeLength()) >
          parameters_->lateral_small_shift_threshold) {
          if (has_large_shift) {
            break;
          }

          has_large_shift = true;
        }

        subsequent.push_back(candidates.at(i));
      }
    };

  // get subsequent shift lines.
  const auto get_subsequent_shift = [&, this](size_t i) {
    AvoidLineArray subsequent{candidates.at(i)};

    if (candidates.size() == i + 1) {
      return subsequent;
    }

    if (
      std::abs(candidates.at(i).getRelativeLength()) < parameters_->lateral_small_shift_threshold) {
      const auto has_large_shift =
        candidates.at(i + 1).getRelativeLength() > parameters_->lateral_small_shift_threshold;

      // candidate.at(i) is small length shift line. add large length shift line.
      subsequent.push_back(candidates.at(i + 1));
      add_straight_shift(subsequent, has_large_shift, i + 2);
    } else {
      // candidate.at(i) is large length shift line. add small length shift lines.
      add_straight_shift(subsequent, true, i + 1);
    }

    return subsequent;
  };

  // check jerk limit.
  const auto is_large_jerk = [this](const auto & s) {
    const auto jerk = PathShifter::calcJerkFromLatLonDistance(
      s.getRelativeLength(), s.getRelativeLongitudinal(), helper_.getAvoidanceEgoSpeed());
    return jerk > helper_.getLateralMaxJerkLimit();
  };

  // check ignore or not.
  const auto is_ignore_shift = [this](const auto & s) {
    return std::abs(helper_.getRelativeShiftToPath(s)) < parameters_->lateral_execution_threshold;
  };

  for (size_t i = 0; i < candidates.size(); ++i) {
    const auto & candidate = candidates.at(i);

    // new shift points must exist in front of Ego
    // this value should be larger than -eps consider path shifter calculation error.
    const double eps = 0.01;
    if (candidate.start_longitudinal < -eps) {
      break;
    }

    if (!is_ignore_shift(candidate)) {
      if (is_large_jerk(candidate)) {
        break;
      }

      return get_subsequent_shift(i);
    }
  }

  DEBUG_PRINT("No new shift point exists.");
  return {};
}

bool AvoidanceModule::isValidShiftLine(
  const AvoidLineArray & shift_lines, const PathShifter & shifter) const
{
  if (shift_lines.empty()) {
    return false;
  }

  auto shifter_for_validate = shifter;

  addNewShiftLines(shifter_for_validate, shift_lines);

  ShiftedPath proposed_shift_path;
  shifter_for_validate.generate(&proposed_shift_path);

  // check offset between new shift path and ego position.
  {
    const auto new_idx = planner_data_->findEgoIndex(proposed_shift_path.path.points);
    const auto new_shift_length = proposed_shift_path.shift_length.at(new_idx);

    constexpr double THRESHOLD = 0.1;
    const auto offset = std::abs(new_shift_length - helper_.getEgoShift());
    if (offset > THRESHOLD) {
      RCLCPP_WARN_THROTTLE(
        getLogger(), *clock_, 1000, "new shift line is invalid. [HUGE OFFSET (%.2f)]", offset);
      return false;
    }
  }

  debug_data_.proposed_spline_shift = proposed_shift_path.shift_length;

  return true;  // valid shift line.
}

void AvoidanceModule::updateData()
{
  using utils::avoidance::toShiftedPath;

  helper_.setData(planner_data_);

  if (!helper_.isInitialized()) {
    helper_.setPreviousSplineShiftPath(toShiftedPath(*getPreviousModuleOutput().path));
    helper_.setPreviousLinearShiftPath(toShiftedPath(*getPreviousModuleOutput().path));
    helper_.setPreviousReferencePath(*getPreviousModuleOutput().path);
    helper_.setPreviousDrivingLanes(utils::avoidance::getCurrentLanesFromPath(
      *getPreviousModuleOutput().reference_path, planner_data_));
  }

  debug_data_ = DebugData();
  avoidance_data_ = calcAvoidancePlanningData(debug_data_);

  utils::avoidance::updateRegisteredObject(
    registered_objects_, avoidance_data_.target_objects, parameters_);
  utils::avoidance::compensateDetectionLost(
    registered_objects_, avoidance_data_.target_objects, avoidance_data_.other_objects);

  std::sort(
    avoidance_data_.target_objects.begin(), avoidance_data_.target_objects.end(),
    [](auto a, auto b) { return a.longitudinal < b.longitudinal; });

  path_shifter_.setPath(avoidance_data_.reference_path);

  // update registered shift point for new reference path & remove past objects
  updateRegisteredRawShiftLines();

  fillShiftLine(avoidance_data_, debug_data_);
  fillEgoStatus(avoidance_data_, debug_data_);
  fillDebugData(avoidance_data_, debug_data_);
}

void AvoidanceModule::processOnEntry()
{
  initVariables();
  waitApproval();
}

void AvoidanceModule::processOnExit()
{
  initVariables();
  initRTCStatus();
}

void AvoidanceModule::initVariables()
{
  helper_.reset();
  path_shifter_ = PathShifter{};

  debug_data_ = DebugData();
  debug_marker_.markers.clear();
  resetPathCandidate();
  resetPathReference();
  registered_raw_shift_lines_ = {};
  current_raw_shift_lines_ = {};
  original_unique_id = 0;
  is_avoidance_maneuver_starts = false;
  arrived_path_end_ = false;
}

void AvoidanceModule::initRTCStatus()
{
  removeRTCStatus();
  clearWaitingApproval();
  left_shift_array_.clear();
  right_shift_array_.clear();
  uuid_map_.at("left") = generateUUID();
  uuid_map_.at("right") = generateUUID();
  candidate_uuid_ = generateUUID();
}

TurnSignalInfo AvoidanceModule::calcTurnSignalInfo(const ShiftedPath & path) const
{
  const auto shift_lines = path_shifter_.getShiftLines();
  if (shift_lines.empty()) {
    return {};
  }

  const auto front_shift_line = shift_lines.front();
  const size_t start_idx = front_shift_line.start_idx;
  const size_t end_idx = front_shift_line.end_idx;

  const auto current_shift_length = helper_.getEgoShift();
  const double start_shift_length = path.shift_length.at(start_idx);
  const double end_shift_length = path.shift_length.at(end_idx);
  const double segment_shift_length = end_shift_length - start_shift_length;

  const double turn_signal_shift_length_threshold =
    planner_data_->parameters.turn_signal_shift_length_threshold;
  const double turn_signal_search_time = planner_data_->parameters.turn_signal_search_time;
  const double turn_signal_minimum_search_distance =
    planner_data_->parameters.turn_signal_minimum_search_distance;

  // If shift length is shorter than the threshold, it does not need to turn on blinkers
  if (std::fabs(segment_shift_length) < turn_signal_shift_length_threshold) {
    return {};
  }

  // If the vehicle does not shift anymore, we turn off the blinker
  if (std::fabs(end_shift_length - current_shift_length) < 0.1) {
    return {};
  }

  // compute blinker start idx and end idx
  size_t blinker_start_idx = [&]() {
    for (size_t idx = start_idx; idx <= end_idx; ++idx) {
      const double current_shift_length = path.shift_length.at(idx);
      if (current_shift_length > 0.1) {
        return idx;
      }
    }
    return start_idx;
  }();
  size_t blinker_end_idx = end_idx;

  // prevent invalid access for out-of-range
  blinker_start_idx =
    std::min(std::max(std::size_t(0), blinker_start_idx), path.path.points.size() - 1);
  blinker_end_idx =
    std::min(std::max(blinker_start_idx, blinker_end_idx), path.path.points.size() - 1);

  const auto blinker_start_pose = path.path.points.at(blinker_start_idx).point.pose;
  const auto blinker_end_pose = path.path.points.at(blinker_end_idx).point.pose;

  const double ego_vehicle_offset =
    planner_data_->parameters.vehicle_info.max_longitudinal_offset_m;
  const auto signal_prepare_distance =
    std::max(getEgoSpeed() * turn_signal_search_time, turn_signal_minimum_search_distance);
  const auto ego_front_to_shift_start =
    calcSignedArcLength(path.path.points, getEgoPosition(), blinker_start_pose.position) -
    ego_vehicle_offset;

  if (signal_prepare_distance < ego_front_to_shift_start) {
    return {};
  }

  bool turn_signal_on_swerving = planner_data_->parameters.turn_signal_on_swerving;

  TurnSignalInfo turn_signal_info{};
  if (turn_signal_on_swerving) {
    if (segment_shift_length > 0.0) {
      turn_signal_info.turn_signal.command = TurnIndicatorsCommand::ENABLE_LEFT;
    } else {
      turn_signal_info.turn_signal.command = TurnIndicatorsCommand::ENABLE_RIGHT;
    }
  } else {
    turn_signal_info.turn_signal.command = TurnIndicatorsCommand::DISABLE;
  }

  if (ego_front_to_shift_start > 0.0) {
    turn_signal_info.desired_start_point = planner_data_->self_odometry->pose.pose;
  } else {
    turn_signal_info.desired_start_point = blinker_start_pose;
  }
  turn_signal_info.desired_end_point = blinker_end_pose;
  turn_signal_info.required_start_point = blinker_start_pose;
  turn_signal_info.required_end_point = blinker_end_pose;

  return turn_signal_info;
}

void AvoidanceModule::updateInfoMarker(const AvoidancePlanningData & data) const
{
  using marker_utils::avoidance_marker::createTargetObjectsMarkerArray;

  info_marker_.markers.clear();
  appendMarkerArray(
    createTargetObjectsMarkerArray(data.target_objects, "target_objects"), &info_marker_);
}

void AvoidanceModule::updateDebugMarker(
  const AvoidancePlanningData & data, const PathShifter & shifter, const DebugData & debug) const
{
  using marker_utils::createLaneletsAreaMarkerArray;
  using marker_utils::createObjectsMarkerArray;
  using marker_utils::createPathMarkerArray;
  using marker_utils::createPoseMarkerArray;
  using marker_utils::createShiftGradMarkerArray;
  using marker_utils::createShiftLengthMarkerArray;
  using marker_utils::createShiftLineMarkerArray;
  using marker_utils::avoidance_marker::createAvoidLineMarkerArray;
  using marker_utils::avoidance_marker::createEgoStatusMarkerArray;
  using marker_utils::avoidance_marker::createOtherObjectsMarkerArray;
  using marker_utils::avoidance_marker::createOverhangFurthestLineStringMarkerArray;
  using marker_utils::avoidance_marker::createPredictedVehiclePositions;
  using marker_utils::avoidance_marker::createSafetyCheckMarkerArray;
  using marker_utils::avoidance_marker::createUnsafeObjectsMarkerArray;
  using marker_utils::avoidance_marker::makeOverhangToRoadShoulderMarkerArray;
  using tier4_autoware_utils::appendMarkerArray;

  debug_marker_.markers.clear();

  if (!parameters_->publish_debug_marker) {
    return;
  }

  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();

  const auto add = [this](const MarkerArray & added) { appendMarkerArray(added, &debug_marker_); };

  const auto addAvoidLine =
    [&](const AvoidLineArray & al_arr, const auto & ns, auto r, auto g, auto b, double w = 0.05) {
      add(createAvoidLineMarkerArray(al_arr, ns, r, g, b, w));
    };

  const auto addShiftLine =
    [&](const ShiftLineArray & sl_arr, const auto & ns, auto r, auto g, auto b, double w = 0.1) {
      add(createShiftLineMarkerArray(sl_arr, shifter.getBaseOffset(), ns, r, g, b, w));
    };

  add(createEgoStatusMarkerArray(data, getEgoPose(), "ego_status"));
  add(createPredictedVehiclePositions(
    debug.path_with_planned_velocity, "predicted_vehicle_positions"));

  const auto & path = data.reference_path;
  add(createPathMarkerArray(debug.center_line, "centerline", 0, 0.0, 0.5, 0.9));
  add(createPathMarkerArray(path, "centerline_resampled", 0, 0.0, 0.9, 0.5));
  add(createPathMarkerArray(
    helper_.getPreviousLinearShiftPath().path, "prev_linear_shift", 0, 0.5, 0.4, 0.6));
  add(createPoseMarkerArray(data.reference_pose, "reference_pose", 0, 0.9, 0.3, 0.3));

  add(createSafetyCheckMarkerArray(data.state, getEgoPose(), debug));

  add(createLaneletsAreaMarkerArray(*debug.current_lanelets, "current_lanelet", 0.0, 1.0, 0.0));
  add(createLaneletsAreaMarkerArray(*debug.expanded_lanelets, "expanded_lanelet", 0.8, 0.8, 0.0));

  add(createOtherObjectsMarkerArray(
    data.other_objects, AvoidanceDebugFactor::OBJECT_IS_BEHIND_THRESHOLD));
  add(createOtherObjectsMarkerArray(
    data.other_objects, AvoidanceDebugFactor::OBJECT_IS_IN_FRONT_THRESHOLD));
  add(createOtherObjectsMarkerArray(
    data.other_objects, AvoidanceDebugFactor::OBJECT_BEHIND_PATH_GOAL));
  add(createOtherObjectsMarkerArray(
    data.other_objects, AvoidanceDebugFactor::TOO_NEAR_TO_CENTERLINE));
  add(createOtherObjectsMarkerArray(data.other_objects, AvoidanceDebugFactor::OBJECT_IS_NOT_TYPE));
  add(createOtherObjectsMarkerArray(data.other_objects, AvoidanceDebugFactor::NOT_PARKING_OBJECT));
  add(createOtherObjectsMarkerArray(data.other_objects, std::string("MovingObject")));
  add(createOtherObjectsMarkerArray(data.other_objects, std::string("CrosswalkUser")));
  add(createOtherObjectsMarkerArray(data.other_objects, std::string("OutOfTargetArea")));
  add(createOtherObjectsMarkerArray(data.other_objects, std::string("NotNeedAvoidance")));
  add(createOtherObjectsMarkerArray(data.other_objects, std::string("LessThanExecutionThreshold")));

  add(makeOverhangToRoadShoulderMarkerArray(data.target_objects, "overhang"));
  add(createOverhangFurthestLineStringMarkerArray(debug.bounds, "bounds", 1.0, 0.0, 1.0));

  add(createUnsafeObjectsMarkerArray(debug.unsafe_objects, "unsafe_objects"));

  // parent object info
  addAvoidLine(debug.registered_raw_shift, "p_registered_shift", 0.8, 0.8, 0.0);
  addAvoidLine(debug.current_raw_shift, "p_current_raw_shift", 0.5, 0.2, 0.2);
  addAvoidLine(debug.extra_return_shift, "p_extra_return_shift", 0.0, 0.5, 0.8);

  // shift length
  {
    const std::string ns = "shift_length";
    add(createShiftLengthMarkerArray(debug.pos_shift, path, ns + "_pos", 0.0, 0.7, 0.5));
    add(createShiftLengthMarkerArray(debug.neg_shift, path, ns + "_neg", 0.0, 0.5, 0.7));
    add(createShiftLengthMarkerArray(debug.total_shift, path, ns + "_total", 0.99, 0.4, 0.2));
  }

  // shift grad
  {
    const std::string ns = "shift_grad";
    add(createShiftGradMarkerArray(
      debug.pos_shift_grad, debug.pos_shift, path, ns + "_pos", 0.0, 0.7, 0.5));
    add(createShiftGradMarkerArray(
      debug.neg_shift_grad, debug.neg_shift, path, ns + "_neg", 0.0, 0.5, 0.7));
    add(createShiftGradMarkerArray(
      debug.total_forward_grad, debug.total_shift, path, ns + "_total_forward", 0.99, 0.4, 0.2));
    add(createShiftGradMarkerArray(
      debug.total_backward_grad, debug.total_shift, path, ns + "_total_backward", 0.4, 0.2, 0.99));
  }

  // shift path
  {
    const std::string ns = "shift_line";
    add(createShiftLengthMarkerArray(
      helper_.getPreviousLinearShiftPath().shift_length, path, ns + "_linear_registered", 0.9, 0.3,
      0.3));
    add(createShiftLengthMarkerArray(
      debug.proposed_spline_shift, path, ns + "_spline_proposed", 1.0, 1.0, 1.0));
  }

  // child shift points
  {
    const std::string ns = "pipeline";
    add(createAvoidLineMarkerArray(debug.gap_filled, ns + "_1_gap_filled", 0.5, 0.8, 1.0, 0.05));
    add(createAvoidLineMarkerArray(debug.merged, ns + "_2_merge", 0.345, 0.968, 1.0, 0.05));
    add(createAvoidLineMarkerArray(
      debug.trim_similar_grad_shift, ns + "_3_concat_by_grad", 0.976, 0.328, 0.910, 0.05));
    add(
      createAvoidLineMarkerArray(debug.quantized, ns + "_4_quantized", 0.505, 0.745, 0.969, 0.05));
    add(createAvoidLineMarkerArray(
      debug.trim_small_shift, ns + "_5_trim_small_shift", 0.663, 0.525, 0.941, 0.05));
    add(createAvoidLineMarkerArray(
      debug.trim_similar_grad_shift_second, ns + "_6_concat_by_grad", 0.97, 0.32, 0.91, 0.05));
    add(createAvoidLineMarkerArray(
      debug.trim_momentary_return, ns + "_7_trim_momentary_return", 0.976, 0.078, 0.878, 0.05));
    add(createAvoidLineMarkerArray(
      debug.trim_too_sharp_shift, ns + "_8_trim_sharp_shift", 0.576, 0.0, 0.978, 0.05));
    add(createAvoidLineMarkerArray(
      debug.trim_similar_grad_shift_third, ns + "_9_concat_by_grad", 1.0, 0.0, 0.0, 0.05));
  }

  addShiftLine(shifter.getShiftLines(), "path_shifter_registered_points", 0.99, 0.99, 0.0, 0.5);
  addAvoidLine(debug.new_shift_lines, "path_shifter_proposed_points", 0.99, 0.0, 0.0, 0.5);
}

void AvoidanceModule::updateAvoidanceDebugData(
  std::vector<AvoidanceDebugMsg> & avoidance_debug_msg_array) const
{
  debug_data_.avoidance_debug_msg_array.avoidance_info.clear();
  auto & debug_data_avoidance = debug_data_.avoidance_debug_msg_array.avoidance_info;
  debug_data_avoidance = avoidance_debug_msg_array;
  if (!debug_avoidance_initializer_for_shift_line_.empty()) {
    const bool is_info_old_ =
      (clock_->now() - debug_avoidance_initializer_for_shift_line_time_).seconds() > 0.1;
    if (!is_info_old_) {
      debug_data_avoidance.insert(
        debug_data_avoidance.end(), debug_avoidance_initializer_for_shift_line_.begin(),
        debug_avoidance_initializer_for_shift_line_.end());
    }
  }
}

double AvoidanceModule::calcDistanceToStopLine(const ObjectData & object) const
{
  const auto & p = parameters_;
  const auto & base_link2front = planner_data_->parameters.base_link2front;
  const auto & vehicle_width = planner_data_->parameters.vehicle_width;

  //         D5
  //      |<---->|                               D4
  //      |<----------------------------------------------------------------------->|
  // +-----------+            D1                 D2                      D3         +-----------+
  // |           |        |<------->|<------------------------->|<----------------->|           |
  // |    ego    |======= x ======= x ========================= x ==================|    obj    |
  // |           |    stop_point  avoid                       avoid                 |           |
  // +-----------+                start                        end                  +-----------+
  //
  // D1: p.min_prepare_distance
  // D2: min_avoid_distance
  // D3: longitudinal_avoid_margin_front (margin + D5)
  // D4: o_front.longitudinal
  // D5: base_link2front

  const auto object_type = utils::getHighestProbLabel(object.object.classification);
  const auto object_parameter = parameters_->object_parameters.at(object_type);

  const auto avoid_margin = object_parameter.safety_buffer_lateral * object.distance_factor +
                            object_parameter.avoid_margin_lateral + 0.5 * vehicle_width;
  const auto variable = helper_.getMinAvoidanceDistance(
    helper_.getShiftLength(object, utils::avoidance::isOnRight(object), avoid_margin));
  const auto constant = p->min_prepare_distance + object_parameter.safety_buffer_longitudinal +
                        base_link2front + p->stop_buffer;

  return object.longitudinal - std::min(variable + constant, p->stop_max_distance);
}

void AvoidanceModule::insertWaitPoint(
  const bool use_constraints_for_decel, ShiftedPath & shifted_path) const
{
  const auto & data = avoidance_data_;

  if (!data.stop_target_object) {
    return;
  }

  if (data.avoiding_now) {
    return;
  }

  // If we don't need to consider deceleration constraints, insert a deceleration point
  // and return immediately
  if (!use_constraints_for_decel) {
    utils::avoidance::insertDecelPoint(
      getEgoPosition(), data.to_stop_line, 0.0, shifted_path.path, stop_pose_);
    return;
  }

  // If the stop distance is not enough for comfortable stop, don't insert wait point.
  const auto is_comfortable_stop = helper_.getFeasibleDecelDistance(0.0) < data.to_stop_line;
  const auto is_slow_speed = getEgoSpeed() < parameters_->min_slow_down_speed;
  if (!is_comfortable_stop && !is_slow_speed) {
    RCLCPP_WARN_THROTTLE(getLogger(), *clock_, 500, "not execute uncomfortable deceleration.");
    return;
  }

  // If target object can be stopped for, insert a deceleration point and return
  if (data.stop_target_object.get().is_stoppable) {
    utils::avoidance::insertDecelPoint(
      getEgoPosition(), data.to_stop_line, 0.0, shifted_path.path, stop_pose_);
    return;
  }

  // If the object cannot be stopped for, calculate a "mild" deceleration distance
  // and insert a deceleration point at that distance
  const auto stop_distance = helper_.getFeasibleDecelDistance(0.0, false);
  utils::avoidance::insertDecelPoint(
    getEgoPosition(), stop_distance, 0.0, shifted_path.path, stop_pose_);
}

void AvoidanceModule::insertStopPoint(
  const bool use_constraints_for_decel, ShiftedPath & shifted_path) const
{
  const auto & data = avoidance_data_;

  if (data.safe) {
    return;
  }

  if (!parameters_->enable_yield_maneuver_during_shifting) {
    return;
  }

  const auto stop_idx = [&]() {
    const auto ego_idx = planner_data_->findEgoIndex(shifted_path.path.points);
    for (size_t idx = ego_idx; idx < shifted_path.path.points.size(); ++idx) {
      const auto & estimated_pose = shifted_path.path.points.at(idx).point.pose;
      if (!utils::isEgoWithinOriginalLane(
            data.current_lanelets, estimated_pose, planner_data_->parameters)) {
        return idx - 1;
      }
    }

    return shifted_path.path.points.size() - 1;
  }();

  const auto stop_distance =
    calcSignedArcLength(shifted_path.path.points, getEgoPosition(), stop_idx);

  // If we don't need to consider deceleration constraints, insert a deceleration point
  // and return immediately
  if (!use_constraints_for_decel) {
    utils::avoidance::insertDecelPoint(
      getEgoPosition(), stop_distance, 0.0, shifted_path.path, stop_pose_);
    return;
  }

  // Otherwise, consider deceleration constraints before inserting deceleration point
  const auto decel_distance = helper_.getFeasibleDecelDistance(0.0, false);
  if (stop_distance < decel_distance) {
    return;
  }

  constexpr double MARGIN = 1.0;
  utils::avoidance::insertDecelPoint(
    getEgoPosition(), stop_distance - MARGIN, 0.0, shifted_path.path, stop_pose_);
}

void AvoidanceModule::insertYieldVelocity(ShiftedPath & shifted_path) const
{
  const auto & p = parameters_;
  const auto & data = avoidance_data_;

  if (data.target_objects.empty()) {
    return;
  }

  if (data.avoiding_now) {
    return;
  }

  const auto decel_distance = helper_.getFeasibleDecelDistance(p->yield_velocity, false);
  utils::avoidance::insertDecelPoint(
    getEgoPosition(), decel_distance, p->yield_velocity, shifted_path.path, slow_pose_);
}

void AvoidanceModule::insertPrepareVelocity(ShiftedPath & shifted_path) const
{
  const auto & data = avoidance_data_;

  // do nothing if there is no avoidance target.
  if (data.target_objects.empty()) {
    return;
  }

  // insert slow down speed only when the avoidance maneuver is not initiated.
  if (data.avoiding_now) {
    return;
  }

  // insert slow down speed only when no shift line is approved.
  if (!path_shifter_.getShiftLines().empty()) {
    return;
  }

  const auto object = data.target_objects.front();

  // calculate shift length for front object.
  const auto & vehicle_width = planner_data_->parameters.vehicle_width;
  const auto object_type = utils::getHighestProbLabel(object.object.classification);
  const auto object_parameter = parameters_->object_parameters.at(object_type);
  const auto avoid_margin = object_parameter.safety_buffer_lateral * object.distance_factor +
                            object_parameter.avoid_margin_lateral + 0.5 * vehicle_width;
  const auto shift_length =
    helper_.getShiftLength(object, utils::avoidance::isOnRight(object), avoid_margin);

  // check slow down feasibility
  const auto min_avoid_distance = helper_.getMinAvoidanceDistance(shift_length);
  const auto distance_to_object = object.longitudinal;
  const auto remaining_distance = distance_to_object - min_avoid_distance;
  const auto decel_distance = helper_.getFeasibleDecelDistance(parameters_->velocity_map.front());
  if (remaining_distance < decel_distance) {
    return;
  }

  // decide slow down lower limit.
  const auto lower_speed = object.avoid_required ? 0.0 : parameters_->min_slow_down_speed;

  // insert slow down speed.
  const auto start_idx = planner_data_->findEgoIndex(shifted_path.path.points);
  for (size_t i = start_idx; i < shifted_path.path.points.size(); ++i) {
    const auto distance_from_ego = calcSignedArcLength(shifted_path.path.points, start_idx, i);

    // slow down speed is inserted only in front of the object.
    const auto shift_longitudinal_distance = distance_to_object - distance_from_ego;
    if (shift_longitudinal_distance < min_avoid_distance) {
      break;
    }

    // target speed with nominal jerk limits.
    const double v_target = PathShifter::calcFeasibleVelocityFromJerk(
      shift_length, helper_.getLateralMinJerkLimit(), shift_longitudinal_distance);
    const double v_original = shifted_path.path.points.at(i).point.longitudinal_velocity_mps;
    const double v_insert = std::max(v_target - parameters_->buf_slow_down_speed, lower_speed);

    shifted_path.path.points.at(i).point.longitudinal_velocity_mps = std::min(v_original, v_insert);
  }

  slow_pose_ = motion_utils::calcLongitudinalOffsetPose(
    shifted_path.path.points, start_idx, distance_to_object);
}

std::shared_ptr<AvoidanceDebugMsgArray> AvoidanceModule::get_debug_msg_array() const
{
  debug_data_.avoidance_debug_msg_array.header.stamp = clock_->now();
  return std::make_shared<AvoidanceDebugMsgArray>(debug_data_.avoidance_debug_msg_array);
}

void AvoidanceModule::acceptVisitor(const std::shared_ptr<SceneModuleVisitor> & visitor) const
{
  if (visitor) {
    visitor->visitAvoidanceModule(this);
  }
}

void SceneModuleVisitor::visitAvoidanceModule(const AvoidanceModule * module) const
{
  avoidance_visitor_ = module->get_debug_msg_array();
}
}  // namespace behavior_path_planner
