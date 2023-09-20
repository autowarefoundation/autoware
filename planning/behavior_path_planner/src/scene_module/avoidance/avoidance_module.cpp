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

#include "behavior_path_planner/marker_utils/avoidance/debug.hpp"
#include "behavior_path_planner/scene_module/scene_module_visitor.hpp"
#include "behavior_path_planner/utils/avoidance/utils.hpp"
#include "behavior_path_planner/utils/path_safety_checker/objects_filtering.hpp"
#include "behavior_path_planner/utils/path_utils.hpp"
#include "behavior_path_planner/utils/utils.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <tier4_autoware_utils/geometry/boost_polygon_utils.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/ros/marker_helper.hpp>
#include <tier4_autoware_utils/ros/uuid_helper.hpp>

#include <tier4_planning_msgs/msg/avoidance_debug_factor.hpp>
#include <tier4_planning_msgs/msg/avoidance_debug_msg.hpp>
#include <tier4_planning_msgs/msg/avoidance_debug_msg_array.hpp>

#include <boost/geometry/algorithms/centroid.hpp>
#include <boost/geometry/strategies/cartesian/centroid_bashein_detmer.hpp>

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

using behavior_path_planner::utils::path_safety_checker::CollisionCheckDebug;
using motion_utils::calcLongitudinalOffsetPose;
using motion_utils::calcSignedArcLength;
using motion_utils::findNearestIndex;
using motion_utils::findNearestSegmentIndex;
using tier4_autoware_utils::appendMarkerArray;
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

bool isBestEffort(const std::string & policy)
{
  return policy == "best_effort";
}

AvoidLine merge(const AvoidLine & line1, const AvoidLine & line2, const uint64_t id)
{
  AvoidLine ret{};

  ret.start_idx = line1.start_idx;
  ret.start_shift_length = line1.start_shift_length;
  ret.start_longitudinal = line1.start_longitudinal;

  ret.end_idx = line2.end_idx;
  ret.end_shift_length = line2.end_shift_length;
  ret.end_longitudinal = line2.end_longitudinal;

  ret.id = id;
  ret.object = line1.object;

  return ret;
}

AvoidLine fill(const AvoidLine & line1, const AvoidLine & line2, const uint64_t id)
{
  AvoidLine ret{};

  ret.start_idx = line1.end_idx;
  ret.start_shift_length = line1.end_shift_length;
  ret.start_longitudinal = line1.end_longitudinal;

  ret.end_idx = line2.start_idx;
  ret.end_shift_length = line2.start_shift_length;
  ret.end_longitudinal = line2.start_longitudinal;

  ret.id = id;
  ret.object = line1.object;

  return ret;
}

AvoidLineArray toArray(const AvoidOutlines & outlines)
{
  AvoidLineArray ret{};
  for (const auto & outline : outlines) {
    ret.push_back(outline.avoid_line);
    ret.push_back(outline.return_line);

    std::for_each(
      outline.middle_lines.begin(), outline.middle_lines.end(),
      [&ret](const auto & line) { ret.push_back(line); });
  }
  return ret;
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

  // Check ego is in preferred lane
  updateInfoMarker(avoid_data_);
  updateDebugMarker(avoid_data_, path_shifter_, debug_data_);

  // there is object that should be avoid. return true.
  if (!!avoid_data_.stop_target_object) {
    return true;
  }

  if (avoid_data_.new_shift_line.empty()) {
    return false;
  }

  return !avoid_data_.target_objects.empty();
}

bool AvoidanceModule::isExecutionReady() const
{
  DEBUG_PRINT("AVOIDANCE isExecutionReady");
  return avoid_data_.safe && avoid_data_.comfortable;
}

bool AvoidanceModule::canTransitSuccessState()
{
  const auto & data = avoid_data_;

  // Change input lane. -> EXIT.
  if (!isDrivingSameLane(helper_.getPreviousDrivingLanes(), data.current_lanelets)) {
    RCLCPP_WARN(getLogger(), "Previous module lane is updated. Exit.");
    return true;
  }

  helper_.setPreviousDrivingLanes(data.current_lanelets);

  // Reach input path end point. -> EXIT.
  {
    const auto idx = planner_data_->findEgoIndex(data.reference_path.points);
    if (idx == data.reference_path.points.size() - 1) {
      arrived_path_end_ = true;
    }

    constexpr double THRESHOLD = 1.0;
    const auto is_further_than_threshold =
      calcDistance2d(getEgoPose(), getPose(data.reference_path.points.back())) > THRESHOLD;
    if (is_further_than_threshold && arrived_path_end_) {
      RCLCPP_WARN(getLogger(), "Reach path end point. Exit.");
      return true;
    }
  }

  const bool has_avoidance_target = !data.target_objects.empty();
  const bool has_shift_point = !path_shifter_.getShiftLines().empty();
  const bool has_base_offset =
    std::abs(path_shifter_.getBaseOffset()) > parameters_->lateral_avoid_check_threshold;

  // Nothing to do. -> EXIT.
  if (!has_avoidance_target) {
    if (!has_shift_point && !has_base_offset) {
      RCLCPP_INFO(getLogger(), "No objects. No approved shift lines. Exit.");
      return true;
    }
  }

  // Be able to canceling avoidance path. -> EXIT.
  if (!has_avoidance_target) {
    if (!helper_.isShifted() && parameters_->enable_cancel_maneuver) {
      RCLCPP_INFO(getLogger(), "No objects. Cancel avoidance path. Exit.");
      return true;
    }
  }

  return false;  // Keep current state.
}

void AvoidanceModule::fillFundamentalData(AvoidancePlanningData & data, DebugData & debug)
{
  // reference pose
  data.reference_pose =
    utils::getUnshiftedEgoPose(getEgoPose(), helper_.getPreviousSplineShiftPath());

  // lanelet info
  data.current_lanelets = utils::avoidance::getCurrentLanesFromPath(
    *getPreviousModuleOutput().reference_path, planner_data_);

  // reference path
  if (isDrivingSameLane(helper_.getPreviousDrivingLanes(), data.current_lanelets)) {
    data.reference_path_rough = extendBackwardLength(*getPreviousModuleOutput().path);
  } else {
    data.reference_path_rough = *getPreviousModuleOutput().path;
    RCLCPP_WARN(getLogger(), "Previous module lane is updated. Don't use latest reference path.");
  }

  // resampled reference path
  data.reference_path = utils::resamplePathWithSpline(
    data.reference_path_rough, parameters_->resample_interval_for_planning);

  // closest index
  data.ego_closest_path_index = planner_data_->findEgoIndex(data.reference_path.points);

  // arclength from ego pose (used in many functions)
  data.arclength_from_ego = utils::calcPathArcLengthArray(
    data.reference_path, 0, data.reference_path.points.size(),
    calcSignedArcLength(data.reference_path.points, getEgoPosition(), 0));

  // target objects for avoidance
  fillAvoidanceTargetObjects(data, debug);

  // lost object compensation
  utils::avoidance::updateRegisteredObject(registered_objects_, data.target_objects, parameters_);
  utils::avoidance::compensateDetectionLost(
    registered_objects_, data.target_objects, data.other_objects);

  // sort object order by longitudinal distance
  std::sort(data.target_objects.begin(), data.target_objects.end(), [](auto a, auto b) {
    return a.longitudinal < b.longitudinal;
  });

  // set base path
  path_shifter_.setPath(data.reference_path);
}

void AvoidanceModule::fillAvoidanceTargetObjects(
  AvoidancePlanningData & data, DebugData & debug) const
{
  using utils::avoidance::fillObjectStoppableJudge;
  using utils::avoidance::filterTargetObjects;
  using utils::avoidance::getTargetLanelets;

  // Separate dynamic objects based on whether they are inside or outside of the expanded lanelets.
  const auto [object_within_target_lane, object_outside_target_lane] =
    utils::avoidance::separateObjectsByPath(
      utils::resamplePathWithSpline(
        helper_.getPreviousSplineShiftPath().path, parameters_->resample_interval_for_output),
      planner_data_, data, parameters_, debug);

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
    object_data, data.reference_path, object_data.overhang_pose.position);

  // Check whether the the ego should avoid the object.
  const auto & vehicle_width = planner_data_->parameters.vehicle_width;
  utils::avoidance::fillAvoidanceNecessity(
    object_data, registered_objects_, vehicle_width, parameters_);

  return object_data;
}

bool AvoidanceModule::canYieldManeuver(const AvoidancePlanningData & data) const
{
  // transit yield maneuver only when the avoidance maneuver is not initiated.
  if (helper_.isShifted()) {
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
  auto path_shifter = path_shifter_;

  /**
   * STEP1: Generate avoid outlines.
   * Basically, avoid outlines are generated per target objects.
   */
  const auto outlines = generateAvoidOutline(data, debug);

  /**
   * STEP2: Create rough shift lines.
   */
  data.raw_shift_line = applyPreProcess(outlines, debug);

  /**
   * STEP3: Create candidate shift lines.
   * Merge rough shift lines, and extract new shift lines.
   */
  data.new_shift_line = generateCandidateShiftLine(data.raw_shift_line, path_shifter, debug);
  const auto found_new_sl = data.new_shift_line.size() > 0;
  const auto registered = path_shifter.getShiftLines().size() > 0;
  data.found_avoidance_path = found_new_sl || registered;

  /**
   * STEP4: Set new shift lines.
   * If there are new shift points, these shift points are registered in path_shifter in order to
   * generate candidate avoidance path.
   */
  if (!data.new_shift_line.empty()) {
    addNewShiftLines(path_shifter, data.new_shift_line);
  }

  /**
   * STEP5: Generate avoidance path.
   */
  ShiftedPath spline_shift_path = utils::avoidance::toShiftedPath(data.reference_path);
  const auto success_spline_path_generation =
    path_shifter.generate(&spline_shift_path, true, SHIFT_TYPE::SPLINE);
  data.candidate_path = success_spline_path_generation
                          ? spline_shift_path
                          : utils::avoidance::toShiftedPath(data.reference_path);

  /**
   * STEP6: Check avoidance path safety.
   * For each target objects and the objects in adjacent lanes,
   * check that there is a certain amount of margin in the lateral and longitudinal direction.
   */
  data.comfortable = isComfortable(data.new_shift_line);
  data.safe = isSafePath(data.candidate_path, debug);
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

  /**
   * If the output path is locked by outside of this module, don't update output path.
   */
  if (isOutputPathLocked()) {
    data.safe_shift_line.clear();
    data.candidate_path = helper_.getPreviousSplineShiftPath();
    RCLCPP_DEBUG_THROTTLE(
      getLogger(), *clock_, 500, "this module is locked now. keep current path.");
    return;
  }

  /**
   * If the avoidance path is safe, use unapproved_new_sl for avoidance path generation.
   */
  if (data.safe) {
    data.yield_required = false;
    data.safe_shift_line = data.new_shift_line;
    return;
  }

  /**
   * If the yield maneuver is disabled, use unapproved_new_sl for avoidance path generation even if
   * the shift line is unsafe.
   */
  if (!parameters_->enable_yield_maneuver) {
    data.yield_required = false;
    data.safe_shift_line = data.new_shift_line;
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
    data.safe_shift_line = data.new_shift_line;
    RCLCPP_WARN_THROTTLE(getLogger(), *clock_, 500, "unsafe. but could not transit yield status.");
    return;
  }

  /**
   * Transit yield maneuver. Clear shift lines and output yield path.
   */
  {
    data.yield_required = true;
    data.safe_shift_line = data.new_shift_line;
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

void AvoidanceModule::fillDebugData(
  const AvoidancePlanningData & data, [[maybe_unused]] DebugData & debug) const
{
  if (!data.stop_target_object) {
    return;
  }

  if (helper_.isShifted()) {
    return;
  }

  if (data.new_shift_line.empty()) {
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
      insertWaitPoint(isBestEffort(parameters_->policy_deceleration), path);
      break;
    }
    case AvoidanceState::AVOID_PATH_NOT_READY: {
      insertWaitPoint(isBestEffort(parameters_->policy_deceleration), path);
      break;
    }
    case AvoidanceState::AVOID_PATH_READY: {
      insertWaitPoint(isBestEffort(parameters_->policy_deceleration), path);
      break;
    }
    case AvoidanceState::AVOID_EXECUTE: {
      insertStopPoint(isBestEffort(parameters_->policy_deceleration), path);
      break;
    }
    default:
      throw std::domain_error("invalid behavior");
  }

  insertReturnDeadLine(isBestEffort(parameters_->policy_deceleration), path);

  setStopReason(StopReason::AVOIDANCE, path.path);
}

void AvoidanceModule::updateRegisteredRawShiftLines()
{
  const auto & data = avoid_data_;

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
  debug_data_.step1_registered_shift_line = registered_raw_shift_lines_;
}

AvoidLineArray AvoidanceModule::applyPreProcess(
  const AvoidOutlines & outlines, DebugData & debug) const
{
  AvoidOutlines processed_outlines = outlines;

  /**
   * Step1: Rough merge process.
   * Merge multiple avoid outlines. If an avoid outlines' return shift line conflicts other
   * outline's avoid shift line, those avoid outlines are merged.
   */
  processed_outlines = applyMergeProcess(processed_outlines, debug);

  /**
   * Step2: Fill gap process.
   * Create and add new shift line to avoid outline in order to fill gaps between avoid shift line
   * and middle shift lines, return shift line and middle shift lines.
   */
  processed_outlines = applyFillGapProcess(processed_outlines, debug);

  /**
   * Step3: Convert to AvoidLineArray from AvoidOutlines.
   */
  AvoidLineArray processed_raw_lines = toArray(processed_outlines);

  /**
   * Step4: Combine process.
   * Use all registered points. For the current points, if the similar one of the current
   * points are already registered, will not use it.
   */
  processed_raw_lines =
    applyCombineProcess(processed_raw_lines, registered_raw_shift_lines_, debug);

  /*
   * Step5: Add return shift line.
   * Add return-to-center shift point from the last shift point, if needed.
   * If there is no shift points, set return-to center shift from ego.
   */
  processed_raw_lines = addReturnShiftLine(processed_raw_lines, debug);

  /*
   * Step6: Fill gap process.
   * Create and add new shift line to avoid lines.
   */
  return applyFillGapProcess(processed_raw_lines, debug);
}

AvoidLineArray AvoidanceModule::generateCandidateShiftLine(
  const AvoidLineArray & shift_lines, const PathShifter & path_shifter, DebugData & debug) const
{
  AvoidLineArray processed_shift_lines = shift_lines;

  /**
   * Step1: Merge process.
   * Merge positive shift avoid lines and negative shift avoid lines.
   */
  processed_shift_lines = applyMergeProcess(processed_shift_lines, debug);

  /**
   * Step2: Clean up process.
   * Remove noisy shift line and concat same gradient shift lines.
   */
  processed_shift_lines = applyTrimProcess(processed_shift_lines, debug);

  /**
   * Step3: Extract new shift lines.
   * Compare processed shift lines and registered shift lines in order to find new shift lines.
   */
  processed_shift_lines = findNewShiftLine(processed_shift_lines, debug);

  /**
   * Step4: Validate new shift lines.
   * Output new shift lines only when the avoidance path which is generated from them doesn't have
   * huge offset from ego.
   */
  return isValidShiftLine(processed_shift_lines, path_shifter) ? processed_shift_lines
                                                               : AvoidLineArray{};
}

void AvoidanceModule::registerRawShiftLines(const AvoidLineArray & future)
{
  if (future.empty()) {
    RCLCPP_ERROR(getLogger(), "future is empty! return.");
    return;
  }

  const auto old_size = registered_raw_shift_lines_.size();

  auto future_with_info = future;
  utils::avoidance::fillAdditionalInfoFromPoint(avoid_data_, future_with_info);
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

AvoidOutlines AvoidanceModule::generateAvoidOutline(
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

    if (!isBestEffort(parameters_->policy_lateral_margin)) {
      return desire_shift_length;
    }

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

    // don't relax shift length since it can stop in front of the object.
    if (object.is_stoppable && !parameters_->use_shorten_margin_immediately) {
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
    if (!isBestEffort(parameters_->policy_deceleration)) {
      object.reason = AvoidanceDebugFactor::TOO_LARGE_JERK;
      return boost::none;
    }

    // output avoidance path under lateral jerk constraints.
    const auto feasible_relative_shift_length = PathShifter::calcLateralDistFromJerk(
      remaining_distance, helper_.getLateralMaxJerkLimit(), helper_.getAvoidanceEgoSpeed());

    if (std::abs(feasible_relative_shift_length) < parameters_->lateral_execution_threshold) {
      object.reason = "LessThanExecutionThreshold";
      return boost::none;
    }

    const auto feasible_shift_length =
      desire_shift_length > 0.0 ? feasible_relative_shift_length + current_ego_shift
                                : -1.0 * feasible_relative_shift_length + current_ego_shift;

    const auto feasible =
      std::abs(feasible_shift_length - object.overhang_dist) <
      0.5 * planner_data_->parameters.vehicle_width + object_parameter.safety_buffer_lateral;
    if (feasible) {
      RCLCPP_WARN_THROTTLE(
        getLogger(), *clock_, 1000, "feasible shift length is not enough to avoid. ");
      object.reason = AvoidanceDebugFactor::INSUFFICIENT_LATERAL_MARGIN;
      return boost::none;
    }

    {
      RCLCPP_WARN_THROTTLE(
        getLogger(), *clock_, 1000, "use feasible shift length. [original: (%.2f) actual: (%.2f)]",
        std::abs(avoiding_shift), feasible_relative_shift_length);
    }

    return feasible_shift_length;
  };

  const auto is_forward_object = [](const auto & object) { return object.longitudinal > 0.0; };

  const auto is_valid_shift_line = [](const auto & s) {
    return s.start_longitudinal > 0.0 && s.start_longitudinal < s.end_longitudinal;
  };

  AvoidOutlines outlines;
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
        avoid_data_.arclength_from_ego.at(avoid_data_.ego_closest_path_index);

      al_avoid.start_longitudinal =
        std::max(o.longitudinal - offset - feasible_avoid_distance, 1e-3);
      al_avoid.start_idx = utils::avoidance::findPathIndexFromArclength(
        avoid_data_.arclength_from_ego, al_avoid.start_longitudinal + path_front_to_ego);
      al_avoid.start = avoid_data_.reference_path.points.at(al_avoid.start_idx).point.pose;
      al_avoid.start_shift_length = helper_.getLinearShift(al_avoid.start.position);

      al_avoid.end_shift_length = feasible_shift_length.get();
      al_avoid.end_longitudinal = o.longitudinal - offset;
      al_avoid.id = getOriginalShiftLineUniqueId();
      al_avoid.object = o;
      al_avoid.object_on_right = utils::avoidance::isOnRight(o);
    }

    AvoidLine al_return;
    {
      const auto offset = object_parameter.safety_buffer_longitudinal + base_link2rear + o.length;
      // The end_margin also has the purpose of preventing the return path from NOT being
      // triggered at the end point.
      const auto return_remaining_distance = std::max(
        data.arclength_from_ego.back() - o.longitudinal - offset -
          parameters_->remain_buffer_distance,
        0.0);

      al_return.start_shift_length = feasible_shift_length.get();
      al_return.end_shift_length = 0.0;
      al_return.start_longitudinal = o.longitudinal + offset;
      al_return.end_longitudinal =
        o.longitudinal + offset + std::min(feasible_return_distance, return_remaining_distance);
      al_return.id = getOriginalShiftLineUniqueId();
      al_return.object = o;
      al_return.object_on_right = utils::avoidance::isOnRight(o);
    }

    if (is_valid_shift_line(al_avoid) && is_valid_shift_line(al_return)) {
      outlines.emplace_back(al_avoid, al_return);
    } else {
      o.reason = "InvalidShiftLine";
      continue;
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

  utils::avoidance::fillAdditionalInfoFromLongitudinal(data, outlines);

  debug.step1_current_shift_line = toArray(outlines);

  return outlines;
}

void AvoidanceModule::generateTotalShiftLine(
  const AvoidLineArray & avoid_lines, ShiftLineData & shift_line_data) const
{
  const auto & path = avoid_data_.reference_path;
  const auto & arcs = avoid_data_.arclength_from_ego;
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
  for (size_t i = 0; i < sl.shift_line.size(); ++i) {
    if (avoid_data_.ego_closest_path_index < i) {
      break;
    }
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

  for (size_t i = avoid_data_.ego_closest_path_index; i <= avoid_lines.front().start_idx; ++i) {
    sl.shift_line.at(i) = helper_.getLinearShift(getPoint(path.points.at(i)));
    sl.shift_line_grad.at(i) = grad_first_shift_line;
  }

  sl.shift_line_history.push_back(sl.shift_line);
}

AvoidLineArray AvoidanceModule::extractShiftLinesFromLine(ShiftLineData & shift_line_data) const
{
  using utils::avoidance::setEndData;
  using utils::avoidance::setStartData;

  const auto & path = avoid_data_.reference_path;
  const auto & arcs = avoid_data_.arclength_from_ego;
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
  for (size_t i = avoid_data_.ego_closest_path_index; i < N - 1; ++i) {
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

AvoidOutlines AvoidanceModule::applyMergeProcess(
  const AvoidOutlines & outlines, DebugData & debug) const
{
  AvoidOutlines ret{};

  if (outlines.size() < 2) {
    return outlines;
  }

  const auto no_conflict = [](const auto & line1, const auto & line2) {
    return line1.end_idx < line2.start_idx || line2.end_idx < line1.start_idx;
  };

  const auto same_side_shift = [](const auto & line1, const auto & line2) {
    return line1.object_on_right == line2.object_on_right;
  };

  const auto within = [](const auto & line, const size_t idx) {
    return line.start_idx < idx && idx < line.end_idx;
  };

  ret.push_back(outlines.front());

  for (size_t i = 1; i < outlines.size(); i++) {
    auto & last_outline = ret.back();
    auto & next_outline = outlines.at(i);

    const auto & return_line = last_outline.return_line;
    const auto & avoid_line = next_outline.avoid_line;

    if (no_conflict(return_line, avoid_line)) {
      ret.push_back(outlines.at(i));
      continue;
    }

    const auto merged_shift_line = merge(return_line, avoid_line, getOriginalShiftLineUniqueId());

    if (!isComfortable(AvoidLineArray{merged_shift_line})) {
      ret.push_back(outlines.at(i));
      continue;
    }

    if (same_side_shift(return_line, avoid_line)) {
      last_outline.middle_lines.push_back(merged_shift_line);
      last_outline.return_line = next_outline.return_line;
      debug.step1_merged_shift_line.push_back(merged_shift_line);
      continue;
    }

    if (within(return_line, avoid_line.end_idx) && within(avoid_line, return_line.start_idx)) {
      last_outline.middle_lines.push_back(merged_shift_line);
      last_outline.return_line = next_outline.return_line;
      debug.step1_merged_shift_line.push_back(merged_shift_line);
      continue;
    }

    if (within(return_line, avoid_line.start_idx) && within(avoid_line, return_line.end_idx)) {
      last_outline.middle_lines.push_back(merged_shift_line);
      last_outline.return_line = next_outline.return_line;
      debug.step1_merged_shift_line.push_back(merged_shift_line);
      continue;
    }
  }

  utils::avoidance::fillAdditionalInfoFromLongitudinal(avoid_data_, ret);
  utils::avoidance::fillAdditionalInfoFromLongitudinal(avoid_data_, debug.step1_merged_shift_line);

  return ret;
}

AvoidOutlines AvoidanceModule::applyFillGapProcess(
  const AvoidOutlines & outlines, DebugData & debug) const
{
  AvoidOutlines ret = outlines;

  for (auto & outline : ret) {
    if (outline.middle_lines.empty()) {
      const auto new_line =
        fill(outline.avoid_line, outline.return_line, getOriginalShiftLineUniqueId());
      outline.middle_lines.push_back(new_line);
      debug.step1_filled_shift_line.push_back(new_line);
    }

    helper_.alignShiftLinesOrder(outline.middle_lines, false);

    if (outline.avoid_line.end_longitudinal < outline.middle_lines.front().start_longitudinal) {
      const auto new_line =
        fill(outline.avoid_line, outline.middle_lines.front(), getOriginalShiftLineUniqueId());
      outline.middle_lines.push_back(new_line);
      debug.step1_filled_shift_line.push_back(new_line);
    }

    helper_.alignShiftLinesOrder(outline.middle_lines, false);

    if (outline.middle_lines.back().end_longitudinal < outline.return_line.start_longitudinal) {
      const auto new_line =
        fill(outline.middle_lines.back(), outline.return_line, getOriginalShiftLineUniqueId());
      outline.middle_lines.push_back(new_line);
      debug.step1_filled_shift_line.push_back(new_line);
    }

    helper_.alignShiftLinesOrder(outline.middle_lines, false);
  }

  utils::avoidance::fillAdditionalInfoFromLongitudinal(avoid_data_, ret);
  utils::avoidance::fillAdditionalInfoFromLongitudinal(avoid_data_, debug.step1_filled_shift_line);

  return ret;
}

AvoidLineArray AvoidanceModule::applyFillGapProcess(
  const AvoidLineArray & shift_lines, DebugData & debug) const
{
  AvoidLineArray sorted = shift_lines;

  helper_.alignShiftLinesOrder(sorted, false);

  AvoidLineArray ret = sorted;

  if (shift_lines.empty()) {
    return ret;
  }

  const auto & data = avoid_data_;

  // fill gap between ego and nearest shift line.
  if (sorted.front().start_longitudinal > 0.0) {
    AvoidLine ego_line{};
    utils::avoidance::setEndData(
      ego_line, helper_.getEgoLinearShift(), data.reference_pose, data.ego_closest_path_index, 0.0);

    const auto new_line = fill(ego_line, sorted.front(), getOriginalShiftLineUniqueId());
    ret.push_back(new_line);
    debug.step1_front_shift_line.push_back(new_line);
  }

  helper_.alignShiftLinesOrder(sorted, false);

  // fill gap among shift lines.
  for (size_t i = 0; i < sorted.size() - 1; ++i) {
    if (sorted.at(i + 1).start_longitudinal < sorted.at(i).end_longitudinal) {
      continue;
    }

    const auto new_line = fill(sorted.at(i), sorted.at(i + 1), getOriginalShiftLineUniqueId());
    ret.push_back(new_line);
    debug.step1_front_shift_line.push_back(new_line);
  }

  helper_.alignShiftLinesOrder(ret, false);

  utils::avoidance::fillAdditionalInfoFromLongitudinal(avoid_data_, ret);
  utils::avoidance::fillAdditionalInfoFromLongitudinal(avoid_data_, debug.step1_front_shift_line);

  return ret;
}

AvoidLineArray AvoidanceModule::applyCombineProcess(
  const AvoidLineArray & shift_lines, const AvoidLineArray & registered_lines,
  [[maybe_unused]] DebugData & debug) const
{
  return utils::avoidance::combineRawShiftLinesWithUniqueCheck(registered_lines, shift_lines);
}

AvoidLineArray AvoidanceModule::applyMergeProcess(
  const AvoidLineArray & shift_lines, DebugData & debug) const
{
  // Generate shift line by merging shift_lines.
  ShiftLineData shift_line_data;
  generateTotalShiftLine(shift_lines, shift_line_data);

  // Re-generate shift points by detecting gradient-change point of the shift line.
  auto merged_shift_lines = extractShiftLinesFromLine(shift_line_data);

  // set parent id
  for (auto & al : merged_shift_lines) {
    al.parent_ids = utils::avoidance::calcParentIds(shift_lines, al);
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
    debug.step2_merged_shift_line = merged_shift_lines;
  }

  return merged_shift_lines;
}

AvoidLineArray AvoidanceModule::applyTrimProcess(
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
    applySmallShiftFilter(sl_array_trimmed, SHIFT_DIFF_THRES);
  }

  // - Combine avoid points that have almost same gradient.
  // this is to remove the noise.
  {
    const auto THRESHOLD = parameters_->same_grad_filter_1_threshold;
    applySimilarGradFilter(sl_array_trimmed, THRESHOLD);
    debug.step3_grad_filtered_1st = sl_array_trimmed;
  }

  // - Quantize the shift length to reduce the shift point noise
  // This is to remove the noise coming from detection accuracy, interpolation, resampling, etc.
  {
    const auto THRESHOLD = parameters_->quantize_filter_threshold;
    applyQuantizeProcess(sl_array_trimmed, THRESHOLD);
    debug.step3_quantize_filtered = sl_array_trimmed;
  }

  // - Change the shift length to the previous one if the deviation is small.
  {
    constexpr double SHIFT_DIFF_THRES = 1.0;
    applySmallShiftFilter(sl_array_trimmed, SHIFT_DIFF_THRES);
    debug.step3_noise_filtered = sl_array_trimmed;
  }

  // - Combine avoid points that have almost same gradient (again)
  {
    const auto THRESHOLD = parameters_->same_grad_filter_2_threshold;
    applySimilarGradFilter(sl_array_trimmed, THRESHOLD);
    debug.step3_grad_filtered_2nd = sl_array_trimmed;
  }

  // - Combine avoid points that have almost same gradient (again)
  {
    const auto THRESHOLD = parameters_->same_grad_filter_3_threshold;
    applySimilarGradFilter(sl_array_trimmed, THRESHOLD);
    debug.step3_grad_filtered_3rd = sl_array_trimmed;
  }

  return sl_array_trimmed;
}

void AvoidanceModule::applyQuantizeProcess(
  AvoidLineArray & shift_lines, const double threshold) const
{
  if (threshold < 1.0e-5) {
    return;  // no need to process
  }

  for (auto & sl : shift_lines) {
    sl.end_shift_length = std::round(sl.end_shift_length / threshold) * threshold;
  }

  helper_.alignShiftLinesOrder(shift_lines);
}

void AvoidanceModule::applySmallShiftFilter(
  AvoidLineArray & shift_lines, const double threshold) const
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

void AvoidanceModule::applySimilarGradFilter(
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

AvoidLineArray AvoidanceModule::addReturnShiftLine(
  const AvoidLineArray & shift_lines, DebugData & debug) const
{
  AvoidLineArray ret = shift_lines;

  constexpr double ep = 1.0e-3;
  const auto & data = avoid_data_;
  const bool has_candidate_point = !ret.empty();
  const bool has_registered_point = !path_shifter_.getShiftLines().empty();

  const auto exist_unavoidable_object = std::any_of(
    data.target_objects.begin(), data.target_objects.end(),
    [](const auto & o) { return !o.is_avoidable && o.longitudinal > 0.0; });

  if (exist_unavoidable_object) {
    return ret;
  }

  // If the return-to-center shift points are already registered, do nothing.
  if (!has_registered_point && std::fabs(getCurrentBaseShift()) < ep) {
    DEBUG_PRINT("No shift points, not base offset. Do not have to add return-shift.");
    return ret;
  }

  constexpr double RETURN_SHIFT_THRESHOLD = 0.1;
  DEBUG_PRINT("registered last shift = %f", path_shifter_.getLastShiftLength());
  if (std::abs(path_shifter_.getLastShiftLength()) < RETURN_SHIFT_THRESHOLD) {
    DEBUG_PRINT("Return shift is already registered. do nothing.");
    return ret;
  }

  // From here, the return-to-center is not registered. But perhaps the candidate is
  // already generated.

  // If it has a shift point, add return shift from the existing last shift point.
  // If not, add return shift from ego point. (prepare distance is considered for both.)
  ShiftLine last_sl;  // the return-shift will be generated after the last shift point.
  {
    // avoidance points: Yes, shift points: No -> select last avoidance point.
    if (has_candidate_point && !has_registered_point) {
      helper_.alignShiftLinesOrder(ret, false);
      last_sl = ret.back();
    }

    // avoidance points: No, shift points: Yes -> select last shift point.
    if (!has_candidate_point && has_registered_point) {
      last_sl = utils::avoidance::fillAdditionalInfo(
        data, AvoidLine{path_shifter_.getLastShiftLine().get()});
    }

    // avoidance points: Yes, shift points: Yes -> select the last one from both.
    if (has_candidate_point && has_registered_point) {
      helper_.alignShiftLinesOrder(ret, false);
      const auto & al = ret.back();
      const auto & sl = utils::avoidance::fillAdditionalInfo(
        data, AvoidLine{path_shifter_.getLastShiftLine().get()});
      last_sl = (sl.end_longitudinal > al.end_longitudinal) ? sl : al;
    }

    // avoidance points: No, shift points: No -> set the ego position to the last shift point
    // so that the return-shift will be generated from ego position.
    if (!has_candidate_point && !has_registered_point) {
      last_sl.end_idx = avoid_data_.ego_closest_path_index;
      last_sl.end = avoid_data_.reference_path.points.at(last_sl.end_idx).point.pose;
      last_sl.end_shift_length = getCurrentBaseShift();
    }
  }

  // There already is a shift point candidates to go back to center line, but it could be too sharp
  // due to detection noise or timing.
  // Here the return-shift from ego is added for the in case.
  if (std::fabs(last_sl.end_shift_length) < RETURN_SHIFT_THRESHOLD) {
    const auto current_base_shift = helper_.getEgoShift();
    if (std::abs(current_base_shift) < ep) {
      DEBUG_PRINT("last shift almost is zero, and current base_shift is zero. do nothing.");
      return ret;
    }

    // Is there a shift point in the opposite direction of the current_base_shift?
    //   No  -> we can overwrite the return shift, because the other shift points that decrease
    //          the shift length are for return-shift.
    //   Yes -> we can NOT overwrite, because it might be not a return-shift, but a avoiding
    //          shift to the opposite direction which can not be overwritten by the return-shift.
    for (const auto & sl : ret) {
      if (
        (current_base_shift > 0.0 && sl.end_shift_length < -ep) ||
        (current_base_shift < 0.0 && sl.end_shift_length > ep)) {
        DEBUG_PRINT(
          "try to put overwrite return shift, but there is shift for opposite direction. Skip "
          "adding return shift.");
        return ret;
      }
    }

    // If return shift already exists in candidate or registered shift lines, skip adding return
    // shift.
    if (has_candidate_point || has_registered_point) {
      return ret;
    }

    // set the return-shift from ego.
    DEBUG_PRINT(
      "return shift already exists, but they are all candidates. Add return shift for overwrite.");
    last_sl.end_idx = avoid_data_.ego_closest_path_index;
    last_sl.end = avoid_data_.reference_path.points.at(last_sl.end_idx).point.pose;
    last_sl.end_shift_length = current_base_shift;
  }

  const auto & arclength_from_ego = avoid_data_.arclength_from_ego;

  const auto nominal_prepare_distance = helper_.getNominalPrepareDistance();
  const auto nominal_avoid_distance = helper_.getMaxAvoidanceDistance(last_sl.end_shift_length);

  if (arclength_from_ego.empty()) {
    return ret;
  }

  const auto remaining_distance = arclength_from_ego.back() - parameters_->remain_buffer_distance;

  // If the avoidance point has already been set, the return shift must be set after the point.
  const auto last_sl_distance = avoid_data_.arclength_from_ego.at(last_sl.end_idx);

  // check if there is enough distance for return.
  if (last_sl_distance > remaining_distance) {  // tmp: add some small number (+1.0)
    RCLCPP_WARN_THROTTLE(getLogger(), *clock_, 1000, "No enough distance for return.");
    return ret;
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
    al.end = avoid_data_.reference_path.points.at(al.end_idx).point.pose;
    al.end_longitudinal = arclength_from_ego.at(al.end_idx);
    al.end_shift_length = last_sl.end_shift_length;
    al.start_shift_length = last_sl.end_shift_length;
    ret.push_back(al);
    debug.step1_return_shift_line.push_back(al);
  }

  // shift point for return to center line
  {
    AvoidLine al;
    al.id = getOriginalShiftLineUniqueId();
    al.start_idx =
      utils::avoidance::findPathIndexFromArclength(arclength_from_ego, prepare_distance_scaled);
    al.start = avoid_data_.reference_path.points.at(al.start_idx).point.pose;
    al.start_longitudinal = arclength_from_ego.at(al.start_idx);
    al.end_idx = utils::avoidance::findPathIndexFromArclength(
      arclength_from_ego, prepare_distance_scaled + avoid_distance_scaled);
    al.end = avoid_data_.reference_path.points.at(al.end_idx).point.pose;
    al.end_longitudinal = arclength_from_ego.at(al.end_idx);
    al.end_shift_length = 0.0;
    al.start_shift_length = last_sl.end_shift_length;
    ret.push_back(al);
    debug.step1_return_shift_line.push_back(al);
  }

  return ret;
}

bool AvoidanceModule::isSafePath(
  ShiftedPath & shifted_path, [[maybe_unused]] DebugData & debug) const
{
  const auto & p = planner_data_->parameters;

  if (!parameters_->enable_safety_check) {
    return true;  // if safety check is disabled, it always return safe.
  }

  const bool limit_to_max_velocity = false;
  const auto ego_predicted_path_for_front_object = utils::avoidance::convertToPredictedPath(
    shifted_path.path, planner_data_, true, limit_to_max_velocity, parameters_);
  const auto ego_predicted_path_for_rear_object = utils::avoidance::convertToPredictedPath(
    shifted_path.path, planner_data_, false, limit_to_max_velocity, parameters_);

  const auto ego_idx = planner_data_->findEgoIndex(shifted_path.path.points);
  const auto is_right_shift = [&]() -> std::optional<bool> {
    for (size_t i = ego_idx; i < shifted_path.shift_length.size(); i++) {
      const auto length = shifted_path.shift_length.at(i);

      if (parameters_->lateral_avoid_check_threshold < length) {
        return false;
      }

      if (parameters_->lateral_avoid_check_threshold < -1.0 * length) {
        return true;
      }
    }

    return std::nullopt;
  }();

  if (!is_right_shift.has_value()) {
    return true;
  }

  const auto hysteresis_factor = safe_ ? 1.0 : parameters_->hysteresis_factor_expand_rate;

  const auto safety_check_target_objects = utils::avoidance::getSafetyCheckTargetObjects(
    avoid_data_, planner_data_, parameters_, is_right_shift.value());

  for (const auto & object : safety_check_target_objects) {
    auto current_debug_data = marker_utils::createObjectDebug(object);

    const auto obj_polygon =
      tier4_autoware_utils::toPolygon2d(object.initial_pose.pose, object.shape);

    const auto is_object_front =
      utils::path_safety_checker::isTargetObjectFront(getEgoPose(), obj_polygon, p.vehicle_info);

    const auto is_object_oncoming =
      utils::path_safety_checker::isTargetObjectOncoming(getEgoPose(), object.initial_pose.pose);

    const auto obj_predicted_paths = utils::path_safety_checker::getPredictedPathFromObj(
      object, parameters_->check_all_predicted_path);

    const auto & ego_predicted_path = is_object_front && !is_object_oncoming
                                        ? ego_predicted_path_for_front_object
                                        : ego_predicted_path_for_rear_object;

    for (const auto & obj_path : obj_predicted_paths) {
      if (!utils::path_safety_checker::checkCollision(
            shifted_path.path, ego_predicted_path, object, obj_path, p, parameters_->rss_params,
            hysteresis_factor, current_debug_data.second)) {
        marker_utils::updateCollisionCheckDebugMap(
          debug.collision_check, current_debug_data, false);

        safe_count_ = 0;
        return false;
      }
    }
    marker_utils::updateCollisionCheckDebugMap(debug.collision_check, current_debug_data, true);
  }

  safe_count_++;

  return safe_ || safe_count_ > parameters_->hysteresis_factor_safe_count;
}

void AvoidanceModule::generateExpandDrivableLanes(BehaviorModuleOutput & output) const
{
  std::vector<DrivableLanes> drivable_lanes;
  for (const auto & lanelet : avoid_data_.current_lanelets) {
    drivable_lanes.push_back(
      utils::avoidance::generateExpandDrivableLanes(lanelet, planner_data_, parameters_));
  }

  {  // for new architecture
    DrivableAreaInfo current_drivable_area_info;
    // generate drivable lanes
    current_drivable_area_info.drivable_lanes = drivable_lanes;
    // generate obstacle polygons
    current_drivable_area_info.obstacles =
      utils::avoidance::generateObstaclePolygonsForDrivableArea(
        avoid_data_.target_objects, parameters_, planner_data_->parameters.vehicle_width / 2.0);
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
  const auto & data = avoid_data_;

  resetPathCandidate();
  resetPathReference();

  updatePathShifter(data.safe_shift_line);

  if (data.yield_required) {
    removeRegisteredShiftLines();
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

  avoid_data_.state = updateEgoState(data);

  // update output data
  {
    updateEgoBehavior(data, spline_shift_path);
    updateInfoMarker(avoid_data_);
    updateDebugMarker(avoid_data_, path_shifter_, debug_data_);
  }

  if (isDrivingSameLane(helper_.getPreviousDrivingLanes(), data.current_lanelets)) {
    output.path = std::make_shared<PathWithLaneId>(spline_shift_path.path);
  } else {
    output.path = getPreviousModuleOutput().path;
    RCLCPP_WARN(getLogger(), "Previous module lane is updated. Do nothing.");
  }

  output.reference_path = getPreviousModuleOutput().reference_path;
  path_reference_ = getPreviousModuleOutput().reference_path;

  const size_t ego_idx = planner_data_->findEgoIndex(output.path->points);
  utils::clipPathLength(*output.path, ego_idx, planner_data_->parameters);

  // Drivable area generation.
  generateExpandDrivableLanes(output);
  setDrivableLanes(output.drivable_area_info.drivable_lanes);

  return output;
}

CandidateOutput AvoidanceModule::planCandidate() const
{
  const auto & data = avoid_data_;

  CandidateOutput output;

  auto shifted_path = data.candidate_path;

  if (!data.safe_shift_line.empty()) {  // clip from shift start index for visualize
    utils::clipPathLength(
      shifted_path.path, data.safe_shift_line.front().start_idx, std::numeric_limits<double>::max(),
      0.0);

    const auto sl = helper_.getMainShiftLine(data.safe_shift_line);
    const auto sl_front = data.safe_shift_line.front();
    const auto sl_back = data.safe_shift_line.back();

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
  BehaviorModuleOutput out = plan();

  if (path_shifter_.getShiftLines().empty()) {
    out.turn_signal_info = getPreviousModuleOutput().turn_signal_info;
  }

  path_candidate_ = std::make_shared<PathWithLaneId>(planCandidate().path_candidate);
  path_reference_ = getPreviousModuleOutput().reference_path;

  return out;
}

void AvoidanceModule::updatePathShifter(const AvoidLineArray & shift_lines)
{
  if (parameters_->disable_path_update) {
    return;
  }

  if (shift_lines.empty()) {
    return;
  }

  if (!isActivated()) {
    return;
  }

  addNewShiftLines(path_shifter_, shift_lines);

  current_raw_shift_lines_ = avoid_data_.raw_shift_line;

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

    if (sl.end_idx > new_shift_end_idx) {
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

  const double road_velocity = avoid_data_.reference_path.points.at(front_new_shift_line.start_idx)
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

AvoidLineArray AvoidanceModule::findNewShiftLine(
  const AvoidLineArray & shift_lines, DebugData & debug) const
{
  if (shift_lines.empty()) {
    return {};
  }

  // add small shift lines.
  const auto add_straight_shift =
    [&, this](auto & subsequent, bool has_large_shift, const size_t start_idx) {
      for (size_t i = start_idx; i < shift_lines.size(); ++i) {
        if (
          std::abs(shift_lines.at(i).getRelativeLength()) >
          parameters_->lateral_small_shift_threshold) {
          if (has_large_shift) {
            return;
          }

          has_large_shift = true;
        }

        if (!isComfortable(AvoidLineArray{shift_lines.at(i)})) {
          return;
        }

        subsequent.push_back(shift_lines.at(i));
      }
    };

  // get subsequent shift lines.
  const auto get_subsequent_shift = [&, this](size_t i) {
    AvoidLineArray subsequent{shift_lines.at(i)};

    if (!isComfortable(subsequent)) {
      return subsequent;
    }

    if (shift_lines.size() == i + 1) {
      return subsequent;
    }

    if (!isComfortable(AvoidLineArray{shift_lines.at(i + 1)})) {
      return subsequent;
    }

    if (
      std::abs(shift_lines.at(i).getRelativeLength()) <
      parameters_->lateral_small_shift_threshold) {
      const auto has_large_shift =
        shift_lines.at(i + 1).getRelativeLength() > parameters_->lateral_small_shift_threshold;

      // candidate.at(i) is small length shift line. add large length shift line.
      subsequent.push_back(shift_lines.at(i + 1));
      add_straight_shift(subsequent, has_large_shift, i + 2);
    } else {
      // candidate.at(i) is large length shift line. add small length shift lines.
      add_straight_shift(subsequent, true, i + 1);
    }

    return subsequent;
  };

  // check ignore or not.
  const auto is_ignore_shift = [this](const auto & s) {
    return std::abs(helper_.getRelativeShiftToPath(s)) < parameters_->lateral_execution_threshold;
  };

  for (size_t i = 0; i < shift_lines.size(); ++i) {
    const auto & candidate = shift_lines.at(i);

    // new shift points must exist in front of Ego
    // this value should be larger than -eps consider path shifter calculation error.
    if (candidate.start_idx < avoid_data_.ego_closest_path_index) {
      break;
    }

    if (!is_ignore_shift(candidate)) {
      const auto new_shift_lines = get_subsequent_shift(i);
      debug.step4_new_shift_line = new_shift_lines;
      return new_shift_lines;
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

  debug_data_.proposed_spline_shift = proposed_shift_path.shift_length;

  // check offset between new shift path and ego position.
  {
    const auto new_idx = planner_data_->findEgoIndex(proposed_shift_path.path.points);
    const auto new_shift_length = proposed_shift_path.shift_length.at(new_idx);

    constexpr double THRESHOLD = 0.1;
    const auto offset = std::abs(new_shift_length - helper_.getEgoShift());
    if (offset > THRESHOLD) {
      RCLCPP_DEBUG_THROTTLE(
        getLogger(), *clock_, 1000, "new shift line is invalid. [HUGE OFFSET (%.2f)]", offset);
      return false;
    }
  }

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
  avoid_data_ = AvoidancePlanningData();

  // update base path and target objects.
  fillFundamentalData(avoid_data_, debug_data_);

  // an empty path will kill further processing
  if (avoid_data_.reference_path.points.empty()) {
    return;
  }

  // update registered shift point for new reference path & remove past objects
  updateRegisteredRawShiftLines();

  // update shift line and check path safety.
  fillShiftLine(avoid_data_, debug_data_);

  // update ego behavior.
  fillEgoStatus(avoid_data_, debug_data_);

  // update debug data.
  fillDebugData(avoid_data_, debug_data_);

  // update rtc status.
  updateRTCData();

  safe_ = avoid_data_.safe;
}

void AvoidanceModule::processOnEntry()
{
  initVariables();
  removeRTCStatus();
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
  left_shift_array_.clear();
  right_shift_array_.clear();
  uuid_map_.at("left") = generateUUID();
  uuid_map_.at("right") = generateUUID();
  candidate_uuid_ = generateUUID();
}

void AvoidanceModule::updateRTCData()
{
  const auto & data = avoid_data_;

  updateRegisteredRTCStatus(helper_.getPreviousSplineShiftPath().path);

  const auto candidates = data.safe ? data.safe_shift_line : data.new_shift_line;

  if (candidates.empty()) {
    removeCandidateRTCStatus();
    return;
  }

  const auto shift_line = helper_.getMainShiftLine(candidates);
  if (helper_.getRelativeShiftToPath(shift_line) > 0.0) {
    removePreviousRTCStatusRight();
  } else if (helper_.getRelativeShiftToPath(shift_line) < 0.0) {
    removePreviousRTCStatusLeft();
  } else {
    RCLCPP_WARN_STREAM(getLogger(), "Direction is UNKNOWN");
  }

  CandidateOutput output;

  const auto sl_front = candidates.front();
  const auto sl_back = candidates.back();

  output.path_candidate = data.candidate_path.path;
  output.lateral_shift = helper_.getRelativeShiftToPath(shift_line);
  output.start_distance_to_path_change = sl_front.start_longitudinal;
  output.finish_distance_to_path_change = sl_back.end_longitudinal;

  updateCandidateRTCStatus(output);
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
  using marker_utils::createPolygonMarkerArray;
  using marker_utils::createPoseMarkerArray;
  using marker_utils::createShiftGradMarkerArray;
  using marker_utils::createShiftLengthMarkerArray;
  using marker_utils::createShiftLineMarkerArray;
  using marker_utils::showPolygon;
  using marker_utils::showPredictedPath;
  using marker_utils::showSafetyCheckInfo;
  using marker_utils::avoidance_marker::createAvoidLineMarkerArray;
  using marker_utils::avoidance_marker::createEgoStatusMarkerArray;
  using marker_utils::avoidance_marker::createOtherObjectsMarkerArray;
  using marker_utils::avoidance_marker::createOverhangFurthestLineStringMarkerArray;
  using marker_utils::avoidance_marker::createPredictedVehiclePositions;
  using marker_utils::avoidance_marker::makeOverhangToRoadShoulderMarkerArray;
  using tier4_autoware_utils::appendMarkerArray;

  debug_marker_.markers.clear();

  if (!parameters_->publish_debug_marker) {
    return;
  }

  const auto & path = data.reference_path;

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

  const auto addObjects = [&](const ObjectDataArray & objects, const auto & ns) {
    add(createOtherObjectsMarkerArray(objects, ns));
  };

  const auto addShiftLength =
    [&](const auto & shift_length, const auto & ns, auto r, auto g, auto b) {
      add(createShiftLengthMarkerArray(shift_length, path, ns, r, g, b));
    };

  const auto addShiftGrad = [&](
                              const auto & shift_grad, const auto & shift_length, const auto & ns,
                              auto r, auto g, auto b) {
    add(createShiftGradMarkerArray(shift_grad, shift_length, path, ns, r, g, b));
  };

  // ignore objects
  {
    addObjects(data.other_objects, AvoidanceDebugFactor::OBJECT_IS_BEHIND_THRESHOLD);
    addObjects(data.other_objects, AvoidanceDebugFactor::OBJECT_IS_IN_FRONT_THRESHOLD);
    addObjects(data.other_objects, AvoidanceDebugFactor::OBJECT_IS_NOT_TYPE);
    addObjects(data.other_objects, AvoidanceDebugFactor::OBJECT_BEHIND_PATH_GOAL);
    addObjects(data.other_objects, AvoidanceDebugFactor::TOO_NEAR_TO_CENTERLINE);
    addObjects(data.other_objects, AvoidanceDebugFactor::NOT_PARKING_OBJECT);
    addObjects(data.other_objects, std::string("MovingObject"));
    addObjects(data.other_objects, std::string("CrosswalkUser"));
    addObjects(data.other_objects, std::string("OutOfTargetArea"));
    addObjects(data.other_objects, std::string("NotNeedAvoidance"));
    addObjects(data.other_objects, std::string("LessThanExecutionThreshold"));
  }

  // shift line pre-process
  {
    addAvoidLine(debug.step1_registered_shift_line, "step1_registered_shift_line", 0.2, 0.2, 1.0);
    addAvoidLine(debug.step1_current_shift_line, "step1_current_shift_line", 0.2, 0.4, 0.8, 0.3);
    addAvoidLine(debug.step1_merged_shift_line, "step1_merged_shift_line", 0.2, 0.6, 0.6, 0.3);
    addAvoidLine(debug.step1_filled_shift_line, "step1_filled_shift_line", 0.2, 0.8, 0.4, 0.3);
    addAvoidLine(debug.step1_return_shift_line, "step1_return_shift_line", 0.2, 1.0, 0.2, 0.3);
  }

  // merge process
  {
    addAvoidLine(debug.step2_merged_shift_line, "step2_merged_shift_line", 0.2, 1.0, 0.0, 0.3);
  }

  // trimming process
  {
    addAvoidLine(debug.step3_grad_filtered_1st, "step3_grad_filtered_1st", 0.2, 0.8, 0.0, 0.3);
    addAvoidLine(debug.step3_grad_filtered_2nd, "step3_grad_filtered_2nd", 0.4, 0.6, 0.0, 0.3);
    addAvoidLine(debug.step3_grad_filtered_3rd, "step3_grad_filtered_3rd", 0.6, 0.4, 0.0, 0.3);
  }

  // registering process
  {
    addShiftLine(shifter.getShiftLines(), "step4_old_shift_line", 1.0, 1.0, 0.0, 0.3);
    addAvoidLine(data.raw_shift_line, "step4_raw_shift_line", 1.0, 0.0, 0.0, 0.3);
    addAvoidLine(data.new_shift_line, "step4_new_shift_line", 1.0, 0.0, 0.0, 0.3);
  }

  // safety check
  {
    add(showSafetyCheckInfo(debug.collision_check, "object_debug_info"));
    add(showPredictedPath(debug.collision_check, "ego_predicted_path"));
    add(showPolygon(debug.collision_check, "ego_and_target_polygon_relation"));
  }

  // shift length
  {
    addShiftLength(debug.pos_shift, "merged_length_pos", 0.0, 0.7, 0.5);
    addShiftLength(debug.neg_shift, "merged_length_neg", 0.0, 0.5, 0.7);
    addShiftLength(debug.total_shift, "merged_length_total", 0.99, 0.4, 0.2);
  }

  // shift grad
  {
    addShiftGrad(debug.pos_shift_grad, debug.pos_shift, "merged_grad_pos", 0.0, 0.7, 0.5);
    addShiftGrad(debug.neg_shift_grad, debug.neg_shift, "merged_grad_neg", 0.0, 0.5, 0.7);
    addShiftGrad(debug.total_forward_grad, debug.total_shift, "grad_forward", 0.99, 0.4, 0.2);
    addShiftGrad(debug.total_backward_grad, debug.total_shift, "grad_backward", 0.4, 0.2, 0.9);
  }

  // misc
  {
    add(createEgoStatusMarkerArray(data, getEgoPose(), "ego_status"));
    add(createPathMarkerArray(path, "centerline_resampled", 0, 0.0, 0.9, 0.5));
    add(createLaneletsAreaMarkerArray(*debug.current_lanelets, "current_lanelet", 0.0, 1.0, 0.0));
    add(createPolygonMarkerArray(debug.detection_area, "detection_area", 0L, 0.16, 1.0, 0.69, 0.1));
    add(makeOverhangToRoadShoulderMarkerArray(data.target_objects, "overhang"));
    add(createOverhangFurthestLineStringMarkerArray(debug.bounds, "bounds", 1.0, 0.0, 1.0));
  }
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

void AvoidanceModule::insertReturnDeadLine(
  const bool use_constraints_for_decel, ShiftedPath & shifted_path) const
{
  const auto & data = avoid_data_;

  if (!planner_data_->route_handler->isInGoalRouteSection(data.current_lanelets.back())) {
    RCLCPP_DEBUG(getLogger(), "goal is far enough.");
    return;
  }

  const auto shift_length = path_shifter_.getLastShiftLength();

  if (std::abs(shift_length) < 1e-3) {
    RCLCPP_DEBUG(getLogger(), "don't have to consider return shift.");
    return;
  }

  const auto min_return_distance = helper_.getMinAvoidanceDistance(shift_length);

  const auto to_goal = calcSignedArcLength(
    shifted_path.path.points, getEgoPosition(), shifted_path.path.points.size() - 1);
  const auto to_stop_line = to_goal - min_return_distance - parameters_->remain_buffer_distance;

  // If we don't need to consider deceleration constraints, insert a deceleration point
  // and return immediately
  if (!use_constraints_for_decel) {
    utils::avoidance::insertDecelPoint(
      getEgoPosition(), to_stop_line - parameters_->stop_buffer, 0.0, shifted_path.path,
      stop_pose_);
    return;
  }

  // If the stop distance is not enough for comfortable stop, don't insert wait point.
  const auto is_comfortable_stop = helper_.getFeasibleDecelDistance(0.0) < to_stop_line;
  if (!is_comfortable_stop) {
    RCLCPP_DEBUG(getLogger(), "stop distance is not enough.");
    return;
  }

  utils::avoidance::insertDecelPoint(
    getEgoPosition(), to_stop_line - parameters_->stop_buffer, 0.0, shifted_path.path, stop_pose_);

  // insert slow down speed.
  const double current_target_velocity = PathShifter::calcFeasibleVelocityFromJerk(
    shift_length, helper_.getLateralMinJerkLimit(), to_stop_line);
  if (current_target_velocity < getEgoSpeed()) {
    RCLCPP_DEBUG(getLogger(), "current velocity exceeds target slow down speed.");
    return;
  }

  const auto start_idx = planner_data_->findEgoIndex(shifted_path.path.points);
  for (size_t i = start_idx; i < shifted_path.path.points.size(); ++i) {
    const auto distance_from_ego = calcSignedArcLength(shifted_path.path.points, start_idx, i);

    // slow down speed is inserted only in front of the object.
    const auto shift_longitudinal_distance = to_stop_line - distance_from_ego;
    if (shift_longitudinal_distance < 0.0) {
      break;
    }

    // target speed with nominal jerk limits.
    const double v_target = PathShifter::calcFeasibleVelocityFromJerk(
      shift_length, helper_.getLateralMinJerkLimit(), shift_longitudinal_distance);
    const double v_original = shifted_path.path.points.at(i).point.longitudinal_velocity_mps;
    const double v_insert =
      std::max(v_target - parameters_->buf_slow_down_speed, parameters_->min_slow_down_speed);

    shifted_path.path.points.at(i).point.longitudinal_velocity_mps = std::min(v_original, v_insert);
  }
}

void AvoidanceModule::insertWaitPoint(
  const bool use_constraints_for_decel, ShiftedPath & shifted_path) const
{
  const auto & data = avoid_data_;

  if (!data.stop_target_object) {
    return;
  }

  if (helper_.isShifted()) {
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
  const auto & data = avoid_data_;

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
  const auto & data = avoid_data_;

  if (data.target_objects.empty()) {
    return;
  }

  if (helper_.isShifted()) {
    return;
  }

  const auto decel_distance = helper_.getFeasibleDecelDistance(p->yield_velocity, false);
  utils::avoidance::insertDecelPoint(
    getEgoPosition(), decel_distance, p->yield_velocity, shifted_path.path, slow_pose_);
}

void AvoidanceModule::insertPrepareVelocity(ShiftedPath & shifted_path) const
{
  const auto & data = avoid_data_;

  // do nothing if there is no avoidance target.
  if (data.target_objects.empty()) {
    return;
  }

  // insert slow down speed only when the avoidance maneuver is not initiated.
  if (helper_.isShifted()) {
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
  const double current_target_velocity = PathShifter::calcFeasibleVelocityFromJerk(
    shift_length, helper_.getLateralMinJerkLimit(), distance_to_object);
  if (current_target_velocity < getEgoSpeed() && decel_distance < remaining_distance) {
    utils::avoidance::insertDecelPoint(
      getEgoPosition(), decel_distance, parameters_->velocity_map.front(), shifted_path.path,
      slow_pose_);
    return;
  }

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
