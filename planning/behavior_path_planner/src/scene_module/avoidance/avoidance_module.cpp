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

#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/scene_module/avoidance/avoidance_utils.hpp"
#include "behavior_path_planner/scene_module/avoidance/debug.hpp"
#include "behavior_path_planner/utilities.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <opencv2/opencv.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <algorithm>
#include <iomanip>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <vector>

// set as macro so that calling function name will be printed.
// debug print is heavy. turn on only when debugging.
#define DEBUG_PRINT(...) \
  RCLCPP_DEBUG_EXPRESSION(getLogger(), parameters_.print_debug_info, __VA_ARGS__)
#define printShiftPoints(p, msg) DEBUG_PRINT("[%s] %s", msg, toStrInfo(p).c_str())

namespace behavior_path_planner
{
using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::calcLateralDeviation;
using tier4_autoware_utils::calcSignedArcLength;
using tier4_autoware_utils::createPoint;
using tier4_autoware_utils::findNearestIndex;

AvoidanceModule::AvoidanceModule(
  const std::string & name, rclcpp::Node & node, const AvoidanceParameters & parameters)
: SceneModuleInterface{name, node}, parameters_{parameters}
{
  using std::placeholders::_1;

  approval_handler_.waitApproval();
}

bool AvoidanceModule::isExecutionRequested() const
{
  DEBUG_PRINT("AVOIDANCE isExecutionRequested");

  if (current_state_ == BT::NodeStatus::RUNNING) {
    return true;
  }

  DebugData debug;
  const auto avoid_data = calcAvoidancePlanningData(debug);

  const bool has_avoidance_target = !avoid_data.objects.empty();
  return has_avoidance_target ? true : false;
}

bool AvoidanceModule::isExecutionReady() const
{
  DEBUG_PRINT("AVOIDANCE isExecutionReady");

  if (current_state_ == BT::NodeStatus::RUNNING) {
    return true;
  }

  return true;
}

BT::NodeStatus AvoidanceModule::updateState()
{
  const auto is_plan_running = isAvoidancePlanRunning();

  DebugData debug;
  const auto avoid_data = calcAvoidancePlanningData(debug);
  const bool has_avoidance_target = !avoid_data.objects.empty();

  if (!is_plan_running && !has_avoidance_target) {
    current_state_ = BT::NodeStatus::SUCCESS;
  } else {
    current_state_ = BT::NodeStatus::RUNNING;
  }

  DEBUG_PRINT(
    "is_plan_running = %d, has_avoidance_target = %d", is_plan_running, has_avoidance_target);

  return current_state_;
}

bool AvoidanceModule::isAvoidancePlanRunning() const
{
  const bool has_base_offset = std::abs(path_shifter_.getBaseOffset()) > 0.01;
  const bool has_shift_point = (path_shifter_.getShiftPointsSize() > 0);
  return has_base_offset || has_shift_point;
}

AvoidancePlanningData AvoidanceModule::calcAvoidancePlanningData(DebugData & debug) const
{
  AvoidancePlanningData data;

  // reference pose
  const auto reference_pose = getUnshiftedEgoPose(prev_output_);
  data.reference_pose = reference_pose.pose;

  // center line path (output of this function must have size > 1)
  const auto center_path = calcCenterLinePath(planner_data_, reference_pose);
  debug.center_line = center_path;
  if (center_path.points.size() < 2) {
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 5000, "calcCenterLinePath() must return path which size > 1");
    return data;
  }

  // reference path
  data.reference_path =
    util::resamplePathWithSpline(center_path, parameters_.resample_interval_for_planning);
  if (data.reference_path.points.size() < 2) {
    // if the resampled path has only 1 point, use original path.
    data.reference_path = center_path;
  }
  data.ego_closest_path_index =
    findNearestIndex(data.reference_path.points, data.reference_pose.position);

  // arclength from ego pose (used in many functions)
  data.arclength_from_ego = util::calcPathArcLengthArray(
    data.reference_path, 0, data.reference_path.points.size(),
    calcSignedArcLength(data.reference_path.points, getEgoPosition(), 0));

  // lanelet info
  data.current_lanelets = calcLaneAroundPose(
    planner_data_, reference_pose.pose, planner_data_->parameters.backward_path_length);

  // target objects for avoidance
  data.objects = calcAvoidanceTargetObjects(data.current_lanelets, data.reference_path, debug);

  DEBUG_PRINT("target object size = %lu", data.objects.size());

  return data;
}

ObjectDataArray AvoidanceModule::calcAvoidanceTargetObjects(
  const lanelet::ConstLanelets & current_lanes, const PathWithLaneId & reference_path,
  DebugData & debug) const
{
  const auto & path_points = reference_path.points;
  const auto & ego_pos = getEgoPosition();

  // velocity filter: only for stopped vehicle
  const auto objects_candidate = util::filterObjectsByVelocity(
    *planner_data_->dynamic_object, parameters_.threshold_speed_object_is_stopped);

  // detection area filter
  // when expanding lanelets, right_offset must be minus.
  // This is because y axis is positive on the left.
  const auto expanded_lanelets = lanelet::utils::getExpandedLanelets(
    current_lanes, parameters_.detection_area_left_expand_dist,
    parameters_.detection_area_right_expand_dist * (-1.0));
  const auto lane_filtered_objects_index =
    util::filterObjectsByLanelets(objects_candidate, expanded_lanelets);

  DEBUG_PRINT("dynamic_objects size = %lu", planner_data_->dynamic_object->objects.size());
  DEBUG_PRINT("object_candidate size = %lu", objects_candidate.objects.size());
  DEBUG_PRINT("lane_filtered_objects size = %lu", lane_filtered_objects_index.size());

  // for goal
  const auto & rh = planner_data_->route_handler;
  const auto dist_to_goal =
    rh->isInGoalRouteSection(expanded_lanelets.back())
      ? calcSignedArcLength(path_points, ego_pos, rh->getGoalPose().position)
      : std::numeric_limits<double>::max();

  // for filtered objects
  ObjectDataArray target_objects;
  for (const auto & i : lane_filtered_objects_index) {
    const auto & object = objects_candidate.objects.at(i);
    const auto & object_pos = object.kinematics.initial_pose_with_covariance.pose.position;

    if (!isTargetObjectType(object)) {
      DEBUG_PRINT("Ignore object: (isTargetObjectType is false)");
      continue;
    }

    ObjectData object_data;
    object_data.object = object;

    // calc longitudinal distance from ego to closest target object footprint point.
    object_data.longitudinal = calcDistanceToClosestFootprintPoint(reference_path, object, ego_pos);

    // object is behind ego or too far.
    if (object_data.longitudinal < -parameters_.object_check_backward_distance) {
      DEBUG_PRINT("Ignore object: (object < -backward_distance threshold)");
      continue;
    }
    if (object_data.longitudinal > parameters_.object_check_forward_distance) {
      DEBUG_PRINT("Ignore object: (object > forward_distance threshold)");
      continue;
    }

    // Target object is behind the path goal -> ignore.
    if (object_data.longitudinal > dist_to_goal) {
      DEBUG_PRINT("Ignore object: (object is behind the path goal)");
      continue;
    }

    // Calc lateral deviation from path to target object.
    const auto object_closest_index = findNearestIndex(path_points, object_pos);
    const auto object_closest_pose = path_points.at(object_closest_index).point.pose;
    object_data.lateral = calcLateralDeviation(object_closest_pose, object_pos);

    // Object is on center line -> ignore.
    if (std::abs(object_data.lateral) < parameters_.threshold_distance_object_is_on_center) {
      DEBUG_PRINT("Ignore object: (object is on center line)");
      continue;
    }

    // Find the footprint point closest to the path, set to object_data.overhang_distance.
    object_data.overhang_dist = calcOverhangDistance(object_data, object_closest_pose);

    DEBUG_PRINT(
      "set object_data: longitudinal = %f, lateral = %f, largest_overhang = %f",
      object_data.longitudinal, object_data.lateral, object_data.overhang_dist);

    // set data
    target_objects.push_back(object_data);
  }

  // debug
  {
    debug.current_lanelets = std::make_shared<lanelet::ConstLanelets>(current_lanes);
    debug.expanded_lanelets = std::make_shared<lanelet::ConstLanelets>(expanded_lanelets);
  }

  return target_objects;
}

/**
 * updateRegisteredRawShiftPoints
 *
 *  - update path index of the registered objects
 *  - remove old objects whose end point is behind ego pose.
 */
void AvoidanceModule::updateRegisteredRawShiftPoints()
{
  fillAdditionalInfoFromPoint(registered_raw_shift_points_);

  AvoidPointArray avoid_points;
  const int margin = 0;
  const auto deadline = static_cast<size_t>(
    std::max(static_cast<int>(avoidance_data_.ego_closest_path_index) - margin, 0));

  for (const auto & ap : registered_raw_shift_points_) {
    if (ap.end_idx > deadline) {
      avoid_points.push_back(ap);
    }
  }

  DEBUG_PRINT(
    "ego_closest_path_index = %lu, registered_raw_shift_points_ size: %lu -> %lu",
    avoidance_data_.ego_closest_path_index, registered_raw_shift_points_.size(),
    avoid_points.size());

  printShiftPoints(registered_raw_shift_points_, "registered_raw_shift_points_ (before)");
  printShiftPoints(avoid_points, "registered_raw_shift_points_ (after)");

  registered_raw_shift_points_ = avoid_points;
  debug_data_.registered_raw_shift = registered_raw_shift_points_;
}

AvoidPointArray AvoidanceModule::calcShiftPoints(
  AvoidPointArray & current_raw_shift_points, DebugData & debug) const
{
  /**
   * Generate raw_shift_points (shift length, avoidance start point, end point, return point, etc)
   * for each object. These raw shift points are merged below to compute appropriate shift points.
   */
  current_raw_shift_points = calcRawShiftPointsFromObjects(avoidance_data_.objects);
  debug.current_raw_shift = current_raw_shift_points;

  /**
   * Use all registered points. For the current points, if the similar one of the current
   * points are already registered, will not use it.
   * TODO(Horibe): enrich this logic to be able to consider the removal of the registered
   *               shift, because it cannot handle the case like "we don't have to avoid
   *               the object anymore".
   */
  auto total_raw_shift_points =
    combineRawShiftPointsWithUniqueCheck(registered_raw_shift_points_, current_raw_shift_points);

  printShiftPoints(current_raw_shift_points, "current_raw_shift_points");
  printShiftPoints(registered_raw_shift_points_, "registered_raw_shift_points");
  printShiftPoints(total_raw_shift_points, "total_raw_shift_points");

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
  addReturnShiftPointFromEgo(total_raw_shift_points, current_raw_shift_points);
  printShiftPoints(total_raw_shift_points, "total_raw_shift_points_with_extra_return_shift");

  /**
   * On each path point, compute shift length with considering the raw shift points.
   * Then create a merged shift points by finding the change point of the gradient of shifting.
   *  - take maximum shift length if there is duplicate shift point
   *  - take sum if there are shifts for opposite direction (right and left)
   *  - shift length is interpolated linearly.
   * Note: Because this function just foolishly extracts points, it includes
   *       insignificant small (useless) shift points, which should be removed in post-process.
   */
  auto merged_shift_points = mergeShiftPoints(total_raw_shift_points, debug);
  debug.merged = merged_shift_points;

  /*
   * Remove unnecessary shift points
   *  - Quantize the shift length to reduce the shift point noise
   *  - Change the shift length to the previous one if the deviation is small.
   *  - Combine shift points that have almost same gradient
   *  - Remove unnecessary return shift (back to the center line).
   */
  auto shift_points = trimShiftPoint(merged_shift_points, debug);
  DEBUG_PRINT("final shift point size = %lu", shift_points.size());

  return shift_points;
}

void AvoidanceModule::registerRawShiftPoints(const AvoidPointArray & future)
{
  if (future.empty()) {
    RCLCPP_ERROR(getLogger(), "future is empty! return.");
    return;
  }

  const auto old_size = registered_raw_shift_points_.size();

  const auto future_with_info = fillAdditionalInfo(future);
  printShiftPoints(future_with_info, "future_with_info");
  printShiftPoints(registered_raw_shift_points_, "registered_raw_shift_points_");
  printShiftPoints(current_raw_shift_points_, "current_raw_shift_points_");

  const auto isAlreadyRegistered = [this](const auto id) {
    const auto & r = registered_raw_shift_points_;
    return std::any_of(r.begin(), r.end(), [id](const auto & r_sp) { return r_sp.id == id; });
  };

  const auto getAvoidPointByID = [this](const auto id) {
    for (const auto & sp : current_raw_shift_points_) {
      if (sp.id == id) {
        return sp;
      }
    }
    return AvoidPoint{};
  };

  for (const auto & ap : future_with_info) {
    if (ap.parent_ids.empty()) {
      RCLCPP_ERROR(getLogger(), "avoid point for path_shifter must have parent_id.");
    }
    for (const auto parent_id : ap.parent_ids) {
      if (!isAlreadyRegistered(parent_id)) {
        registered_raw_shift_points_.push_back(getAvoidPointByID(parent_id));
      }
    }
  }

  DEBUG_PRINT("registered object size: %lu -> %lu", old_size, registered_raw_shift_points_.size());
}

/**
 * calcRawShiftPointsFromObjects
 *
 * Calculate the shift points (start/end point, shift length) from the object lateral
 * and longitudinal positions in the Frenet coordinate. The jerk limit is also considered here.
 */
AvoidPointArray AvoidanceModule::calcRawShiftPointsFromObjects(
  const ObjectDataArray & objects) const
{
  const auto avoid_margin =
    parameters_.lateral_collision_margin + 0.5 * planner_data_->parameters.vehicle_width;
  const auto prepare_distance = getNominalPrepareDistance();

  // To be consistent with changes in the ego position, the current shift length is considered.
  const auto current_ego_shift = getCurrentShift();

  AvoidPointArray avoid_points;
  for (auto & o : objects) {
    // calc shift length with margin and shift limit
    const auto shift_length = isOnRight(o)
                                ? std::min(o.overhang_dist + avoid_margin, getLeftShiftBound())
                                : std::max(o.overhang_dist - avoid_margin, getRightShiftBound());

    const auto avoiding_shift = shift_length - current_ego_shift;
    const auto return_shift = shift_length;

    // use absolute dist for return-to-center, relative dist from current for avoiding.
    const auto nominal_avoid_distance = getNominalAvoidanceDistance(avoiding_shift);
    const auto nominal_return_distance = getNominalAvoidanceDistance(return_shift);

    /**
     * Is there enough distance from ego to object for avoidance?
     *   - Yes -> use the nominal distance.
     *   - No -> check if it is possible to avoid within maximum jerk limit.
     *     - Yes -> use the stronger jerk.
     *     - No -> ignore this object. Expected behavior is that the vehicle will stop in front
     *             of the obstacle, then start avoidance.
     */
    const bool has_enough_distance = o.longitudinal > (prepare_distance + nominal_avoid_distance);
    const auto remaining_distance = o.longitudinal - prepare_distance;
    if (!has_enough_distance) {
      if (remaining_distance <= 0.0) {
        // TODO(Horibe) Even if there is no enough distance for avoidance shift, the
        // return-to-center shift must be considered for each object if the current_shift
        // is not zero.
        DEBUG_PRINT("object is ignored since remaining_distance <= 0");
        continue;
      }

      // This is the case of exceeding the jerk limit. Use the sharp avoidance ego speed.
      const auto required_jerk = path_shifter_.calcJerkFromLatLonDistance(
        avoiding_shift, remaining_distance, getSharpAvoidanceEgoSpeed());
      if (required_jerk > parameters_.max_lateral_jerk) {
        DEBUG_PRINT(
          "object is ignored required_jerk is too large (req: %f, max: %f)", required_jerk,
          parameters_.max_lateral_jerk);
        continue;
      }
    }
    const auto avoiding_distance =
      has_enough_distance ? nominal_avoid_distance : remaining_distance;

    DEBUG_PRINT(
      "nominal_lateral_jerk = %f, getNominalAvoidanceEgoSpeed() = %f, prepare_distance = %f, "
      "has_enough_distance = %d",
      parameters_.nominal_lateral_jerk, getNominalAvoidanceEgoSpeed(), prepare_distance,
      has_enough_distance);

    // TODO(Horibe): add margin with object length. __/\__ -> __/¯¯\__
    AvoidPoint ap_avoid;
    ap_avoid.length = shift_length;
    ap_avoid.start_length = current_ego_shift;
    ap_avoid.end_longitudinal = o.longitudinal;
    ap_avoid.start_longitudinal = o.longitudinal - avoiding_distance;
    ap_avoid.id = getOriginalShiftPointUniqueId();
    ap_avoid.object = o;
    avoid_points.push_back(ap_avoid);

    // The end_margin also has the purpose of preventing the return path from NOT being
    // triggered at the end point.
    const auto end_margin = 1.0;
    const auto return_remaining_distance =
      std::max(avoidance_data_.arclength_from_ego.back() - o.longitudinal - end_margin, 0.0);

    AvoidPoint ap_return;
    ap_return.length = 0.0;
    ap_return.start_length = shift_length;
    ap_return.start_longitudinal = o.longitudinal;
    ap_return.end_longitudinal =
      o.longitudinal + std::min(nominal_return_distance, return_remaining_distance);
    ap_return.id = getOriginalShiftPointUniqueId();
    ap_return.object = o;
    avoid_points.push_back(ap_return);

    DEBUG_PRINT(
      "object is set: avoid_shift = %f, return_shift = %f, dist = (avoidStart: %3.3f, avoidEnd: "
      "%3.3f, returnEnd: %3.3f), avoiding_dist = (nom:%f, res:%f), avoid_margin = %f, return_dist "
      "= %f",
      avoiding_shift, return_shift, ap_avoid.start_longitudinal, ap_avoid.end_longitudinal,
      ap_return.end_longitudinal, nominal_avoid_distance, avoiding_distance, avoid_margin,
      nominal_return_distance);
  }

  fillAdditionalInfoFromLongitudinal(avoid_points);

  return avoid_points;
}

AvoidPointArray AvoidanceModule::fillAdditionalInfo(const AvoidPointArray & shift_points) const
{
  if (shift_points.empty()) {
    return shift_points;
  }

  auto out_points = shift_points;

  const auto & path = avoidance_data_.reference_path;
  const auto arclength = avoidance_data_.arclength_from_ego;

  // calc longitudinal
  for (auto & sp : out_points) {
    sp.start_idx = findNearestIndex(path.points, sp.start.position);
    sp.start_longitudinal = arclength.at(sp.start_idx);
    sp.end_idx = findNearestIndex(path.points, sp.end.position);
    sp.end_longitudinal = arclength.at(sp.end_idx);
  }

  // sort by longitudinal
  std::sort(out_points.begin(), out_points.end(), [](auto a, auto b) {
    return a.end_longitudinal < b.end_longitudinal;
  });

  // calc relative lateral length
  out_points.front().start_length = getCurrentBaseShift();
  for (size_t i = 1; i < shift_points.size(); ++i) {
    out_points.at(i).start_length = shift_points.at(i - 1).length;
  }

  return out_points;
}
AvoidPoint AvoidanceModule::fillAdditionalInfo(const AvoidPoint & shift_point) const
{
  const auto ret = fillAdditionalInfo(AvoidPointArray{shift_point});
  return ret.front();
}

void AvoidanceModule::fillAdditionalInfoFromPoint(AvoidPointArray & shift_points) const
{
  if (shift_points.empty()) {
    return;
  }

  const auto & path = avoidance_data_.reference_path;
  const auto arclength = util::calcPathArcLengthArray(path);
  const auto dist_path_front_to_ego =
    calcSignedArcLength(path.points, 0, avoidance_data_.ego_closest_path_index);

  // calc longitudinal
  for (auto & sp : shift_points) {
    sp.start_idx = findNearestIndex(path.points, sp.start.position);
    sp.start_longitudinal = arclength.at(sp.start_idx) - dist_path_front_to_ego;
    sp.end_idx = findNearestIndex(path.points, sp.end.position);
    sp.end_longitudinal = arclength.at(sp.end_idx) - dist_path_front_to_ego;
  }
}

void AvoidanceModule::fillAdditionalInfoFromLongitudinal(AvoidPointArray & shift_points) const
{
  const auto & path = avoidance_data_.reference_path;
  const auto arclength = util::calcPathArcLengthArray(path);
  const auto path_front_to_ego =
    calcSignedArcLength(path.points, 0, avoidance_data_.ego_closest_path_index);

  for (auto & sp : shift_points) {
    sp.start_idx = findPathIndexFromArclength(arclength, sp.start_longitudinal + path_front_to_ego);
    sp.start = path.points.at(sp.start_idx).point.pose;
    sp.end_idx = findPathIndexFromArclength(arclength, sp.end_longitudinal + path_front_to_ego);
    sp.end = path.points.at(sp.end_idx).point.pose;
  }
}
/*
 * combineRawShiftPointsWithUniqueCheck
 *
 * Combine points A into B. If shift_point of A which has same object_id and
 * similar shape is already in B, it will not be added into B.
 */
AvoidPointArray AvoidanceModule::combineRawShiftPointsWithUniqueCheck(
  const AvoidPointArray & base_points, const AvoidPointArray & added_points) const
{
  // TODO(Horibe) parametrize
  const auto isSimilar = [](const AvoidPoint & a, const AvoidPoint & b) {
    using tier4_autoware_utils::calcDistance2d;
    if (calcDistance2d(a.start, b.start) > 1.0) {
      return false;
    }
    if (calcDistance2d(a.end, b.end) > 1.0) {
      return false;
    }
    if (std::abs(a.length - b.length) > 0.5) {
      return false;
    }
    return true;
  };
  const auto hasSameObjectId = [](const auto & a, const auto & b) {
    return a.object.object.object_id == b.object.object.object_id;
  };

  auto combined = base_points;  // initialized
  for (const auto & o : added_points) {
    bool skip = false;

    for (const auto & b : base_points) {
      if (hasSameObjectId(o, b) && isSimilar(o, b)) {
        skip = true;
        break;
      }
    }
    if (!skip) {
      combined.push_back(o);
    }
  }

  return combined;
}

void AvoidanceModule::generateTotalShiftLine(
  const AvoidPointArray & avoid_points, ShiftLineData & shift_line_data) const
{
  const auto & path = avoidance_data_.reference_path;
  const auto & arclengths = avoidance_data_.arclength_from_ego;
  const auto N = path.points.size();

  auto & sl = shift_line_data;

  sl.shift_line = std::vector<double>(N, 0.0);
  sl.shift_line_grad = std::vector<double>(N, 0.0);

  sl.pos_shift_line = std::vector<double>(N, 0.0);
  sl.neg_shift_line = std::vector<double>(N, 0.0);

  sl.pos_shift_line_grad = std::vector<double>(N, 0.0);
  sl.neg_shift_line_grad = std::vector<double>(N, 0.0);

  // debug
  sl.shift_line_history = std::vector<std::vector<double>>(avoid_points.size(), sl.shift_line);

  // take minmax for same directional shift length
  for (size_t j = 0; j < avoid_points.size(); ++j) {
    const auto & ap = avoid_points.at(j);
    for (size_t i = 0; i < N; ++i) {
      // calc current interpolated shift
      const auto i_shift = lerpShiftLengthOnArc(arclengths.at(i), ap);

      // update maximum shift for positive direction
      if (i_shift > sl.pos_shift_line.at(i)) {
        sl.pos_shift_line.at(i) = i_shift;
        sl.pos_shift_line_grad.at(i) = ap.getGradient();
      }

      // update minumum shift for negative direction
      if (i_shift < sl.neg_shift_line.at(i)) {
        sl.neg_shift_line.at(i) = i_shift;
        sl.neg_shift_line_grad.at(i) = ap.getGradient();
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
  const auto current_shift = getCurrentLinearShift();
  for (size_t i = 0; i <= avoidance_data_.ego_closest_path_index; ++i) {
    sl.shift_line.at(i) = current_shift;
    sl.shift_line_grad.at(i) = 0.0;
  }

  // If the shift point does not have an associated object,
  // use previous value.
  for (size_t i = 1; i < N; ++i) {
    bool has_object = false;
    for (const auto & ap : avoid_points) {
      if (ap.start_idx < i && i < ap.end_idx) {
        has_object = true;
        break;
      }
    }
    if (!has_object) {
      sl.shift_line.at(i) = sl.shift_line.at(i - 1);
    }
  }
  sl.shift_line_history.push_back(sl.shift_line);
}

AvoidPointArray AvoidanceModule::extractShiftPointsFromLine(ShiftLineData & shift_line_data) const
{
  const auto & path = avoidance_data_.reference_path;
  const auto & arclengths = avoidance_data_.arclength_from_ego;
  const auto N = path.points.size();

  auto & sl = shift_line_data;

  const auto getBwdGrad = [&](const size_t i) {
    if (i == 0) {
      return sl.shift_line_grad.at(i);
    }
    const double ds = arclengths.at(i) - arclengths.at(i - 1);
    if (ds < 1.0e-5) {
      return sl.shift_line_grad.at(i);
    }  // use theoretical value when ds is too small.
    return (sl.shift_line.at(i) - sl.shift_line.at(i - 1)) / ds;
  };

  const auto getFwdGrad = [&](const size_t i) {
    if (i == arclengths.size() - 1) {
      return sl.shift_line_grad.at(i);
    }
    const double ds = arclengths.at(i + 1) - arclengths.at(i);
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
    sl.forward_grad.at(i) = getFwdGrad(i);
    sl.backward_grad.at(i) = getBwdGrad(i);
  }

  AvoidPointArray merged_avoid_points;
  AvoidPoint ap{};
  bool found_first_start = false;
  constexpr auto CREATE_SHIFT_GRAD_THR = 0.001;
  constexpr auto IS_ALREADY_SHIFTING_THR = 0.001;
  for (size_t i = avoidance_data_.ego_closest_path_index; i < N - 1; ++i) {
    const auto & p = path.points.at(i).point.pose;
    const auto shift = sl.shift_line.at(i);

    // If the vehicle is already on the avoidance (checked by the first point has shift),
    // set a start point at the first path point.
    if (!found_first_start && std::abs(shift) > IS_ALREADY_SHIFTING_THR) {
      setStartData(ap, 0.0, p, i, arclengths.at(i));  // start length is overwritten later.
      found_first_start = true;
      DEBUG_PRINT("shift (= %f) is not zero at i = %lu. set start shift here.", shift, i);
    }

    // find the point where the gradient of the shift is changed
    const bool set_shift_point_flag =
      std::abs(sl.forward_grad.at(i) - sl.backward_grad.at(i)) > CREATE_SHIFT_GRAD_THR;

    if (!set_shift_point_flag) {
      continue;
    }

    if (!found_first_start) {
      setStartData(ap, 0.0, p, i, arclengths.at(i));  // start length is overwritten later.
      found_first_start = true;
      DEBUG_PRINT("grad change detected. start at i = %lu", i);
    } else {
      setEndData(ap, shift, p, i, arclengths.at(i));
      ap.id = getOriginalShiftPointUniqueId();
      merged_avoid_points.push_back(ap);
      setStartData(ap, 0.0, p, i, arclengths.at(i));  // start length is overwritten later.
      DEBUG_PRINT("end and start point found at i = %lu", i);
    }
  }
  return merged_avoid_points;
}

AvoidPointArray AvoidanceModule::mergeShiftPoints(
  const AvoidPointArray & raw_shift_points, DebugData & debug) const
{
  // Generate shift line by merging raw_shift_points.
  ShiftLineData shift_line_data;
  generateTotalShiftLine(raw_shift_points, shift_line_data);

  // Re-generate shift points by detecting gradient-change point of the shift line.
  auto merged_shift_points = extractShiftPointsFromLine(shift_line_data);

  // set parent id
  for (auto & ap : merged_shift_points) {
    ap.parent_ids = calcParentIds(raw_shift_points, ap);
  }

  // sort by distance from ego.
  alignShiftPointsOrder(merged_shift_points);

  // debug visualize
  {
    debug.pos_shift = shift_line_data.pos_shift_line;
    debug.neg_shift = shift_line_data.neg_shift_line;
    debug.total_shift = shift_line_data.shift_line;
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
       << closest << ", raw_shift_points size = " << raw_shift_points.size() << std::endl;
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

  printShiftPoints(merged_shift_points, "merged_shift_points");

  return merged_shift_points;
}

std::vector<size_t> AvoidanceModule::calcParentIds(
  const AvoidPointArray & parent_candidates, const AvoidPoint & child) const
{
  // Get the ID of the original AP whose transition area overlaps with the given AP,
  // and set it to the parent id.
  std::set<uint64_t> ids;
  for (const auto & ap : parent_candidates) {
    const auto p_s = ap.start_longitudinal;
    const auto p_e = ap.end_longitudinal;
    const auto has_overlap = !(p_e < child.start_longitudinal || child.end_longitudinal < p_s);

    if (!has_overlap) {
      continue;
    }

    // Id the shift is overlapped, insert the shift point. Additionally, the shift which refers
    // to the same object id (created by the same object) will be set.
    //
    // Why? : think that there are two shifts, avoiding and .
    // If you register only the avoiding shift, the return-to-center shift will not be generated
    // when you get too close to or over the obstacle. The return-shift can be handled with
    // addReturnShift(), but it maybe reasonable to register the return-to-center shift for the
    // object at the same time as registering the avoidance shift to remove the complexity of the
    // addReturnShift().
    for (const auto & ap_local : parent_candidates) {
      if (ap_local.object.object.object_id == ap.object.object.object_id) {
        ids.insert(ap_local.id);
      }
    }
  }
  return std::vector<size_t>(ids.begin(), ids.end());
}

/*
 * Remove unnecessary avoid points
 * - Combine avoid points that have almost same gradient
 * - Quantize the shift length to reduce the shift point noise
 * - Change the shift length to the previous one if the deviation is small.
 * - Remove unnecessary return shift (back to the center line).
 */
AvoidPointArray AvoidanceModule::trimShiftPoint(
  const AvoidPointArray & shift_points, DebugData & debug) const
{
  if (shift_points.empty()) {
    return shift_points;
  }

  AvoidPointArray sp_array_trimmed = shift_points;

  // sort shift points from front to back.
  alignShiftPointsOrder(sp_array_trimmed);

  // - Combine avoid points that have almost same gradient.
  // this is to remove the noise.
  {
    const auto CHANGE_SHIFT_THRESHOLD_FOR_NOISE = 0.1;
    trimSimilarGradShiftPoint(sp_array_trimmed, CHANGE_SHIFT_THRESHOLD_FOR_NOISE);
    debug.trim_similar_grad_shift = sp_array_trimmed;
    printShiftPoints(sp_array_trimmed, "after trim_similar_grad_shift");
  }

  // - Quantize the shift length to reduce the shift point noise
  // This is to remove the noise coming from detection accuracy, interpolation, resampling, etc.
  {
    constexpr double QUANTIZATION_DISTANCE = 0.2;
    quantizeShiftPoint(sp_array_trimmed, QUANTIZATION_DISTANCE);
    printShiftPoints(sp_array_trimmed, "after sp_array_trimmed");
    debug.quantized = sp_array_trimmed;
  }

  // - Change the shift length to the previous one if the deviation is small.
  {
    // constexpr double SHIFT_DIFF_THRES = 0.5;
    // trimSmallShiftPoint(sp_array_trimmed, SHIFT_DIFF_THRES);
    debug.trim_small_shift = sp_array_trimmed;
    printShiftPoints(sp_array_trimmed, "after trim_small_shift");
  }

  // - Combine avoid points that have almost same gradient (again)
  {
    const auto CHANGE_SHIFT_THRESHOLD = 0.2;
    trimSimilarGradShiftPoint(sp_array_trimmed, CHANGE_SHIFT_THRESHOLD);
    debug.trim_similar_grad_shift_second = sp_array_trimmed;
    printShiftPoints(sp_array_trimmed, "after trim_similar_grad_shift_second");
  }

  // - trimTooSharpShift
  // Check if it is not too sharp for the return-to-center shift point.
  // If the shift is sharp, it is combined with the next shift point until it gets non-sharp.
  {
    trimSharpReturn(sp_array_trimmed);
    debug.trim_too_sharp_shift = sp_array_trimmed;
    printShiftPoints(sp_array_trimmed, "after trimSharpReturn");
  }

  return sp_array_trimmed;
}

void AvoidanceModule::alignShiftPointsOrder(
  AvoidPointArray & shift_points, const bool recalc_start_length) const
{
  if (shift_points.empty()) {
    return;
  }

  // sort shift points from front to back.
  std::sort(shift_points.begin(), shift_points.end(), [](auto a, auto b) {
    return a.end_longitudinal < b.end_longitudinal;
  });

  // calc relative length
  // NOTE: the input shift point must not have conflict range. Otherwise relative
  // length value will be broken.
  if (recalc_start_length) {
    shift_points.front().start_length = getCurrentLinearShift();
    for (size_t i = 1; i < shift_points.size(); ++i) {
      shift_points.at(i).start_length = shift_points.at(i - 1).length;
    }
  }
}

void AvoidanceModule::quantizeShiftPoint(
  AvoidPointArray & shift_points, const double interval) const
{
  if (interval < 1.0e-5) {
    return;  // no need to process
  }

  for (auto & sp : shift_points) {
    sp.length = std::round(sp.length / interval) * interval;
  }

  alignShiftPointsOrder(shift_points);
}

void AvoidanceModule::trimSmallShiftPoint(
  AvoidPointArray & shift_points, const double shift_diff_thres) const
{
  AvoidPointArray shift_points_orig = shift_points;
  shift_points.clear();

  shift_points.push_back(shift_points_orig.front());  // Take the first one anyway (think later)

  for (size_t i = 1; i < shift_points_orig.size(); ++i) {
    auto sp_now = shift_points_orig.at(i);
    const auto sp_prev = shift_points.back();
    const auto shift_diff = sp_now.length - sp_prev.length;

    auto sp_modified = sp_now;

    // remove the shift point if the length is almost same as the previous one.
    if (std::abs(shift_diff) < shift_diff_thres) {
      sp_modified.length = sp_prev.length;
      sp_modified.start_length = sp_prev.length;
      DEBUG_PRINT(
        "i = %lu, relative shift = %f is small. set with relative shift = 0.", i, shift_diff);
    } else {
      DEBUG_PRINT("i = %lu, shift = %f is large. take this one normally.", i, shift_diff);
    }

    shift_points.push_back(sp_modified);
  }

  alignShiftPointsOrder(shift_points);

  DEBUG_PRINT("size %lu -> %lu", shift_points_orig.size(), shift_points.size());
}

void AvoidanceModule::trimSimilarGradShiftPoint(
  AvoidPointArray & avoid_points, const double change_shift_dist_threshold) const
{
  AvoidPointArray avoid_points_orig = avoid_points;
  avoid_points.clear();

  avoid_points.push_back(avoid_points_orig.front());  // Take the first one anyway (think later)

  // Save the points being merged. When merging consecutively, also check previously merged points.
  AvoidPointArray being_merged_points;

  for (size_t i = 1; i < avoid_points_orig.size(); ++i) {
    const auto ap_now = avoid_points_orig.at(i);
    const auto ap_prev = avoid_points.back();

    being_merged_points.push_back(ap_prev);  // This point is about to be merged.

    auto combined_ap = ap_prev;
    setEndData(combined_ap, ap_now.length, ap_now.end, ap_now.end_idx, ap_now.end_longitudinal);
    combined_ap.parent_ids = concatParentIds(combined_ap.parent_ids, ap_prev.parent_ids);

    const auto has_large_length_change = [&]() {
      for (const auto & original : being_merged_points) {
        const auto longitudinal = original.end_longitudinal - combined_ap.start_longitudinal;
        const auto new_length = combined_ap.getGradient() * longitudinal + combined_ap.start_length;
        const bool has_large_change =
          std::abs(new_length - original.length) > change_shift_dist_threshold;

        DEBUG_PRINT(
          "original.length: %f, original.end_longitudinal: %f, combined_ap.start_longitudinal: "
          "%f, combined_ap.Gradient: %f, new_length: %f, has_large_change: %d",
          original.length, original.end_longitudinal, combined_ap.start_longitudinal,
          combined_ap.getGradient(), new_length, has_large_change);

        if (std::abs(new_length - original.length) > change_shift_dist_threshold) {
          return true;
        }
      }
      return false;
    }();

    if (has_large_length_change) {
      // If this point is merged with the previous points, it makes a large changes.
      // Do not merge this.
      avoid_points.push_back(ap_now);
      being_merged_points.clear();
      DEBUG_PRINT("use this point. has_large_length_change = %d", has_large_length_change);
    } else {
      avoid_points.back() = combined_ap;  // Update the last points by merging the current point
      being_merged_points.push_back(ap_prev);
      DEBUG_PRINT("trim! has_large_length_change = %d", has_large_length_change);
    }
  }

  alignShiftPointsOrder(avoid_points);

  DEBUG_PRINT("size %lu -> %lu", avoid_points_orig.size(), avoid_points.size());
}

/**
 * Remove short "return to center" shift point. ¯¯\_/¯¯　-> ¯¯¯¯¯¯
 *
 * Is the shift point for "return to center"?
 *  - no : Do not trim anything.
 *  - yes: Is it short distance enough to be removed?
 *     - no : Do not trim anything.
 *     - yes: Remove the "return" shift point.
 *            Recalculate longitudinal distance and modify the shift point.
 */
void AvoidanceModule::trimMomentaryReturn(AvoidPointArray & shift_points) const
{
  const auto isZero = [](double v) { return std::abs(v) < 1.0e-5; };

  AvoidPointArray shift_points_orig = shift_points;
  shift_points.clear();

  const double DISTANCE_AFTER_RETURN_THR = 5.0 * getNominalAvoidanceEgoSpeed();

  const auto & arclength = avoidance_data_.arclength_from_ego;

  const auto check_reduce_shift = [](const double now_length, const double prev_length) {
    const auto abs_shift_diff = std::abs(now_length) - std::abs(prev_length);
    const auto has_same_sign = (now_length * prev_length >= 0.0);
    const bool is_reduce_shift = (abs_shift_diff < 0.0 && has_same_sign);
    return is_reduce_shift;
  };

  for (size_t i = 0; i < shift_points_orig.size(); ++i) {
    const auto sp_now = shift_points_orig.at(i);
    const auto sp_prev_length =
      shift_points.empty() ? getCurrentLinearShift() : shift_points.back().length;
    const auto abs_shift_diff = std::abs(sp_now.length) - std::abs(sp_prev_length);
    const bool is_reduce_shift = check_reduce_shift(sp_now.length, sp_prev_length);

    // Do nothing for non-reduce shift point
    if (!is_reduce_shift) {
      shift_points.push_back(sp_now);
      DEBUG_PRINT(
        "i = %lu, not reduce shift. take this one.　abs_shift_diff = %f, sp_now.length = %f, "
        "sp_prev_length = %f, sp_now.length * sp_prev_length = %f",
        i, abs_shift_diff, sp_now.length, sp_prev_length, sp_now.length * sp_prev_length);
      continue;
    }

    // The last point is out of target of this function.
    const bool is_last_sp = (i == shift_points_orig.size() - 1);
    if (is_last_sp) {
      shift_points.push_back(sp_now);
      DEBUG_PRINT("i = %lu, last shift. take this one.", i);
      continue;
    }

    // --- From here, the shift point is "return to center" or "straight". ---
    // -----------------------------------------------------------------------

    const auto sp_next = shift_points_orig.at(i + 1);

    // there is no straight interval, combine them. ¯¯\/¯¯ -> ¯¯¯¯¯¯
    if (!isZero(sp_next.getRelativeLength())) {
      DEBUG_PRINT(
        "i = %lu, return-shift is detected, next shift_diff (%f) is nonzero. combine them. (skip "
        "next shift).",
        i, sp_next.getRelativeLength());
      auto sp_modified = sp_next;
      setStartData(
        sp_modified, sp_now.length, sp_now.start, sp_now.start_idx, sp_now.start_longitudinal);
      sp_modified.parent_ids = concatParentIds(sp_modified.parent_ids, sp_now.parent_ids);
      shift_points.push_back(sp_modified);
      ++i;  // skip next shift point
      continue;
    }

    // Find next shifting point, i.e.  ¯¯\____"/"¯¯
    //                               now ↑     ↑ target
    const auto next_avoid_idx = [&]() {
      for (size_t j = i + 1; j < shift_points_orig.size(); ++j) {
        if (!isZero(shift_points_orig.at(j).getRelativeLength())) {
          return j;
        }
      }
      return shift_points_orig.size();
    }();

    // The straight distance lasts until end. take this one.
    // ¯¯\______
    if (next_avoid_idx == shift_points_orig.size()) {
      shift_points.push_back(sp_now);
      DEBUG_PRINT("i = %lu, back -> straight lasts until end. take this one.", i);
      continue;
    }

    const auto sp_next_avoid = shift_points_orig.at(next_avoid_idx);
    const auto straight_distance = sp_next_avoid.start_longitudinal - sp_now.end_longitudinal;

    // The straight distance after "return to center" is long enough. take this one.
    // ¯¯\______/¯¯ (enough long straight line!)
    if (straight_distance > DISTANCE_AFTER_RETURN_THR) {
      shift_points.push_back(sp_now);
      DEBUG_PRINT("i = %lu, back -> straight: distance is long. take this one", i);
      continue;
    }

    // From here, back to center and go straight, straight distance is too short.
    // ¯¯\______/¯¯ (short straight line!)

    const auto relative_shift = sp_next_avoid.length - sp_now.length;
    const auto avoid_distance = getNominalAvoidanceDistance(relative_shift);

    // Calculate start point from end point and avoidance distance.
    auto sp_next_modified = sp_next_avoid;
    sp_next_modified.start_length = sp_prev_length;
    sp_next_modified.start_longitudinal =
      std::max(sp_next_avoid.end_longitudinal - avoid_distance, sp_now.start_longitudinal);
    sp_next_modified.start_idx =
      findPathIndexFromArclength(arclength, sp_next_modified.start_longitudinal);
    sp_next_modified.start =
      avoidance_data_.reference_path.points.at(sp_next_modified.start_idx).point.pose;
    sp_next_modified.parent_ids = calcParentIds(current_raw_shift_points_, sp_next_modified);

    // Straight shift point
    if (sp_next_modified.start_idx > sp_now.start_idx) {  // the case where a straight route exists.
      auto sp_now_modified = sp_now;
      sp_now_modified.start_length = sp_prev_length;
      setEndData(
        sp_now_modified, sp_prev_length, sp_next_modified.start, sp_next_modified.start_idx,
        sp_next_modified.start_longitudinal);
      sp_now_modified.parent_ids = calcParentIds(current_raw_shift_points_, sp_now_modified);
      shift_points.push_back(sp_now_modified);
    }
    shift_points.push_back(sp_next_modified);

    DEBUG_PRINT(
      "i = %lu, find remove target!: next_avoid_idx = %lu, shift length = (now: %f, prev: %f, "
      "next_avoid: %f, next_mod: %f).",
      i, next_avoid_idx, sp_now.length, sp_prev_length, sp_next_avoid.length,
      sp_next_modified.length);

    i = next_avoid_idx;  // skip shifting until next_avoid_idx.
  }

  alignShiftPointsOrder(shift_points);

  DEBUG_PRINT(
    "trimMomentaryReturn: size %lu -> %lu", shift_points_orig.size(), shift_points.size());
}

void AvoidanceModule::trimSharpReturn(AvoidPointArray & shift_points) const
{
  AvoidPointArray shift_points_orig = shift_points;
  shift_points.clear();

  const auto isZero = [](double v) { return std::abs(v) < 0.01; };

  // check if the shift point is positive (avoiding) shift
  const auto isPositive = [&](const auto & sp) {
    constexpr auto POSITIVE_SHIFT_THR = 0.1;
    return std::abs(sp.length) - std::abs(sp.start_length) > POSITIVE_SHIFT_THR;
  };

  // check if the shift point is negative (returning) shift
  const auto isNegative = [&](const auto & sp) {
    constexpr auto NEGATIVE_SHIFT_THR = -0.1;
    return std::abs(sp.length) - std::abs(sp.start_length) < NEGATIVE_SHIFT_THR;
  };

  // combine two shift points. Be careful the order of "now" and "next".
  const auto combineShiftPoint = [this](const auto & sp_next, const auto & sp_now) {
    auto sp_modified = sp_now;
    setEndData(sp_modified, sp_next.length, sp_next.end, sp_next.end_idx, sp_next.end_longitudinal);
    sp_modified.parent_ids = concatParentIds(sp_modified.parent_ids, sp_now.parent_ids);
    return sp_modified;
  };

  // Check if the merged shift has a conflict with the original shifts.
  const auto hasViolation = [this](const auto & combined, const auto & combined_src) {
    constexpr auto VIOLATION_SHIFT_THR = 0.3;
    for (const auto & sp : combined_src) {
      const auto combined_shift = lerpShiftLengthOnArc(sp.end_longitudinal, combined);
      if (sp.length < -0.01 && combined_shift > sp.length + VIOLATION_SHIFT_THR) {
        return true;
      }
      if (sp.length > 0.01 && combined_shift < sp.length - VIOLATION_SHIFT_THR) {
        return true;
      }
    }
    return false;
  };

  // check for all shift points
  for (size_t i = 0; i < shift_points_orig.size(); ++i) {
    auto sp_now = shift_points_orig.at(i);
    sp_now.start_length =
      shift_points.empty() ? getCurrentLinearShift() : shift_points.back().length;

    if (sp_now.length * sp_now.start_length < -0.01) {
      DEBUG_PRINT("i = %lu, This is avoid shift for opposite direction. take this one", i);
      continue;
    }

    // Do nothing for non-reduce shift point
    if (!isNegative(sp_now)) {
      shift_points.push_back(sp_now);
      DEBUG_PRINT(
        "i = %lu, positive shift. take this one. sp_now.length * sp_now.start_length = %f", i,
        sp_now.length * sp_now.start_length);
      continue;
    }

    // The last point is out of target of this function.
    if (i == shift_points_orig.size() - 1) {
      shift_points.push_back(sp_now);
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
      auto sp_combined = sp_now;
      auto sp_combined_prev = sp_combined;
      AvoidPointArray sp_combined_array{sp_now};
      size_t j = i + 1;
      for (; i < shift_points_orig.size(); ++j) {
        const auto sp_combined = combineShiftPoint(shift_points_orig.at(j), sp_now);

        {
          std::stringstream ss;
          ss << "i = " << i << ", j = " << j << ": sp_combined = " << toStrInfo(sp_combined);
          DEBUG_PRINT("%s", ss.str().c_str());
        }

        // it gets positive. Finish merging.
        if (isPositive(sp_combined)) {
          shift_points.push_back(sp_combined);
          DEBUG_PRINT("reach positive.");
          break;
        }

        // Still negative, but it violates the original shift points.
        // Finish with the previous merge result.
        if (hasViolation(sp_combined, sp_combined_array)) {
          shift_points.push_back(sp_combined_prev);
          DEBUG_PRINT("violation found.");
          --j;
          break;
        }

        // Still negative, but it has an enough long distance. Finish merging.
        const auto nominal_distance = getNominalAvoidanceDistance(sp_combined.getRelativeLength());
        const auto long_distance =
          isZero(sp_combined.length) ? nominal_distance : nominal_distance * 5.0;
        if (sp_combined.getRelativeLongitudinal() > long_distance) {
          shift_points.push_back(sp_combined);
          DEBUG_PRINT("still negative, but long enough. Threshold = %f", long_distance);
          break;
        }

        // It reaches the last point. Still the shift is sharp, but merge with the current result.
        if (j == shift_points_orig.size() - 1) {
          shift_points.push_back(sp_combined);
          DEBUG_PRINT("reach end point.");
          break;
        }

        // Still negative shift, and the distance is not enough. Search next.
        sp_combined_prev = sp_combined;
        sp_combined_array.push_back(shift_points_orig.at(j));
      }
      i = j;
      continue;
    }
  }

  alignShiftPointsOrder(shift_points);

  DEBUG_PRINT("trimSharpReturn: size %lu -> %lu", shift_points_orig.size(), shift_points.size());
}

void AvoidanceModule::trimTooSharpShift(AvoidPointArray & avoid_points) const
{
  if (avoid_points.empty()) {
    return;
  }

  AvoidPointArray avoid_points_orig = avoid_points;
  avoid_points.clear();

  const auto isInJerkLimit = [this](const auto & ap) {
    const auto required_jerk = path_shifter_.calcJerkFromLatLonDistance(
      ap.getRelativeLength(), ap.getRelativeLongitudinal(), getSharpAvoidanceEgoSpeed());
    return std::fabs(required_jerk) < parameters_.max_lateral_jerk;
  };

  for (size_t i = 0; i < avoid_points_orig.size(); ++i) {
    auto ap_now = avoid_points_orig.at(i);

    if (isInJerkLimit(ap_now)) {
      avoid_points.push_back(ap_now);
      continue;
    }

    DEBUG_PRINT("over jerk is detected: i = %lu", i);
    printShiftPoints(AvoidPointArray{ap_now}, "points with over jerk");

    // The avoidance_point_now exceeds jerk limit, so merge it with the next avoidance_point.
    for (size_t j = i + 1; j < avoid_points_orig.size(); ++j) {
      auto ap_next = avoid_points_orig.at(j);
      setEndData(ap_now, ap_next.length, ap_next.end, ap_next.end_idx, ap_next.end_longitudinal);
      if (isInJerkLimit(ap_now)) {
        avoid_points.push_back(ap_now);
        DEBUG_PRINT("merge finished. i = %lu, j = %lu", i, j);
        i = j;  // skip check until j index.
        break;
      }
    }
  }

  alignShiftPointsOrder(avoid_points);

  DEBUG_PRINT("size %lu -> %lu", avoid_points_orig.size(), avoid_points.size());
}

/*
 * addReturnShiftPoint
 *
 * Pick up the last shift point, which is the most farthest from ego, from the current candidate
 * avoidance points and registered points in the shifter. If the last shift length of the point is
 * non-zero, add a return-shift to center line from the point. If there is no shift point in
 * candidate avoidance points nor registered points, and base_shift > 0, add a return-shift to
 * center line from ego.
 */
void AvoidanceModule::addReturnShiftPointFromEgo(
  AvoidPointArray & sp_candidates, AvoidPointArray & current_raw_shift_points) const
{
  constexpr double ep = 1.0e-3;
  const bool has_candidate_point = !sp_candidates.empty();
  const bool has_registered_point = !path_shifter_.getShiftPoints().empty();

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
  ShiftPoint last_sp;  // the return-shift will be generated after the last shift point.
  {
    // avoidance points: Yes, shift points: No -> select last avoidance point.
    if (has_candidate_point && !has_registered_point) {
      alignShiftPointsOrder(sp_candidates, false);
      last_sp = sp_candidates.back();
    }

    // avoidance points: No, shift points: Yes -> select last shift point.
    if (!has_candidate_point && has_registered_point) {
      last_sp = fillAdditionalInfo(AvoidPoint{path_shifter_.getLastShiftPoint().get()});
    }

    // avoidance points: Yes, shift points: Yes -> select the last one from both.
    if (has_candidate_point && has_registered_point) {
      alignShiftPointsOrder(sp_candidates, false);
      const auto & ap = sp_candidates.back();
      const auto & sp = fillAdditionalInfo(AvoidPoint{path_shifter_.getLastShiftPoint().get()});
      last_sp = (sp.end_longitudinal > ap.end_longitudinal) ? sp : ap;
    }

    // avoidance points: No, shift points: No -> set the ego position to the last shift point
    // so that the return-shift will be generated from ego position.
    if (!has_candidate_point && !has_registered_point) {
      last_sp.end = getEgoPose().pose;
      last_sp.end_idx = avoidance_data_.ego_closest_path_index;
      last_sp.length = getCurrentBaseShift();
    }
  }
  printShiftPoints(ShiftPointArray{last_sp}, "last shift point");

  // There already is a shift point candidates to go back to center line, but it could be too sharp
  // due to detection noise or timing.
  // Here the return-shift from ego is added for the in case.
  if (std::fabs(last_sp.length) < RETURN_SHIFT_THRESHOLD) {
    const auto current_base_shift = getCurrentShift();
    if (std::abs(current_base_shift) < ep) {
      DEBUG_PRINT("last shift almost is zero, and current base_shift is zero. do nothing.");
      return;
    }

    // Is there a shift point in the opposite direction of the current_base_shift?
    //   No  -> we can overwrite the return shift, because the other shift points that decrease
    //          the shift length are for return-shift.
    //   Yes -> we can NOT overwrite, because it might be not a return-shift, but a avoiding
    //          shift to the opposite direction which can not be overwritten by the return-shift.
    for (const auto & sp : sp_candidates) {
      if (
        (current_base_shift > 0.0 && sp.length < -ep) ||
        (current_base_shift < 0.0 && sp.length > ep)) {
        DEBUG_PRINT(
          "try to put overwrite return shift, but there is shift for opposite direction. Skip "
          "adding return shift.");
        return;
      }
    }

    // set the return-shift from ego.
    DEBUG_PRINT(
      "return shift already exists, but they are all candidates. Add return shift for overwrite.");
    last_sp.end = getEgoPose().pose;
    last_sp.end_idx = avoidance_data_.ego_closest_path_index;
    last_sp.length = current_base_shift;
  }

  const auto & arclength_from_ego = avoidance_data_.arclength_from_ego;

  const auto nominal_prepare_distance = getNominalPrepareDistance();
  const auto nominal_avoid_distance = getNominalAvoidanceDistance(last_sp.length);

  if (arclength_from_ego.empty()) {
    return;
  }

  const auto remaining_distance = arclength_from_ego.back();

  // If the avoidance point has already been set, the return shift must be set after the point.
  const auto last_sp_distance = avoidance_data_.arclength_from_ego.at(last_sp.end_idx);

  // check if there is enough distance for return.
  if (last_sp_distance + 1.0 > remaining_distance) {  // tmp: add some small number (+1.0)
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
  //  ego              last_sp_end             prepare_end            path_end    avoid_end
  // ==o====================o----------------------o----------------------o------------o
  //   |            prepare_dist                   |          avoid_dist               |
  //
  // [After Scaling]
  // ==o====================o------------------o--------------------------o
  //   |        prepare_dist_scaled            |    avoid_dist_scaled     |
  //
  const double variable_prepare_distance =
    std::max(nominal_prepare_distance - last_sp_distance, 0.0);

  double prepare_distance_scaled = std::max(nominal_prepare_distance, last_sp_distance);
  double avoid_distance_scaled = nominal_avoid_distance;
  if (remaining_distance < prepare_distance_scaled + avoid_distance_scaled) {
    const auto scale = (remaining_distance - last_sp_distance) /
                       std::max(nominal_avoid_distance + variable_prepare_distance, 0.1);
    prepare_distance_scaled = last_sp_distance + scale * nominal_prepare_distance;
    avoid_distance_scaled *= scale;
    DEBUG_PRINT(
      "last_sp_distance = %f, nominal_prepare_distance = %f, nominal_avoid_distance = %f, "
      "remaining_distance = %f, variable_prepare_distance = %f, scale = %f, "
      "prepare_distance_scaled = %f,avoid_distance_scaled = %f",
      last_sp_distance, nominal_prepare_distance, nominal_avoid_distance, remaining_distance,
      variable_prepare_distance, scale, prepare_distance_scaled, avoid_distance_scaled);
  } else {
    DEBUG_PRINT("there is enough distance. Use nominal for prepare & avoidance.");
  }

  // shift point for prepare distance: from last shift to return-start point.
  if (nominal_prepare_distance > last_sp_distance) {
    AvoidPoint ap;
    ap.id = getOriginalShiftPointUniqueId();
    ap.start_idx = last_sp.end_idx;
    ap.start = last_sp.end;
    ap.start_longitudinal = arclength_from_ego.at(ap.start_idx);
    ap.end_idx = findPathIndexFromArclength(arclength_from_ego, prepare_distance_scaled);
    ap.end = avoidance_data_.reference_path.points.at(ap.end_idx).point.pose;
    ap.end_longitudinal = arclength_from_ego.at(ap.end_idx);
    ap.length = last_sp.length;
    ap.start_length = last_sp.length;
    sp_candidates.push_back(ap);
    printShiftPoints(AvoidPointArray{ap}, "prepare for return");
    debug_data_.extra_return_shift.push_back(ap);

    // TODO(Horibe) think how to store the current object
    current_raw_shift_points.push_back(ap);
  }

  // shift point for return to center line
  {
    AvoidPoint ap;
    ap.id = getOriginalShiftPointUniqueId();
    ap.start_idx = findPathIndexFromArclength(arclength_from_ego, prepare_distance_scaled);
    ap.start = avoidance_data_.reference_path.points.at(ap.start_idx).point.pose;
    ap.start_longitudinal = arclength_from_ego.at(ap.start_idx);
    ap.end_idx = findPathIndexFromArclength(
      arclength_from_ego, prepare_distance_scaled + avoid_distance_scaled);
    ap.end = avoidance_data_.reference_path.points.at(ap.end_idx).point.pose;
    ap.end_longitudinal = arclength_from_ego.at(ap.end_idx);
    ap.length = 0.0;
    ap.start_length = last_sp.length;
    sp_candidates.push_back(ap);
    printShiftPoints(AvoidPointArray{ap}, "return point");
    debug_data_.extra_return_shift = AvoidPointArray{ap};

    // TODO(Horibe) think how to store the current object
    current_raw_shift_points.push_back(ap);
  }

  DEBUG_PRINT("Return Shift is added.");
}

double AvoidanceModule::getRightShiftBound() const
{
  // TODO(Horibe) write me. Real lane boundary must be considered here.
  return -parameters_.max_right_shift_length;
}

double AvoidanceModule::getLeftShiftBound() const
{
  // TODO(Horibe) write me. Real lane boundary must be considered here.
  return parameters_.max_left_shift_length;
}

void AvoidanceModule::generateExtendedDrivableArea(ShiftedPath * shifted_path, double margin) const
{
  const auto right_extend_elem =
    std::min_element(shifted_path->shift_length.begin(), shifted_path->shift_length.end());
  const auto left_extend_elem =
    std::max_element(shifted_path->shift_length.begin(), shifted_path->shift_length.end());

  double right_extend = std::min(*right_extend_elem, 0.0);
  double left_extend = std::max(*left_extend_elem, 0.0);

  constexpr double THRESHOLD = 0.01;
  right_extend -= (right_extend < -THRESHOLD) ? margin : 0.0;
  left_extend += (left_extend > THRESHOLD) ? margin : 0.0;

  const auto extended_lanelets = lanelet::utils::getExpandedLanelets(
    avoidance_data_.current_lanelets, left_extend, right_extend);

  {
    const auto & p = planner_data_->parameters;
    shifted_path->path.drivable_area = util::generateDrivableArea(
      extended_lanelets, getEgoPose(), p.drivable_area_width, p.drivable_area_height,
      p.drivable_area_resolution, p.vehicle_length, *(planner_data_->route_handler));
  }
}

void AvoidanceModule::modifyPathVelocityToPreventAccelerationOnAvoidance(ShiftedPath & path) const
{
  const auto ego_idx = avoidance_data_.ego_closest_path_index;
  const auto N = path.shift_length.size();

  // find first shift-change point from ego
  constexpr auto SHIFT_DIFF_THR = 0.1;
  size_t target_idx = N;
  const auto current_shift = path.shift_length.at(ego_idx);
  for (size_t i = ego_idx + 1; i < N; ++i) {
    if (std::abs(path.shift_length.at(i) - current_shift) > SHIFT_DIFF_THR) {
      // this index do not have to be accurate, so it can be i or i + 1.
      // but if the ego point is already on the shift-change point, ego index should be a target_idx
      // so that the distance for acceleration will be 0 and the ego speed is directly applied
      // to the path velocity (no acceleration while avoidance)
      target_idx = i - 1;
      break;
    }
  }
  if (target_idx == N) {
    DEBUG_PRINT("shift length has no changes. No velocity limit is applied.");
    return;
  }

  // calc time to the shift-change point
  constexpr auto NO_ACCEL_TIME_THR = 3.0;
  const auto s = avoidance_data_.arclength_from_ego.at(target_idx) -
                 avoidance_data_.arclength_from_ego.at(ego_idx);
  const auto t = s / std::max(getEgoSpeed(), 1.0);
  if (t > NO_ACCEL_TIME_THR) {
    DEBUG_PRINT(
      "shift point is far (s: %f, t: %f, ego_i: %lu, target_i: %lu). No velocity limit is applied.",
      s, t, ego_idx, target_idx);
    return;
  }

  // calc max velocity with given acceleration
  const auto v0 = getEgoSpeed();
  const auto vmax = std::max(
    parameters_.min_avoidance_speed_for_acc_prevention,
    std::sqrt(v0 * v0 + 2.0 * s * parameters_.max_avoidance_acceleration));

  // apply velocity limit
  constexpr size_t VLIM_APPLY_IDX_MARGIN = 0;
  for (size_t i = ego_idx + VLIM_APPLY_IDX_MARGIN; i < N; ++i) {
    path.path.points.at(i).point.longitudinal_velocity_mps =
      std::min(path.path.points.at(i).point.longitudinal_velocity_mps, static_cast<float>(vmax));
  }

  DEBUG_PRINT(
    "s: %f, t: %f, v0: %f, a: %f, vmax: %f, ego_i: %lu, target_i: %lu", s, t, v0,
    parameters_.max_avoidance_acceleration, vmax, ego_idx, target_idx);
}

// TODO(Horibe) clean up functions: there is a similar code in util as well.
PathWithLaneId AvoidanceModule::calcCenterLinePath(
  const std::shared_ptr<const PlannerData> & planner_data, const PoseStamped & pose) const
{
  const auto & p = planner_data->parameters;
  const auto & route_handler = planner_data->route_handler;

  PathWithLaneId centerline_path;

  // special for avoidance: take behind distance upt ot shift-start-point if it exist.
  const auto longest_dist_to_shift_point = [&]() {
    double max_dist = 0.0;
    for (const auto & pnt : path_shifter_.getShiftPoints()) {
      max_dist = std::max(max_dist, calcDistance2d(getEgoPose(), pnt.start));
    }
    for (const auto & sp : registered_raw_shift_points_) {
      max_dist = std::max(max_dist, calcDistance2d(getEgoPose(), sp.start));
    }
    return max_dist;
  }();

  printShiftPoints(path_shifter_.getShiftPoints(), "path_shifter_.getShiftPoints()");
  printShiftPoints(registered_raw_shift_points_, "registered_raw_shift_points_");

  const auto extra_margin = 10.0;  // Since distance does not consider arclength, but just line.
  const auto backward_length =
    std::max(p.backward_path_length, longest_dist_to_shift_point + extra_margin);

  DEBUG_PRINT(
    "p.backward_path_length = %f, longest_dist_to_shift_point = %f, backward_length = %f",
    p.backward_path_length, longest_dist_to_shift_point, backward_length);

  const lanelet::ConstLanelets current_lanes =
    calcLaneAroundPose(planner_data, pose.pose, backward_length);
  centerline_path = util::getCenterLinePath(
    *route_handler, current_lanes, pose.pose, backward_length, p.forward_path_length, p);

  // for debug: check if the path backward distance is same as the desired length.
  // {
  //   const auto back_to_ego = tier4_autoware_utils::calcSignedArcLength(
  //     centerline_path.points, centerline_path.points.front().point.pose.position,
  //     getEgoPosition());
  //   RCLCPP_INFO(getLogger(), "actual back_to_ego distance = %f", back_to_ego);
  // }

  centerline_path.header = route_handler->getRouteHeader();

  return centerline_path;
}

boost::optional<AvoidPoint> AvoidanceModule::calcIntersectionShiftPoint(
  const AvoidancePlanningData & data) const
{
  boost::optional<PathPointWithLaneId> intersection_point{};
  for (const auto & p : avoidance_data_.reference_path.points) {
    for (const auto & id : p.lane_ids) {
      const lanelet::ConstLanelet ll = planner_data_->route_handler->getLaneletsFromId(id);
      std::string turn_direction = ll.attributeOr("turn_direction", "else");
      if (turn_direction == "right" || turn_direction == "left") {
        intersection_point = p;
        RCLCPP_INFO(getLogger(), "intersection is found.");
        break;
      }
    }
    if (intersection_point) {
      break;
    }
  }

  const auto calcBehindPose = [&data](const Point & p, const double dist) {
    const auto & path = data.reference_path;
    const size_t start = findNearestIndex(path.points, p);
    double sum = 0.0;
    for (size_t i = start - 1; i > 1; --i) {
      sum += calcDistance2d(path.points.at(i), path.points.at(i + 1));
      if (sum > dist) {
        return path.points.at(i).point.pose;
      }
    }
    return path.points.at(0).point.pose;
  };

  const auto intersection_shift_point = [&]() {
    boost::optional<AvoidPoint> shift_point{};
    if (!intersection_point) {
      RCLCPP_INFO(getLogger(), "No intersection.");
      return shift_point;
    }

    const double ego_to_intersection_dist = calcSignedArcLength(
      data.reference_path.points, getEgoPosition(), intersection_point->point.pose.position);

    if (ego_to_intersection_dist <= 5.0) {
      RCLCPP_INFO(getLogger(), "No enough margin to intersection.");
      return shift_point;
    }

    // Search obstacles around the intersection.
    // If it exists, use its shift distance on the intersection.
    constexpr double intersection_obstacle_check_dist = 10.0;
    constexpr double intersection_shift_margin = 1.0;

    double shift_length = 0.0;  // default (no obstacle) is zero.
    for (const auto & obj : avoidance_data_.objects) {
      if (
        std::abs(obj.longitudinal - ego_to_intersection_dist) > intersection_obstacle_check_dist) {
        continue;
      }
      if (isOnRight(obj)) {
        continue;  // TODO(Horibe) Now only think about the left side obstacle.
      }
      shift_length = std::min(shift_length, obj.overhang_dist - intersection_shift_margin);
    }
    RCLCPP_INFO(getLogger(), "Intersection shift_length = %f", shift_length);

    AvoidPoint p{};
    p.length = shift_length;
    p.start =
      calcBehindPose(intersection_point->point.pose.position, intersection_obstacle_check_dist);
    p.end = intersection_point->point.pose;
    shift_point = p;
    return shift_point;
  }();

  return intersection_shift_point;
}

BehaviorModuleOutput AvoidanceModule::plan()
{
  DEBUG_PRINT("AVOIDANCE plan");

  const auto shift_points = calcShiftPoints(current_raw_shift_points_, debug_data_);

  const auto new_shift_points = findNewShiftPoint(shift_points, path_shifter_);

  /**
   * Has new shift point?
   *   Yes -> Is it approved?
   *       Yes -> add the shift point.
   *       No  -> set approval_handler to WAIT_APPROVAL state.
   *   No -> waiting approval?
   *       Yes -> clear WAIT_APPROVAL state.
   *       No  -> do nothing.
   */
  if (new_shift_points) {
    debug_data_.new_shift_points = *new_shift_points;
    DEBUG_PRINT("new_shift_points size = %lu", new_shift_points->size());
    printShiftPoints(*new_shift_points, "new_shift_points");
    addShiftPointIfApproved(*new_shift_points);
  } else if (approval_handler_.isWaitingApproval()) {
    approval_handler_.clearWaitApproval();
  }

  // generate path with shift points that have been inserted.
  auto avoidance_path = generateAvoidancePath(path_shifter_);
  debug_data_.output_shift = avoidance_path.shift_length;

  // Drivable area generation.
  constexpr double extend_margin = 0.5;
  generateExtendedDrivableArea(&avoidance_path, extend_margin);

  // modify max speed to prevent acceleration in avoidance maneuver.
  modifyPathVelocityToPreventAccelerationOnAvoidance(avoidance_path);

  // post processing
  {
    postProcess(path_shifter_);  // remove old shift points
    prev_output_ = avoidance_path;
    prev_linear_shift_path_ = toShiftedPath(avoidance_data_.reference_path);
    path_shifter_.generate(&prev_linear_shift_path_, true, SHIFT_TYPE::LINEAR);
    prev_reference_ = avoidance_data_.reference_path;
    if (parameters_.publish_debug_marker) {
      setDebugData(path_shifter_, debug_data_);
    }
  }

  BehaviorModuleOutput output;
  output.turn_signal_info = calcTurnSignalInfo(avoidance_path);
  // sparse resampling for computational cost
  {
    avoidance_path.path =
      util::resamplePathWithSpline(avoidance_path.path, parameters_.resample_interval_for_output);
  }
  output.path = std::make_shared<PathWithLaneId>(avoidance_path.path);

  clipPathLength(*output.path);

  DEBUG_PRINT("exit plan(): set prev output (back().lat = %f)", prev_output_.shift_length.back());

  return output;
}

PathWithLaneId AvoidanceModule::planCandidate() const
{
  DEBUG_PRINT("AVOIDANCE planCandidate start");

  auto path_shifter = path_shifter_;
  auto debug_data = debug_data_;
  auto current_raw_shift_points = current_raw_shift_points_;

  const auto shift_points = calcShiftPoints(current_raw_shift_points, debug_data);
  const auto new_shift_points = findNewShiftPoint(shift_points, path_shifter);
  if (new_shift_points) {
    addNewShiftPoints(path_shifter, *new_shift_points);
  }

  auto shifted_path = generateAvoidancePath(path_shifter);

  if (new_shift_points) {  // clip from shift start index for visualize
    clipByMinStartIdx(*new_shift_points, shifted_path.path);
  }

  clipPathLength(shifted_path.path);

  return shifted_path.path;
}

BehaviorModuleOutput AvoidanceModule::planWaitingApproval()
{
  // we can execute the plan() since it handles the approval appropriately.
  BehaviorModuleOutput out = plan();
  out.path_candidate = std::make_shared<PathWithLaneId>(planCandidate());
  return out;
}

void AvoidanceModule::addShiftPointIfApproved(const AvoidPointArray & shift_points)
{
  if (approval_handler_.isApproved()) {
    DEBUG_PRINT("We want to add this shift point, and approved. ADD SHIFT POINT!");
    const size_t prev_size = path_shifter_.getShiftPointsSize();
    addNewShiftPoints(path_shifter_, shift_points);

    // register original points for consistency
    registerRawShiftPoints(shift_points);

    DEBUG_PRINT("shift_point size: %lu -> %lu", prev_size, path_shifter_.getShiftPointsSize());

    // use this approval.
    approval_handler_.clearApproval();  // TODO(Horibe) will be fixed with service-call?
  } else {
    DEBUG_PRINT("We want to add this shift point, but NOT approved. waiting...");
    approval_handler_.waitApproval();
  }
}

/**
 * set new shift points. remove old shift points if it has a conflict.
 */
void AvoidanceModule::addNewShiftPoints(
  PathShifter & path_shifter, const AvoidPointArray & new_shift_points) const
{
  ShiftPointArray future = toShiftPointArray(new_shift_points);

  size_t min_start_idx = std::numeric_limits<size_t>::max();
  for (const auto & sp : new_shift_points) {
    min_start_idx = std::min(min_start_idx, sp.start_idx);
  }

  const auto current_shift_points = path_shifter.getShiftPoints();

  DEBUG_PRINT("min_start_idx = %lu", min_start_idx);

  // Remove shift points that starts later than the new_shift_point from path_shifter.
  //
  // Why? Because shifter sorts by start position and applies shift points, so if there is a
  // shift point that starts after the one you are going to put in, new one will be affected
  // by the old one.
  //
  // Is it ok? This is a situation where the vehicle was originally going to avoid at the farther
  // point, but decided to avoid it at a closer point. In this case, it is reasonable to cancel the
  // farther avoidance.
  for (const auto & sp : current_shift_points) {
    if (sp.start_idx >= min_start_idx) {
      DEBUG_PRINT(
        "sp.start_idx = %lu, this sp starts after new proposal. remove this one.", sp.start_idx);
    } else {
      DEBUG_PRINT("sp.start_idx = %lu, no conflict. keep this one.", sp.start_idx);
      future.push_back(sp);
    }
  }

  path_shifter.setShiftPoints(future);
}

boost::optional<AvoidPointArray> AvoidanceModule::findNewShiftPoint(
  const AvoidPointArray & candidates, const PathShifter & shifter) const
{
  (void)shifter;

  if (candidates.empty()) {
    DEBUG_PRINT("shift candidates is empty. return None.");
    return {};
  }

  printShiftPoints(candidates, "findNewShiftPoint: candidates");

  // Retrieve the subsequent linear shift point from the given index point.
  const auto getShiftPointWithSubsequentStraight = [this, &candidates](size_t i) {
    AvoidPointArray subsequent{candidates.at(i)};
    for (size_t j = i + 1; j < candidates.size(); ++j) {
      const auto next_shift = candidates.at(j);
      if (std::abs(next_shift.getRelativeLength()) < 1.0e-2) {
        subsequent.push_back(next_shift);
        DEBUG_PRINT("j = %lu, relative shift is zero. add together.", j);
      } else {
        DEBUG_PRINT("j = %lu, relative shift is not zero = %f.", j, next_shift.getRelativeLength());
        break;
      }
    }
    return subsequent;
  };

  const auto calcJerk = [this](const auto & ap) {
    return path_shifter_.calcJerkFromLatLonDistance(
      ap.getRelativeLength(), ap.getRelativeLongitudinal(), getSharpAvoidanceEgoSpeed());
  };

  // TODO(Horibe) maybe this value must be same with trimSmallShift's.
  constexpr double NEW_POINT_THRESHOLD = 0.5 - 1.0e-3;

  for (size_t i = 0; i < candidates.size(); ++i) {
    const auto & candidate = candidates.at(i);
    std::stringstream ss;
    ss << "i = " << i << ", id = " << candidate.id;
    const auto pfx = ss.str().c_str();

    if (prev_reference_.points.size() != prev_linear_shift_path_.path.points.size()) {
      throw std::logic_error("prev_reference_ and prev_linear_shift_path_ must have same size.");
    }

    // TODO(Horibe): this code prohibits the changes on ego pose. Think later.
    // if (candidate.start_idx < avoidance_data_.ego_closest_path_index) {
    //   DEBUG_PRINT("%s, start_idx is behind ego. skip.", pfx);
    //   continue;
    // }

    if (calcJerk(candidate) > parameters_.max_lateral_jerk) {
      DEBUG_PRINT("%s, this shift exceeds jerk limit (%f). skip.", pfx, calcJerk(candidate));
      continue;
    }

    const auto current_shift = prev_linear_shift_path_.shift_length.at(
      findNearestIndex(prev_reference_.points, candidate.end.position));

    // TODO(Horibe) test fails with this print. why?
    // DEBUG_PRINT("%s, shift current: %f, candidate: %f", pfx, current_shift, candidate.length);

    if (std::abs(candidate.length - current_shift) > NEW_POINT_THRESHOLD) {
      DEBUG_PRINT(
        "%s, New shift point is found!!! shift change: %f -> %f", pfx, current_shift,
        candidate.length);
      return getShiftPointWithSubsequentStraight(i);
    }
  }

  DEBUG_PRINT("No new shift point exists.");
  return {};
}

double AvoidanceModule::getEgoSpeed() const
{
  return std::abs(planner_data_->self_odometry->twist.twist.linear.x);
}

double AvoidanceModule::getNominalAvoidanceEgoSpeed() const
{
  return std::max(getEgoSpeed(), parameters_.min_nominal_avoidance_speed);
}
double AvoidanceModule::getSharpAvoidanceEgoSpeed() const
{
  return std::max(getEgoSpeed(), parameters_.min_sharp_avoidance_speed);
}

Point AvoidanceModule::getEgoPosition() const { return planner_data_->self_pose->pose.position; }

PoseStamped AvoidanceModule::getEgoPose() const { return *(planner_data_->self_pose); }

PoseStamped AvoidanceModule::getUnshiftedEgoPose(const ShiftedPath & prev_path) const
{
  const auto ego_pose = getEgoPose();

  if (prev_path.path.points.empty()) {
    return ego_pose;
  }

  // un-shifted fot current ideal pose
  const auto closest = findNearestIndex(prev_path.path.points, ego_pose.pose.position);

  PoseStamped unshifted_pose = ego_pose;
  unshifted_pose.pose.orientation = prev_path.path.points.at(closest).point.pose.orientation;

  util::shiftPose(&unshifted_pose.pose, -prev_path.shift_length.at(closest));
  unshifted_pose.pose.orientation = ego_pose.pose.orientation;

  return unshifted_pose;
}

double AvoidanceModule::getNominalAvoidanceDistance(const double shift_length) const
{
  const auto & p = parameters_;
  const auto distance_by_jerk = path_shifter_.calcLongitudinalDistFromJerk(
    shift_length, parameters_.nominal_lateral_jerk, getNominalAvoidanceEgoSpeed());

  return std::max(p.min_avoidance_distance, distance_by_jerk);
}

double AvoidanceModule::getSharpAvoidanceDistance(const double shift_length) const
{
  const auto & p = parameters_;
  const auto distance_by_jerk = path_shifter_.calcLongitudinalDistFromJerk(
    shift_length, parameters_.max_lateral_jerk, getSharpAvoidanceEgoSpeed());

  return std::max(p.min_avoidance_distance, distance_by_jerk);
}

double AvoidanceModule::getNominalPrepareDistance() const
{
  const auto & p = parameters_;
  const auto epsilon_m = 0.01;  // for floating error to pass "has_enough_distance" check.
  const auto nominal_distance = std::max(getEgoSpeed() * p.prepare_time, p.min_prepare_distance);
  return nominal_distance + epsilon_m;
}

ShiftedPath AvoidanceModule::generateAvoidancePath(PathShifter & path_shifter) const
{
  DEBUG_PRINT("path_shifter: base shift = %f", getCurrentBaseShift());
  printShiftPoints(path_shifter.getShiftPoints(), "path_shifter shift points");

  ShiftedPath shifted_path;
  if (!path_shifter.generate(&shifted_path)) {
    RCLCPP_ERROR(getLogger(), "failed to generate shifted path.");
    return toShiftedPath(avoidance_data_.reference_path);
  }

  return shifted_path;
}

void AvoidanceModule::postProcess(PathShifter & path_shifter) const
{
  path_shifter.removeBehindShiftPointAndSetBaseOffset(getEgoPosition());
}

void AvoidanceModule::updateData()
{
  debug_data_ = DebugData();
  avoidance_data_ = calcAvoidancePlanningData(debug_data_);

  // TODO(Horibe): this is not tested yet, disable now.
  updateRegisteredObject(avoidance_data_.objects);
  CompensateDetectionLost(avoidance_data_.objects);

  path_shifter_.setPath(avoidance_data_.reference_path);

  // update registered shift point for new reference path & remove past objects
  updateRegisteredRawShiftPoints();

  // for the first time
  if (prev_output_.path.points.empty()) {
    prev_output_.path = avoidance_data_.reference_path;
    prev_output_.shift_length = std::vector<double>(prev_output_.path.points.size(), 0.0);
  }
  if (prev_linear_shift_path_.path.points.empty()) {
    prev_linear_shift_path_.path = avoidance_data_.reference_path;
    prev_linear_shift_path_.shift_length =
      std::vector<double>(prev_linear_shift_path_.path.points.size(), 0.0);
  }
  if (prev_reference_.points.empty()) {
    prev_reference_ = avoidance_data_.reference_path;
  }
}

std::string getUuidStr(const ObjectData & obj)
{
  return std::to_string(obj.object.object_id.uuid.at(0)) +
         std::to_string(obj.object.object_id.uuid.at(1)) +
         std::to_string(obj.object.object_id.uuid.at(2));
}

std::string getUuidStr(const ObjectDataArray & objs)
{
  std::stringstream ss;
  for (const auto & o : objs) {
    ss << getUuidStr(o) << ", ";
  }
  return ss.str();
}

/*
 * updateRegisteredObject
 *
 * Same object is observed this time -> update registered object with the new one.
 * Not observed -> increment the lost_count. if it exceeds the threshold, remove it.
 * How to check if it is same object?
 *   - it has same ID
 *   - it has different id, but sn object is found around similar position
 */
void AvoidanceModule::updateRegisteredObject(const ObjectDataArray & now_objects)
{
  const auto updateIfDetectedNow = [&now_objects, this](auto & registered_object) {
    const auto & n = now_objects;
    const auto r_id = registered_object.object.object_id;
    const auto same_id_obj = std::find_if(
      n.begin(), n.end(), [&r_id](const auto & o) { return o.object.object_id == r_id; });

    // same id object is detected. update registered.
    if (same_id_obj != n.end()) {
      registered_object = *same_id_obj;
      return true;
    }

    constexpr auto POS_THR = 1.5;
    const auto r_pos = registered_object.object.kinematics.initial_pose_with_covariance.pose;
    const auto similar_pos_obj = std::find_if(n.begin(), n.end(), [&](const auto & o) {
      return calcDistance2d(r_pos, o.object.kinematics.initial_pose_with_covariance.pose) < POS_THR;
    });

    // same id object is not detected, but object is found around registered. update registered.
    if (similar_pos_obj != n.end()) {
      registered_object = *similar_pos_obj;
      return true;
    }

    // Same ID nor similar position object does not found.
    return false;
  };

  // -- check registered_objects, remove if lost_count exceeds limit. --
  for (int i = static_cast<int>(registered_objects_.size()) - 1; i >= 0; --i) {
    auto & r = registered_objects_.at(i);
    const std::string s = getUuidStr(r);

    // registered object is not detected this time. lost count up.
    if (!updateIfDetectedNow(r)) {
      ++r.lost_count;

      // lost count exceeds threshold. remove object from register.
      if (r.lost_count > parameters_.object_hold_max_count) {
        registered_objects_.erase(registered_objects_.begin() + i);
      }
    }
  }

  const auto isAlreadyRegistered = [this](const auto & n_id) {
    const auto & r = registered_objects_;
    return std::any_of(
      r.begin(), r.end(), [&n_id](const auto & o) { return o.object.object_id == n_id; });
  };

  // -- check now_objects, add it if it has new object id --
  for (const auto now_obj : now_objects) {
    if (!isAlreadyRegistered(now_obj.object.object_id)) {
      registered_objects_.push_back(now_obj);
    }
  }
}

/*
 * CompensateDetectionLost
 *
 * add registered object if the now_objects does not contain the same object_id.
 *
 */
void AvoidanceModule::CompensateDetectionLost(ObjectDataArray & now_objects) const
{
  const auto old_size = now_objects.size();  // for debug

  const auto isDetectedNow = [&](const auto & r_id) {
    const auto & n = now_objects;
    return std::any_of(
      n.begin(), n.end(), [&r_id](const auto & o) { return o.object.object_id == r_id; });
  };

  for (const auto & registered : registered_objects_) {
    if (!isDetectedNow(registered.object.object_id)) {
      now_objects.push_back(registered);
    }
  }
  DEBUG_PRINT("object size: %lu -> %lu", old_size, now_objects.size());
}

void AvoidanceModule::onEntry()
{
  DEBUG_PRINT("AVOIDANCE onEntry. wait approval!");
  initVariables();
  current_state_ = BT::NodeStatus::SUCCESS;
  approval_handler_.waitApproval();
}

void AvoidanceModule::onExit()
{
  DEBUG_PRINT("AVOIDANCE onExit");
  initVariables();
  current_state_ = BT::NodeStatus::IDLE;
  approval_handler_.clearWaitApproval();
}

void AvoidanceModule::setParameters(const AvoidanceParameters & parameters)
{
  parameters_ = parameters;
}

void AvoidanceModule::initVariables()
{
  prev_output_ = ShiftedPath();
  prev_linear_shift_path_ = ShiftedPath();
  prev_reference_ = PathWithLaneId();
  path_shifter_ = PathShifter{};

  debug_data_ = DebugData();

  registered_raw_shift_points_ = {};
  current_raw_shift_points_ = {};
  original_unique_id = 0;
}

void AvoidanceModule::clipPathLength(PathWithLaneId & path) const
{
  const double forward = planner_data_->parameters.forward_path_length;
  const double backward = planner_data_->parameters.backward_path_length;

  util::clipPathLength(path, getEgoPosition(), forward, backward);
}

bool AvoidanceModule::isTargetObjectType(const PredictedObject & object) const
{
  using autoware_auto_perception_msgs::msg::ObjectClassification;
  const auto t = util::getHighestProbLabel(object.classification);
  const auto is_object_type =
    (t == ObjectClassification::CAR || t == ObjectClassification::TRUCK ||
     t == ObjectClassification::BUS);
  return is_object_type;
}

TurnSignalInfo AvoidanceModule::calcTurnSignalInfo(const ShiftedPath & path) const
{
  TurnSignalInfo turn_signal;

  const auto shift_points = path_shifter_.getShiftPoints();
  if (shift_points.empty()) {
    return {};
  }

  const auto latest_shift_point = shift_points.front();  // assuming it is sorted.

  const auto turn_info = util::getPathTurnSignal(
    avoidance_data_.current_lanelets, path, latest_shift_point, planner_data_->self_pose->pose,
    planner_data_->self_odometry->twist.twist.linear.x, planner_data_->parameters,
    parameters_.avoidance_search_distance);

  // Set turn signal if the vehicle across the lane.
  if (!path.shift_length.empty()) {
    if (isAvoidancePlanRunning()) {
      turn_signal.turn_signal.command = turn_info.first.command;
    }
  }

  // calc distance from ego to latest_shift_point end point.
  if (turn_info.second >= 0.0) {
    turn_signal.signal_distance = turn_info.second;
  }

  return turn_signal;
}

double AvoidanceModule::getCurrentShift() const
{
  return prev_output_.shift_length.at(findNearestIndex(prev_output_.path.points, getEgoPosition()));
}
double AvoidanceModule::getCurrentLinearShift() const
{
  return prev_linear_shift_path_.shift_length.at(
    findNearestIndex(prev_linear_shift_path_.path.points, getEgoPosition()));
}

void AvoidanceModule::setDebugData(const PathShifter & shifter, const DebugData & debug)
{
  using marker_utils::createAvoidanceObjectsMarkerArray;
  using marker_utils::createAvoidPointMarkerArray;
  using marker_utils::createLaneletsAreaMarkerArray;
  using marker_utils::createObjectsMarkerArray;
  using marker_utils::createPathMarkerArray;
  using marker_utils::createPoseMarkerArray;
  using marker_utils::createShiftLengthMarkerArray;
  using marker_utils::createShiftPointMarkerArray;

  debug_marker_.markers.clear();

  const auto add = [this](const MarkerArray & added) {
    tier4_autoware_utils::appendMarkerArray(added, &debug_marker_);
  };

  const auto addAvoidPoint =
    [&](const AvoidPointArray & ap_arr, const auto & ns, auto r, auto g, auto b, double w = 0.1) {
      add(createAvoidPointMarkerArray(ap_arr, ns, r, g, b, w));
    };

  const auto addShiftPoint =
    [&](const ShiftPointArray & sp_arr, const auto & ns, auto r, auto g, auto b, double w = 0.1) {
      add(createShiftPointMarkerArray(sp_arr, shifter.getBaseOffset(), ns, r, g, b, w));
    };

  const auto & path = avoidance_data_.reference_path;
  add(createPathMarkerArray(debug.center_line, "centerline", 0, 0.0, 0.5, 0.9));
  add(createPathMarkerArray(path, "centerline_resampled", 0, 0.0, 0.9, 0.5));
  add(createPathMarkerArray(prev_linear_shift_path_.path, "prev_linear_shift", 0, 0.5, 0.4, 0.6));
  add(createPoseMarkerArray(avoidance_data_.reference_pose, "reference_pose", 0, 0.9, 0.3, 0.3));

  add(createLaneletsAreaMarkerArray(*debug.current_lanelets, "current_lanelet", 0.0, 1.0, 0.0));
  add(createLaneletsAreaMarkerArray(*debug.expanded_lanelets, "expanded_lanelet", 0.8, 0.8, 0.0));
  add(createAvoidanceObjectsMarkerArray(avoidance_data_.objects, "avoidance_object"));

  // parent object info
  addAvoidPoint(debug.registered_raw_shift, "p_registered_shift", 0.8, 0.8, 0.0);
  addAvoidPoint(debug.current_raw_shift, "p_current_raw_shift", 0.5, 0.2, 0.2);
  addAvoidPoint(debug.extra_return_shift, "p_extra_return_shift", 0.0, 0.5, 0.8);

  // merged shift
  const auto & linear_shift = prev_linear_shift_path_.shift_length;
  add(createShiftLengthMarkerArray(debug.pos_shift, path, "m_pos_shift_line", 0, 0.7, 0.5));
  add(createShiftLengthMarkerArray(debug.neg_shift, path, "m_neg_shift_line", 0, 0.5, 0.7));
  add(createShiftLengthMarkerArray(debug.total_shift, path, "m_total_shift_line", 0.99, 0.4, 0.2));
  add(createShiftLengthMarkerArray(debug.output_shift, path, "m_output_shift_line", 0.8, 0.8, 0.2));
  add(createShiftLengthMarkerArray(linear_shift, path, "m_output_linear_line", 0.9, 0.3, 0.3));

  // child shift points
  addAvoidPoint(debug.merged, "c_0_merged", 0.345, 0.968, 1.0);
  addAvoidPoint(debug.trim_similar_grad_shift, "c_1_trim_similar_grad_shift", 0.976, 0.328, 0.910);
  addAvoidPoint(debug.quantized, "c_2_quantized", 0.505, 0.745, 0.969);
  addAvoidPoint(debug.trim_small_shift, "c_3_trim_small_shift", 0.663, 0.525, 0.941);
  addAvoidPoint(
    debug.trim_similar_grad_shift_second, "c_4_trim_similar_grad_shift", 0.97, 0.32, 0.91);
  addAvoidPoint(debug.trim_momentary_return, "c_5_trim_momentary_return", 0.976, 0.078, 0.878);
  addAvoidPoint(debug.trim_too_sharp_shift, "c_6_trim_too_sharp_shift", 0.576, 0.0, 0.978);

  addShiftPoint(shifter.getShiftPoints(), "path_shifter_registered_points", 0.99, 0.99, 0.0, 0.5);
  addAvoidPoint(debug.new_shift_points, "path_shifter_proposed_points", 0.99, 0.0, 0.0, 0.5);
}

}  // namespace behavior_path_planner
