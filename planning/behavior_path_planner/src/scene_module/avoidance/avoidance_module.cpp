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
#include "behavior_path_planner/scene_module/scene_module_visitor.hpp"
#include "behavior_path_planner/utilities.hpp"

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
using motion_utils::calcSignedArcLength;
using motion_utils::findNearestIndex;
using motion_utils::findNearestSegmentIndex;
using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::calcLateralDeviation;
using tier4_planning_msgs::msg::AvoidanceDebugFactor;

AvoidanceModule::AvoidanceModule(
  const std::string & name, rclcpp::Node & node, std::shared_ptr<AvoidanceParameters> parameters)
: SceneModuleInterface{name, node},
  parameters_{std::move(parameters)},
  rtc_interface_left_(&node, "avoidance_left"),
  rtc_interface_right_(&node, "avoidance_right"),
  uuid_left_{generateUUID()},
  uuid_right_{generateUUID()}
{
  using std::placeholders::_1;
  steering_factor_interface_ptr_ = std::make_unique<SteeringFactorInterface>(&node, "avoidance");
}

bool AvoidanceModule::isExecutionRequested() const
{
  DEBUG_PRINT("AVOIDANCE isExecutionRequested");

  if (current_state_ == BT::NodeStatus::RUNNING) {
    return true;
  }

  const auto avoid_data = calcAvoidancePlanningData(debug_data_);

  if (parameters_->publish_debug_marker) {
    setDebugData(avoid_data, path_shifter_, debug_data_);
  }

  return !avoid_data.target_objects.empty();
}

bool AvoidanceModule::isExecutionReady() const
{
  DEBUG_PRINT("AVOIDANCE isExecutionReady");

  {
    DebugData debug;
    static_cast<void>(calcAvoidancePlanningData(debug));
  }

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
  const bool has_avoidance_target = !avoid_data.target_objects.empty();
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
  const bool has_shift_line = (path_shifter_.getShiftLinesSize() > 0);
  return has_base_offset || has_shift_line;
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
    util::resamplePathWithSpline(center_path, parameters_->resample_interval_for_planning);
  if (data.reference_path.points.size() < 2) {
    // if the resampled path has only 1 point, use original path.
    data.reference_path = center_path;
  }

  const size_t nearest_segment_index =
    findNearestSegmentIndex(data.reference_path.points, data.reference_pose.position);
  data.ego_closest_path_index =
    std::min(nearest_segment_index + 1, data.reference_path.points.size() - 1);

  // arclength from ego pose (used in many functions)
  data.arclength_from_ego = util::calcPathArcLengthArray(
    data.reference_path, 0, data.reference_path.points.size(),
    calcSignedArcLength(data.reference_path.points, getEgoPosition(), 0));

  // lanelet info
  data.current_lanelets = util::calcLaneAroundPose(
    planner_data_->route_handler, reference_pose.pose,
    planner_data_->parameters.forward_path_length, planner_data_->parameters.backward_path_length);

  // target objects for avoidance
  fillAvoidanceTargetObjects(data, debug);

  DEBUG_PRINT("target object size = %lu", data.target_objects.size());

  return data;
}

void AvoidanceModule::fillAvoidanceTargetObjects(
  AvoidancePlanningData & data, DebugData & debug) const
{
  using lanelet::geometry::distance2d;
  using lanelet::utils::getId;
  using lanelet::utils::to2D;

  const auto & path_points = data.reference_path.points;
  const auto & ego_pos = getEgoPosition();

  // detection area filter
  // when expanding lanelets, right_offset must be minus.
  // This is because y axis is positive on the left.
  const auto expanded_lanelets = lanelet::utils::getExpandedLanelets(
    data.current_lanelets, parameters_->detection_area_left_expand_dist,
    parameters_->detection_area_right_expand_dist * (-1.0));
  const auto lane_filtered_objects_index =
    util::filterObjectIndicesByLanelets(*planner_data_->dynamic_object, expanded_lanelets);

  DEBUG_PRINT("dynamic_objects size = %lu", planner_data_->dynamic_object->objects.size());
  DEBUG_PRINT("lane_filtered_objects size = %lu", lane_filtered_objects_index.size());

  // for goal
  const auto & rh = planner_data_->route_handler;
  const auto dist_to_goal =
    rh->isInGoalRouteSection(expanded_lanelets.back())
      ? calcSignedArcLength(path_points, ego_pos, rh->getGoalPose().position)
      : std::numeric_limits<double>::max();

  lanelet::ConstLineStrings3d debug_linestring;
  debug_linestring.clear();
  // for filtered objects
  ObjectDataArray target_objects;
  std::vector<AvoidanceDebugMsg> avoidance_debug_msg_array;
  for (const auto & i : lane_filtered_objects_index) {
    const auto & object = planner_data_->dynamic_object->objects.at(i);
    const auto & object_pos = object.kinematics.initial_pose_with_covariance.pose.position;
    AvoidanceDebugMsg avoidance_debug_msg;
    const auto avoidance_debug_array_false_and_push_back =
      [&avoidance_debug_msg, &avoidance_debug_msg_array](const std::string & failed_reason) {
        avoidance_debug_msg.allow_avoidance = false;
        avoidance_debug_msg.failed_reason = failed_reason;
        avoidance_debug_msg_array.push_back(avoidance_debug_msg);
      };

    if (!isTargetObjectType(object)) {
      avoidance_debug_array_false_and_push_back(AvoidanceDebugFactor::OBJECT_IS_NOT_TYPE);
      continue;
    }

    ObjectData object_data;
    object_data.object = object;
    avoidance_debug_msg.object_id = getUuidStr(object_data);

    const auto object_closest_index = findNearestIndex(path_points, object_pos);
    const auto object_closest_pose = path_points.at(object_closest_index).point.pose;

    // Calc envelop polygon.
    fillObjectEnvelopePolygon(object_closest_pose, object_data);

    // calc longitudinal distance from ego to closest target object footprint point.
    fillLongitudinalAndLengthByClosestEnvelopeFootprint(data.reference_path, ego_pos, object_data);
    avoidance_debug_msg.longitudinal_distance = object_data.longitudinal;

    // Calc moving time.
    fillObjectMovingTime(object_data);

    if (object_data.move_time > parameters_->threshold_time_object_is_moving) {
      avoidance_debug_array_false_and_push_back("MovingObject");
      data.other_objects.push_back(object_data);
      continue;
    }

    // object is behind ego or too far.
    if (object_data.longitudinal < -parameters_->object_check_backward_distance) {
      avoidance_debug_array_false_and_push_back(AvoidanceDebugFactor::OBJECT_IS_BEHIND_THRESHOLD);
      data.other_objects.push_back(object_data);
      continue;
    }
    if (object_data.longitudinal > parameters_->object_check_forward_distance) {
      avoidance_debug_array_false_and_push_back(AvoidanceDebugFactor::OBJECT_IS_IN_FRONT_THRESHOLD);
      data.other_objects.push_back(object_data);
      continue;
    }

    // Target object is behind the path goal -> ignore.
    if (object_data.longitudinal > dist_to_goal) {
      avoidance_debug_array_false_and_push_back(AvoidanceDebugFactor::OBJECT_BEHIND_PATH_GOAL);
      data.other_objects.push_back(object_data);
      continue;
    }

    // Calc lateral deviation from path to target object.
    object_data.lateral = calcLateralDeviation(object_closest_pose, object_pos);
    avoidance_debug_msg.lateral_distance_from_centerline = object_data.lateral;

    // Find the footprint point closest to the path, set to object_data.overhang_distance.
    object_data.overhang_dist = calcEnvelopeOverhangDistance(
      object_data, object_closest_pose, object_data.overhang_pose.position);

    lanelet::ConstLanelet overhang_lanelet;
    if (!rh->getClosestLaneletWithinRoute(object_closest_pose, &overhang_lanelet)) {
      continue;
    }

    if (overhang_lanelet.id()) {
      object_data.overhang_lanelet = overhang_lanelet;
      lanelet::BasicPoint3d overhang_basic_pose(
        object_data.overhang_pose.position.x, object_data.overhang_pose.position.y,
        object_data.overhang_pose.position.z);
      const bool get_left =
        isOnRight(object_data) && parameters_->enable_avoidance_over_same_direction;
      const bool get_right =
        !isOnRight(object_data) && parameters_->enable_avoidance_over_same_direction;

      const auto target_lines = rh->getFurthestLinestring(
        overhang_lanelet, get_right, get_left,
        parameters_->enable_avoidance_over_opposite_direction);

      if (isOnRight(object_data)) {
        object_data.to_road_shoulder_distance =
          distance2d(to2D(overhang_basic_pose), to2D(target_lines.back().basicLineString()));
        debug_linestring.push_back(target_lines.back());
      } else {
        object_data.to_road_shoulder_distance =
          distance2d(to2D(overhang_basic_pose), to2D(target_lines.front().basicLineString()));
        debug_linestring.push_back(target_lines.front());
      }
    }

    DEBUG_PRINT(
      "set object_data: longitudinal = %f, lateral = %f, largest_overhang = %f,"
      "to_road_shoulder_distance = %f",
      object_data.longitudinal, object_data.lateral, object_data.overhang_dist,
      object_data.to_road_shoulder_distance);

    // Object is on center line -> ignore.
    avoidance_debug_msg.lateral_distance_from_centerline = object_data.lateral;
    if (std::abs(object_data.lateral) < parameters_->threshold_distance_object_is_on_center) {
      avoidance_debug_array_false_and_push_back(AvoidanceDebugFactor::TOO_NEAR_TO_CENTERLINE);
      data.other_objects.push_back(object_data);
      continue;
    }

    object_data.last_seen = clock_->now();

    // set data
    data.target_objects.push_back(object_data);
  }

  // debug
  {
    updateAvoidanceDebugData(avoidance_debug_msg_array);
    debug.farthest_linestring_from_overhang =
      std::make_shared<lanelet::ConstLineStrings3d>(debug_linestring);
    debug.current_lanelets = std::make_shared<lanelet::ConstLanelets>(data.current_lanelets);
    debug.expanded_lanelets = std::make_shared<lanelet::ConstLanelets>(expanded_lanelets);
  }
}

void AvoidanceModule::fillObjectEnvelopePolygon(
  const Pose & closest_pose, ObjectData & object_data) const
{
  using boost::geometry::within;

  const auto id = object_data.object.object_id;
  const auto same_id_obj = std::find_if(
    registered_objects_.begin(), registered_objects_.end(),
    [&id](const auto & o) { return o.object.object_id == id; });

  if (same_id_obj == registered_objects_.end()) {
    object_data.envelope_poly =
      createEnvelopePolygon(object_data, closest_pose, parameters_->object_envelope_buffer);
    return;
  }

  Polygon2d object_polygon{};
  util::calcObjectPolygon(object_data.object, &object_polygon);

  if (!within(object_polygon, same_id_obj->envelope_poly)) {
    object_data.envelope_poly =
      createEnvelopePolygon(object_data, closest_pose, parameters_->object_envelope_buffer);
    return;
  }

  object_data.envelope_poly = same_id_obj->envelope_poly;
}

void AvoidanceModule::fillObjectMovingTime(ObjectData & object_data) const
{
  const auto & object_vel =
    object_data.object.kinematics.initial_twist_with_covariance.twist.linear.x;
  const auto is_faster_than_threshold = object_vel > parameters_->threshold_speed_object_is_stopped;

  const auto id = object_data.object.object_id;
  const auto same_id_obj = std::find_if(
    stopped_objects_.begin(), stopped_objects_.end(),
    [&id](const auto & o) { return o.object.object_id == id; });

  const auto is_new_object = same_id_obj == stopped_objects_.end();

  if (!is_faster_than_threshold) {
    object_data.last_stop = clock_->now();
    object_data.move_time = 0.0;
    if (is_new_object) {
      stopped_objects_.push_back(object_data);
    } else {
      same_id_obj->last_stop = clock_->now();
      same_id_obj->move_time = 0.0;
    }
    return;
  }

  if (is_new_object) {
    object_data.move_time = std::numeric_limits<double>::max();
    return;
  }

  object_data.last_stop = same_id_obj->last_stop;
  object_data.move_time = (clock_->now() - same_id_obj->last_stop).seconds();

  if (object_data.move_time > parameters_->threshold_time_object_is_moving) {
    stopped_objects_.erase(same_id_obj);
  }
}

/**
 * updateRegisteredRawShiftLines
 *
 *  - update path index of the registered objects
 *  - remove old objects whose end point is behind ego pose.
 */
void AvoidanceModule::updateRegisteredRawShiftLines()
{
  fillAdditionalInfoFromPoint(registered_raw_shift_lines_);

  AvoidLineArray avoid_lines;
  const int margin = 0;
  const auto deadline = static_cast<size_t>(
    std::max(static_cast<int>(avoidance_data_.ego_closest_path_index) - margin, 0));

  for (const auto & al : registered_raw_shift_lines_) {
    if (al.end_idx > deadline) {
      avoid_lines.push_back(al);
    }
  }

  DEBUG_PRINT(
    "ego_closest_path_index = %lu, registered_raw_shift_lines_ size: %lu -> %lu",
    avoidance_data_.ego_closest_path_index, registered_raw_shift_lines_.size(), avoid_lines.size());

  printShiftLines(registered_raw_shift_lines_, "registered_raw_shift_lines_ (before)");
  printShiftLines(avoid_lines, "registered_raw_shift_lines_ (after)");

  registered_raw_shift_lines_ = avoid_lines;
  debug_data_.registered_raw_shift = registered_raw_shift_lines_;
}

AvoidLineArray AvoidanceModule::calcShiftLines(
  AvoidLineArray & current_raw_shift_lines, DebugData & debug) const
{
  /**
   * Generate raw_shift_lines (shift length, avoidance start point, end point, return point, etc)
   * for each object. These raw shift points are merged below to compute appropriate shift points.
   */
  current_raw_shift_lines = calcRawShiftLinesFromObjects(avoidance_data_.target_objects);
  debug.current_raw_shift = current_raw_shift_lines;

  /**
   * Use all registered points. For the current points, if the similar one of the current
   * points are already registered, will not use it.
   * TODO(Horibe): enrich this logic to be able to consider the removal of the registered
   *               shift, because it cannot handle the case like "we don't have to avoid
   *               the object anymore".
   */
  auto total_raw_shift_lines =
    combineRawShiftLinesWithUniqueCheck(registered_raw_shift_lines_, current_raw_shift_lines);

  printShiftLines(current_raw_shift_lines, "current_raw_shift_lines");
  printShiftLines(registered_raw_shift_lines_, "registered_raw_shift_lines");
  printShiftLines(total_raw_shift_lines, "total_raw_shift_lines");

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
  addReturnShiftLineFromEgo(total_raw_shift_lines, current_raw_shift_lines);
  printShiftLines(total_raw_shift_lines, "total_raw_shift_lines_with_extra_return_shift");

  /**
   * On each path point, compute shift length with considering the raw shift points.
   * Then create a merged shift points by finding the change point of the gradient of shifting.
   *  - take maximum shift length if there is duplicate shift point
   *  - take sum if there are shifts for opposite direction (right and left)
   *  - shift length is interpolated linearly.
   * Note: Because this function just foolishly extracts points, it includes
   *       insignificant small (useless) shift points, which should be removed in post-process.
   */
  auto merged_shift_lines = mergeShiftLines(total_raw_shift_lines, debug);
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

  const auto future_with_info = fillAdditionalInfo(future);
  printShiftLines(future_with_info, "future_with_info");
  printShiftLines(registered_raw_shift_lines_, "registered_raw_shift_lines_");
  printShiftLines(current_raw_shift_lines_, "current_raw_shift_lines_");

  const auto isAlreadyRegistered = [this](const auto id) {
    const auto & r = registered_raw_shift_lines_;
    return std::any_of(r.begin(), r.end(), [id](const auto & r_sl) { return r_sl.id == id; });
  };

  const auto getAvoidLineByID = [this](const auto id) {
    for (const auto & sl : current_raw_shift_lines_) {
      if (sl.id == id) {
        return sl;
      }
    }
    return AvoidLine{};
  };

  for (const auto & al : future_with_info) {
    if (al.parent_ids.empty()) {
      RCLCPP_ERROR(getLogger(), "avoid line for path_shifter must have parent_id.");
    }
    for (const auto parent_id : al.parent_ids) {
      if (!isAlreadyRegistered(parent_id)) {
        registered_raw_shift_lines_.push_back(getAvoidLineByID(parent_id));
      }
    }
  }

  DEBUG_PRINT("registered object size: %lu -> %lu", old_size, registered_raw_shift_lines_.size());
}

double AvoidanceModule::getShiftLength(
  const ObjectData & object, const bool & is_object_on_right, const double & avoid_margin) const
{
  const auto shift_length =
    behavior_path_planner::calcShiftLength(is_object_on_right, object.overhang_dist, avoid_margin);
  return is_object_on_right ? std::min(shift_length, getLeftShiftBound())
                            : std::max(shift_length, getRightShiftBound());
}
/**
 * calcRawShiftLinesFromObjects
 *
 * Calculate the shift points (start/end point, shift length) from the object lateral
 * and longitudinal positions in the Frenet coordinate. The jerk limit is also considered here.
 */
AvoidLineArray AvoidanceModule::calcRawShiftLinesFromObjects(const ObjectDataArray & objects) const
{
  debug_avoidance_initializer_for_shift_line_.clear();
  const auto prepare_distance = getNominalPrepareDistance();

  // To be consistent with changes in the ego position, the current shift length is considered.
  const auto current_ego_shift = getCurrentShift();
  // // implement lane detection here.
  const auto & lat_collision_safety_buffer = parameters_->lateral_collision_safety_buffer;
  const auto & lat_collision_margin = parameters_->lateral_collision_margin;
  const auto & vehicle_width = planner_data_->parameters.vehicle_width;
  const auto & road_shoulder_safety_margin = parameters_->road_shoulder_safety_margin;

  auto avoid_margin = lat_collision_safety_buffer + lat_collision_margin + 0.5 * vehicle_width;

  AvoidLineArray avoid_lines;
  std::vector<AvoidanceDebugMsg> avoidance_debug_msg_array;
  avoidance_debug_msg_array.reserve(objects.size());
  for (auto & o : objects) {
    AvoidanceDebugMsg avoidance_debug_msg;
    const auto avoidance_debug_array_false_and_push_back =
      [&avoidance_debug_msg, &avoidance_debug_msg_array](const std::string & failed_reason) {
        avoidance_debug_msg.allow_avoidance = false;
        avoidance_debug_msg.failed_reason = failed_reason;
        avoidance_debug_msg_array.push_back(avoidance_debug_msg);
      };

    const auto max_allowable_lateral_distance =
      o.to_road_shoulder_distance - road_shoulder_safety_margin - 0.5 * vehicle_width;

    avoidance_debug_msg.object_id = getUuidStr(o);
    avoidance_debug_msg.longitudinal_distance = o.longitudinal;
    avoidance_debug_msg.lateral_distance_from_centerline = o.lateral;
    avoidance_debug_msg.to_furthest_linestring_distance = o.to_road_shoulder_distance;
    avoidance_debug_msg.max_shift_length = max_allowable_lateral_distance;

    auto const min_safety_lateral_distance = lat_collision_safety_buffer + 0.5 * vehicle_width;

    if (max_allowable_lateral_distance < avoid_margin) {
      if (max_allowable_lateral_distance >= min_safety_lateral_distance) {
        avoid_margin = max_allowable_lateral_distance;
      } else {
        avoidance_debug_array_false_and_push_back(
          AvoidanceDebugFactor::INSUFFICIENT_LATERAL_MARGIN);
        continue;
      }
    }

    const auto is_object_on_right = isOnRight(o);
    const auto shift_length = getShiftLength(o, is_object_on_right, avoid_margin);
    if (isSameDirectionShift(is_object_on_right, shift_length)) {
      avoidance_debug_array_false_and_push_back("IgnoreSameDirectionShift");
      continue;
    }

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
        avoidance_debug_array_false_and_push_back(
          AvoidanceDebugFactor::REMAINING_DISTANCE_LESS_THAN_ZERO);
        continue;
      }

      // This is the case of exceeding the jerk limit. Use the sharp avoidance ego speed.
      const auto required_jerk = path_shifter_.calcJerkFromLatLonDistance(
        avoiding_shift, remaining_distance, getSharpAvoidanceEgoSpeed());
      avoidance_debug_msg.required_jerk = required_jerk;
      avoidance_debug_msg.maximum_jerk = parameters_->max_lateral_jerk;
      if (required_jerk > parameters_->max_lateral_jerk) {
        avoidance_debug_array_false_and_push_back(AvoidanceDebugFactor::TOO_LARGE_JERK);
        continue;
      }
    }
    const auto avoiding_distance =
      has_enough_distance ? nominal_avoid_distance : remaining_distance;

    DEBUG_PRINT(
      "nominal_lateral_jerk = %f, getNominalAvoidanceEgoSpeed() = %f, prepare_distance = %f, "
      "has_enough_distance = %d",
      parameters_->nominal_lateral_jerk, getNominalAvoidanceEgoSpeed(), prepare_distance,
      has_enough_distance);

    AvoidLine al_avoid;
    al_avoid.end_shift_length = shift_length;
    al_avoid.start_shift_length = current_ego_shift;
    al_avoid.end_longitudinal = o.longitudinal;
    al_avoid.start_longitudinal = o.longitudinal - avoiding_distance;
    al_avoid.id = getOriginalShiftLineUniqueId();
    al_avoid.object = o;
    avoid_lines.push_back(al_avoid);

    // The end_margin also has the purpose of preventing the return path from NOT being
    // triggered at the end point.
    const auto end_margin = 1.0;
    const auto return_remaining_distance =
      std::max(avoidance_data_.arclength_from_ego.back() - o.longitudinal - end_margin, 0.0);

    AvoidLine al_return;
    al_return.end_shift_length = 0.0;
    al_return.start_shift_length = shift_length;
    al_return.start_longitudinal = o.longitudinal + o.length;
    al_return.end_longitudinal =
      o.longitudinal + o.length + std::min(nominal_return_distance, return_remaining_distance);
    al_return.id = getOriginalShiftLineUniqueId();
    al_return.object = o;
    avoid_lines.push_back(al_return);

    DEBUG_PRINT(
      "object is set: avoid_shift = %f, return_shift = %f, dist = (avoidStart: %3.3f, avoidEnd: "
      "%3.3f, returnEnd: %3.3f), avoiding_dist = (nom:%f, res:%f), avoid_margin = %f, return_dist "
      "= %f",
      avoiding_shift, return_shift, al_avoid.start_longitudinal, al_avoid.end_longitudinal,
      al_return.end_longitudinal, nominal_avoid_distance, avoiding_distance, avoid_margin,
      nominal_return_distance);
    avoidance_debug_msg.allow_avoidance = true;
    avoidance_debug_msg_array.push_back(avoidance_debug_msg);
  }

  debug_avoidance_initializer_for_shift_line_ = std::move(avoidance_debug_msg_array);
  debug_avoidance_initializer_for_shift_line_time_ = clock_->now();
  fillAdditionalInfoFromLongitudinal(avoid_lines);

  return avoid_lines;
}

AvoidLineArray AvoidanceModule::fillAdditionalInfo(const AvoidLineArray & shift_lines) const
{
  if (shift_lines.empty()) {
    return shift_lines;
  }

  auto out_points = shift_lines;

  const auto & path = avoidance_data_.reference_path;
  const auto arclength = avoidance_data_.arclength_from_ego;

  // calc longitudinal
  for (auto & sl : out_points) {
    sl.start_idx = findNearestIndex(path.points, sl.start.position);
    sl.start_longitudinal = arclength.at(sl.start_idx);
    sl.end_idx = findNearestIndex(path.points, sl.end.position);
    sl.end_longitudinal = arclength.at(sl.end_idx);
  }

  // sort by longitudinal
  std::sort(out_points.begin(), out_points.end(), [](auto a, auto b) {
    return a.end_longitudinal < b.end_longitudinal;
  });

  // calc relative lateral length
  out_points.front().start_shift_length = getCurrentBaseShift();
  for (size_t i = 1; i < shift_lines.size(); ++i) {
    out_points.at(i).start_shift_length = shift_lines.at(i - 1).end_shift_length;
  }

  return out_points;
}
AvoidLine AvoidanceModule::fillAdditionalInfo(const AvoidLine & shift_line) const
{
  const auto ret = fillAdditionalInfo(AvoidLineArray{shift_line});
  return ret.front();
}

void AvoidanceModule::fillAdditionalInfoFromPoint(AvoidLineArray & shift_lines) const
{
  if (shift_lines.empty()) {
    return;
  }

  const auto & path = avoidance_data_.reference_path;
  const auto arclength = util::calcPathArcLengthArray(path);
  const auto dist_path_front_to_ego =
    calcSignedArcLength(path.points, 0, avoidance_data_.ego_closest_path_index);

  // calc longitudinal
  for (auto & sl : shift_lines) {
    sl.start_idx = findNearestIndex(path.points, sl.start.position);
    sl.start_longitudinal = arclength.at(sl.start_idx) - dist_path_front_to_ego;
    sl.end_idx = findNearestIndex(path.points, sl.end.position);
    sl.end_longitudinal = arclength.at(sl.end_idx) - dist_path_front_to_ego;
  }
}

void AvoidanceModule::fillAdditionalInfoFromLongitudinal(AvoidLineArray & shift_lines) const
{
  const auto & path = avoidance_data_.reference_path;
  const auto arclength = util::calcPathArcLengthArray(path);
  const auto path_front_to_ego =
    calcSignedArcLength(path.points, 0, avoidance_data_.ego_closest_path_index);

  for (auto & sl : shift_lines) {
    sl.start_idx = findPathIndexFromArclength(arclength, sl.start_longitudinal + path_front_to_ego);
    sl.start = path.points.at(sl.start_idx).point.pose;
    sl.end_idx = findPathIndexFromArclength(arclength, sl.end_longitudinal + path_front_to_ego);
    sl.end = path.points.at(sl.end_idx).point.pose;
  }
}
/*
 * combineRawShiftLinesWithUniqueCheck
 *
 * Combine points A into B. If shift_line of A which has same object_id and
 * similar shape is already in B, it will not be added into B.
 */
AvoidLineArray AvoidanceModule::combineRawShiftLinesWithUniqueCheck(
  const AvoidLineArray & base_lines, const AvoidLineArray & added_lines) const
{
  // TODO(Horibe) parametrize
  const auto isSimilar = [](const AvoidLine & a, const AvoidLine & b) {
    using tier4_autoware_utils::calcDistance2d;
    if (calcDistance2d(a.start, b.start) > 1.0) {
      return false;
    }
    if (calcDistance2d(a.end, b.end) > 1.0) {
      return false;
    }
    if (std::abs(a.end_shift_length - b.end_shift_length) > 0.5) {
      return false;
    }
    return true;
  };
  const auto hasSameObjectId = [](const auto & a, const auto & b) {
    return a.object.object.object_id == b.object.object.object_id;
  };

  auto combined = base_lines;  // initialized
  for (const auto & added_line : added_lines) {
    bool skip = false;

    for (const auto & base_line : base_lines) {
      if (hasSameObjectId(added_line, base_line) && isSimilar(added_line, base_line)) {
        skip = true;
        break;
      }
    }
    if (!skip) {
      combined.push_back(added_line);
    }
  }

  return combined;
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
      const auto i_shift = lerpShiftLengthOnArc(arcs.at(i), al);

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
  const auto current_shift = getCurrentLinearShift();
  for (size_t i = 0; i <= avoidance_data_.ego_closest_path_index; ++i) {
    sl.shift_line.at(i) = current_shift;
    sl.shift_line_grad.at(i) = 0.0;
  }

  // If the shift point does not have an associated object,
  // use previous value.
  for (size_t i = 1; i < N; ++i) {
    bool has_object = false;
    for (const auto & al : avoid_lines) {
      if (al.start_idx < i && i < al.end_idx) {
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

AvoidLineArray AvoidanceModule::extractShiftLinesFromLine(ShiftLineData & shift_line_data) const
{
  const auto & path = avoidance_data_.reference_path;
  const auto & arcs = avoidance_data_.arclength_from_ego;
  const auto N = path.points.size();

  auto & sl = shift_line_data;

  const auto getBwdGrad = [&](const size_t i) {
    if (i == 0) {
      return sl.shift_line_grad.at(i);
    }
    const double ds = arcs.at(i) - arcs.at(i - 1);
    if (ds < 1.0e-5) {
      return sl.shift_line_grad.at(i);
    }  // use theoretical value when ds is too small.
    return (sl.shift_line.at(i) - sl.shift_line.at(i - 1)) / ds;
  };

  const auto getFwdGrad = [&](const size_t i) {
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
    sl.forward_grad.at(i) = getFwdGrad(i);
    sl.backward_grad.at(i) = getBwdGrad(i);
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
  return merged_avoid_lines;
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
    al.parent_ids = calcParentIds(raw_shift_lines, al);
  }

  // sort by distance from ego.
  alignShiftLinesOrder(merged_shift_lines);

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

std::vector<size_t> AvoidanceModule::calcParentIds(
  const AvoidLineArray & parent_candidates, const AvoidLine & child) const
{
  // Get the ID of the original AP whose transition area overlaps with the given AP,
  // and set it to the parent id.
  std::set<uint64_t> ids;
  for (const auto & al : parent_candidates) {
    const auto p_s = al.start_longitudinal;
    const auto p_e = al.end_longitudinal;
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
    for (const auto & al_local : parent_candidates) {
      if (al_local.object.object.object_id == al.object.object.object_id) {
        ids.insert(al_local.id);
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
AvoidLineArray AvoidanceModule::trimShiftLine(
  const AvoidLineArray & shift_lines, DebugData & debug) const
{
  if (shift_lines.empty()) {
    return shift_lines;
  }

  AvoidLineArray sl_array_trimmed = shift_lines;

  // sort shift points from front to back.
  alignShiftLinesOrder(sl_array_trimmed);

  // - Combine avoid points that have almost same gradient.
  // this is to remove the noise.
  {
    const auto CHANGE_SHIFT_THRESHOLD_FOR_NOISE = 0.1;
    trimSimilarGradShiftLine(sl_array_trimmed, CHANGE_SHIFT_THRESHOLD_FOR_NOISE);
    debug.trim_similar_grad_shift = sl_array_trimmed;
    printShiftLines(sl_array_trimmed, "after trim_similar_grad_shift");
  }

  // - Quantize the shift length to reduce the shift point noise
  // This is to remove the noise coming from detection accuracy, interpolation, resampling, etc.
  {
    constexpr double QUANTIZATION_DISTANCE = 0.2;
    quantizeShiftLine(sl_array_trimmed, QUANTIZATION_DISTANCE);
    printShiftLines(sl_array_trimmed, "after sl_array_trimmed");
    debug.quantized = sl_array_trimmed;
  }

  // - Change the shift length to the previous one if the deviation is small.
  {
    // constexpr double SHIFT_DIFF_THRES = 0.5;
    // trimSmallShiftLine(sl_array_trimmed, SHIFT_DIFF_THRES);
    debug.trim_small_shift = sl_array_trimmed;
    printShiftLines(sl_array_trimmed, "after trim_small_shift");
  }

  // - Combine avoid points that have almost same gradient (again)
  {
    const auto CHANGE_SHIFT_THRESHOLD = 0.2;
    trimSimilarGradShiftLine(sl_array_trimmed, CHANGE_SHIFT_THRESHOLD);
    debug.trim_similar_grad_shift_second = sl_array_trimmed;
    printShiftLines(sl_array_trimmed, "after trim_similar_grad_shift_second");
  }

  // - trimTooSharpShift
  // Check if it is not too sharp for the return-to-center shift point.
  // If the shift is sharp, it is combined with the next shift point until it gets non-sharp.
  {
    trimSharpReturn(sl_array_trimmed);
    debug.trim_too_sharp_shift = sl_array_trimmed;
    printShiftLines(sl_array_trimmed, "after trimSharpReturn");
  }

  return sl_array_trimmed;
}

void AvoidanceModule::alignShiftLinesOrder(
  AvoidLineArray & shift_lines, const bool recalculate_start_length) const
{
  if (shift_lines.empty()) {
    return;
  }

  // sort shift points from front to back.
  std::sort(shift_lines.begin(), shift_lines.end(), [](auto a, auto b) {
    return a.end_longitudinal < b.end_longitudinal;
  });

  // calc relative length
  // NOTE: the input shift point must not have conflict range. Otherwise relative
  // length value will be broken.
  if (recalculate_start_length) {
    shift_lines.front().start_shift_length = getCurrentLinearShift();
    for (size_t i = 1; i < shift_lines.size(); ++i) {
      shift_lines.at(i).start_shift_length = shift_lines.at(i - 1).end_shift_length;
    }
  }
}

void AvoidanceModule::quantizeShiftLine(AvoidLineArray & shift_lines, const double interval) const
{
  if (interval < 1.0e-5) {
    return;  // no need to process
  }

  for (auto & sl : shift_lines) {
    sl.end_shift_length = std::round(sl.end_shift_length / interval) * interval;
  }

  alignShiftLinesOrder(shift_lines);
}

void AvoidanceModule::trimSmallShiftLine(
  AvoidLineArray & shift_lines, const double shift_diff_thres) const
{
  AvoidLineArray shift_lines_orig = shift_lines;
  shift_lines.clear();

  shift_lines.push_back(shift_lines_orig.front());  // Take the first one anyway (think later)

  for (size_t i = 1; i < shift_lines_orig.size(); ++i) {
    auto sl_now = shift_lines_orig.at(i);
    const auto sl_prev = shift_lines.back();
    const auto shift_diff = sl_now.end_shift_length - sl_prev.end_shift_length;

    auto sl_modified = sl_now;

    // remove the shift point if the length is almost same as the previous one.
    if (std::abs(shift_diff) < shift_diff_thres) {
      sl_modified.end_shift_length = sl_prev.end_shift_length;
      sl_modified.start_shift_length = sl_prev.end_shift_length;
      DEBUG_PRINT(
        "i = %lu, relative shift = %f is small. set with relative shift = 0.", i, shift_diff);
    } else {
      DEBUG_PRINT("i = %lu, shift = %f is large. take this one normally.", i, shift_diff);
    }

    shift_lines.push_back(sl_modified);
  }

  alignShiftLinesOrder(shift_lines);

  DEBUG_PRINT("size %lu -> %lu", shift_lines_orig.size(), shift_lines.size());
}

void AvoidanceModule::trimSimilarGradShiftLine(
  AvoidLineArray & avoid_lines, const double change_shift_dist_threshold) const
{
  AvoidLineArray avoid_lines_orig = avoid_lines;
  avoid_lines.clear();

  avoid_lines.push_back(avoid_lines_orig.front());  // Take the first one anyway (think later)

  // Save the points being merged. When merging consecutively, also check previously merged points.
  AvoidLineArray being_merged_points;

  for (size_t i = 1; i < avoid_lines_orig.size(); ++i) {
    const auto al_now = avoid_lines_orig.at(i);
    const auto al_prev = avoid_lines.back();

    being_merged_points.push_back(al_prev);  // This point is about to be merged.

    auto combined_al = al_prev;
    setEndData(
      combined_al, al_now.end_shift_length, al_now.end, al_now.end_idx, al_now.end_longitudinal);
    combined_al.parent_ids = concatParentIds(combined_al.parent_ids, al_prev.parent_ids);

    const auto has_large_length_change = [&]() {
      for (const auto & original : being_merged_points) {
        const auto longitudinal = original.end_longitudinal - combined_al.start_longitudinal;
        const auto new_length =
          combined_al.getGradient() * longitudinal + combined_al.start_shift_length;
        const bool has_large_change =
          std::abs(new_length - original.end_shift_length) > change_shift_dist_threshold;

        DEBUG_PRINT(
          "original.end_shift_length: %f, original.end_longitudinal: %f, "
          "combined_al.start_longitudinal: "
          "%f, combined_al.Gradient: %f, new_length: %f, has_large_change: %d",
          original.end_shift_length, original.end_longitudinal, combined_al.start_longitudinal,
          combined_al.getGradient(), new_length, has_large_change);

        if (std::abs(new_length - original.end_shift_length) > change_shift_dist_threshold) {
          return true;
        }
      }
      return false;
    }();

    if (has_large_length_change) {
      // If this point is merged with the previous points, it makes a large changes.
      // Do not merge this.
      avoid_lines.push_back(al_now);
      being_merged_points.clear();
      DEBUG_PRINT("use this point. has_large_length_change = %d", has_large_length_change);
    } else {
      avoid_lines.back() = combined_al;  // Update the last points by merging the current point
      being_merged_points.push_back(al_prev);
      DEBUG_PRINT("trim! has_large_length_change = %d", has_large_length_change);
    }
  }

  alignShiftLinesOrder(avoid_lines);

  DEBUG_PRINT("size %lu -> %lu", avoid_lines_orig.size(), avoid_lines.size());
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
void AvoidanceModule::trimMomentaryReturn(AvoidLineArray & shift_lines) const
{
  const auto isZero = [](double v) { return std::abs(v) < 1.0e-5; };

  AvoidLineArray shift_lines_orig = shift_lines;
  shift_lines.clear();

  const double DISTANCE_AFTER_RETURN_THR = 5.0 * getNominalAvoidanceEgoSpeed();

  const auto & arclength = avoidance_data_.arclength_from_ego;

  const auto check_reduce_shift = [](const double now_length, const double prev_length) {
    const auto abs_shift_diff = std::abs(now_length) - std::abs(prev_length);
    const auto has_same_sign = (now_length * prev_length >= 0.0);
    const bool is_reduce_shift = (abs_shift_diff < 0.0 && has_same_sign);
    return is_reduce_shift;
  };

  for (size_t i = 0; i < shift_lines_orig.size(); ++i) {
    const auto sl_now = shift_lines_orig.at(i);
    const auto sl_prev_length =
      shift_lines.empty() ? getCurrentLinearShift() : shift_lines.back().end_shift_length;
    const auto abs_shift_diff = std::abs(sl_now.end_shift_length) - std::abs(sl_prev_length);
    const bool is_reduce_shift = check_reduce_shift(sl_now.end_shift_length, sl_prev_length);

    // Do nothing for non-reduce shift point
    if (!is_reduce_shift) {
      shift_lines.push_back(sl_now);
      DEBUG_PRINT(
        "i = %lu, not reduce shift. take this one.abs_shift_diff = %f, sl_now.length = %f, "
        "sl_prev_length = %f, sl_now.length * sl_prev_length = %f",
        i, abs_shift_diff, sl_now.end_shift_length, sl_prev_length,
        sl_now.end_shift_length * sl_prev_length);
      continue;
    }

    // The last point is out of target of this function.
    const bool is_last_sl = (i == shift_lines_orig.size() - 1);
    if (is_last_sl) {
      shift_lines.push_back(sl_now);
      DEBUG_PRINT("i = %lu, last shift. take this one.", i);
      continue;
    }

    // --- From here, the shift point is "return to center" or "straight". ---
    // -----------------------------------------------------------------------

    const auto sl_next = shift_lines_orig.at(i + 1);

    // there is no straight interval, combine them. ¯¯\/¯¯ -> ¯¯¯¯¯¯
    if (!isZero(sl_next.getRelativeLength())) {
      DEBUG_PRINT(
        "i = %lu, return-shift is detected, next shift_diff (%f) is nonzero. combine them. (skip "
        "next shift).",
        i, sl_next.getRelativeLength());
      auto sl_modified = sl_next;
      setStartData(
        sl_modified, sl_now.end_shift_length, sl_now.start, sl_now.start_idx,
        sl_now.start_longitudinal);
      sl_modified.parent_ids = concatParentIds(sl_modified.parent_ids, sl_now.parent_ids);
      shift_lines.push_back(sl_modified);
      ++i;  // skip next shift point
      continue;
    }

    // Find next shifting point, i.e.  ¯¯\____"/"¯¯
    //                               now ↑     ↑ target
    const auto next_avoid_idx = [&]() {
      for (size_t j = i + 1; j < shift_lines_orig.size(); ++j) {
        if (!isZero(shift_lines_orig.at(j).getRelativeLength())) {
          return j;
        }
      }
      return shift_lines_orig.size();
    }();

    // The straight distance lasts until end. take this one.
    // ¯¯\______
    if (next_avoid_idx == shift_lines_orig.size()) {
      shift_lines.push_back(sl_now);
      DEBUG_PRINT("i = %lu, back -> straight lasts until end. take this one.", i);
      continue;
    }

    const auto sl_next_avoid = shift_lines_orig.at(next_avoid_idx);
    const auto straight_distance = sl_next_avoid.start_longitudinal - sl_now.end_longitudinal;

    // The straight distance after "return to center" is long enough. take this one.
    // ¯¯\______/¯¯ (enough long straight line!)
    if (straight_distance > DISTANCE_AFTER_RETURN_THR) {
      shift_lines.push_back(sl_now);
      DEBUG_PRINT("i = %lu, back -> straight: distance is long. take this one", i);
      continue;
    }

    // From here, back to center and go straight, straight distance is too short.
    // ¯¯\______/¯¯ (short straight line!)

    const auto relative_shift = sl_next_avoid.end_shift_length - sl_now.end_shift_length;
    const auto avoid_distance = getNominalAvoidanceDistance(relative_shift);

    // Calculate start point from end point and avoidance distance.
    auto sl_next_modified = sl_next_avoid;
    sl_next_modified.start_shift_length = sl_prev_length;
    sl_next_modified.start_longitudinal =
      std::max(sl_next_avoid.end_longitudinal - avoid_distance, sl_now.start_longitudinal);
    sl_next_modified.start_idx =
      findPathIndexFromArclength(arclength, sl_next_modified.start_longitudinal);
    sl_next_modified.start =
      avoidance_data_.reference_path.points.at(sl_next_modified.start_idx).point.pose;
    sl_next_modified.parent_ids = calcParentIds(current_raw_shift_lines_, sl_next_modified);

    // Straight shift point
    if (sl_next_modified.start_idx > sl_now.start_idx) {  // the case where a straight route exists.
      auto sl_now_modified = sl_now;
      sl_now_modified.start_shift_length = sl_prev_length;
      setEndData(
        sl_now_modified, sl_prev_length, sl_next_modified.start, sl_next_modified.start_idx,
        sl_next_modified.start_longitudinal);
      sl_now_modified.parent_ids = calcParentIds(current_raw_shift_lines_, sl_now_modified);
      shift_lines.push_back(sl_now_modified);
    }
    shift_lines.push_back(sl_next_modified);

    DEBUG_PRINT(
      "i = %lu, find remove target!: next_avoid_idx = %lu, shift length = (now: %f, prev: %f, "
      "next_avoid: %f, next_mod: %f).",
      i, next_avoid_idx, sl_now.end_shift_length, sl_prev_length, sl_next_avoid.end_shift_length,
      sl_next_modified.end_shift_length);

    i = next_avoid_idx;  // skip shifting until next_avoid_idx.
  }

  alignShiftLinesOrder(shift_lines);

  DEBUG_PRINT("trimMomentaryReturn: size %lu -> %lu", shift_lines_orig.size(), shift_lines.size());
}

void AvoidanceModule::trimSharpReturn(AvoidLineArray & shift_lines) const
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
    setEndData(
      sl_modified, sl_next.end_shift_length, sl_next.end, sl_next.end_idx,
      sl_next.end_longitudinal);
    sl_modified.parent_ids = concatParentIds(sl_modified.parent_ids, sl_now.parent_ids);
    return sl_modified;
  };

  // Check if the merged shift has a conflict with the original shifts.
  const auto hasViolation = [this](const auto & combined, const auto & combined_src) {
    constexpr auto VIOLATION_SHIFT_THR = 0.3;
    for (const auto & sl : combined_src) {
      const auto combined_shift = lerpShiftLengthOnArc(sl.end_longitudinal, combined);
      if (
        sl.end_shift_length < -0.01 && combined_shift > sl.end_shift_length + VIOLATION_SHIFT_THR) {
        return true;
      }
      if (
        sl.end_shift_length > 0.01 && combined_shift < sl.end_shift_length - VIOLATION_SHIFT_THR) {
        return true;
      }
    }
    return false;
  };

  // check for all shift points
  for (size_t i = 0; i < shift_lines_orig.size(); ++i) {
    auto sl_now = shift_lines_orig.at(i);
    sl_now.start_shift_length =
      shift_lines.empty() ? getCurrentLinearShift() : shift_lines.back().end_shift_length;

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
        const auto nominal_distance = getNominalAvoidanceDistance(sl_combined.getRelativeLength());
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

  alignShiftLinesOrder(shift_lines);

  DEBUG_PRINT("trimSharpReturn: size %lu -> %lu", shift_lines_orig.size(), shift_lines.size());
}

void AvoidanceModule::trimTooSharpShift(AvoidLineArray & avoid_lines) const
{
  if (avoid_lines.empty()) {
    return;
  }

  AvoidLineArray avoid_lines_orig = avoid_lines;
  avoid_lines.clear();

  const auto isInJerkLimit = [this](const auto & al) {
    const auto required_jerk = path_shifter_.calcJerkFromLatLonDistance(
      al.getRelativeLength(), al.getRelativeLongitudinal(), getSharpAvoidanceEgoSpeed());
    return std::fabs(required_jerk) < parameters_->max_lateral_jerk;
  };

  for (size_t i = 0; i < avoid_lines_orig.size(); ++i) {
    auto al_now = avoid_lines_orig.at(i);

    if (isInJerkLimit(al_now)) {
      avoid_lines.push_back(al_now);
      continue;
    }

    DEBUG_PRINT("over jerk is detected: i = %lu", i);
    printShiftLines(AvoidLineArray{al_now}, "points with over jerk");

    // The avoidance_point_now exceeds jerk limit, so merge it with the next avoidance_point.
    for (size_t j = i + 1; j < avoid_lines_orig.size(); ++j) {
      auto al_next = avoid_lines_orig.at(j);
      setEndData(
        al_now, al_next.end_shift_length, al_next.end, al_next.end_idx, al_next.end_longitudinal);
      if (isInJerkLimit(al_now)) {
        avoid_lines.push_back(al_now);
        DEBUG_PRINT("merge finished. i = %lu, j = %lu", i, j);
        i = j;  // skip check until j index.
        break;
      }
    }
  }

  alignShiftLinesOrder(avoid_lines);

  DEBUG_PRINT("size %lu -> %lu", avoid_lines_orig.size(), avoid_lines.size());
}

/*
 * addReturnShiftLine
 *
 * Pick up the last shift point, which is the most farthest from ego, from the current candidate
 * avoidance points and registered points in the shifter. If the last shift length of the point is
 * non-zero, add a return-shift to center line from the point. If there is no shift point in
 * candidate avoidance points nor registered points, and base_shift > 0, add a return-shift to
 * center line from ego.
 */
void AvoidanceModule::addReturnShiftLineFromEgo(
  AvoidLineArray & sl_candidates, AvoidLineArray & current_raw_shift_lines) const
{
  constexpr double ep = 1.0e-3;
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
      alignShiftLinesOrder(sl_candidates, false);
      last_sl = sl_candidates.back();
    }

    // avoidance points: No, shift points: Yes -> select last shift point.
    if (!has_candidate_point && has_registered_point) {
      last_sl = fillAdditionalInfo(AvoidLine{path_shifter_.getLastShiftLine().get()});
    }

    // avoidance points: Yes, shift points: Yes -> select the last one from both.
    if (has_candidate_point && has_registered_point) {
      alignShiftLinesOrder(sl_candidates, false);
      const auto & al = sl_candidates.back();
      const auto & sl = fillAdditionalInfo(AvoidLine{path_shifter_.getLastShiftLine().get()});
      last_sl = (sl.end_longitudinal > al.end_longitudinal) ? sl : al;
    }

    // avoidance points: No, shift points: No -> set the ego position to the last shift point
    // so that the return-shift will be generated from ego position.
    if (!has_candidate_point && !has_registered_point) {
      last_sl.end = getEgoPose().pose;
      last_sl.end_idx = avoidance_data_.ego_closest_path_index;
      last_sl.end_shift_length = getCurrentBaseShift();
    }
  }
  printShiftLines(ShiftLineArray{last_sl}, "last shift point");

  // There already is a shift point candidates to go back to center line, but it could be too sharp
  // due to detection noise or timing.
  // Here the return-shift from ego is added for the in case.
  if (std::fabs(last_sl.end_shift_length) < RETURN_SHIFT_THRESHOLD) {
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

    // set the return-shift from ego.
    DEBUG_PRINT(
      "return shift already exists, but they are all candidates. Add return shift for overwrite.");
    last_sl.end = getEgoPose().pose;
    last_sl.end_idx = avoidance_data_.ego_closest_path_index;
    last_sl.end_shift_length = current_base_shift;
  }

  const auto & arclength_from_ego = avoidance_data_.arclength_from_ego;

  const auto nominal_prepare_distance = getNominalPrepareDistance();
  const auto nominal_avoid_distance = getNominalAvoidanceDistance(last_sl.end_shift_length);

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
    al.end_idx = findPathIndexFromArclength(arclength_from_ego, prepare_distance_scaled);
    al.end = avoidance_data_.reference_path.points.at(al.end_idx).point.pose;
    al.end_longitudinal = arclength_from_ego.at(al.end_idx);
    al.end_shift_length = last_sl.end_shift_length;
    al.start_shift_length = last_sl.end_shift_length;
    sl_candidates.push_back(al);
    printShiftLines(AvoidLineArray{al}, "prepare for return");
    debug_data_.extra_return_shift.push_back(al);

    // TODO(Horibe) think how to store the current object
    current_raw_shift_lines.push_back(al);
  }

  // shift point for return to center line
  {
    AvoidLine al;
    al.id = getOriginalShiftLineUniqueId();
    al.start_idx = findPathIndexFromArclength(arclength_from_ego, prepare_distance_scaled);
    al.start = avoidance_data_.reference_path.points.at(al.start_idx).point.pose;
    al.start_longitudinal = arclength_from_ego.at(al.start_idx);
    al.end_idx = findPathIndexFromArclength(
      arclength_from_ego, prepare_distance_scaled + avoid_distance_scaled);
    al.end = avoidance_data_.reference_path.points.at(al.end_idx).point.pose;
    al.end_longitudinal = arclength_from_ego.at(al.end_idx);
    al.end_shift_length = 0.0;
    al.start_shift_length = last_sl.end_shift_length;
    sl_candidates.push_back(al);
    printShiftLines(AvoidLineArray{al}, "return point");
    debug_data_.extra_return_shift = AvoidLineArray{al};

    // TODO(Horibe) think how to store the current object
    current_raw_shift_lines.push_back(al);
  }

  DEBUG_PRINT("Return Shift is added.");
}

double AvoidanceModule::getRightShiftBound() const
{
  // TODO(Horibe) write me. Real lane boundary must be considered here.
  return -parameters_->max_right_shift_length;
}

double AvoidanceModule::getLeftShiftBound() const
{
  // TODO(Horibe) write me. Real lane boundary must be considered here.
  return parameters_->max_left_shift_length;
}

// TODO(murooka) judge when and which way to extend drivable area. current implementation is keep
// extending during avoidance module
// TODO(murooka) freespace during turning in intersection where there is no neighbour lanes
// NOTE: Assume that there is no situation where there is an object in the middle lane of more than
// two lanes since which way to avoid is not obvious
void AvoidanceModule::generateExtendedDrivableArea(ShiftedPath * shifted_path) const
{
  const auto has_same_lane =
    [](const lanelet::ConstLanelets lanes, const lanelet::ConstLanelet & lane) {
      if (lanes.empty()) return false;
      const auto has_same = [&](const auto & ll) { return ll.id() == lane.id(); };
      return std::find_if(lanes.begin(), lanes.end(), has_same) != lanes.end();
    };

  const auto & route_handler = planner_data_->route_handler;
  const auto & current_lanes = avoidance_data_.current_lanelets;
  const auto & enable_opposite = parameters_->enable_avoidance_over_opposite_direction;
  std::vector<DrivableLanes> drivable_lanes;

  for (const auto & current_lane : current_lanes) {
    DrivableLanes current_drivable_lanes;
    current_drivable_lanes.left_lane = current_lane;
    current_drivable_lanes.right_lane = current_lane;

    if (!parameters_->enable_avoidance_over_same_direction) {
      drivable_lanes.push_back(current_drivable_lanes);
      continue;
    }

    // get left side lane
    const lanelet::ConstLanelets all_left_lanelets =
      route_handler->getAllLeftSharedLinestringLanelets(current_lane, enable_opposite, true);
    if (!all_left_lanelets.empty()) {
      current_drivable_lanes.left_lane = all_left_lanelets.back();  // leftmost lanelet

      for (int i = all_left_lanelets.size() - 2; i >= 0; --i) {
        current_drivable_lanes.middle_lanes.push_back(all_left_lanelets.at(i));
      }
    }

    // get right side lane
    const lanelet::ConstLanelets all_right_lanelets =
      route_handler->getAllRightSharedLinestringLanelets(current_lane, enable_opposite, true);
    if (!all_right_lanelets.empty()) {
      current_drivable_lanes.right_lane = all_right_lanelets.back();  // rightmost lanelet
      if (current_drivable_lanes.left_lane.id() != current_lane.id()) {
        current_drivable_lanes.middle_lanes.push_back(current_lane);
      }

      for (size_t i = 0; i < all_right_lanelets.size() - 1; ++i) {
        current_drivable_lanes.middle_lanes.push_back(all_right_lanelets.at(i));
      }
    }

    // 2. when there are multiple turning lanes whose previous lanelet is the same in
    // intersection
    const lanelet::ConstLanelets next_lanes_from_intersection = std::invoke(
      [&route_handler](const lanelet::ConstLanelet & lane) {
        if (!lane.hasAttribute("turn_direction")) {
          return lanelet::ConstLanelets{};
        }

        // get previous lane, and return false if previous lane does not exist
        lanelet::ConstLanelets prev_lanes;
        if (!route_handler->getPreviousLaneletsWithinRoute(lane, &prev_lanes)) {
          return lanelet::ConstLanelets{};
        }

        lanelet::ConstLanelets next_lanes;
        for (const auto & prev_lane : prev_lanes) {
          const auto next_lanes_from_prev = route_handler->getNextLanelets(prev_lane);
          next_lanes.reserve(next_lanes.size() + next_lanes_from_prev.size());
          next_lanes.insert(
            next_lanes.end(), next_lanes_from_prev.begin(), next_lanes_from_prev.end());
        }
        return next_lanes;
      },
      current_lane);

    // 2.1 look for neighbour lane, where end line of the lane is connected to end line of the
    // original lane
    for (const auto & next_lane : next_lanes_from_intersection) {
      if (current_lane.id() == next_lane.id()) {
        continue;
      }
      constexpr double epsilon = 1e-5;
      const auto & next_left_back_point_2d = next_lane.leftBound2d().back().basicPoint();
      const auto & next_right_back_point_2d = next_lane.rightBound2d().back().basicPoint();
      const auto & orig_left_back_point_2d = current_lane.leftBound2d().back().basicPoint();
      const auto & orig_right_back_point_2d = current_lane.rightBound2d().back().basicPoint();

      if ((next_right_back_point_2d - orig_left_back_point_2d).norm() < epsilon) {
        current_drivable_lanes.left_lane = next_lane;
        if (
          current_drivable_lanes.right_lane.id() != current_lane.id() &&
          !has_same_lane(current_drivable_lanes.middle_lanes, current_lane)) {
          current_drivable_lanes.middle_lanes.push_back(current_lane);
        }
      } else if (
        (next_left_back_point_2d - orig_right_back_point_2d).norm() < epsilon &&
        !has_same_lane(current_drivable_lanes.middle_lanes, current_lane)) {
        current_drivable_lanes.right_lane = next_lane;
        if (current_drivable_lanes.left_lane.id() != current_lane.id()) {
          current_drivable_lanes.middle_lanes.push_back(current_lane);
        }
      }
    }
    drivable_lanes.push_back(current_drivable_lanes);
  }

  drivable_lanes = util::expandLanelets(
    drivable_lanes, parameters_->drivable_area_left_bound_offset,
    parameters_->drivable_area_right_bound_offset, {"road_border"});

  {
    const auto & p = planner_data_->parameters;
    shifted_path->path.drivable_area = util::generateDrivableArea(
      shifted_path->path, drivable_lanes, p.drivable_area_resolution, p.vehicle_length,
      planner_data_);
  }
}

void AvoidanceModule::modifyPathVelocityToPreventAccelerationOnAvoidance(ShiftedPath & path)
{
  const auto ego_idx = avoidance_data_.ego_closest_path_index;
  const auto N = path.shift_length.size();

  if (!ego_velocity_starting_avoidance_ptr_) {
    ego_velocity_starting_avoidance_ptr_ = std::make_shared<double>(getEgoSpeed());
  }

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

  constexpr auto NO_ACCEL_TIME_THR = 3.0;

  // update ego velocity if the shift point is far
  const auto s_from_ego = avoidance_data_.arclength_from_ego.at(target_idx) -
                          avoidance_data_.arclength_from_ego.at(ego_idx);
  const auto t_from_ego = s_from_ego / std::max(getEgoSpeed(), 1.0);
  if (t_from_ego > NO_ACCEL_TIME_THR) {
    *ego_velocity_starting_avoidance_ptr_ = getEgoSpeed();
  }

  // calc index and velocity to NO_ACCEL_TIME_THR
  const auto v0 = *ego_velocity_starting_avoidance_ptr_;
  auto vmax = 0.0;
  size_t insert_idx = ego_idx;
  for (size_t i = ego_idx; i <= target_idx; ++i) {
    const auto s =
      avoidance_data_.arclength_from_ego.at(target_idx) - avoidance_data_.arclength_from_ego.at(i);
    const auto t = s / std::max(v0, 1.0);
    if (t < NO_ACCEL_TIME_THR) {
      insert_idx = i;
      vmax = std::max(
        parameters_->min_avoidance_speed_for_acc_prevention,
        std::sqrt(v0 * v0 + 2.0 * s * parameters_->max_avoidance_acceleration));
      break;
    }
  }

  // apply velocity limit
  constexpr size_t V_LIM_APPLY_IDX_MARGIN = 0;
  for (size_t i = insert_idx + V_LIM_APPLY_IDX_MARGIN; i < N; ++i) {
    path.path.points.at(i).point.longitudinal_velocity_mps =
      std::min(path.path.points.at(i).point.longitudinal_velocity_mps, static_cast<float>(vmax));
  }

  DEBUG_PRINT(
    "s: %f, t: %f, v0: %f, a: %f, vmax: %f, ego_i: %lu, target_i: %lu", s_from_ego, t_from_ego, v0,
    parameters_->max_avoidance_acceleration, vmax, ego_idx, target_idx);
}

// TODO(Horibe) clean up functions: there is a similar code in util as well.
PathWithLaneId AvoidanceModule::calcCenterLinePath(
  const std::shared_ptr<const PlannerData> & planner_data, const PoseStamped & pose) const
{
  const auto & p = planner_data->parameters;
  const auto & route_handler = planner_data->route_handler;

  PathWithLaneId centerline_path;

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

  printShiftLines(path_shifter_.getShiftLines(), "path_shifter_.getShiftLines()");
  printShiftLines(registered_raw_shift_lines_, "registered_raw_shift_lines_");

  const auto extra_margin = 10.0;  // Since distance does not consider arclength, but just line.
  const auto backward_length =
    std::max(p.backward_path_length, longest_dist_to_shift_line + extra_margin);

  DEBUG_PRINT(
    "p.backward_path_length = %f, longest_dist_to_shift_line = %f, backward_length = %f",
    p.backward_path_length, longest_dist_to_shift_line, backward_length);

  const lanelet::ConstLanelets current_lanes =
    util::calcLaneAroundPose(route_handler, pose.pose, p.forward_path_length, backward_length);
  centerline_path = util::getCenterLinePath(
    *route_handler, current_lanes, pose.pose, backward_length, p.forward_path_length, p);

  // for debug: check if the path backward distance is same as the desired length.
  // {
  //   const auto back_to_ego = motion_utils::calcSignedArcLength(
  //     centerline_path.points, centerline_path.points.front().point.pose.position,
  //     getEgoPosition());
  //   RCLCPP_INFO(getLogger(), "actual back_to_ego distance = %f", back_to_ego);
  // }

  centerline_path.header = route_handler->getRouteHeader();

  return centerline_path;
}

boost::optional<AvoidLine> AvoidanceModule::calcIntersectionShiftLine(
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

  const auto intersection_shift_line = [&]() {
    boost::optional<AvoidLine> shift_line{};
    if (!intersection_point) {
      RCLCPP_INFO(getLogger(), "No intersection.");
      return shift_line;
    }

    const double ego_to_intersection_dist = calcSignedArcLength(
      data.reference_path.points, getEgoPosition(), intersection_point->point.pose.position);

    if (ego_to_intersection_dist <= 5.0) {
      RCLCPP_INFO(getLogger(), "No enough margin to intersection.");
      return shift_line;
    }

    // Search obstacles around the intersection.
    // If it exists, use its shift distance on the intersection.
    constexpr double intersection_obstacle_check_dist = 10.0;
    constexpr double intersection_shift_margin = 1.0;

    double shift_length = 0.0;  // default (no obstacle) is zero.
    for (const auto & obj : avoidance_data_.target_objects) {
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

    AvoidLine p{};
    p.end_shift_length = shift_length;
    p.start =
      calcBehindPose(intersection_point->point.pose.position, intersection_obstacle_check_dist);
    p.end = intersection_point->point.pose;
    shift_line = p;
    return shift_line;
  }();

  return intersection_shift_line;
}

BehaviorModuleOutput AvoidanceModule::plan()
{
  DEBUG_PRINT("AVOIDANCE plan");

  const auto shift_lines = calcShiftLines(current_raw_shift_lines_, debug_data_);

  const auto new_shift_lines = findNewShiftLine(shift_lines, path_shifter_);

  /**
   * Has new shift point?
   *   Yes -> Is it approved?
   *       Yes -> add the shift point.
   *       No  -> set approval_handler to WAIT_APPROVAL state.
   *   No -> waiting approval?
   *       Yes -> clear WAIT_APPROVAL state.
   *       No  -> do nothing.
   */
  if (new_shift_lines) {
    debug_data_.new_shift_lines = *new_shift_lines;
    DEBUG_PRINT("new_shift_lines size = %lu", new_shift_lines->size());
    printShiftLines(*new_shift_lines, "new_shift_lines");
    int i = new_shift_lines->size() - 1;
    for (; i > 0; i--) {
      if (fabs(new_shift_lines->at(i).getRelativeLength()) < 0.01) {
        continue;
      } else {
        break;
      }
    }
    if (new_shift_lines->at(i).getRelativeLength() > 0.0) {
      removePreviousRTCStatusRight();
    } else if (new_shift_lines->at(i).getRelativeLength() < 0.0) {
      removePreviousRTCStatusLeft();
    } else {
      RCLCPP_WARN_STREAM(getLogger(), "Direction is UNKNOWN");
    }
    addShiftLineIfApproved(*new_shift_lines);
  } else if (isWaitingApproval()) {
    clearWaitingApproval();
    removeCandidateRTCStatus();
  }

  // generate path with shift points that have been inserted.
  auto avoidance_path = generateAvoidancePath(path_shifter_);
  debug_data_.output_shift = avoidance_path.shift_length;

  // Drivable area generation.
  generateExtendedDrivableArea(&avoidance_path);

  // modify max speed to prevent acceleration in avoidance maneuver.
  modifyPathVelocityToPreventAccelerationOnAvoidance(avoidance_path);

  // post processing
  {
    postProcess(path_shifter_);  // remove old shift points
    prev_output_ = avoidance_path;
    prev_linear_shift_path_ = toShiftedPath(avoidance_data_.reference_path);
    path_shifter_.generate(&prev_linear_shift_path_, true, SHIFT_TYPE::LINEAR);
    prev_reference_ = avoidance_data_.reference_path;
    if (parameters_->publish_debug_marker) {
      setDebugData(avoidance_data_, path_shifter_, debug_data_);
    } else {
      debug_marker_.markers.clear();
    }
  }

  BehaviorModuleOutput output;
  output.turn_signal_info = calcTurnSignalInfo(avoidance_path);
  // sparse resampling for computational cost
  {
    avoidance_path.path =
      util::resamplePathWithSpline(avoidance_path.path, parameters_->resample_interval_for_output);
  }
  output.path = std::make_shared<PathWithLaneId>(avoidance_path.path);

  const size_t ego_idx = findEgoIndex(output.path->points);
  util::clipPathLength(*output.path, ego_idx, planner_data_->parameters);

  DEBUG_PRINT("exit plan(): set prev output (back().lat = %f)", prev_output_.shift_length.back());

  updateRegisteredRTCStatus(avoidance_path.path);

  return output;
}

CandidateOutput AvoidanceModule::planCandidate() const
{
  DEBUG_PRINT("AVOIDANCE planCandidate start");
  CandidateOutput output;

  auto path_shifter = path_shifter_;
  auto debug_data = debug_data_;
  auto current_raw_shift_lines = current_raw_shift_lines_;

  const auto shift_lines = calcShiftLines(current_raw_shift_lines, debug_data);
  const auto new_shift_lines = findNewShiftLine(shift_lines, path_shifter);
  if (new_shift_lines) {
    addNewShiftLines(path_shifter, *new_shift_lines);
  }

  auto shifted_path = generateAvoidancePath(path_shifter);

  if (new_shift_lines) {  // clip from shift start index for visualize
    clipByMinStartIdx(*new_shift_lines, shifted_path.path);

    int i = new_shift_lines->size() - 1;
    for (; i > 0; i--) {
      if (fabs(new_shift_lines->at(i).getRelativeLength()) < 0.01) {
        continue;
      } else {
        break;
      }
    }
    output.lateral_shift = new_shift_lines->at(i).getRelativeLength();
    output.start_distance_to_path_change = new_shift_lines->front().start_longitudinal;
    output.finish_distance_to_path_change = new_shift_lines->back().end_longitudinal;

    const uint16_t steering_factor_direction = std::invoke([&output]() {
      if (output.lateral_shift > 0.0) {
        return SteeringFactor::LEFT;
      }
      return SteeringFactor::RIGHT;
    });
    steering_factor_interface_ptr_->updateSteeringFactor(
      {new_shift_lines->front().start, new_shift_lines->back().end},
      {output.start_distance_to_path_change, output.finish_distance_to_path_change},
      SteeringFactor::AVOIDANCE_PATH_CHANGE, steering_factor_direction, SteeringFactor::APPROACHING,
      "");
  }

  const size_t ego_idx = findEgoIndex(shifted_path.path.points);
  util::clipPathLength(shifted_path.path, ego_idx, planner_data_->parameters);

  output.path_candidate = shifted_path.path;

  return output;
}

BehaviorModuleOutput AvoidanceModule::planWaitingApproval()
{
  // we can execute the plan() since it handles the approval appropriately.
  BehaviorModuleOutput out = plan();
  const auto candidate = planCandidate();
  constexpr double threshold_to_update_status = -1.0e-03;
  if (candidate.start_distance_to_path_change > threshold_to_update_status) {
    updateCandidateRTCStatus(candidate);
    waitApproval();
  } else {
    clearWaitingApproval();
    removeCandidateRTCStatus();
  }
  out.path_candidate = std::make_shared<PathWithLaneId>(candidate.path_candidate);
  return out;
}

void AvoidanceModule::addShiftLineIfApproved(const AvoidLineArray & shift_lines)
{
  if (isActivated()) {
    DEBUG_PRINT("We want to add this shift point, and approved. ADD SHIFT POINT!");
    const size_t prev_size = path_shifter_.getShiftLinesSize();
    addNewShiftLines(path_shifter_, shift_lines);

    // register original points for consistency
    registerRawShiftLines(shift_lines);

    int i = shift_lines.size() - 1;
    for (; i > 0; i--) {
      if (fabs(shift_lines.at(i).getRelativeLength()) < 0.01) {
        continue;
      } else {
        break;
      }
    }

    if (shift_lines.at(i).getRelativeLength() > 0.0) {
      left_shift_array_.push_back({uuid_left_, shift_lines.front().start, shift_lines.back().end});
    } else if (shift_lines.at(i).getRelativeLength() < 0.0) {
      right_shift_array_.push_back(
        {uuid_right_, shift_lines.front().start, shift_lines.back().end});
    }

    uuid_left_ = generateUUID();
    uuid_right_ = generateUUID();
    candidate_uuid_ = generateUUID();

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
  ShiftLineArray future = toShiftLineArray(new_shift_lines);

  size_t min_start_idx = std::numeric_limits<size_t>::max();
  for (const auto & sl : new_shift_lines) {
    min_start_idx = std::min(min_start_idx, sl.start_idx);
  }

  const auto current_shift_lines = path_shifter.getShiftLines();

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
    } else {
      DEBUG_PRINT("sl.start_idx = %lu, no conflict. keep this one.", sl.start_idx);
      future.push_back(sl);
    }
  }

  path_shifter.setShiftLines(future);
}

boost::optional<AvoidLineArray> AvoidanceModule::findNewShiftLine(
  const AvoidLineArray & candidates, const PathShifter & shifter) const
{
  (void)shifter;

  if (candidates.empty()) {
    DEBUG_PRINT("shift candidates is empty. return None.");
    return {};
  }

  printShiftLines(candidates, "findNewShiftLine: candidates");

  // Retrieve the subsequent linear shift point from the given index point.
  const auto getShiftLineWithSubsequentStraight = [this, &candidates](size_t i) {
    AvoidLineArray subsequent{candidates.at(i)};
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

  const auto calcJerk = [this](const auto & al) {
    return path_shifter_.calcJerkFromLatLonDistance(
      al.getRelativeLength(), al.getRelativeLongitudinal(), getSharpAvoidanceEgoSpeed());
  };

  for (size_t i = 0; i < candidates.size(); ++i) {
    const auto & candidate = candidates.at(i);
    std::stringstream ss;
    ss << "i = " << i << ", id = " << candidate.id;
    const auto pfx = ss.str().c_str();

    if (prev_reference_.points.size() != prev_linear_shift_path_.path.points.size()) {
      throw std::logic_error("prev_reference_ and prev_linear_shift_path_ must have same size.");
    }

    // new shift points must exist in front of Ego
    // this value should be larger than -eps consider path shifter calculation error.
    const double eps = 0.01;
    if (candidate.start_longitudinal < -eps) {
      continue;
    }

    // TODO(Horibe): this code prohibits the changes on ego pose. Think later.
    // if (candidate.start_idx < avoidance_data_.ego_closest_path_index) {
    //   DEBUG_PRINT("%s, start_idx is behind ego. skip.", pfx);
    //   continue;
    // }

    const auto current_shift = prev_linear_shift_path_.shift_length.at(
      findNearestIndex(prev_reference_.points, candidate.end.position));

    // TODO(Horibe) test fails with this print. why?
    // DEBUG_PRINT("%s, shift current: %f, candidate: %f", pfx, current_shift,
    // candidate.end_shift_length);

    const auto new_point_threshold = parameters_->avoidance_execution_lateral_threshold;
    if (std::abs(candidate.end_shift_length - current_shift) > new_point_threshold) {
      if (calcJerk(candidate) > parameters_->max_lateral_jerk) {
        DEBUG_PRINT(
          "%s, Failed to find new shift: jerk limit over (%f).", pfx, calcJerk(candidate));
        break;
      }

      DEBUG_PRINT(
        "%s, New shift point is found!!! shift change: %f -> %f", pfx, current_shift,
        candidate.end_shift_length);
      return getShiftLineWithSubsequentStraight(i);
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
  return std::max(getEgoSpeed(), parameters_->min_nominal_avoidance_speed);
}
double AvoidanceModule::getSharpAvoidanceEgoSpeed() const
{
  return std::max(getEgoSpeed(), parameters_->min_sharp_avoidance_speed);
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
    shift_length, parameters_->nominal_lateral_jerk, getNominalAvoidanceEgoSpeed());

  return std::max(p->min_avoidance_distance, distance_by_jerk);
}

double AvoidanceModule::getSharpAvoidanceDistance(const double shift_length) const
{
  const auto & p = parameters_;
  const auto distance_by_jerk = path_shifter_.calcLongitudinalDistFromJerk(
    shift_length, parameters_->max_lateral_jerk, getSharpAvoidanceEgoSpeed());

  return std::max(p->min_avoidance_distance, distance_by_jerk);
}

double AvoidanceModule::getNominalPrepareDistance() const
{
  const auto & p = parameters_;
  const auto epsilon_m = 0.01;  // for floating error to pass "has_enough_distance" check.
  const auto nominal_distance = std::max(getEgoSpeed() * p->prepare_time, p->min_prepare_distance);
  return nominal_distance + epsilon_m;
}

ShiftedPath AvoidanceModule::generateAvoidancePath(PathShifter & path_shifter) const
{
  DEBUG_PRINT("path_shifter: base shift = %f", getCurrentBaseShift());
  printShiftLines(path_shifter.getShiftLines(), "path_shifter shift points");

  ShiftedPath shifted_path;
  if (!path_shifter.generate(&shifted_path)) {
    RCLCPP_ERROR(getLogger(), "failed to generate shifted path.");
    return toShiftedPath(avoidance_data_.reference_path);
  }

  return shifted_path;
}

void AvoidanceModule::postProcess(PathShifter & path_shifter) const
{
  const size_t nearest_idx = findEgoIndex(path_shifter.getReferencePath().points);
  path_shifter.removeBehindShiftLineAndSetBaseOffset(nearest_idx);
}

void AvoidanceModule::updateData()
{
  debug_data_ = DebugData();
  avoidance_data_ = calcAvoidancePlanningData(debug_data_);

  // TODO(Horibe): this is not tested yet, disable now.
  updateRegisteredObject(avoidance_data_.target_objects);
  compensateDetectionLost(avoidance_data_.target_objects, avoidance_data_.other_objects);

  path_shifter_.setPath(avoidance_data_.reference_path);

  // update registered shift point for new reference path & remove past objects
  updateRegisteredRawShiftLines();

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
      r.lost_time = (clock_->now() - r.last_seen).seconds();
    } else {
      r.last_seen = clock_->now();
      r.lost_time = 0.0;
    }

    // lost count exceeds threshold. remove object from register.
    if (r.lost_time > parameters_->object_last_seen_threshold) {
      registered_objects_.erase(registered_objects_.begin() + i);
    }
  }

  const auto isAlreadyRegistered = [this](const auto & n_id) {
    const auto & r = registered_objects_;
    return std::any_of(
      r.begin(), r.end(), [&n_id](const auto & o) { return o.object.object_id == n_id; });
  };

  // -- check now_objects, add it if it has new object id --
  for (const auto & now_obj : now_objects) {
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
void AvoidanceModule::compensateDetectionLost(
  ObjectDataArray & now_objects, ObjectDataArray & other_objects) const
{
  const auto old_size = now_objects.size();  // for debug

  const auto isDetectedNow = [&](const auto & r_id) {
    const auto & n = now_objects;
    return std::any_of(
      n.begin(), n.end(), [&r_id](const auto & o) { return o.object.object_id == r_id; });
  };

  const auto isIgnoreObject = [&](const auto & r_id) {
    const auto & n = other_objects;
    return std::any_of(
      n.begin(), n.end(), [&r_id](const auto & o) { return o.object.object_id == r_id; });
  };

  for (const auto & registered : registered_objects_) {
    if (
      !isDetectedNow(registered.object.object_id) && !isIgnoreObject(registered.object.object_id)) {
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
}

void AvoidanceModule::onExit()
{
  DEBUG_PRINT("AVOIDANCE onExit");
  initVariables();
  current_state_ = BT::NodeStatus::SUCCESS;
  clearWaitingApproval();
  removeRTCStatus();
  steering_factor_interface_ptr_->clearSteeringFactors();
}

void AvoidanceModule::initVariables()
{
  prev_output_ = ShiftedPath();
  prev_linear_shift_path_ = ShiftedPath();
  prev_reference_ = PathWithLaneId();
  path_shifter_ = PathShifter{};
  left_shift_array_.clear();
  right_shift_array_.clear();

  debug_data_ = DebugData();
  debug_marker_.markers.clear();
  registered_raw_shift_lines_ = {};
  current_raw_shift_lines_ = {};
  original_unique_id = 0;
}

bool AvoidanceModule::isTargetObjectType(const PredictedObject & object) const
{
  using autoware_auto_perception_msgs::msg::ObjectClassification;
  const auto t = util::getHighestProbLabel(object.classification);
  const auto is_object_type =
    ((t == ObjectClassification::CAR && parameters_->avoid_car) ||
     (t == ObjectClassification::TRUCK && parameters_->avoid_truck) ||
     (t == ObjectClassification::BUS && parameters_->avoid_bus) ||
     (t == ObjectClassification::TRAILER && parameters_->avoid_trailer) ||
     (t == ObjectClassification::UNKNOWN && parameters_->avoid_unknown) ||
     (t == ObjectClassification::BICYCLE && parameters_->avoid_bicycle) ||
     (t == ObjectClassification::MOTORCYCLE && parameters_->avoid_motorcycle) ||
     (t == ObjectClassification::PEDESTRIAN && parameters_->avoid_pedestrian));
  return is_object_type;
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

  const auto current_shift_length = getCurrentShift();
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
  const size_t blinker_start_idx = [&]() {
    for (size_t idx = start_idx; idx <= end_idx; ++idx) {
      const double current_shift_length = path.shift_length.at(idx);
      if (current_shift_length > 0.1) {
        return idx;
      }
    }
    return start_idx;
  }();
  const size_t blinker_end_idx = end_idx;

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

  TurnSignalInfo turn_signal_info{};
  if (parameters_->turn_signal_on_swerving) {
    if (segment_shift_length > 0.0) {
      turn_signal_info.turn_signal.command = TurnIndicatorsCommand::ENABLE_LEFT;
    } else {
      turn_signal_info.turn_signal.command = TurnIndicatorsCommand::ENABLE_RIGHT;
    }
  } else {
    turn_signal_info.turn_signal.command = TurnIndicatorsCommand::DISABLE;
  }

  if (ego_front_to_shift_start > 0.0) {
    turn_signal_info.desired_start_point = planner_data_->self_pose->pose;
  } else {
    turn_signal_info.desired_start_point = blinker_start_pose;
  }
  turn_signal_info.desired_end_point = blinker_end_pose;
  turn_signal_info.required_start_point = blinker_start_pose;
  turn_signal_info.required_end_point = blinker_end_pose;

  return turn_signal_info;
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

void AvoidanceModule::setDebugData(
  const AvoidancePlanningData & data, const PathShifter & shifter, const DebugData & debug) const
{
  using marker_utils::createLaneletsAreaMarkerArray;
  using marker_utils::createObjectsMarkerArray;
  using marker_utils::createPathMarkerArray;
  using marker_utils::createPoseMarkerArray;
  using marker_utils::createShiftLengthMarkerArray;
  using marker_utils::createShiftLineMarkerArray;
  using marker_utils::avoidance_marker::createAvoidLineMarkerArray;
  using marker_utils::avoidance_marker::createOtherObjectsMarkerArray;
  using marker_utils::avoidance_marker::createOverhangFurthestLineStringMarkerArray;
  using marker_utils::avoidance_marker::createTargetObjectsMarkerArray;
  using marker_utils::avoidance_marker::makeOverhangToRoadShoulderMarkerArray;

  debug_marker_.markers.clear();

  const auto add = [this](const MarkerArray & added) {
    tier4_autoware_utils::appendMarkerArray(added, &debug_marker_);
  };

  const auto addAvoidLine =
    [&](const AvoidLineArray & al_arr, const auto & ns, auto r, auto g, auto b, double w = 0.1) {
      add(createAvoidLineMarkerArray(al_arr, ns, r, g, b, w));
    };

  const auto addShiftLine =
    [&](const ShiftLineArray & sl_arr, const auto & ns, auto r, auto g, auto b, double w = 0.1) {
      add(createShiftLineMarkerArray(sl_arr, shifter.getBaseOffset(), ns, r, g, b, w));
    };

  const auto & path = data.reference_path;
  add(createPathMarkerArray(debug.center_line, "centerline", 0, 0.0, 0.5, 0.9));
  add(createPathMarkerArray(path, "centerline_resampled", 0, 0.0, 0.9, 0.5));
  add(createPathMarkerArray(prev_linear_shift_path_.path, "prev_linear_shift", 0, 0.5, 0.4, 0.6));
  add(createPoseMarkerArray(data.reference_pose, "reference_pose", 0, 0.9, 0.3, 0.3));

  add(createLaneletsAreaMarkerArray(*debug.current_lanelets, "current_lanelet", 0.0, 1.0, 0.0));
  add(createLaneletsAreaMarkerArray(*debug.expanded_lanelets, "expanded_lanelet", 0.8, 0.8, 0.0));
  add(createTargetObjectsMarkerArray(data.target_objects, "target_objects"));
  add(createOtherObjectsMarkerArray(data.other_objects, "other_objects"));
  add(makeOverhangToRoadShoulderMarkerArray(data.target_objects, "overhang"));
  add(createOverhangFurthestLineStringMarkerArray(
    *debug.farthest_linestring_from_overhang, "farthest_linestring_from_overhang", 1.0, 0.0, 1.0));

  // parent object info
  addAvoidLine(debug.registered_raw_shift, "p_registered_shift", 0.8, 0.8, 0.0);
  addAvoidLine(debug.current_raw_shift, "p_current_raw_shift", 0.5, 0.2, 0.2);
  addAvoidLine(debug.extra_return_shift, "p_extra_return_shift", 0.0, 0.5, 0.8);

  // merged shift
  const auto & linear_shift = prev_linear_shift_path_.shift_length;
  add(createShiftLengthMarkerArray(debug.pos_shift, path, "m_pos_shift_line", 0, 0.7, 0.5));
  add(createShiftLengthMarkerArray(debug.neg_shift, path, "m_neg_shift_line", 0, 0.5, 0.7));
  add(createShiftLengthMarkerArray(debug.total_shift, path, "m_total_shift_line", 0.99, 0.4, 0.2));
  add(createShiftLengthMarkerArray(debug.output_shift, path, "m_output_shift_line", 0.8, 0.8, 0.2));
  add(createShiftLengthMarkerArray(linear_shift, path, "m_output_linear_line", 0.9, 0.3, 0.3));

  // child shift points
  addAvoidLine(debug.merged, "c_0_merged", 0.345, 0.968, 1.0);
  addAvoidLine(debug.trim_similar_grad_shift, "c_1_trim_similar_grad_shift", 0.976, 0.328, 0.910);
  addAvoidLine(debug.quantized, "c_2_quantized", 0.505, 0.745, 0.969);
  addAvoidLine(debug.trim_small_shift, "c_3_trim_small_shift", 0.663, 0.525, 0.941);
  addAvoidLine(
    debug.trim_similar_grad_shift_second, "c_4_trim_similar_grad_shift", 0.97, 0.32, 0.91);
  addAvoidLine(debug.trim_momentary_return, "c_5_trim_momentary_return", 0.976, 0.078, 0.878);
  addAvoidLine(debug.trim_too_sharp_shift, "c_6_trim_too_sharp_shift", 0.576, 0.0, 0.978);

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
