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

#include "behavior_path_planner/utils/utils.hpp"

#include "behavior_path_planner/utils/avoidance/avoidance_module_data.hpp"
#include "behavior_path_planner/utils/avoidance/utils.hpp"
#include "behavior_path_planner/utils/path_utils.hpp"

#include <autoware_auto_tf2/tf2_autoware_auto_msgs.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <tier4_planning_msgs/msg/avoidance_debug_factor.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace behavior_path_planner::utils::avoidance
{

using motion_utils::calcLongitudinalOffsetPoint;
using motion_utils::calcSignedArcLength;
using motion_utils::findNearestIndex;
using motion_utils::findNearestSegmentIndex;
using motion_utils::insertTargetPoint;
using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::calcLateralDeviation;
using tier4_autoware_utils::calcOffsetPose;
using tier4_autoware_utils::calcYawDeviation;
using tier4_autoware_utils::createQuaternionFromRPY;
using tier4_autoware_utils::getPose;
using tier4_autoware_utils::pose2transform;
using tier4_autoware_utils::toHexString;
using tier4_planning_msgs::msg::AvoidanceDebugFactor;
using tier4_planning_msgs::msg::AvoidanceDebugMsg;

namespace
{
geometry_msgs::msg::Point32 createPoint32(const double x, const double y, const double z)
{
  geometry_msgs::msg::Point32 p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

geometry_msgs::msg::Polygon toMsg(const tier4_autoware_utils::Polygon2d & polygon, const double z)
{
  geometry_msgs::msg::Polygon ret;
  for (const auto & p : polygon.outer()) {
    ret.points.push_back(createPoint32(p.x(), p.y(), z));
  }
  return ret;
}

boost::optional<Point> intersect(
  const Point & p1, const Point & p2, const Point & p3, const Point & p4)
{
  // calculate intersection point
  const double det = (p1.x - p2.x) * (p4.y - p3.y) - (p4.x - p3.x) * (p1.y - p2.y);
  if (det == 0.0) {
    return {};
  }

  const double t = ((p4.y - p3.y) * (p4.x - p2.x) + (p3.x - p4.x) * (p4.y - p2.y)) / det;
  const double s = ((p2.y - p1.y) * (p4.x - p2.x) + (p1.x - p2.x) * (p4.y - p2.y)) / det;
  if (t < 0 || 1 < t || s < 0 || 1 < s) {
    return {};
  }

  Point intersect_point;
  intersect_point.x = t * p1.x + (1.0 - t) * p2.x;
  intersect_point.y = t * p1.y + (1.0 - t) * p2.y;
  intersect_point.z = t * p1.z + (1.0 - t) * p2.z;
  return intersect_point;
}
}  // namespace

bool isOnRight(const ObjectData & obj)
{
  return obj.lateral < 0.0;
}

bool isTargetObjectType(
  const PredictedObject & object, const std::shared_ptr<AvoidanceParameters> & parameters)
{
  const auto object_type = utils::getHighestProbLabel(object.classification);

  if (parameters->object_parameters.count(object_type) == 0) {
    return false;
  }

  return parameters->object_parameters.at(object_type).is_target;
}

bool isVehicleTypeObject(const ObjectData & object)
{
  const auto object_type = utils::getHighestProbLabel(object.object.classification);

  if (object_type == ObjectClassification::UNKNOWN) {
    return false;
  }

  if (object_type == ObjectClassification::PEDESTRIAN) {
    return false;
  }

  if (object_type == ObjectClassification::BICYCLE) {
    return false;
  }

  return true;
}

bool isWithinCrosswalk(
  const ObjectData & object,
  const std::shared_ptr<const lanelet::routing::RoutingGraphContainer> & overall_graphs)
{
  using Point = boost::geometry::model::d2::point_xy<double>;

  const auto & p = object.object.kinematics.initial_pose_with_covariance.pose.position;
  const Point p_object{p.x, p.y};

  // get conflicting crosswalk crosswalk
  constexpr int PEDESTRIAN_GRAPH_ID = 1;
  const auto conflicts =
    overall_graphs->conflictingInGraph(object.overhang_lanelet, PEDESTRIAN_GRAPH_ID);

  constexpr double THRESHOLD = 2.0;
  for (const auto & crosswalk : conflicts) {
    auto polygon = crosswalk.polygon2d().basicPolygon();

    boost::geometry::correct(polygon);

    // ignore objects around the crosswalk
    if (boost::geometry::distance(p_object, polygon) < THRESHOLD) {
      return true;
    }
  }

  return false;
}

double calcShiftLength(
  const bool & is_object_on_right, const double & overhang_dist, const double & avoid_margin)
{
  const auto shift_length =
    is_object_on_right ? (overhang_dist + avoid_margin) : (overhang_dist - avoid_margin);
  return std::fabs(shift_length) > 1e-3 ? shift_length : 0.0;
}

bool isShiftNecessary(const bool & is_object_on_right, const double & shift_length)
{
  /**
   *   ^
   *   |
   * --+----x-------------------------------x--->
   *   |                 x     x
   *   |                 ==obj==
   */
  if (is_object_on_right && shift_length < 0.0) {
    return false;
  }

  /**
   *   ^                 ==obj==
   *   |                 x     x
   * --+----x-------------------------------x--->
   *   |
   */
  if (!is_object_on_right && shift_length > 0.0) {
    return false;
  }

  return true;
}

bool isSameDirectionShift(const bool & is_object_on_right, const double & shift_length)
{
  return (is_object_on_right == std::signbit(shift_length));
}

ShiftedPath toShiftedPath(const PathWithLaneId & path)
{
  ShiftedPath out;
  out.path = path;
  out.shift_length.resize(path.points.size());
  std::fill(out.shift_length.begin(), out.shift_length.end(), 0.0);
  return out;
}

ShiftLineArray toShiftLineArray(const AvoidLineArray & avoid_points)
{
  ShiftLineArray shift_lines;
  for (const auto & ap : avoid_points) {
    shift_lines.push_back(ap);
  }
  return shift_lines;
}

size_t findPathIndexFromArclength(
  const std::vector<double> & path_arclength_arr, const double target_arc)
{
  if (path_arclength_arr.empty()) {
    return 0;
  }

  for (size_t i = 0; i < path_arclength_arr.size(); ++i) {
    if (path_arclength_arr.at(i) > target_arc) {
      return i;
    }
  }
  return path_arclength_arr.size() - 1;
}

std::vector<size_t> concatParentIds(
  const std::vector<size_t> & ids1, const std::vector<size_t> & ids2)
{
  std::set<size_t> id_set{ids1.begin(), ids1.end()};
  for (const auto id : ids2) {
    id_set.insert(id);
  }
  const auto v = std::vector<size_t>{id_set.begin(), id_set.end()};
  return v;
}

std::vector<size_t> calcParentIds(const AvoidLineArray & lines1, const AvoidLine & lines2)
{
  // Get the ID of the original AP whose transition area overlaps with the given AP,
  // and set it to the parent id.
  std::set<uint64_t> ids;
  for (const auto & al : lines1) {
    const auto p_s = al.start_longitudinal;
    const auto p_e = al.end_longitudinal;
    const auto has_overlap = !(p_e < lines2.start_longitudinal || lines2.end_longitudinal < p_s);

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
    for (const auto & al_local : lines1) {
      if (al_local.object.object.object_id == al.object.object.object_id) {
        ids.insert(al_local.id);
      }
    }
  }
  return std::vector<size_t>(ids.begin(), ids.end());
}

double lerpShiftLengthOnArc(double arc, const AvoidLine & ap)
{
  if (ap.start_longitudinal <= arc && arc < ap.end_longitudinal) {
    if (std::abs(ap.getRelativeLongitudinal()) < 1.0e-5) {
      return ap.end_shift_length;
    }
    const auto start_weight = (ap.end_longitudinal - arc) / ap.getRelativeLongitudinal();
    return start_weight * ap.start_shift_length + (1.0 - start_weight) * ap.end_shift_length;
  }
  return 0.0;
}

void fillLongitudinalAndLengthByClosestEnvelopeFootprint(
  const PathWithLaneId & path, const Point & ego_pos, ObjectData & obj)
{
  double min_distance = std::numeric_limits<double>::max();
  double max_distance = std::numeric_limits<double>::lowest();
  for (const auto & p : obj.envelope_poly.outer()) {
    const auto point = tier4_autoware_utils::createPoint(p.x(), p.y(), 0.0);
    const double arc_length = motion_utils::calcSignedArcLength(path.points, ego_pos, point);
    min_distance = std::min(min_distance, arc_length);
    max_distance = std::max(max_distance, arc_length);
  }
  obj.longitudinal = min_distance;
  obj.length = max_distance - min_distance;
  return;
}

double calcEnvelopeOverhangDistance(
  const ObjectData & object_data, const Pose & base_pose, Point & overhang_pose)
{
  double largest_overhang = isOnRight(object_data) ? -100.0 : 100.0;  // large number

  for (const auto & p : object_data.envelope_poly.outer()) {
    const auto point = tier4_autoware_utils::createPoint(p.x(), p.y(), 0.0);
    const auto lateral = tier4_autoware_utils::calcLateralDeviation(base_pose, point);

    const auto & overhang_pose_on_right = [&overhang_pose, &largest_overhang, &point, &lateral]() {
      if (lateral > largest_overhang) {
        overhang_pose = point;
      }
      return std::max(largest_overhang, lateral);
    };

    const auto & overhang_pose_on_left = [&overhang_pose, &largest_overhang, &point, &lateral]() {
      if (lateral < largest_overhang) {
        overhang_pose = point;
      }
      return std::min(largest_overhang, lateral);
    };

    largest_overhang = isOnRight(object_data) ? overhang_pose_on_right() : overhang_pose_on_left();
  }
  return largest_overhang;
}

void setEndData(
  AvoidLine & ap, const double length, const geometry_msgs::msg::Pose & end, const size_t end_idx,
  const double end_dist)
{
  ap.end_shift_length = length;
  ap.end = end;
  ap.end_idx = end_idx;
  ap.end_longitudinal = end_dist;
}

void setStartData(
  AvoidLine & ap, const double start_shift_length, const geometry_msgs::msg::Pose & start,
  const size_t start_idx, const double start_dist)
{
  ap.start_shift_length = start_shift_length;
  ap.start = start;
  ap.start_idx = start_idx;
  ap.start_longitudinal = start_dist;
}

Polygon2d createEnvelopePolygon(
  const ObjectData & object_data, const Pose & closest_pose, const double envelope_buffer)
{
  namespace bg = boost::geometry;
  using tier4_autoware_utils::Point2d;
  using tier4_autoware_utils::Polygon2d;
  using Box = bg::model::box<Point2d>;

  const auto object_polygon = tier4_autoware_utils::toPolygon2d(object_data.object);

  const auto toPolygon2d = [](const geometry_msgs::msg::Polygon & polygon) {
    Polygon2d ret{};

    for (const auto & p : polygon.points) {
      ret.outer().push_back(Point2d(p.x, p.y));
    }

    return ret;
  };

  Pose pose_2d = closest_pose;
  pose_2d.orientation = createQuaternionFromRPY(0.0, 0.0, tf2::getYaw(closest_pose.orientation));

  TransformStamped geometry_tf{};
  geometry_tf.transform = pose2transform(pose_2d);

  tf2::Transform tf;
  tf2::fromMsg(geometry_tf.transform, tf);
  TransformStamped inverse_geometry_tf{};
  inverse_geometry_tf.transform = tf2::toMsg(tf.inverse());

  geometry_msgs::msg::Polygon out_ros_polygon{};
  tf2::doTransform(
    toMsg(object_polygon, closest_pose.position.z), out_ros_polygon, inverse_geometry_tf);

  const auto envelope_box = bg::return_envelope<Box>(toPolygon2d(out_ros_polygon));

  Polygon2d envelope_poly{};
  bg::convert(envelope_box, envelope_poly);

  geometry_msgs::msg::Polygon envelope_ros_polygon{};
  tf2::doTransform(
    toMsg(envelope_poly, closest_pose.position.z), envelope_ros_polygon, geometry_tf);

  const auto expanded_polygon =
    tier4_autoware_utils::expandPolygon(toPolygon2d(envelope_ros_polygon), envelope_buffer);
  return expanded_polygon;
}

std::vector<DrivableAreaInfo::Obstacle> generateObstaclePolygonsForDrivableArea(
  const ObjectDataArray & objects, const std::shared_ptr<AvoidanceParameters> & parameters,
  const double vehicle_width)
{
  std::vector<DrivableAreaInfo::Obstacle> obstacles_for_drivable_area;

  if (objects.empty() || !parameters->enable_bound_clipping) {
    return obstacles_for_drivable_area;
  }

  for (const auto & object : objects) {
    // If avoidance is executed by both behavior and motion, only non-avoidable object will be
    // extracted from the drivable area.
    if (!parameters->disable_path_update) {
      if (object.is_avoidable) {
        continue;
      }
    }

    // check if avoid marin is calculated
    if (!object.avoid_margin) {
      continue;
    }

    const auto object_type = utils::getHighestProbLabel(object.object.classification);
    const auto object_parameter = parameters->object_parameters.at(object_type);

    // generate obstacle polygon
    const double diff_poly_buffer =
      object.avoid_margin.get() - object_parameter.envelope_buffer_margin - vehicle_width / 2.0;
    const auto obj_poly =
      tier4_autoware_utils::expandPolygon(object.envelope_poly, diff_poly_buffer);
    const bool is_left = 0 < object.lateral;
    obstacles_for_drivable_area.push_back(
      {object.object.kinematics.initial_pose_with_covariance.pose, obj_poly, is_left});
  }
  return obstacles_for_drivable_area;
}

double getLongitudinalVelocity(const Pose & p_ref, const Pose & p_target, const double v)
{
  return v * std::cos(calcYawDeviation(p_ref, p_target));
}

bool isCentroidWithinLanelets(
  const PredictedObject & object, const lanelet::ConstLanelets & target_lanelets)
{
  if (target_lanelets.empty()) {
    return false;
  }

  const auto & object_pos = object.kinematics.initial_pose_with_covariance.pose.position;
  lanelet::BasicPoint2d object_centroid(object_pos.x, object_pos.y);

  for (const auto & llt : target_lanelets) {
    if (boost::geometry::within(object_centroid, llt.polygon2d().basicPolygon())) {
      return true;
    }
  }

  return false;
}

lanelet::ConstLanelets getTargetLanelets(
  const std::shared_ptr<const PlannerData> & planner_data, lanelet::ConstLanelets & route_lanelets,
  const double left_offset, const double right_offset)
{
  const auto & rh = planner_data->route_handler;

  lanelet::ConstLanelets target_lanelets{};
  for (const auto & lane : route_lanelets) {
    auto l_offset = 0.0;
    auto r_offset = 0.0;

    const auto opt_left_lane = rh->getLeftLanelet(lane);
    if (opt_left_lane) {
      target_lanelets.push_back(opt_left_lane.get());
    } else {
      l_offset = left_offset;
    }

    const auto opt_right_lane = rh->getRightLanelet(lane);
    if (opt_right_lane) {
      target_lanelets.push_back(opt_right_lane.get());
    } else {
      r_offset = right_offset;
    }

    const auto right_opposite_lanes = rh->getRightOppositeLanelets(lane);
    if (!right_opposite_lanes.empty()) {
      target_lanelets.push_back(right_opposite_lanes.front());
    }

    const auto expand_lane = lanelet::utils::getExpandedLanelet(lane, l_offset, r_offset);
    target_lanelets.push_back(expand_lane);
  }

  return target_lanelets;
}

lanelet::ConstLanelets getCurrentLanesFromPath(
  const PathWithLaneId & path, const std::shared_ptr<const PlannerData> & planner_data)
{
  if (path.points.empty()) {
    throw std::logic_error("empty path.");
  }

  if (path.points.front().lane_ids.empty()) {
    throw std::logic_error("empty lane ids.");
  }

  const auto start_id = path.points.front().lane_ids.front();
  const auto start_lane = planner_data->route_handler->getLaneletsFromId(start_id);
  const auto & p = planner_data->parameters;

  return planner_data->route_handler->getLaneletSequence(
    start_lane, p.backward_path_length, p.forward_path_length);
}

void insertDecelPoint(
  const Point & p_src, const double offset, const double velocity, PathWithLaneId & path,
  boost::optional<Pose> & p_out)
{
  const auto decel_point = calcLongitudinalOffsetPoint(path.points, p_src, offset);

  if (!decel_point) {
    // TODO(Satoshi OTA)  Think later the process in the case of no decel point found.
    return;
  }

  const auto seg_idx = findNearestSegmentIndex(path.points, decel_point.get());
  const auto insert_idx = insertTargetPoint(seg_idx, decel_point.get(), path.points);

  if (!insert_idx) {
    // TODO(Satoshi OTA)  Think later the process in the case of no decel point found.
    return;
  }

  const auto insertVelocity = [&insert_idx](PathWithLaneId & path, const float v) {
    for (size_t i = insert_idx.get(); i < path.points.size(); ++i) {
      const auto & original_velocity = path.points.at(i).point.longitudinal_velocity_mps;
      path.points.at(i).point.longitudinal_velocity_mps = std::min(original_velocity, v);
    }
  };

  insertVelocity(path, velocity);

  p_out = getPose(path.points.at(insert_idx.get()));
}

void fillObjectEnvelopePolygon(
  ObjectData & object_data, const ObjectDataArray & registered_objects, const Pose & closest_pose,
  const std::shared_ptr<AvoidanceParameters> & parameters)
{
  using boost::geometry::within;

  const auto object_type = utils::getHighestProbLabel(object_data.object.classification);
  const auto object_parameter = parameters->object_parameters.at(object_type);

  const auto & envelope_buffer_margin =
    object_parameter.envelope_buffer_margin * object_data.distance_factor;

  const auto id = object_data.object.object_id;
  const auto same_id_obj = std::find_if(
    registered_objects.begin(), registered_objects.end(),
    [&id](const auto & o) { return o.object.object_id == id; });

  if (same_id_obj == registered_objects.end()) {
    object_data.envelope_poly =
      createEnvelopePolygon(object_data, closest_pose, envelope_buffer_margin);
    return;
  }

  const auto object_polygon = tier4_autoware_utils::toPolygon2d(object_data.object);

  if (!within(object_polygon, same_id_obj->envelope_poly)) {
    object_data.envelope_poly =
      createEnvelopePolygon(object_data, closest_pose, envelope_buffer_margin);
    return;
  }

  object_data.envelope_poly = same_id_obj->envelope_poly;
}

void fillObjectMovingTime(
  ObjectData & object_data, ObjectDataArray & stopped_objects,
  const std::shared_ptr<AvoidanceParameters> & parameters)
{
  const auto object_type = utils::getHighestProbLabel(object_data.object.classification);
  const auto object_parameter = parameters->object_parameters.at(object_type);

  const auto & object_vel =
    object_data.object.kinematics.initial_twist_with_covariance.twist.linear.x;
  const auto is_faster_than_threshold = object_vel > object_parameter.moving_speed_threshold;

  const auto id = object_data.object.object_id;
  const auto same_id_obj = std::find_if(
    stopped_objects.begin(), stopped_objects.end(),
    [&id](const auto & o) { return o.object.object_id == id; });

  const auto is_new_object = same_id_obj == stopped_objects.end();
  const rclcpp::Time now = rclcpp::Clock(RCL_ROS_TIME).now();

  if (!is_faster_than_threshold) {
    object_data.last_stop = now;
    object_data.move_time = 0.0;
    if (is_new_object) {
      object_data.stop_time = 0.0;
      object_data.last_move = now;
      stopped_objects.push_back(object_data);
    } else {
      same_id_obj->stop_time = (now - same_id_obj->last_move).seconds();
      same_id_obj->last_stop = now;
      same_id_obj->move_time = 0.0;
      object_data.stop_time = same_id_obj->stop_time;
    }
    return;
  }

  if (is_new_object) {
    object_data.move_time = std::numeric_limits<double>::infinity();
    object_data.stop_time = 0.0;
    object_data.last_move = now;
    return;
  }

  object_data.last_stop = same_id_obj->last_stop;
  object_data.move_time = (now - same_id_obj->last_stop).seconds();
  object_data.stop_time = 0.0;

  if (object_data.move_time > object_parameter.moving_time_threshold) {
    stopped_objects.erase(same_id_obj);
  }
}

void fillAvoidanceNecessity(
  ObjectData & object_data, const ObjectDataArray & registered_objects, const double vehicle_width,
  const std::shared_ptr<AvoidanceParameters> & parameters)
{
  const auto object_type = utils::getHighestProbLabel(object_data.object.classification);
  const auto object_parameter = parameters->object_parameters.at(object_type);
  const auto safety_margin =
    0.5 * vehicle_width + object_parameter.safety_buffer_lateral * object_data.distance_factor;

  const auto check_necessity = [&](const auto hysteresis_factor) {
    return (isOnRight(object_data) &&
            std::abs(object_data.overhang_dist) < safety_margin * hysteresis_factor) ||
           (!isOnRight(object_data) &&
            object_data.overhang_dist < safety_margin * hysteresis_factor);
  };

  const auto id = object_data.object.object_id;
  const auto same_id_obj = std::find_if(
    registered_objects.begin(), registered_objects.end(),
    [&id](const auto & o) { return o.object.object_id == id; });

  // First time
  if (same_id_obj == registered_objects.end()) {
    object_data.avoid_required = check_necessity(1.0);
    return;
  }

  // FALSE -> FALSE or FALSE -> TRUE
  if (!same_id_obj->avoid_required) {
    object_data.avoid_required = check_necessity(1.0);
    return;
  }

  // TRUE -> ? (check with hysteresis factor)
  object_data.avoid_required = check_necessity(parameters->safety_check_hysteresis_factor);
}

void fillObjectStoppableJudge(
  ObjectData & object_data, const ObjectDataArray & registered_objects,
  const double feasible_stop_distance, const std::shared_ptr<AvoidanceParameters> & parameters)
{
  if (!parameters->use_constraints_for_decel) {
    object_data.is_stoppable = true;
    return;
  }

  if (!object_data.avoid_required) {
    object_data.is_stoppable = false;
    return;
  }

  const auto id = object_data.object.object_id;
  const auto same_id_obj = std::find_if(
    registered_objects.begin(), registered_objects.end(),
    [&id](const auto & o) { return o.object.object_id == id; });

  const auto is_stoppable = object_data.to_stop_line > feasible_stop_distance;
  if (is_stoppable) {
    object_data.is_stoppable = true;
    return;
  }

  if (same_id_obj == registered_objects.end()) {
    object_data.is_stoppable = false;
    return;
  }

  object_data.is_stoppable = same_id_obj->is_stoppable;
}

void updateRegisteredObject(
  ObjectDataArray & registered_objects, const ObjectDataArray & now_objects,
  const std::shared_ptr<AvoidanceParameters> & parameters)
{
  const auto updateIfDetectedNow = [&now_objects](auto & registered_object) {
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

  const rclcpp::Time now = rclcpp::Clock(RCL_ROS_TIME).now();

  // -- check registered_objects, remove if lost_count exceeds limit. --
  for (int i = static_cast<int>(registered_objects.size()) - 1; i >= 0; --i) {
    auto & r = registered_objects.at(i);

    // registered object is not detected this time. lost count up.
    if (!updateIfDetectedNow(r)) {
      r.lost_time = (now - r.last_seen).seconds();
    } else {
      r.last_seen = now;
      r.lost_time = 0.0;
    }

    // lost count exceeds threshold. remove object from register.
    if (r.lost_time > parameters->object_last_seen_threshold) {
      registered_objects.erase(registered_objects.begin() + i);
    }
  }

  const auto isAlreadyRegistered = [&](const auto & n_id) {
    const auto & r = registered_objects;
    return std::any_of(
      r.begin(), r.end(), [&n_id](const auto & o) { return o.object.object_id == n_id; });
  };

  // -- check now_objects, add it if it has new object id --
  for (const auto & now_obj : now_objects) {
    if (!isAlreadyRegistered(now_obj.object.object_id)) {
      registered_objects.push_back(now_obj);
    }
  }
}

void compensateDetectionLost(
  const ObjectDataArray & registered_objects, ObjectDataArray & now_objects,
  ObjectDataArray & other_objects)
{
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

  for (const auto & registered : registered_objects) {
    if (
      !isDetectedNow(registered.object.object_id) && !isIgnoreObject(registered.object.object_id)) {
      now_objects.push_back(registered);
    }
  }
}

void filterTargetObjects(
  ObjectDataArray & objects, AvoidancePlanningData & data, DebugData & debug,
  const std::shared_ptr<const PlannerData> & planner_data,
  const std::shared_ptr<AvoidanceParameters> & parameters)
{
  using boost::geometry::return_centroid;
  using boost::geometry::within;
  using lanelet::geometry::distance2d;
  using lanelet::geometry::toArcCoordinates;
  using lanelet::utils::to2D;

  const auto & rh = planner_data->route_handler;
  const auto & path_points = data.reference_path_rough.points;
  const auto & ego_pos = planner_data->self_odometry->pose.pose.position;
  const auto & vehicle_width = planner_data->parameters.vehicle_width;
  const rclcpp::Time now = rclcpp::Clock(RCL_ROS_TIME).now();

  // for goal
  const auto ego_idx = planner_data->findEgoIndex(path_points);
  const auto dist_to_goal = rh->isInGoalRouteSection(data.current_lanelets.back())
                              ? calcSignedArcLength(path_points, ego_idx, path_points.size() - 1)
                              : std::numeric_limits<double>::max();

  // extend lanelets if the reference path is cut for lane change.
  const auto & ego_pose = planner_data->self_odometry->pose.pose;
  lanelet::ConstLanelets extend_lanelets = data.current_lanelets;
  while (rclcpp::ok()) {
    const double lane_length = lanelet::utils::getLaneletLength2d(extend_lanelets);
    const auto arclength = lanelet::utils::getArcCoordinates(extend_lanelets, ego_pose);
    const auto next_lanelets = rh->getNextLanelets(extend_lanelets.back());

    if (next_lanelets.empty()) {
      break;
    }

    if (lane_length - arclength.length < planner_data->parameters.forward_path_length) {
      extend_lanelets.push_back(next_lanelets.front());
    } else {
      break;
    }
  }

  for (auto & o : objects) {
    const auto & object_pose = o.object.kinematics.initial_pose_with_covariance.pose;
    const auto object_closest_index = findNearestIndex(path_points, object_pose.position);
    const auto object_closest_pose = path_points.at(object_closest_index).point.pose;

    const auto object_type = utils::getHighestProbLabel(o.object.classification);
    const auto object_parameter = parameters->object_parameters.at(object_type);

    if (!isTargetObjectType(o.object, parameters)) {
      o.reason = AvoidanceDebugFactor::OBJECT_IS_NOT_TYPE;
      data.other_objects.push_back(o);
      continue;
    }

    // if following condition are satisfied, ignored the objects as moving objects.
    // 1. speed is higher than threshold.
    // 2. keep that speed longer than the time threshold.
    if (o.move_time > object_parameter.moving_time_threshold) {
      o.reason = AvoidanceDebugFactor::MOVING_OBJECT;
      data.other_objects.push_back(o);
      continue;
    }

    // calc longitudinal distance from ego to closest target object footprint point.
    fillLongitudinalAndLengthByClosestEnvelopeFootprint(data.reference_path_rough, ego_pos, o);

    // object is behind ego or too far.
    if (o.longitudinal < -parameters->object_check_backward_distance) {
      o.reason = AvoidanceDebugFactor::OBJECT_IS_BEHIND_THRESHOLD;
      data.other_objects.push_back(o);
      continue;
    }
    if (o.longitudinal > parameters->object_check_forward_distance) {
      o.reason = AvoidanceDebugFactor::OBJECT_IS_IN_FRONT_THRESHOLD;
      data.other_objects.push_back(o);
      continue;
    }

    // Target object is behind the path goal -> ignore.
    if (o.longitudinal > dist_to_goal) {
      o.reason = AvoidanceDebugFactor::OBJECT_BEHIND_PATH_GOAL;
      data.other_objects.push_back(o);
      continue;
    }

    if (o.longitudinal + o.length / 2 + parameters->object_check_goal_distance > dist_to_goal) {
      o.reason = "TooNearToGoal";
      data.other_objects.push_back(o);
      continue;
    }

    lanelet::ConstLanelet overhang_lanelet;
    if (!rh->getClosestLaneletWithinRoute(object_closest_pose, &overhang_lanelet)) {
      continue;
    }

    if (overhang_lanelet.id()) {
      o.overhang_lanelet = overhang_lanelet;
      lanelet::BasicPoint3d overhang_basic_pose(
        o.overhang_pose.position.x, o.overhang_pose.position.y, o.overhang_pose.position.z);

      const bool get_left = isOnRight(o) && parameters->use_adjacent_lane;
      const bool get_right = !isOnRight(o) && parameters->use_adjacent_lane;
      const bool get_opposite = parameters->use_opposite_lane;

      lanelet::ConstLineString3d target_line{};
      o.to_road_shoulder_distance = std::numeric_limits<double>::max();
      const auto update_road_to_shoulder_distance = [&](const auto & target_lanelet) {
        const auto lines =
          rh->getFurthestLinestring(target_lanelet, get_right, get_left, get_opposite, true);
        const auto & line = isOnRight(o) ? lines.back() : lines.front();
        const auto d = distance2d(to2D(overhang_basic_pose), to2D(line.basicLineString()));
        if (d < o.to_road_shoulder_distance) {
          o.to_road_shoulder_distance = d;
          target_line = line;
        }
      };

      // current lanelet
      update_road_to_shoulder_distance(overhang_lanelet);
      // previous lanelet
      lanelet::ConstLanelets previous_lanelet{};
      if (rh->getPreviousLaneletsWithinRoute(overhang_lanelet, &previous_lanelet)) {
        update_road_to_shoulder_distance(previous_lanelet.front());
      }
      // next lanelet
      lanelet::ConstLanelet next_lanelet{};
      if (rh->getNextLaneletWithinRoute(overhang_lanelet, &next_lanelet)) {
        update_road_to_shoulder_distance(next_lanelet);
      }
      debug.bounds.push_back(target_line);

      {
        o.to_road_shoulder_distance = extendToRoadShoulderDistanceWithPolygon(
          rh, target_line, o.to_road_shoulder_distance, overhang_lanelet, o.overhang_pose.position,
          overhang_basic_pose, parameters->use_hatched_road_markings,
          parameters->use_intersection_areas);
      }
    }

    // calculate avoid_margin dynamically
    // NOTE: This calculation must be after calculating to_road_shoulder_distance.
    const double max_avoid_margin = object_parameter.safety_buffer_lateral * o.distance_factor +
                                    object_parameter.avoid_margin_lateral + 0.5 * vehicle_width;
    const double min_safety_lateral_distance =
      object_parameter.safety_buffer_lateral + 0.5 * vehicle_width;
    const auto max_allowable_lateral_distance =
      o.to_road_shoulder_distance - parameters->road_shoulder_safety_margin - 0.5 * vehicle_width;

    const auto avoid_margin = [&]() -> boost::optional<double> {
      if (max_allowable_lateral_distance < min_safety_lateral_distance) {
        return boost::none;
      }
      return std::min(max_allowable_lateral_distance, max_avoid_margin);
    }();

    if (!!avoid_margin) {
      const auto shift_length = calcShiftLength(isOnRight(o), o.overhang_dist, avoid_margin.get());
      if (!isShiftNecessary(isOnRight(o), shift_length)) {
        o.reason = "NotNeedAvoidance";
        data.other_objects.push_back(o);
        continue;
      }

      if (std::abs(shift_length) < parameters->lateral_execution_threshold) {
        o.reason = "LessThanExecutionThreshold";
        data.other_objects.push_back(o);
        continue;
      }
    }

    // for non vehicle type object
    if (!isVehicleTypeObject(o)) {
      if (isWithinCrosswalk(o, rh->getOverallGraphPtr())) {
        // avoidance module ignore pedestrian and bicycle around crosswalk
        o.reason = "CrosswalkUser";
        data.other_objects.push_back(o);
      } else {
        // if there is no crosswalk near the object, avoidance module avoids pedestrian and bicycle
        // no matter how it is shifted.
        o.last_seen = now;
        o.avoid_margin = avoid_margin;
        data.target_objects.push_back(o);
      }
      continue;
    }

    // from here condition check for vehicle type objects.

    const auto stop_time_longer_than_threshold =
      o.stop_time > parameters->threshold_time_force_avoidance_for_stopped_vehicle;

    if (stop_time_longer_than_threshold && parameters->enable_force_avoidance_for_stopped_vehicle) {
      // force avoidance for stopped vehicle
      bool not_parked_object = true;

      // check traffic light
      const auto to_traffic_light =
        utils::getDistanceToNextTrafficLight(object_pose, extend_lanelets);
      {
        not_parked_object =
          to_traffic_light < parameters->object_ignore_section_traffic_light_in_front_distance;
      }

      // check crosswalk
      const auto to_crosswalk =
        utils::getDistanceToCrosswalk(ego_pose, extend_lanelets, *rh->getOverallGraphPtr()) -
        o.longitudinal;
      {
        const auto stop_for_crosswalk =
          to_crosswalk < parameters->object_ignore_section_crosswalk_in_front_distance &&
          to_crosswalk > -1.0 * parameters->object_ignore_section_crosswalk_behind_distance;
        not_parked_object = not_parked_object || stop_for_crosswalk;
      }

      o.to_stop_factor_distance = std::min(to_traffic_light, to_crosswalk);

      if (!not_parked_object) {
        o.last_seen = now;
        o.avoid_margin = avoid_margin;
        data.target_objects.push_back(o);
        continue;
      }
    }

    // Object is on center line -> ignore.
    if (std::abs(o.lateral) < parameters->threshold_distance_object_is_on_center) {
      o.reason = AvoidanceDebugFactor::TOO_NEAR_TO_CENTERLINE;
      data.other_objects.push_back(o);
      continue;
    }

    lanelet::BasicPoint2d object_centroid(o.centroid.x(), o.centroid.y());

    /**
     * Is not object in adjacent lane?
     *   - Yes -> Is parking object?
     *     - Yes -> the object is avoidance target.
     *     - No -> ignore this object.
     *   - No -> the object is avoidance target no matter whether it is parking object or not.
     */
    const auto is_in_ego_lane =
      within(object_centroid, overhang_lanelet.polygon2d().basicPolygon());
    if (is_in_ego_lane) {
      /**
       * TODO(Satoshi Ota) use intersection area
       * under the assumption that there is no parking vehicle inside intersection,
       * ignore all objects that is in the ego lane as not parking objects.
       */
      std::string turn_direction = overhang_lanelet.attributeOr("turn_direction", "else");
      if (turn_direction == "right" || turn_direction == "left" || turn_direction == "straight") {
        o.reason = AvoidanceDebugFactor::NOT_PARKING_OBJECT;
        data.other_objects.push_back(o);
        continue;
      }

      const auto centerline_pose =
        lanelet::utils::getClosestCenterPose(overhang_lanelet, object_pose.position);
      lanelet::BasicPoint3d centerline_point(
        centerline_pose.position.x, centerline_pose.position.y, centerline_pose.position.z);

      // ============================================ <- most_left_lanelet.leftBound()
      // y              road shoulder
      // ^ ------------------------------------------
      // |   x                                +
      // +---> --- object closest lanelet --- o ----- <- object_closest_lanelet.centerline()
      //
      // --------------------------------------------
      // +: object position
      // o: nearest point on centerline

      bool is_left_side_parked_vehicle = false;
      if (!isOnRight(o)) {
        auto [object_shiftable_distance, sub_type] = [&]() {
          const auto most_left_road_lanelet = rh->getMostLeftLanelet(overhang_lanelet);
          const auto most_left_lanelet_candidates =
            rh->getLaneletMapPtr()->laneletLayer.findUsages(most_left_road_lanelet.leftBound());

          lanelet::ConstLanelet most_left_lanelet = most_left_road_lanelet;
          const lanelet::Attribute sub_type =
            most_left_lanelet.attribute(lanelet::AttributeName::Subtype);

          for (const auto & ll : most_left_lanelet_candidates) {
            const lanelet::Attribute sub_type = ll.attribute(lanelet::AttributeName::Subtype);
            if (sub_type.value() == "road_shoulder") {
              most_left_lanelet = ll;
            }
          }

          const auto center_to_left_boundary = distance2d(
            to2D(most_left_lanelet.leftBound().basicLineString()), to2D(centerline_point));

          return std::make_pair(
            center_to_left_boundary - 0.5 * o.object.shape.dimensions.y, sub_type);
        }();

        if (sub_type.value() != "road_shoulder") {
          object_shiftable_distance += parameters->object_check_min_road_shoulder_width;
        }

        const auto arc_coordinates =
          toArcCoordinates(to2D(overhang_lanelet.centerline().basicLineString()), object_centroid);
        o.shiftable_ratio = arc_coordinates.distance / object_shiftable_distance;

        is_left_side_parked_vehicle = o.shiftable_ratio > parameters->object_check_shiftable_ratio;
      }

      bool is_right_side_parked_vehicle = false;
      if (isOnRight(o)) {
        auto [object_shiftable_distance, sub_type] = [&]() {
          const auto most_right_road_lanelet = rh->getMostRightLanelet(overhang_lanelet);
          const auto most_right_lanelet_candidates =
            rh->getLaneletMapPtr()->laneletLayer.findUsages(most_right_road_lanelet.rightBound());

          lanelet::ConstLanelet most_right_lanelet = most_right_road_lanelet;
          const lanelet::Attribute sub_type =
            most_right_lanelet.attribute(lanelet::AttributeName::Subtype);

          for (const auto & ll : most_right_lanelet_candidates) {
            const lanelet::Attribute sub_type = ll.attribute(lanelet::AttributeName::Subtype);
            if (sub_type.value() == "road_shoulder") {
              most_right_lanelet = ll;
            }
          }

          const auto center_to_right_boundary = distance2d(
            to2D(most_right_lanelet.rightBound().basicLineString()), to2D(centerline_point));

          return std::make_pair(
            center_to_right_boundary - 0.5 * o.object.shape.dimensions.y, sub_type);
        }();

        if (sub_type.value() != "road_shoulder") {
          object_shiftable_distance += parameters->object_check_min_road_shoulder_width;
        }

        const auto arc_coordinates =
          toArcCoordinates(to2D(overhang_lanelet.centerline().basicLineString()), object_centroid);
        o.shiftable_ratio = -1.0 * arc_coordinates.distance / object_shiftable_distance;

        is_right_side_parked_vehicle = o.shiftable_ratio > parameters->object_check_shiftable_ratio;
      }

      if (!is_left_side_parked_vehicle && !is_right_side_parked_vehicle) {
        o.reason = AvoidanceDebugFactor::NOT_PARKING_OBJECT;
        data.other_objects.push_back(o);
        continue;
      }
    }

    o.last_seen = now;
    o.avoid_margin = avoid_margin;

    // set data
    data.target_objects.push_back(o);
  }
}

double extendToRoadShoulderDistanceWithPolygon(
  const std::shared_ptr<route_handler::RouteHandler> & rh,
  const lanelet::ConstLineString3d & target_line, const double to_road_shoulder_distance,
  const lanelet::ConstLanelet & overhang_lanelet, const geometry_msgs::msg::Point & overhang_pos,
  const lanelet::BasicPoint3d & overhang_basic_pose, const bool use_hatched_road_markings,
  const bool use_intersection_areas)
{
  // get expandable polygons for avoidance (e.g. hatched road markings)
  std::vector<lanelet::Polygon3d> expandable_polygons;

  const auto exist_polygon = [&](const auto & candidate_polygon) {
    return std::any_of(
      expandable_polygons.begin(), expandable_polygons.end(),
      [&](const auto & polygon) { return polygon.id() == candidate_polygon.id(); });
  };

  if (use_hatched_road_markings) {
    for (const auto & point : target_line) {
      const auto new_polygon_candidate =
        utils::getPolygonByPoint(rh, point, "hatched_road_markings");

      if (!!new_polygon_candidate && !exist_polygon(*new_polygon_candidate)) {
        expandable_polygons.push_back(*new_polygon_candidate);
      }
    }
  }

  if (use_intersection_areas) {
    const std::string area_id_str = overhang_lanelet.attributeOr("intersection_area", "else");

    if (area_id_str != "else") {
      expandable_polygons.push_back(
        rh->getLaneletMapPtr()->polygonLayer.get(std::atoi(area_id_str.c_str())));
    }
  }

  if (expandable_polygons.empty()) {
    return to_road_shoulder_distance;
  }

  // calculate point laterally offset from overhang position to calculate intersection with
  // polygon
  Point lat_offset_overhang_pos;
  {
    auto arc_coordinates = lanelet::geometry::toArcCoordinates(
      lanelet::utils::to2D(target_line), lanelet::utils::to2D(overhang_basic_pose));
    arc_coordinates.distance = 0.0;
    const auto closest_target_line_point =
      lanelet::geometry::fromArcCoordinates(target_line, arc_coordinates);

    const double ratio = 100.0 / to_road_shoulder_distance;
    lat_offset_overhang_pos.x =
      closest_target_line_point.x() + (closest_target_line_point.x() - overhang_pos.x) * ratio;
    lat_offset_overhang_pos.y =
      closest_target_line_point.y() + (closest_target_line_point.y() - overhang_pos.y) * ratio;
  }

  // update to_road_shoulder_distance with valid expandable polygon
  double updated_to_road_shoulder_distance = to_road_shoulder_distance;
  for (const auto & polygon : expandable_polygons) {
    std::vector<double> intersect_dist_vec;
    for (size_t i = 0; i < polygon.size(); ++i) {
      const auto polygon_current_point =
        geometry_msgs::build<Point>().x(polygon[i].x()).y(polygon[i].y()).z(0.0);
      const auto polygon_next_point = geometry_msgs::build<Point>()
                                        .x(polygon[(i + 1) % polygon.size()].x())
                                        .y(polygon[(i + 1) % polygon.size()].y())
                                        .z(0.0);

      const auto intersect_pos =
        intersect(overhang_pos, lat_offset_overhang_pos, polygon_current_point, polygon_next_point);
      if (intersect_pos) {
        intersect_dist_vec.push_back(calcDistance2d(*intersect_pos, overhang_pos));
      }
    }

    if (intersect_dist_vec.empty()) {
      continue;
    }

    std::sort(intersect_dist_vec.begin(), intersect_dist_vec.end());
    updated_to_road_shoulder_distance =
      std::max(updated_to_road_shoulder_distance, intersect_dist_vec.back());
  }
  return updated_to_road_shoulder_distance;
}

AvoidLine fillAdditionalInfo(const AvoidancePlanningData & data, const AvoidLine & line)
{
  AvoidLineArray ret{line};
  fillAdditionalInfoFromPoint(data, ret);
  return ret.front();
}

void fillAdditionalInfoFromPoint(const AvoidancePlanningData & data, AvoidLineArray & lines)
{
  if (lines.empty()) {
    return;
  }

  const auto & path = data.reference_path;
  const auto & arc = data.arclength_from_ego;

  // calc longitudinal
  for (auto & sl : lines) {
    sl.start_idx = findNearestIndex(path.points, sl.start.position);
    sl.start_longitudinal = arc.at(sl.start_idx);
    sl.end_idx = findNearestIndex(path.points, sl.end.position);
    sl.end_longitudinal = arc.at(sl.end_idx);
  }
}

void fillAdditionalInfoFromLongitudinal(const AvoidancePlanningData & data, AvoidLineArray & lines)
{
  const auto & path = data.reference_path;
  const auto & arc = data.arclength_from_ego;

  for (auto & sl : lines) {
    sl.start_idx = findPathIndexFromArclength(arc, sl.start_longitudinal);
    sl.start = path.points.at(sl.start_idx).point.pose;
    sl.end_idx = findPathIndexFromArclength(arc, sl.end_longitudinal);
    sl.end = path.points.at(sl.end_idx).point.pose;
  }
}

AvoidLineArray combineRawShiftLinesWithUniqueCheck(
  const AvoidLineArray & base_lines, const AvoidLineArray & added_lines)
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
}  // namespace behavior_path_planner::utils::avoidance
