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
}  // namespace

bool isOnRight(const ObjectData & obj)
{
  return obj.lateral < 0.0;
}

bool isTargetObjectType(
  const PredictedObject & object, const std::shared_ptr<AvoidanceParameters> & parameters)
{
  const auto t = utils::getHighestProbLabel(object.classification);

  if (parameters->object_parameters.count(t) == 0) {
    return false;
  }

  return parameters->object_parameters.at(t).enable;
}

double calcShiftLength(
  const bool & is_object_on_right, const double & overhang_dist, const double & avoid_margin)
{
  const auto shift_length =
    is_object_on_right ? (overhang_dist + avoid_margin) : (overhang_dist - avoid_margin);
  return std::fabs(shift_length) > 1e-3 ? shift_length : 0.0;
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
  double max_distance = std::numeric_limits<double>::min();
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

std::vector<tier4_autoware_utils::Polygon2d> generateObstaclePolygonsForDrivableArea(
  const ObjectDataArray & objects, const std::shared_ptr<AvoidanceParameters> & parameters,
  const double vehicle_width)
{
  std::vector<tier4_autoware_utils::Polygon2d> obj_polys;

  if (objects.empty() || !parameters->enable_bound_clipping) {
    return obj_polys;
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

    const auto t = utils::getHighestProbLabel(object.object.classification);
    const auto object_parameter = parameters->object_parameters.at(t);

    // generate obstacle polygon
    const double diff_poly_buffer =
      object.avoid_margin.get() - object_parameter.envelope_buffer_margin - vehicle_width / 2.0;
    const auto obj_poly =
      tier4_autoware_utils::expandPolygon(object.envelope_poly, diff_poly_buffer);
    obj_polys.push_back(obj_poly);
  }
  return obj_polys;
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

  const auto t = utils::getHighestProbLabel(object_data.object.classification);
  const auto object_parameter = parameters->object_parameters.at(t);
  const auto & envelope_buffer_margin = object_parameter.envelope_buffer_margin;

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
  const auto & object_vel =
    object_data.object.kinematics.initial_twist_with_covariance.twist.linear.x;
  const auto is_faster_than_threshold = object_vel > parameters->threshold_speed_object_is_stopped;

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

  if (object_data.move_time > parameters->threshold_time_object_is_moving) {
    stopped_objects.erase(same_id_obj);
  }
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
  const auto dist_to_goal =
    rh->isInGoalRouteSection(data.current_lanelets.back())
      ? calcSignedArcLength(path_points, ego_pos, rh->getGoalPose().position)
      : std::numeric_limits<double>::max();

  for (auto & o : objects) {
    const auto & object_pose = o.object.kinematics.initial_pose_with_covariance.pose;
    const auto object_closest_index = findNearestIndex(path_points, object_pose.position);
    const auto object_closest_pose = path_points.at(object_closest_index).point.pose;

    const auto t = utils::getHighestProbLabel(o.object.classification);
    const auto object_parameter = parameters->object_parameters.at(t);

    if (!isTargetObjectType(o.object, parameters)) {
      data.other_objects.push_back(o);
      continue;
    }

    if (o.move_time > parameters->threshold_time_object_is_moving) {
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
      const bool get_left = isOnRight(o) && parameters->enable_avoidance_over_same_direction;
      const bool get_right = !isOnRight(o) && parameters->enable_avoidance_over_same_direction;

      const auto target_lines = rh->getFurthestLinestring(
        overhang_lanelet, get_right, get_left,
        parameters->enable_avoidance_over_opposite_direction);

      if (isOnRight(o)) {
        o.to_road_shoulder_distance =
          distance2d(to2D(overhang_basic_pose), to2D(target_lines.back().basicLineString()));
        debug.bounds.push_back(target_lines.back());
      } else {
        o.to_road_shoulder_distance =
          distance2d(to2D(overhang_basic_pose), to2D(target_lines.front().basicLineString()));
        debug.bounds.push_back(target_lines.front());
      }
    }

    // calculate avoid_margin dynamically
    // NOTE: This calculation must be after calculating to_road_shoulder_distance.
    const double max_avoid_margin = object_parameter.safety_buffer_lateral +
                                    parameters->lateral_collision_margin + 0.5 * vehicle_width;
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

    // force avoidance for stopped vehicle
    {
      const auto to_traffic_light =
        utils::getDistanceToNextTrafficLight(object_pose, data.current_lanelets);

      o.to_stop_factor_distance = std::min(to_traffic_light, o.to_stop_factor_distance);
    }

    if (
      o.stop_time > parameters->threshold_time_force_avoidance_for_stopped_vehicle &&
      parameters->enable_force_avoidance_for_stopped_vehicle) {
      if (o.to_stop_factor_distance > parameters->object_check_force_avoidance_clearance) {
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
}  // namespace behavior_path_planner::utils::avoidance
