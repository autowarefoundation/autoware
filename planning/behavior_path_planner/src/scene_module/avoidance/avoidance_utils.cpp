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

#include "behavior_path_planner/scene_module/avoidance/avoidance_utils.hpp"

#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/scene_module/avoidance/avoidance_module.hpp"
#include "behavior_path_planner/scene_module/avoidance/avoidance_module_data.hpp"
#include "behavior_path_planner/utilities.hpp"

#include <autoware_auto_tf2/tf2_autoware_auto_msgs.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace behavior_path_planner
{

using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::calcOffsetPose;
using tier4_autoware_utils::calcYawDeviation;
using tier4_autoware_utils::createQuaternionFromRPY;
using tier4_autoware_utils::pose2transform;

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

bool isOnRight(const ObjectData & obj) { return obj.lateral < 0.0; }

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

void clipByMinStartIdx(const AvoidLineArray & shift_lines, PathWithLaneId & path)
{
  if (path.points.empty()) {
    return;
  }

  size_t min_start_idx = std::numeric_limits<size_t>::max();
  for (const auto & sl : shift_lines) {
    min_start_idx = std::min(min_start_idx, sl.start_idx);
  }
  min_start_idx = std::min(min_start_idx, path.points.size() - 1);
  path.points =
    std::vector<PathPointWithLaneId>{path.points.begin() + min_start_idx, path.points.end()};
}

void fillLongitudinalAndLengthByClosestFootprint(
  const PathWithLaneId & path, const PredictedObject & object, const Point & ego_pos,
  ObjectData & obj)
{
  tier4_autoware_utils::Polygon2d object_poly{};
  util::calcObjectPolygon(object, &object_poly);

  const double distance = motion_utils::calcSignedArcLength(
    path.points, ego_pos, object.kinematics.initial_pose_with_covariance.pose.position);
  double min_distance = distance;
  double max_distance = distance;
  for (const auto & p : object_poly.outer()) {
    const auto point = tier4_autoware_utils::createPoint(p.x(), p.y(), 0.0);
    const double arc_length = motion_utils::calcSignedArcLength(path.points, ego_pos, point);
    min_distance = std::min(min_distance, arc_length);
    max_distance = std::max(max_distance, arc_length);
  }
  obj.longitudinal = min_distance;
  obj.length = max_distance - min_distance;
  return;
}

void fillLongitudinalAndLengthByClosestEnvelopeFootprint(
  const PathWithLaneId & path, const Point & ego_pos, ObjectData & obj)
{
  const double distance = motion_utils::calcSignedArcLength(
    path.points, ego_pos, obj.object.kinematics.initial_pose_with_covariance.pose.position);
  double min_distance = distance;
  double max_distance = distance;
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

double calcOverhangDistance(
  const ObjectData & object_data, const Pose & base_pose, Point & overhang_pose)
{
  double largest_overhang = isOnRight(object_data) ? -100.0 : 100.0;  // large number

  tier4_autoware_utils::Polygon2d object_poly{};
  util::calcObjectPolygon(object_data.object, &object_poly);

  for (const auto & p : object_poly.outer()) {
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

std::string getUuidStr(const ObjectData & obj)
{
  std::stringstream hex_value;
  for (const auto & uuid : obj.object.object_id.uuid) {
    hex_value << std::hex << std::setfill('0') << std::setw(2) << +uuid;
  }
  return hex_value.str();
}

std::vector<std::string> getUuidStr(const ObjectDataArray & objs)
{
  std::vector<std::string> uuids;
  uuids.reserve(objs.size());
  for (const auto & o : objs) {
    uuids.push_back(getUuidStr(o));
  }
  return uuids;
}

Polygon2d createEnvelopePolygon(
  const ObjectData & object_data, const Pose & closest_pose, const double envelope_buffer)
{
  namespace bg = boost::geometry;
  using tier4_autoware_utils::Point2d;
  using tier4_autoware_utils::Polygon2d;
  using Box = bg::model::box<Point2d>;

  Polygon2d object_polygon{};
  util::calcObjectPolygon(object_data.object, &object_polygon);

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

  std::vector<Polygon2d> offset_polygons{};
  bg::strategy::buffer::distance_symmetric<double> distance_strategy(envelope_buffer);
  bg::strategy::buffer::join_miter join_strategy;
  bg::strategy::buffer::end_flat end_strategy;
  bg::strategy::buffer::side_straight side_strategy;
  bg::strategy::buffer::point_circle point_strategy;
  bg::buffer(
    toPolygon2d(envelope_ros_polygon), offset_polygons, distance_strategy, side_strategy,
    join_strategy, end_strategy, point_strategy);

  return offset_polygons.front();
}

void getEdgePoints(
  const Polygon2d & object_polygon, const double threshold, std::vector<Point> & edge_points)
{
  if (object_polygon.outer().size() < 2) {
    return;
  }

  const size_t num_points = object_polygon.outer().size();
  for (size_t i = 0; i < num_points - 1; ++i) {
    const auto & curr_p = object_polygon.outer().at(i);
    const auto & next_p = object_polygon.outer().at(i + 1);
    const auto & prev_p =
      i == 0 ? object_polygon.outer().at(num_points - 2) : object_polygon.outer().at(i - 1);
    const Eigen::Vector2d current_to_next(next_p.x() - curr_p.x(), next_p.y() - curr_p.y());
    const Eigen::Vector2d current_to_prev(prev_p.x() - curr_p.x(), prev_p.y() - curr_p.y());
    const double inner_val = current_to_next.dot(current_to_prev);
    if (std::fabs(inner_val) > threshold) {
      continue;
    }

    const auto edge_point = tier4_autoware_utils::createPoint(curr_p.x(), curr_p.y(), 0.0);
    edge_points.push_back(edge_point);
  }
}

void sortPolygonPoints(
  const std::vector<PolygonPoint> & points, std::vector<PolygonPoint> & sorted_points)
{
  sorted_points = points;
  if (points.size() <= 2) {
    // sort data based on longitudinal distance to the boundary
    std::sort(
      sorted_points.begin(), sorted_points.end(),
      [](const PolygonPoint & a, const PolygonPoint & b) { return a.lon_dist < b.lon_dist; });
    return;
  }

  // sort data based on lateral distance to the boundary
  std::sort(
    sorted_points.begin(), sorted_points.end(), [](const PolygonPoint & a, const PolygonPoint & b) {
      return std::fabs(a.lat_dist_to_bound) > std::fabs(b.lat_dist_to_bound);
    });
  PolygonPoint first_point;
  PolygonPoint second_point;
  if (sorted_points.at(0).lon_dist < sorted_points.at(1).lon_dist) {
    first_point = sorted_points.at(0);
    second_point = sorted_points.at(1);
  } else {
    first_point = sorted_points.at(1);
    second_point = sorted_points.at(0);
  }

  for (size_t i = 2; i < sorted_points.size(); ++i) {
    const auto & next_point = sorted_points.at(i);
    if (next_point.lon_dist < first_point.lon_dist) {
      sorted_points = {next_point, first_point, second_point};
      return;
    } else if (second_point.lon_dist < next_point.lon_dist) {
      sorted_points = {first_point, second_point, next_point};
      return;
    }
  }

  sorted_points = {first_point, second_point};
}

void getEdgePoints(
  const std::vector<Point> & bound, const std::vector<Point> & edge_points,
  const double lat_dist_to_path, std::vector<PolygonPoint> & edge_points_data,
  size_t & start_segment_idx, size_t & end_segment_idx)
{
  for (const auto & edge_point : edge_points) {
    const size_t segment_idx = motion_utils::findNearestSegmentIndex(bound, edge_point);
    start_segment_idx = std::min(start_segment_idx, segment_idx);
    end_segment_idx = std::max(end_segment_idx, segment_idx);

    PolygonPoint edge_point_data;
    edge_point_data.point = edge_point;
    edge_point_data.lat_dist_to_bound = motion_utils::calcLateralOffset(bound, edge_point);
    edge_point_data.lon_dist = motion_utils::calcSignedArcLength(bound, 0, edge_point);
    if (lat_dist_to_path >= 0.0 && edge_point_data.lat_dist_to_bound > 0.0) {
      continue;
    } else if (lat_dist_to_path < 0.0 && edge_point_data.lat_dist_to_bound < 0.0) {
      continue;
    }

    edge_points_data.push_back(edge_point_data);
  }
}

std::vector<Point> updateBoundary(
  const std::vector<Point> & original_bound, const std::vector<PolygonPoint> & points,
  const size_t start_segment_idx, const size_t end_segment_idx)
{
  if (start_segment_idx >= end_segment_idx) {
    return original_bound;
  }

  std::vector<Point> updated_bound;
  std::copy(
    original_bound.begin(), original_bound.begin() + start_segment_idx + 1,
    std::back_inserter(updated_bound));
  for (size_t i = 0; i < points.size(); ++i) {
    updated_bound.push_back(points.at(i).point);
  }
  std::copy(
    original_bound.begin() + end_segment_idx + 1, original_bound.end(),
    std::back_inserter(updated_bound));
  return updated_bound;
}

void generateDrivableArea(
  PathWithLaneId & path, const std::vector<DrivableLanes> & lanes, const double vehicle_length,
  const std::shared_ptr<const PlannerData> planner_data, const ObjectDataArray & objects,
  const bool enable_bound_clipping)
{
  util::generateDrivableArea(path, lanes, vehicle_length, planner_data);
  if (objects.empty() || !enable_bound_clipping) {
    return;
  }

  path.left_bound = motion_utils::resamplePointVector(path.left_bound, 1.0, true);
  path.right_bound = motion_utils::resamplePointVector(path.right_bound, 1.0, true);

  for (const auto & object : objects) {
    const auto & obj_pose = object.object.kinematics.initial_pose_with_covariance.pose;
    const auto & obj_poly = object.envelope_poly;
    constexpr double threshold = 0.01;

    // get edge points
    std::vector<Point> edge_points;
    getEdgePoints(obj_poly, threshold, edge_points);

    // get boundary
    const double lat_dist_to_path = motion_utils::calcLateralOffset(path.points, obj_pose.position);
    auto & bound = lat_dist_to_path < 0.0 ? path.right_bound : path.left_bound;

    size_t start_segment_idx = (bound.size() == 1 ? 0 : (bound.size() - 2));
    size_t end_segment_idx = 0;

    // get edge points data
    std::vector<PolygonPoint> edge_points_data;
    getEdgePoints(
      bound, edge_points, lat_dist_to_path, edge_points_data, start_segment_idx, end_segment_idx);

    // sort points
    std::vector<PolygonPoint> sorted_points;
    sortPolygonPoints(edge_points_data, sorted_points);

    // update boundary
    bound = updateBoundary(bound, sorted_points, start_segment_idx, end_segment_idx);
  }
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

}  // namespace behavior_path_planner
