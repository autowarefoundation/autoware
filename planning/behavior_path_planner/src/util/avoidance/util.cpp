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

#include "behavior_path_planner/util/avoidance/util.hpp"

#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/util/avoidance/avoidance_module_data.hpp"
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

using motion_utils::calcLongitudinalOffsetPoint;
using motion_utils::findNearestSegmentIndex;
using motion_utils::insertTargetPoint;
using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::calcOffsetPose;
using tier4_autoware_utils::calcYawDeviation;
using tier4_autoware_utils::createQuaternionFromRPY;
using tier4_autoware_utils::getPose;
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

boost::optional<geometry_msgs::msg::Point> intersect(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2,
  const geometry_msgs::msg::Point & p3, const geometry_msgs::msg::Point & p4)
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

  geometry_msgs::msg::Point intersect_point;
  intersect_point.x = t * p1.x + (1.0 - t) * p2.x;
  intersect_point.y = t * p1.y + (1.0 - t) * p2.y;
  intersect_point.z = t * p1.z + (1.0 - t) * p2.z;
  return intersect_point;
}

boost::optional<std::pair<size_t, geometry_msgs::msg::Point>> intersectBound(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2,
  const std::vector<geometry_msgs::msg::Point> & bound, const size_t seg_idx1,
  const size_t seg_idx2)
{
  const size_t start_idx =
    static_cast<size_t>(std::max(0, static_cast<int>(std::min(seg_idx1, seg_idx2)) - 5));
  const size_t end_idx = static_cast<size_t>(std::min(
    static_cast<int>(bound.size()) - 1, static_cast<int>(std::max(seg_idx1, seg_idx2)) + 1 + 5));
  for (int i = start_idx; i < static_cast<int>(end_idx); ++i) {
    const auto intersect_point = intersect(p1, p2, bound.at(i), bound.at(i + 1));
    if (intersect_point) {
      std::pair<size_t, geometry_msgs::msg::Point> result;
      result.first = static_cast<size_t>(i);
      result.second = intersect_point.get();
      return result;
    }
  }
  return boost::none;
}

PolygonPoint transformBoundFrenetCoordinate(
  const std::vector<geometry_msgs::msg::Point> & points, const geometry_msgs::msg::Point & point)
{
  const size_t seg_idx = motion_utils::findNearestSegmentIndex(points, point);
  const double lon_dist_to_segment =
    motion_utils::calcLongitudinalOffsetToSegment(points, seg_idx, point);
  const double lat_dist = motion_utils::calcLateralOffset(points, point, seg_idx);
  return PolygonPoint{point, seg_idx, lon_dist_to_segment, lat_dist};
}

std::vector<PolygonPoint> generatePolygonInsideBounds(
  const std::vector<Point> & bound, const std::vector<Point> & edge_points,
  const bool is_object_right)
{
  std::vector<PolygonPoint> full_polygon;
  for (const auto & edge_point : edge_points) {
    const auto polygon_point = transformBoundFrenetCoordinate(bound, edge_point);
    full_polygon.push_back(polygon_point);
  }

  std::vector<PolygonPoint> inside_poly;
  for (int i = 0; i < static_cast<int>(full_polygon.size()); ++i) {
    const auto & curr_poly = full_polygon.at(i);
    const auto & prev_poly = full_polygon.at(i == 0 ? full_polygon.size() - 1 : i - 1);

    const bool is_curr_outside = curr_poly.is_outside_bounds(is_object_right);
    const bool is_prev_outside = prev_poly.is_outside_bounds(is_object_right);

    if (is_curr_outside && is_prev_outside) {
      continue;
    }
    if (!is_curr_outside && !is_prev_outside) {
      inside_poly.push_back(curr_poly);
      continue;
    }

    const auto intersection = intersectBound(
      prev_poly.point, curr_poly.point, bound, prev_poly.bound_seg_idx, curr_poly.bound_seg_idx);
    if (!intersection) {
      continue;
    }
    const double lon_dist = motion_utils::calcLongitudinalOffsetToSegment(
      bound, intersection->first, intersection->second);
    const auto intersect_point =
      PolygonPoint{intersection->second, intersection->first, lon_dist, 0.0};

    if (is_prev_outside && !is_curr_outside) {
      inside_poly.push_back(intersect_point);
      inside_poly.push_back(curr_poly);
      continue;
    }
    // Here is if (!is_prev_outside && is_curr_outside).
    inside_poly.push_back(prev_poly);
    inside_poly.push_back(intersect_point);
    continue;
  }

  return inside_poly;
}

std::vector<geometry_msgs::msg::Point> convertToGeometryPoints(
  const std::vector<PolygonPoint> & polygon_points)
{
  std::vector<geometry_msgs::msg::Point> points;
  points.reserve(polygon_points.size());

  for (const auto & polygon_point : polygon_points) {
    points.push_back(polygon_point.point);
  }
  return points;
}

std::vector<PolygonPoint> concatenateTwoPolygons(
  const std::vector<PolygonPoint> & front_polygon, const std::vector<PolygonPoint> & back_polygon)
{
  // At first, the front polygon is the outside polygon
  bool is_front_polygon_outside = true;
  size_t outside_idx = 0;

  const auto get_out_poly = [&]() {
    return is_front_polygon_outside ? front_polygon : back_polygon;
  };
  const auto get_in_poly = [&]() {
    return is_front_polygon_outside ? back_polygon : front_polygon;
  };

  std::vector<PolygonPoint> concatenated_polygon;
  while (rclcpp::ok()) {
    concatenated_polygon.push_back(get_out_poly().at(outside_idx));
    if (outside_idx == get_out_poly().size() - 1) {
      break;
    }
    const size_t curr_idx = outside_idx;
    const size_t next_idx = outside_idx + 1;

    for (size_t i = 0; i < get_in_poly().size() - 1; ++i) {
      const auto intersection = intersect(
        get_out_poly().at(curr_idx).point, get_out_poly().at(next_idx).point,
        get_in_poly().at(i).point, get_in_poly().at(i + 1).point);
      if (intersection) {
        const auto intersect_point = PolygonPoint{intersection.get(), 0, 0.0, 0.0};
        concatenated_polygon.push_back(intersect_point);

        is_front_polygon_outside = !is_front_polygon_outside;
        outside_idx = i;
        break;
      }
    }
    outside_idx += 1;
  }

  return concatenated_polygon;
}

std::vector<std::vector<PolygonPoint>> concatenatePolygons(
  const std::vector<std::vector<PolygonPoint>> & polygons)
{
  auto unique_polygons = polygons;

  while (rclcpp::ok()) {
    bool is_updated = false;

    for (size_t i = 0; i < unique_polygons.size(); ++i) {
      for (size_t j = 0; j < i; ++j) {
        const auto & p1 = unique_polygons.at(i);
        const auto & p2 = unique_polygons.at(j);

        // if p1 and p2 overlaps
        if (p1.back().is_after(p2.front()) && p2.back().is_after(p1.front())) {
          is_updated = true;

          const auto concatenated_polygon = [&]() {
            if (p2.front().is_after(p1.front())) {
              return concatenateTwoPolygons(p1, p2);
            }
            return concatenateTwoPolygons(p2, p1);
          }();

          // NOTE: remove i's element first since is larger than j.
          unique_polygons.erase(unique_polygons.begin() + i);
          unique_polygons.erase(unique_polygons.begin() + j);

          unique_polygons.push_back(concatenated_polygon);
          break;
        }
      }
      if (is_updated) {
        break;
      }
    }

    if (!is_updated) {
      break;
    }
  }
  return unique_polygons;
}

std::vector<PolygonPoint> getPolygonPointsInsideBounds(
  const std::vector<Point> & bound, const std::vector<Point> & edge_points,
  const bool is_object_right)
{
  // NOTE: Polygon is defined at lest by three points.
  if (edge_points.size() < 3) {
    return std::vector<PolygonPoint>();
  }

  // convert to vector of PolygonPoint
  const auto inside_polygon = [&]() {
    auto tmp_polygon = generatePolygonInsideBounds(bound, edge_points, is_object_right);

    // In order to make the order of points the same as the order of lon_dist_to_segment.
    // The order of points is clockwise.
    if (!is_object_right) {
      std::reverse(tmp_polygon.begin(), tmp_polygon.end());
    }
    return tmp_polygon;
  }();
  if (inside_polygon.empty()) {
    return std::vector<PolygonPoint>();
  }

  // search start and end index by longitudinal distance
  std::vector<int> polygon_indices(inside_polygon.size());
  std::iota(polygon_indices.begin(), polygon_indices.end(), 0);
  std::sort(polygon_indices.begin(), polygon_indices.end(), [&](int i1, int i2) {
    return inside_polygon.at(i2).is_after(inside_polygon.at(i1));
  });
  const int start_idx = polygon_indices.front();
  const int end_idx = polygon_indices.back();

  // calculate valid inside polygon
  std::vector<PolygonPoint> valid_inside_polygon;
  for (int i = 0; i < (end_idx - start_idx + static_cast<int>(polygon_indices.size())) %
                          static_cast<int>(polygon_indices.size()) +
                        1;
       ++i) {
    const int poly_idx = (start_idx + i) % static_cast<int>(inside_polygon.size());
    valid_inside_polygon.push_back(inside_polygon.at(poly_idx));
  }

  // add start and end points projected to bound if necessary
  if (inside_polygon.at(start_idx).lat_dist_to_bound != 0.0) {  // not on bound
    auto start_point = inside_polygon.at(start_idx);
    const auto start_point_on_bound = motion_utils::calcLongitudinalOffsetPoint(
      bound, start_point.bound_seg_idx, start_point.lon_dist_to_segment);
    if (start_point_on_bound) {
      start_point.point = start_point_on_bound.get();
      valid_inside_polygon.insert(valid_inside_polygon.begin(), start_point);
    }
  }
  if (inside_polygon.at(end_idx).lat_dist_to_bound != 0.0) {  // not on bound
    auto end_point = inside_polygon.at(end_idx);
    const auto end_point_on_bound = motion_utils::calcLongitudinalOffsetPoint(
      bound, end_point.bound_seg_idx, end_point.lon_dist_to_segment);
    if (end_point_on_bound) {
      end_point.point = end_point_on_bound.get();
      valid_inside_polygon.insert(valid_inside_polygon.end(), end_point);
    }
  }
  return valid_inside_polygon;
}

std::vector<Point> updateBoundary(
  const std::vector<Point> & original_bound,
  const std::vector<std::vector<PolygonPoint>> & sorted_polygons)
{
  if (sorted_polygons.empty()) {
    return original_bound;
  }

  auto reversed_polygons = sorted_polygons;
  std::reverse(reversed_polygons.begin(), reversed_polygons.end());

  auto updated_bound = original_bound;

  // NOTE: Further obstacle is applied first since a part of the updated_bound is erased.
  for (const auto & polygon : reversed_polygons) {
    const auto & start_poly = polygon.front();
    const auto & end_poly = polygon.back();

    const double front_offset = motion_utils::calcLongitudinalOffsetToSegment(
      updated_bound, start_poly.bound_seg_idx, start_poly.point);

    const size_t removed_start_idx =
      0 < front_offset ? start_poly.bound_seg_idx + 1 : start_poly.bound_seg_idx;
    const size_t removed_end_idx = end_poly.bound_seg_idx;

    updated_bound.erase(
      updated_bound.begin() + removed_start_idx, updated_bound.begin() + removed_end_idx + 1);

    const auto obj_points = convertToGeometryPoints(polygon);
    updated_bound.insert(
      updated_bound.begin() + removed_start_idx, obj_points.begin(), obj_points.end());
  }
  return updated_bound;
}
}  // namespace

bool isOnRight(const ObjectData & obj)
{
  return obj.lateral < 0.0;
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
  const auto object_poly = tier4_autoware_utils::toPolygon2d(object);
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

  const auto object_poly = tier4_autoware_utils::toPolygon2d(object_data.object);
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

void generateDrivableArea(
  PathWithLaneId & path, const std::vector<DrivableLanes> & lanes, const double vehicle_length,
  const std::shared_ptr<const PlannerData> planner_data, const ObjectDataArray & objects,
  const bool enable_bound_clipping, const bool disable_path_update,
  const double original_object_buffer)
{
  util::generateDrivableArea(path, lanes, vehicle_length, planner_data);
  if (objects.empty() || !enable_bound_clipping) {
    return;
  }

  std::vector<std::vector<PolygonPoint>> right_polygons;
  std::vector<std::vector<PolygonPoint>> left_polygons;
  for (const auto & object : objects) {
    // If avoidance is executed by both behavior and motion, only non-avoidable object will be
    // extracted from the drivable area.
    if (!disable_path_update) {
      if (object.is_avoidable) {
        continue;
      }
    }

    // check if avoid marin is calculated
    if (!object.avoid_margin) {
      continue;
    }

    // generate obstacle polygon
    const auto & obj_pose = object.object.kinematics.initial_pose_with_covariance.pose;
    const double diff_poly_buffer = object.avoid_margin.get() - original_object_buffer -
                                    planner_data->parameters.vehicle_width / 2.0;
    const auto obj_poly =
      tier4_autoware_utils::expandPolygon(object.envelope_poly, diff_poly_buffer);

    // get edge points of the object
    const size_t nearest_path_idx = motion_utils::findNearestIndex(
      path.points, obj_pose.position);  // to get z for object polygon
    std::vector<Point> edge_points;
    for (size_t i = 0; i < obj_poly.outer().size() - 1;
         ++i) {  // NOTE: There is a duplicated points
      edge_points.push_back(tier4_autoware_utils::createPoint(
        obj_poly.outer().at(i).x(), obj_poly.outer().at(i).y(),
        path.points.at(nearest_path_idx).point.pose.position.z));
    }

    // get a boundary that we have to change
    const double lat_dist_to_path = motion_utils::calcLateralOffset(path.points, obj_pose.position);
    const bool is_object_right = lat_dist_to_path < 0.0;
    const auto & bound = is_object_right ? path.right_bound : path.left_bound;

    // get polygon points inside the bounds
    const auto inside_polygon = getPolygonPointsInsideBounds(bound, edge_points, is_object_right);
    if (!inside_polygon.empty()) {
      if (is_object_right) {
        right_polygons.push_back(inside_polygon);
      } else {
        left_polygons.push_back(inside_polygon);
      }
    }
  }

  for (size_t i = 0; i < 2; ++i) {  // for loop for right and left
    const bool is_object_right = (i == 0);
    const auto & polygons = is_object_right ? right_polygons : left_polygons;
    if (polygons.empty()) {
      continue;
    }

    // concatenate polygons if they are longitudinal overlapped.
    auto unique_polygons = concatenatePolygons(polygons);

    // sort bounds longitudinally
    std::sort(
      unique_polygons.begin(), unique_polygons.end(),
      [](const std::vector<PolygonPoint> & p1, const std::vector<PolygonPoint> & p2) {
        return p2.front().is_after(p1.front());
      });

    // update boundary
    auto & bound = is_object_right ? path.right_bound : path.left_bound;
    bound = updateBoundary(bound, unique_polygons);
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
}  // namespace behavior_path_planner
