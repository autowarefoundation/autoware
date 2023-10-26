// Copyright 2021-2023 Tier IV, Inc.
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

#include "behavior_path_planner/utils/drivable_area_expansion/drivable_area_expansion.hpp"
#include "behavior_path_planner/utils/path_shifter/path_shifter.hpp"
#include "motion_utils/trajectory/path_with_lane_id.hpp"
#include "object_recognition_utils/predicted_path_utils.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <motion_utils/resample/resample.hpp>
#include <tier4_autoware_utils/geometry/boost_polygon_utils.hpp>
#include <tier4_autoware_utils/math/unit_conversion.hpp>

#include <boost/geometry/algorithms/is_valid.hpp>

#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_routing/RoutingGraphContainer.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace
{
double calcInterpolatedZ(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & input,
  const geometry_msgs::msg::Point target_pos, const size_t seg_idx)
{
  const double closest_to_target_dist = motion_utils::calcSignedArcLength(
    input.points, input.points.at(seg_idx).point.pose.position,
    target_pos);  // TODO(murooka) implement calcSignedArcLength(points, idx, point)
  const double seg_dist = motion_utils::calcSignedArcLength(input.points, seg_idx, seg_idx + 1);

  const double closest_z = input.points.at(seg_idx).point.pose.position.z;
  const double next_z = input.points.at(seg_idx + 1).point.pose.position.z;
  const double interpolated_z =
    std::abs(seg_dist) < 1e-6
      ? next_z
      : closest_z + (next_z - closest_z) * closest_to_target_dist / seg_dist;
  return interpolated_z;
}

double calcInterpolatedVelocity(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & input, const size_t seg_idx)
{
  const double seg_dist = motion_utils::calcSignedArcLength(input.points, seg_idx, seg_idx + 1);

  const double closest_vel = input.points.at(seg_idx).point.longitudinal_velocity_mps;
  const double next_vel = input.points.at(seg_idx + 1).point.longitudinal_velocity_mps;
  const double interpolated_vel = std::abs(seg_dist) < 1e-06 ? next_vel : closest_vel;
  return interpolated_vel;
}

template <class T>
size_t findNearestSegmentIndex(
  const std::vector<T> & points, const geometry_msgs::msg::Pose & pose, const double dist_threshold,
  const double yaw_threshold)
{
  const auto nearest_idx =
    motion_utils::findNearestSegmentIndex(points, pose, dist_threshold, yaw_threshold);
  if (nearest_idx) {
    return nearest_idx.get();
  }

  return motion_utils::findNearestSegmentIndex(points, pose.position);
}

template <class T>
size_t findNearestSegmentIndexFromLateralDistance(
  const std::vector<T> & points, const geometry_msgs::msg::Point & target_point)
{
  std::optional<size_t> closest_idx{std::nullopt};
  double min_lateral_dist = std::numeric_limits<double>::max();
  for (size_t seg_idx = 0; seg_idx < points.size() - 1; ++seg_idx) {
    const double lon_dist =
      motion_utils::calcLongitudinalOffsetToSegment(points, seg_idx, target_point);
    const double segment_length =
      tier4_autoware_utils::calcDistance2d(points.at(seg_idx), points.at(seg_idx + 1));
    const double lat_dist = [&]() {
      if (lon_dist < 0.0) {
        return tier4_autoware_utils::calcDistance2d(points.at(seg_idx), target_point);
      }
      if (segment_length < lon_dist) {
        return tier4_autoware_utils::calcDistance2d(points.at(seg_idx + 1), target_point);
      }
      return std::abs(motion_utils::calcLateralOffset(points, target_point, seg_idx));
    }();
    if (lat_dist < min_lateral_dist) {
      closest_idx = seg_idx;
      min_lateral_dist = lat_dist;
    }
  }

  if (closest_idx) {
    return *closest_idx;
  }

  return motion_utils::findNearestSegmentIndex(points, target_point);
}

bool checkHasSameLane(
  const lanelet::ConstLanelets & lanelets, const lanelet::ConstLanelet & target_lane)
{
  if (lanelets.empty()) return false;

  const auto has_same = [&](const auto & ll) { return ll.id() == target_lane.id(); };
  return std::find_if(lanelets.begin(), lanelets.end(), has_same) != lanelets.end();
}

bool isSamePoint(const geometry_msgs::msg::Point & point1, const geometry_msgs::msg::Point & point2)
{
  constexpr double epsilon = 1e-3;
  return std::abs(point1.x - point2.x) < epsilon && std::abs(point1.y - point2.y) < epsilon;
}
}  // namespace

namespace behavior_path_planner::utils
{
using autoware_auto_perception_msgs::msg::ObjectClassification;
using autoware_auto_perception_msgs::msg::Shape;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using tf2::fromMsg;
using tier4_autoware_utils::Point2d;

namespace drivable_area_processing
{
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
    const auto intersect_point =
      tier4_autoware_utils::intersect(p1, p2, bound.at(i), bound.at(i + 1));
    if (intersect_point) {
      std::pair<size_t, geometry_msgs::msg::Point> result;
      result.first = static_cast<size_t>(i);
      result.second = *intersect_point;
      return result;
    }
  }
  return boost::none;
}

double calcDistanceFromPointToSegment(
  const geometry_msgs::msg::Point & segment_start_point,
  const geometry_msgs::msg::Point & segment_end_point,
  const geometry_msgs::msg::Point & target_point)
{
  const auto & a = segment_start_point;
  const auto & b = segment_end_point;
  const auto & p = target_point;

  const double dot_val = (b.x - a.x) * (p.x - a.x) + (b.y - a.y) * (p.y - a.y);
  const double squared_segment_length = tier4_autoware_utils::calcSquaredDistance2d(a, b);
  if (0 <= dot_val && dot_val <= squared_segment_length) {
    const double numerator = std::abs((p.x - a.x) * (a.y - b.y) - (p.y - a.y) * (a.x - b.x));
    const double denominator = std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    return numerator / denominator;
  }

  // target_point is outside the segment.
  return std::min(
    tier4_autoware_utils::calcDistance2d(a, p), tier4_autoware_utils::calcDistance2d(b, p));
}

PolygonPoint transformBoundFrenetCoordinate(
  const std::vector<geometry_msgs::msg::Point> & bound_points,
  const geometry_msgs::msg::Point & target_point)
{
  // NOTE: findNearestSegmentIndex cannot be used since a bound's interval is sometimes too large to
  // find wrong nearest index.
  std::vector<double> dist_to_bound_segment_vec;
  for (size_t i = 0; i < bound_points.size() - 1; ++i) {
    const double dist_to_bound_segment =
      calcDistanceFromPointToSegment(bound_points.at(i), bound_points.at(i + 1), target_point);
    dist_to_bound_segment_vec.push_back(dist_to_bound_segment);
  }

  const size_t min_dist_seg_idx = std::distance(
    dist_to_bound_segment_vec.begin(),
    std::min_element(dist_to_bound_segment_vec.begin(), dist_to_bound_segment_vec.end()));
  const double lon_dist_to_segment =
    motion_utils::calcLongitudinalOffsetToSegment(bound_points, min_dist_seg_idx, target_point);
  const double lat_dist_to_segment =
    motion_utils::calcLateralOffset(bound_points, target_point, min_dist_seg_idx);
  return PolygonPoint{target_point, min_dist_seg_idx, lon_dist_to_segment, lat_dist_to_segment};
}

std::vector<PolygonPoint> generatePolygonInsideBounds(
  const std::vector<Point> & bound, const std::vector<Point> & edge_points,
  const bool is_object_right)
{
  constexpr double invalid_lat_dist_to_bound = 10.0;

  std::vector<PolygonPoint> full_polygon;
  for (const auto & edge_point : edge_points) {
    const auto polygon_point = transformBoundFrenetCoordinate(bound, edge_point);

    // check lat dist for U-turn roads.
    if (
      (is_object_right && invalid_lat_dist_to_bound < polygon_point.lat_dist_to_bound) ||
      (!is_object_right && polygon_point.lat_dist_to_bound < -invalid_lat_dist_to_bound)) {
      return {};
    }
    full_polygon.push_back(polygon_point);
  }

  // 1. check the case where the polygon intersects the bound
  std::vector<PolygonPoint> inside_poly;
  bool has_intersection = false;  // NOTE: between obstacle polygon and bound
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
    has_intersection = true;

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
  if (has_intersection) {
    return inside_poly;
  }

  // 2. check the case where the polygon does not intersect the bound
  const bool is_polygon_fully_inside_bounds = [&]() {
    for (const auto & curr_poly : full_polygon) {
      const bool is_curr_outside = curr_poly.is_outside_bounds(is_object_right);
      if (is_curr_outside) {
        return false;
      }
    }
    return true;
  }();
  if (is_polygon_fully_inside_bounds) {
    return full_polygon;
  }

  return std::vector<PolygonPoint>{};
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

// NOTE: See the PR's figure. https://github.com/autowarefoundation/autoware.universe/pull/2880
std::vector<PolygonPoint> concatenateTwoPolygons(
  const std::vector<PolygonPoint> & front_polygon, const std::vector<PolygonPoint> & back_polygon)
{
  const auto make_unique_polygon = [&](const auto & polygon) {
    std::vector<PolygonPoint> unique_polygon;
    for (const auto & point : polygon) {
      if (!unique_polygon.empty() && isSamePoint(unique_polygon.back().point, point.point)) {
        continue;
      }
      unique_polygon.push_back(point);
    }
    return unique_polygon;
  };
  const auto unique_front_polygon = make_unique_polygon(front_polygon);
  const auto unique_back_polygon = make_unique_polygon(back_polygon);

  // At first, the front polygon is the outside polygon
  bool is_front_polygon_outside = true;
  size_t before_outside_idx = 0;

  const auto get_out_poly = [&]() {
    return is_front_polygon_outside ? unique_front_polygon : unique_back_polygon;
  };
  const auto get_in_poly = [&]() {
    return is_front_polygon_outside ? unique_back_polygon : unique_front_polygon;
  };

  // NOTE: Polygon points is assumed to be clock-wise.
  std::vector<PolygonPoint> concatenated_polygon;
  // NOTE: Maximum number of loop is set to avoid infinity loop calculation just in case.
  const size_t max_loop_num = (unique_front_polygon.size() + unique_back_polygon.size()) * 2;
  for (size_t loop_idx = 0; loop_idx < max_loop_num; ++loop_idx) {
    concatenated_polygon.push_back(get_out_poly().at(before_outside_idx));
    if (before_outside_idx == get_out_poly().size() - 1) {
      break;
    }
    const size_t curr_idx = before_outside_idx;
    const size_t next_idx = before_outside_idx + 1;

    // NOTE: Two polygons may have two intersection points. Therefore the closest intersection
    //       point is used.
    std::optional<size_t> closest_idx = std::nullopt;
    double min_dist_to_intersection = std::numeric_limits<double>::max();
    PolygonPoint closest_intersect_point;
    for (size_t i = 0; i < get_in_poly().size() - 1; ++i) {
      const auto intersection = tier4_autoware_utils::intersect(
        get_out_poly().at(curr_idx).point, get_out_poly().at(next_idx).point,
        get_in_poly().at(i).point, get_in_poly().at(i + 1).point);
      if (!intersection) {
        continue;
      }
      if (
        isSamePoint(get_out_poly().at(curr_idx).point, get_in_poly().at(i).point) ||
        isSamePoint(get_out_poly().at(curr_idx).point, get_in_poly().at(i + 1).point) ||
        isSamePoint(get_out_poly().at(next_idx).point, get_in_poly().at(i).point) ||
        isSamePoint(get_out_poly().at(next_idx).point, get_in_poly().at(i + 1).point)) {
        // NOTE: If the segments shares one point, the while loop will not end.
        continue;
      }

      const auto intersect_point = PolygonPoint{*intersection, 0, 0.0, 0.0};
      const double dist_to_intersection =
        tier4_autoware_utils::calcDistance2d(get_out_poly().at(curr_idx).point, *intersection);
      if (dist_to_intersection < min_dist_to_intersection) {
        closest_idx = i;
        min_dist_to_intersection = dist_to_intersection;
        closest_intersect_point = intersect_point;
      }
    }

    if (closest_idx) {
      before_outside_idx = *closest_idx;
      concatenated_polygon.push_back(closest_intersect_point);
      is_front_polygon_outside = !is_front_polygon_outside;
    }

    before_outside_idx += 1;

    if (loop_idx == max_loop_num - 1) {
      return front_polygon;
    }
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

[[maybe_unused]] geometry_msgs::msg::Point calcCenterOfGeometry(const Polygon2d & obj_poly)
{
  geometry_msgs::msg::Point center_pos;
  for (const auto & point : obj_poly.outer()) {
    center_pos.x += point.x();
    center_pos.y += point.y();
  }

  center_pos.x = center_pos.x / obj_poly.outer().size();
  center_pos.y = center_pos.y / obj_poly.outer().size();
  center_pos.z = center_pos.z / obj_poly.outer().size();

  return center_pos;
}
}  // namespace drivable_area_processing

std::optional<lanelet::Polygon3d> getPolygonByPoint(
  const std::shared_ptr<RouteHandler> & route_handler, const lanelet::ConstPoint3d & point,
  const std::string & polygon_name)
{
  const auto polygons = route_handler->getLaneletMapPtr()->polygonLayer.findUsages(point);
  for (const auto & polygon : polygons) {
    const std::string type = polygon.attributeOr(lanelet::AttributeName::Type, "none");
    if (type == polygon_name) {
      // NOTE: If there are multiple polygons on a point, only the front one is used.
      return polygon;
    }
  }
  return std::nullopt;
}

double l2Norm(const Vector3 vector)
{
  return std::sqrt(std::pow(vector.x, 2) + std::pow(vector.y, 2) + std::pow(vector.z, 2));
}

double getDistanceBetweenPredictedPaths(
  const PredictedPath & object_path, const PredictedPath & ego_path, const double start_time,
  const double end_time, const double resolution)
{
  double min_distance = std::numeric_limits<double>::max();
  for (double t = start_time; t < end_time; t += resolution) {
    const auto object_pose = object_recognition_utils::calcInterpolatedPose(object_path, t);
    if (!object_pose) {
      continue;
    }
    const auto ego_pose = object_recognition_utils::calcInterpolatedPose(ego_path, t);
    if (!ego_pose) {
      continue;
    }
    double distance = tier4_autoware_utils::calcDistance3d(*object_pose, *ego_pose);
    if (distance < min_distance) {
      min_distance = distance;
    }
  }
  return min_distance;
}

double getDistanceBetweenPredictedPathAndObject(
  const PredictedObject & object, const PredictedPath & ego_path, const double start_time,
  const double end_time, const double resolution)
{
  auto clock{rclcpp::Clock{RCL_ROS_TIME}};
  auto t_delta{rclcpp::Duration::from_seconds(resolution)};
  double min_distance = std::numeric_limits<double>::max();
  rclcpp::Time ros_start_time = clock.now() + rclcpp::Duration::from_seconds(start_time);
  rclcpp::Time ros_end_time = clock.now() + rclcpp::Duration::from_seconds(end_time);
  const auto obj_polygon = tier4_autoware_utils::toPolygon2d(object);
  for (double t = start_time; t < end_time; t += resolution) {
    const auto ego_pose = object_recognition_utils::calcInterpolatedPose(ego_path, t);
    if (!ego_pose) {
      continue;
    }
    Point2d ego_point{ego_pose->position.x, ego_pose->position.y};

    double distance = boost::geometry::distance(obj_polygon, ego_point);
    if (distance < min_distance) {
      min_distance = distance;
    }
  }
  return min_distance;
}

bool checkCollisionBetweenPathFootprintsAndObjects(
  const tier4_autoware_utils::LinearRing2d & local_vehicle_footprint,
  const PathWithLaneId & ego_path, const PredictedObjects & dynamic_objects, const double margin)
{
  for (const auto & p : ego_path.points) {
    if (checkCollisionBetweenFootprintAndObjects(
          local_vehicle_footprint, p.point.pose, dynamic_objects, margin)) {
      return true;
    }
  }
  return false;
}

bool checkCollisionBetweenFootprintAndObjects(
  const tier4_autoware_utils::LinearRing2d & local_vehicle_footprint, const Pose & ego_pose,
  const PredictedObjects & dynamic_objects, const double margin)
{
  const auto vehicle_footprint =
    transformVector(local_vehicle_footprint, tier4_autoware_utils::pose2transform(ego_pose));

  for (const auto & object : dynamic_objects.objects) {
    const auto obj_polygon = tier4_autoware_utils::toPolygon2d(object);
    const double distance = boost::geometry::distance(obj_polygon, vehicle_footprint);
    if (distance < margin) return true;
  }
  return false;
}

double calcLateralDistanceFromEgoToObject(
  const Pose & ego_pose, const double vehicle_width, const PredictedObject & dynamic_object)
{
  double min_distance = std::numeric_limits<double>::max();
  const auto obj_polygon = tier4_autoware_utils::toPolygon2d(dynamic_object);
  const auto vehicle_left_pose =
    tier4_autoware_utils::calcOffsetPose(ego_pose, 0, vehicle_width / 2, 0);
  const auto vehicle_right_pose =
    tier4_autoware_utils::calcOffsetPose(ego_pose, 0, -vehicle_width / 2, 0);

  for (const auto & p : obj_polygon.outer()) {
    const auto point = tier4_autoware_utils::createPoint(p.x(), p.y(), 0.0);
    const double signed_distance_from_left =
      tier4_autoware_utils::calcLateralDeviation(vehicle_left_pose, point);
    const double signed_distance_from_right =
      tier4_autoware_utils::calcLateralDeviation(vehicle_right_pose, point);

    if (signed_distance_from_left < 0.0 && signed_distance_from_right > 0.0) {
      // point is between left and right
      return 0.0;
    }

    const double distance_from_ego =
      std::min(std::abs(signed_distance_from_left), std::abs(signed_distance_from_right));
    min_distance = std::min(min_distance, distance_from_ego);
  }
  return min_distance;
}

double calcLongitudinalDistanceFromEgoToObject(
  const Pose & ego_pose, const double base_link2front, const double base_link2rear,
  const PredictedObject & dynamic_object)
{
  double min_distance = std::numeric_limits<double>::max();
  const auto obj_polygon = tier4_autoware_utils::toPolygon2d(dynamic_object);
  const auto vehicle_front_pose =
    tier4_autoware_utils::calcOffsetPose(ego_pose, base_link2front, 0, 0);
  const auto vehicle_rear_pose =
    tier4_autoware_utils::calcOffsetPose(ego_pose, base_link2rear, 0, 0);

  for (const auto & p : obj_polygon.outer()) {
    const auto point = tier4_autoware_utils::createPoint(p.x(), p.y(), 0.0);

    // forward is positive
    const double signed_distance_from_front =
      tier4_autoware_utils::calcLongitudinalDeviation(vehicle_front_pose, point);
    // backward is positive
    const double signed_distance_from_rear =
      -tier4_autoware_utils::calcLongitudinalDeviation(vehicle_rear_pose, point);

    if (signed_distance_from_front < 0.0 && signed_distance_from_rear < 0.0) {
      // point is between front and rear
      return 0.0;
    }

    const double distance_from_ego =
      std::min(std::abs(signed_distance_from_front), std::abs(signed_distance_from_rear));
    min_distance = std::min(min_distance, distance_from_ego);
  }
  return min_distance;
}

double calcLongitudinalDistanceFromEgoToObjects(
  const Pose & ego_pose, double base_link2front, double base_link2rear,
  const PredictedObjects & dynamic_objects)
{
  double min_distance = std::numeric_limits<double>::max();
  for (const auto & object : dynamic_objects.objects) {
    min_distance = std::min(
      min_distance,
      calcLongitudinalDistanceFromEgoToObject(ego_pose, base_link2front, base_link2rear, object));
  }
  return min_distance;
}

std::vector<double> calcObjectsDistanceToPath(
  const PredictedObjects & objects, const PathWithLaneId & ego_path)
{
  std::vector<double> distance_array;
  for (const auto & obj : objects.objects) {
    const auto obj_polygon = tier4_autoware_utils::toPolygon2d(obj);
    LineString2d ego_path_line;
    ego_path_line.reserve(ego_path.points.size());
    for (const auto & p : ego_path.points) {
      boost::geometry::append(
        ego_path_line, Point2d(p.point.pose.position.x, p.point.pose.position.y));
    }
    const double distance = boost::geometry::distance(obj_polygon, ego_path_line);
    distance_array.push_back(distance);
  }
  return distance_array;
}

template <typename T>
bool exists(std::vector<T> vec, T element)
{
  return std::find(vec.begin(), vec.end(), element) != vec.end();
}

boost::optional<size_t> findIndexOutOfGoalSearchRange(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const Pose & goal, const int64_t goal_lane_id,
  const double max_dist = std::numeric_limits<double>::max())
{
  if (points.empty()) {
    return boost::none;
  }

  // find goal index
  size_t min_dist_index;
  double min_dist = std::numeric_limits<double>::max();
  {
    bool found = false;
    for (size_t i = 0; i < points.size(); ++i) {
      const auto & lane_ids = points.at(i).lane_ids;

      const double dist_to_goal =
        tier4_autoware_utils::calcDistance2d(points.at(i).point.pose, goal);
      const bool is_goal_lane_id_in_point =
        std::find(lane_ids.begin(), lane_ids.end(), goal_lane_id) != lane_ids.end();
      if (dist_to_goal < max_dist && dist_to_goal < min_dist && is_goal_lane_id_in_point) {
        min_dist_index = i;
        min_dist = dist_to_goal;
        found = true;
      }
    }
    if (!found) {
      return boost::none;
    }
  }

  // find index out of goal search range
  size_t min_dist_out_of_range_index = min_dist_index;
  for (int i = min_dist_index; 0 <= i; --i) {
    const double dist = tier4_autoware_utils::calcDistance2d(points.at(i).point, goal);
    min_dist_out_of_range_index = i;
    if (max_dist < dist) {
      break;
    }
  }

  return min_dist_out_of_range_index;
}

// goal does not have z
bool setGoal(
  const double search_radius_range, [[maybe_unused]] const double search_rad_range,
  const PathWithLaneId & input, const Pose & goal, const int64_t goal_lane_id,
  PathWithLaneId * output_ptr)
{
  try {
    if (input.points.empty()) {
      return false;
    }

    // calculate refined_goal with interpolation
    // NOTE: goal does not have valid z, that will be calculated by interpolation here
    PathPointWithLaneId refined_goal{};
    const size_t closest_seg_idx_for_goal =
      findNearestSegmentIndex(input.points, goal, 3.0, M_PI_4);
    refined_goal.point.pose = goal;
    refined_goal.point.pose.position.z =
      calcInterpolatedZ(input, goal.position, closest_seg_idx_for_goal);
    refined_goal.point.longitudinal_velocity_mps = 0.0;

    // calculate pre_refined_goal with interpolation
    // NOTE: z and velocity are filled
    PathPointWithLaneId pre_refined_goal{};
    constexpr double goal_to_pre_goal_distance = -1.0;
    pre_refined_goal.point.pose =
      tier4_autoware_utils::calcOffsetPose(goal, goal_to_pre_goal_distance, 0.0, 0.0);
    const size_t closest_seg_idx_for_pre_goal =
      findNearestSegmentIndex(input.points, pre_refined_goal.point.pose, 3.0, M_PI_4);
    pre_refined_goal.point.pose.position.z =
      calcInterpolatedZ(input, pre_refined_goal.point.pose.position, closest_seg_idx_for_pre_goal);
    pre_refined_goal.point.longitudinal_velocity_mps =
      calcInterpolatedVelocity(input, closest_seg_idx_for_pre_goal);

    // find min_dist_out_of_circle_index whose distance to goal is longer than search_radius_range
    const auto min_dist_out_of_circle_index_opt =
      findIndexOutOfGoalSearchRange(input.points, goal, goal_lane_id, search_radius_range);
    if (!min_dist_out_of_circle_index_opt) {
      return false;
    }
    const size_t min_dist_out_of_circle_index = min_dist_out_of_circle_index_opt.get();

    // create output points
    output_ptr->points.reserve(output_ptr->points.size() + min_dist_out_of_circle_index + 3);
    for (size_t i = 0; i <= min_dist_out_of_circle_index; ++i) {
      output_ptr->points.push_back(input.points.at(i));
    }
    output_ptr->points.push_back(pre_refined_goal);
    output_ptr->points.push_back(refined_goal);

    {  // fill skipped lane ids
      // pre refined goal
      auto & pre_goal = output_ptr->points.at(output_ptr->points.size() - 2);
      for (size_t i = min_dist_out_of_circle_index + 1; i < input.points.size(); ++i) {
        for (const auto target_lane_id : input.points.at(i).lane_ids) {
          const bool is_lane_id_found =
            std::find(pre_goal.lane_ids.begin(), pre_goal.lane_ids.end(), target_lane_id) !=
            pre_goal.lane_ids.end();
          if (!is_lane_id_found) {
            pre_goal.lane_ids.push_back(target_lane_id);
          }
        }
      }

      // goal
      output_ptr->points.back().lane_ids = input.points.back().lane_ids;
    }

    output_ptr->left_bound = input.left_bound;
    output_ptr->right_bound = input.right_bound;
    return true;
  } catch (std::out_of_range & ex) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("behavior_path_planner").get_child("utils"),
      "failed to set goal: " << ex.what());
    return false;
  }
}

const Pose refineGoal(const Pose & goal, const lanelet::ConstLanelet & goal_lanelet)
{
  // return goal;
  const auto lanelet_point = lanelet::utils::conversion::toLaneletPoint(goal.position);
  const double distance = boost::geometry::distance(
    goal_lanelet.polygon2d().basicPolygon(), lanelet::utils::to2D(lanelet_point).basicPoint());
  if (distance < std::numeric_limits<double>::epsilon()) {
    return goal;
  }

  const auto segment = lanelet::utils::getClosestSegment(
    lanelet::utils::to2D(lanelet_point), goal_lanelet.centerline());
  if (segment.empty()) {
    return goal;
  }

  Pose refined_goal;
  {
    // find position
    const auto p1 = segment.front().basicPoint();
    const auto p2 = segment.back().basicPoint();
    const auto direction_vector = (p2 - p1).normalized();
    const auto p1_to_goal = lanelet_point.basicPoint() - p1;
    const double s = direction_vector.dot(p1_to_goal);
    const auto refined_point = p1 + direction_vector * s;

    refined_goal.position.x = refined_point.x();
    refined_goal.position.y = refined_point.y();
    refined_goal.position.z = refined_point.z();

    // find orientation
    const double yaw = std::atan2(direction_vector.y(), direction_vector.x());
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(0, 0, yaw);
    refined_goal.orientation = tf2::toMsg(tf_quat);
  }
  return refined_goal;
}

PathWithLaneId refinePathForGoal(
  const double search_radius_range, const double search_rad_range, const PathWithLaneId & input,
  const Pose & goal, const int64_t goal_lane_id)
{
  PathWithLaneId filtered_path = input;
  PathWithLaneId path_with_goal;
  filtered_path.points = motion_utils::removeOverlapPoints(filtered_path.points);

  // always set zero velocity at the end of path for safety
  if (!filtered_path.points.empty()) {
    filtered_path.points.back().point.longitudinal_velocity_mps = 0.0;
  }

  if (setGoal(
        search_radius_range, search_rad_range, filtered_path, goal, goal_lane_id,
        &path_with_goal)) {
    return path_with_goal;
  } else {
    return filtered_path;
  }
}

bool containsGoal(const lanelet::ConstLanelets & lanes, const lanelet::Id & goal_id)
{
  for (const auto & lane : lanes) {
    if (lane.id() == goal_id) {
      return true;
    }
  }
  return false;
}

BehaviorModuleOutput createGoalAroundPath(const std::shared_ptr<const PlannerData> & planner_data)
{
  BehaviorModuleOutput output;

  const auto & route_handler = planner_data->route_handler;
  const auto & modified_goal = planner_data->prev_modified_goal;

  const Pose goal_pose = modified_goal ? modified_goal->pose : route_handler->getGoalPose();
  const auto shoulder_lanes = route_handler->getShoulderLanelets();

  lanelet::ConstLanelet goal_lane;
  const bool is_failed_getting_lanelet = std::invoke([&]() {
    if (isInLanelets(goal_pose, shoulder_lanes)) {
      return !lanelet::utils::query::getClosestLanelet(shoulder_lanes, goal_pose, &goal_lane);
    }
    return !route_handler->getGoalLanelet(&goal_lane);
  });
  if (is_failed_getting_lanelet) {
    return output;
  }

  constexpr double backward_length = 1.0;
  const auto arc_coord = lanelet::utils::getArcCoordinates({goal_lane}, goal_pose);
  const double s_start = std::max(arc_coord.length - backward_length, 0.0);
  const double s_end = arc_coord.length;

  auto reference_path = route_handler->getCenterLinePath({goal_lane}, s_start, s_end);

  const auto drivable_lanelets = getLaneletsFromPath(reference_path, route_handler);
  const auto drivable_lanes = generateDrivableLanes(drivable_lanelets);

  const auto & dp = planner_data->drivable_area_expansion_parameters;

  const auto shorten_lanes = cutOverlappedLanes(reference_path, drivable_lanes);
  const auto expanded_lanes = expandLanelets(
    shorten_lanes, dp.drivable_area_left_bound_offset, dp.drivable_area_right_bound_offset,
    dp.drivable_area_types_to_skip);

  output.path = std::make_shared<PathWithLaneId>(reference_path);
  output.reference_path = std::make_shared<PathWithLaneId>(reference_path);
  output.drivable_area_info.drivable_lanes = drivable_lanes;

  return output;
}

bool isInLanelets(const Pose & pose, const lanelet::ConstLanelets & lanes)
{
  for (const auto & lane : lanes) {
    if (lanelet::utils::isInLanelet(pose, lane)) {
      return true;
    }
  }
  return false;
}

bool isInLaneletWithYawThreshold(
  const Pose & current_pose, const lanelet::ConstLanelet & lanelet, const double yaw_threshold,
  const double radius)
{
  const double pose_yaw = tf2::getYaw(current_pose.orientation);
  const double lanelet_angle = lanelet::utils::getLaneletAngle(lanelet, current_pose.position);
  const double angle_diff =
    std::abs(tier4_autoware_utils::normalizeRadian(lanelet_angle - pose_yaw));

  return (angle_diff < std::abs(yaw_threshold)) &&
         lanelet::utils::isInLanelet(current_pose, lanelet, radius);
}

bool isEgoOutOfRoute(
  const Pose & self_pose, const std::optional<PoseWithUuidStamped> & modified_goal,
  const std::shared_ptr<RouteHandler> & route_handler)
{
  const Pose & goal_pose = (modified_goal && modified_goal->uuid == route_handler->getRouteUuid())
                             ? modified_goal->pose
                             : route_handler->getGoalPose();
  const auto shoulder_lanes = route_handler->getShoulderLanelets();

  lanelet::ConstLanelet goal_lane;
  const bool is_failed_getting_lanelet = std::invoke([&]() {
    if (utils::isInLanelets(goal_pose, shoulder_lanes)) {
      return !lanelet::utils::query::getClosestLanelet(shoulder_lanes, goal_pose, &goal_lane);
    }
    return !route_handler->getGoalLanelet(&goal_lane);
  });
  if (is_failed_getting_lanelet) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("behavior_path_planner").get_child("util"), "cannot find goal lanelet");
    return true;
  }

  // If ego vehicle is over goal on goal lane, return true
  const double yaw_threshold = tier4_autoware_utils::deg2rad(90);
  if (isInLaneletWithYawThreshold(self_pose, goal_lane, yaw_threshold)) {
    constexpr double buffer = 1.0;
    const auto ego_arc_coord = lanelet::utils::getArcCoordinates({goal_lane}, self_pose);
    const auto goal_arc_coord =
      lanelet::utils::getArcCoordinates({goal_lane}, route_handler->getGoalPose());
    if (ego_arc_coord.length > goal_arc_coord.length + buffer) {
      return true;
    } else {
      return false;
    }
  }

  // If ego vehicle is out of the closest lanelet, return true
  // Check if ego vehicle is in shoulder lane
  const bool is_in_shoulder_lane = std::invoke([&]() {
    lanelet::Lanelet closest_shoulder_lanelet;
    if (!lanelet::utils::query::getClosestLanelet(
          shoulder_lanes, self_pose, &closest_shoulder_lanelet)) {
      return false;
    }
    return lanelet::utils::isInLanelet(self_pose, closest_shoulder_lanelet);
  });
  // Check if ego vehicle is in road lane
  const bool is_in_road_lane = std::invoke([&]() {
    lanelet::ConstLanelet closest_road_lane;
    if (!route_handler->getClosestLaneletWithinRoute(self_pose, &closest_road_lane)) {
      RCLCPP_WARN_STREAM(
        rclcpp::get_logger("behavior_path_planner").get_child("util"),
        "cannot find closest road lanelet");
      return false;
    }

    if (lanelet::utils::isInLanelet(self_pose, closest_road_lane)) {
      return true;
    }

    // check previous lanes for backward driving (e.g. pull out)
    const auto prev_lanes = route_handler->getPreviousLanelets(closest_road_lane);
    for (const auto & lane : prev_lanes) {
      if (lanelet::utils::isInLanelet(self_pose, lane)) {
        return true;
      }
    }

    return false;
  });
  if (!is_in_shoulder_lane && !is_in_road_lane) {
    return true;
  }

  return false;
}

bool isEgoWithinOriginalLane(
  const lanelet::ConstLanelets & current_lanes, const Pose & current_pose,
  const BehaviorPathPlannerParameters & common_param, const double outer_margin)
{
  const auto lane_length = lanelet::utils::getLaneletLength2d(current_lanes);
  const auto lane_poly = lanelet::utils::getPolygonFromArcLength(current_lanes, 0, lane_length);
  const auto base_link2front = common_param.base_link2front;
  const auto base_link2rear = common_param.base_link2rear;
  const auto vehicle_width = common_param.vehicle_width;
  const auto vehicle_poly =
    tier4_autoware_utils::toFootprint(current_pose, base_link2front, base_link2rear, vehicle_width);

  // Check if the ego vehicle is entirely within the lane with a given outer margin.
  for (const auto & p : vehicle_poly.outer()) {
    // When the point is in the polygon, the distance is 0. When it is out of the polygon, return a
    // positive value.
    const auto dist = boost::geometry::distance(p, lanelet::utils::to2D(lane_poly).basicPolygon());
    if (dist > std::max(outer_margin, 0.0)) {
      return false;  // out of polygon
    }
  }

  return true;  // inside polygon
}

lanelet::ConstLanelets transformToLanelets(const DrivableLanes & drivable_lanes)
{
  lanelet::ConstLanelets lanes;

  const auto has_same_lane = [&](const auto & lane) {
    if (lanes.empty()) return false;
    const auto has_same = [&](const auto & ll) { return ll.id() == lane.id(); };
    return std::find_if(lanes.begin(), lanes.end(), has_same) != lanes.end();
  };

  lanes.push_back(drivable_lanes.right_lane);
  if (!has_same_lane(drivable_lanes.left_lane)) {
    lanes.push_back(drivable_lanes.left_lane);
  }

  for (const auto & ml : drivable_lanes.middle_lanes) {
    if (!has_same_lane(ml)) {
      lanes.push_back(ml);
    }
  }

  return lanes;
}

lanelet::ConstLanelets transformToLanelets(const std::vector<DrivableLanes> & drivable_lanes)
{
  lanelet::ConstLanelets lanes;

  for (const auto & drivable_lane : drivable_lanes) {
    const auto transformed_lane = transformToLanelets(drivable_lane);
    lanes.insert(lanes.end(), transformed_lane.begin(), transformed_lane.end());
  }

  return lanes;
}

boost::optional<lanelet::ConstLanelet> getRightLanelet(
  const lanelet::ConstLanelet & current_lane, const lanelet::ConstLanelets & shoulder_lanes)
{
  for (const auto & shoulder_lane : shoulder_lanes) {
    if (shoulder_lane.leftBound().id() == current_lane.rightBound().id()) {
      return shoulder_lane;
    }
  }

  return {};
}

boost::optional<lanelet::ConstLanelet> getLeftLanelet(
  const lanelet::ConstLanelet & current_lane, const lanelet::ConstLanelets & shoulder_lanes)
{
  for (const auto & shoulder_lane : shoulder_lanes) {
    if (shoulder_lane.rightBound().id() == current_lane.leftBound().id()) {
      return shoulder_lane;
    }
  }

  return {};
}

std::vector<DrivableLanes> generateDrivableLanes(const lanelet::ConstLanelets & lanes)
{
  std::vector<DrivableLanes> drivable_lanes(lanes.size());
  for (size_t i = 0; i < lanes.size(); ++i) {
    drivable_lanes.at(i).left_lane = lanes.at(i);
    drivable_lanes.at(i).right_lane = lanes.at(i);
  }
  return drivable_lanes;
}

std::vector<DrivableLanes> generateDrivableLanesWithShoulderLanes(
  const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelets & shoulder_lanes)
{
  std::vector<DrivableLanes> drivable_lanes;
  for (const auto & current_lane : current_lanes) {
    DrivableLanes drivable_lane;

    const auto right_lane = getRightLanelet(current_lane, shoulder_lanes);
    const auto left_lane = getLeftLanelet(current_lane, shoulder_lanes);

    if (right_lane && left_lane) {
      drivable_lane.right_lane = *right_lane;
      drivable_lane.left_lane = *left_lane;
      drivable_lane.middle_lanes.push_back(current_lane);
    } else if (right_lane) {
      drivable_lane.right_lane = *right_lane;
      drivable_lane.left_lane = current_lane;
    } else if (left_lane) {
      drivable_lane.right_lane = current_lane;
      drivable_lane.left_lane = *left_lane;
    } else {
      drivable_lane.right_lane = current_lane;
      drivable_lane.left_lane = current_lane;
    }

    drivable_lanes.push_back(drivable_lane);
  }

  return drivable_lanes;
}

std::vector<DrivableLanes> getNonOverlappingExpandedLanes(
  PathWithLaneId & path, const std::vector<DrivableLanes> & lanes,
  const DrivableAreaExpansionParameters & parameters)
{
  const auto shorten_lanes = utils::cutOverlappedLanes(path, lanes);
  return utils::expandLanelets(
    shorten_lanes, parameters.drivable_area_left_bound_offset,
    parameters.drivable_area_right_bound_offset, parameters.drivable_area_types_to_skip);
}

boost::optional<size_t> getOverlappedLaneletId(const std::vector<DrivableLanes> & lanes)
{
  auto overlaps = [](const DrivableLanes & lanes, const DrivableLanes & target_lanes) {
    const auto lanelets = transformToLanelets(lanes);
    const auto target_lanelets = transformToLanelets(target_lanes);

    for (const auto & lanelet : lanelets) {
      for (const auto & target_lanelet : target_lanelets) {
        std::vector<Point2d> intersections{};
        boost::geometry::intersection(
          lanelet.polygon2d().basicPolygon(), target_lanelet.polygon2d().basicPolygon(),
          intersections);

        // if only one point intersects, it is assumed not to be overlapped
        if (intersections.size() > 1) {
          return true;
        }
      }
    }

    // No overlapping
    return false;
  };

  if (lanes.size() <= 2) {
    return {};
  }

  size_t overlapped_idx = lanes.size();
  for (size_t i = 0; i < lanes.size() - 2; ++i) {
    for (size_t j = i + 2; j < lanes.size(); ++j) {
      if (overlaps(lanes.at(i), lanes.at(j))) {
        overlapped_idx = std::min(overlapped_idx, j);
      }
    }
  }

  if (overlapped_idx == lanes.size()) {
    return {};
  }

  return overlapped_idx;
}

std::vector<DrivableLanes> cutOverlappedLanes(
  PathWithLaneId & path, const std::vector<DrivableLanes> & lanes)
{
  const auto overlapped_lanelet_idx = getOverlappedLaneletId(lanes);
  if (!overlapped_lanelet_idx) {
    return lanes;
  }

  std::vector<DrivableLanes> shorten_lanes{lanes.begin(), lanes.begin() + *overlapped_lanelet_idx};
  const auto shorten_lanelets = utils::transformToLanelets(shorten_lanes);

  const auto original_points = path.points;

  path.points.clear();

  const auto has_same_id_lane = [](const auto & lanelet, const auto & p) {
    return std::any_of(p.lane_ids.begin(), p.lane_ids.end(), [&lanelet](const auto id) {
      return lanelet.id() == id;
    });
  };

  const auto has_same_id_lanes = [&has_same_id_lane](const auto & lanelets, const auto & p) {
    return std::any_of(
      lanelets.begin(), lanelets.end(),
      [&has_same_id_lane, &p](const auto & lanelet) { return has_same_id_lane(lanelet, p); });
  };

  // Step1. find first path point within drivable lanes
  size_t start_point_idx = std::numeric_limits<size_t>::max();
  for (const auto & lanes : shorten_lanes) {
    for (size_t i = 0; i < original_points.size(); ++i) {
      // check right lane
      if (has_same_id_lane(lanes.right_lane, original_points.at(i))) {
        start_point_idx = std::min(start_point_idx, i);
      }

      // check left lane
      if (has_same_id_lane(lanes.left_lane, original_points.at(i))) {
        start_point_idx = std::min(start_point_idx, i);
      }

      // check middle lanes
      if (has_same_id_lanes(lanes.middle_lanes, original_points.at(i))) {
        start_point_idx = std::min(start_point_idx, i);
      }
    }
  }

  // Step2. pick up only path points within drivable lanes
  for (const auto & lanes : shorten_lanes) {
    for (size_t i = start_point_idx; i < original_points.size(); ++i) {
      // check right lane
      if (has_same_id_lane(lanes.right_lane, original_points.at(i))) {
        path.points.push_back(original_points.at(i));
        continue;
      }

      // check left lane
      if (has_same_id_lane(lanes.left_lane, original_points.at(i))) {
        path.points.push_back(original_points.at(i));
        continue;
      }

      // check middle lanes
      if (has_same_id_lanes(lanes.middle_lanes, original_points.at(i))) {
        path.points.push_back(original_points.at(i));
        continue;
      }

      start_point_idx = i;
      break;
    }
  }

  return shorten_lanes;
}

geometry_msgs::msg::Point calcLongitudinalOffsetStartPoint(
  const std::vector<geometry_msgs::msg::Point> & points, const geometry_msgs::msg::Pose & pose,
  const size_t nearest_segment_idx, const double offset)
{
  const double offset_length =
    motion_utils::calcLongitudinalOffsetToSegment(points, nearest_segment_idx, pose.position);
  const auto offset_point =
    motion_utils::calcLongitudinalOffsetPoint(points, nearest_segment_idx, offset_length + offset);

  return offset_point ? offset_point.get() : points.at(nearest_segment_idx);
}

geometry_msgs::msg::Point calcLongitudinalOffsetGoalPoint(
  const std::vector<geometry_msgs::msg::Point> & points, const geometry_msgs::msg::Pose & pose,
  const size_t nearest_segment_idx, const double offset)
{
  const double offset_length =
    motion_utils::calcLongitudinalOffsetToSegment(points, nearest_segment_idx, pose.position);
  const auto offset_point =
    motion_utils::calcLongitudinalOffsetPoint(points, nearest_segment_idx, offset_length + offset);

  return offset_point ? offset_point.get() : points.at(nearest_segment_idx + 1);
}

void generateDrivableArea(
  PathWithLaneId & path, const std::vector<DrivableLanes> & lanes,
  const bool enable_expanding_hatched_road_markings, const bool enable_expanding_intersection_areas,
  const double vehicle_length, const std::shared_ptr<const PlannerData> planner_data,
  const bool is_driving_forward)
{
  // extract data
  const auto transformed_lanes = utils::transformToLanelets(lanes);
  const auto current_pose = planner_data->self_odometry->pose.pose;
  const auto route_handler = planner_data->route_handler;
  constexpr double overlap_threshold = 0.01;

  if (path.points.empty()) {
    return;
  }

  auto addPoints =
    [](const lanelet::ConstLineString3d & points, std::vector<geometry_msgs::msg::Point> & bound) {
      for (const auto & bound_p : points) {
        const auto cp = lanelet::utils::conversion::toGeomMsgPt(bound_p);
        if (bound.empty()) {
          bound.push_back(cp);
        } else if (tier4_autoware_utils::calcDistance2d(cp, bound.back()) > overlap_threshold) {
          bound.push_back(cp);
        }
      }
    };

  const auto has_overlap =
    [&](const lanelet::ConstLanelet & lane, const lanelet::ConstLanelets & ignore_lanelets = {}) {
      for (const auto & transformed_lane : transformed_lanes) {
        if (checkHasSameLane(ignore_lanelets, transformed_lane)) {
          continue;
        }
        if (boost::geometry::intersects(
              lane.polygon2d().basicPolygon(), transformed_lane.polygon2d().basicPolygon())) {
          return true;
        }
      }
      return false;
    };

  // Insert Position
  auto left_bound = calcBound(
    route_handler, lanes, enable_expanding_hatched_road_markings,
    enable_expanding_intersection_areas, true);
  auto right_bound = calcBound(
    route_handler, lanes, enable_expanding_hatched_road_markings,
    enable_expanding_intersection_areas, false);

  if (left_bound.empty() || right_bound.empty()) {
    auto clock{rclcpp::Clock{RCL_ROS_TIME}};
    RCLCPP_ERROR_STREAM_THROTTLE(
      rclcpp::get_logger("behavior_path_planner").get_child("utils"), clock, 1000,
      "The right or left bound of drivable area is empty");
    return;
  }

  // Insert points after goal
  lanelet::ConstLanelet goal_lanelet;
  if (
    route_handler->getGoalLanelet(&goal_lanelet) &&
    checkHasSameLane(transformed_lanes, goal_lanelet)) {
    const auto lanes_after_goal = route_handler->getLanesAfterGoal(vehicle_length);
    const auto next_lanes_after_goal = route_handler->getNextLanelets(goal_lanelet);
    const auto goal_left_lanelet = route_handler->getLeftLanelet(goal_lanelet);
    const auto goal_right_lanelet = route_handler->getRightLanelet(goal_lanelet);
    lanelet::ConstLanelets goal_lanelets = {goal_lanelet};
    if (goal_left_lanelet) {
      goal_lanelets.push_back(*goal_left_lanelet);
    }
    if (goal_right_lanelet) {
      goal_lanelets.push_back(*goal_right_lanelet);
    }

    for (const auto & lane : lanes_after_goal) {
      // If lane is already in the transformed lanes, ignore it
      if (checkHasSameLane(transformed_lanes, lane)) {
        continue;
      }
      // Check if overlapped
      const bool is_overlapped =
        (checkHasSameLane(next_lanes_after_goal, lane) ? has_overlap(lane, goal_lanelets)
                                                       : has_overlap(lane));
      if (is_overlapped) {
        continue;
      }

      addPoints(lane.leftBound3d(), left_bound);
      addPoints(lane.rightBound3d(), right_bound);
    }
  }

  if (!is_driving_forward) {
    std::reverse(left_bound.begin(), left_bound.end());
    std::reverse(right_bound.begin(), right_bound.end());
  }

  path.left_bound.clear();
  path.right_bound.clear();

  const auto [left_start_idx, right_start_idx] = [&]() {
    const size_t current_seg_idx = planner_data->findEgoSegmentIndex(path.points);
    const auto cropped_path_points = motion_utils::cropPoints(
      path.points, current_pose.position, current_seg_idx,
      planner_data->parameters.forward_path_length,
      planner_data->parameters.backward_path_length + planner_data->parameters.input_path_interval);

    constexpr double front_length = 0.5;
    const auto front_pose =
      cropped_path_points.empty() ? current_pose : cropped_path_points.front().point.pose;
    const size_t front_left_start_idx =
      findNearestSegmentIndexFromLateralDistance(left_bound, front_pose.position);
    const size_t front_right_start_idx =
      findNearestSegmentIndexFromLateralDistance(right_bound, front_pose.position);
    const auto left_start_point =
      calcLongitudinalOffsetStartPoint(left_bound, front_pose, front_left_start_idx, -front_length);
    const auto right_start_point = calcLongitudinalOffsetStartPoint(
      right_bound, front_pose, front_right_start_idx, -front_length);
    const size_t left_start_idx =
      findNearestSegmentIndexFromLateralDistance(left_bound, left_start_point);
    const size_t right_start_idx =
      findNearestSegmentIndexFromLateralDistance(right_bound, right_start_point);

    // Insert a start point
    path.left_bound.push_back(left_start_point);
    path.right_bound.push_back(right_start_point);

    return std::make_pair(left_start_idx, right_start_idx);
  }();

  // Get Closest segment for the goal point
  const auto goal_pose = path.points.empty() ? current_pose : path.points.back().point.pose;
  const size_t goal_left_start_idx =
    findNearestSegmentIndexFromLateralDistance(left_bound, goal_pose.position);
  const size_t goal_right_start_idx =
    findNearestSegmentIndexFromLateralDistance(right_bound, goal_pose.position);
  const auto left_goal_point =
    calcLongitudinalOffsetGoalPoint(left_bound, goal_pose, goal_left_start_idx, vehicle_length);
  const auto right_goal_point =
    calcLongitudinalOffsetGoalPoint(right_bound, goal_pose, goal_right_start_idx, vehicle_length);
  const size_t left_goal_idx = std::max(
    goal_left_start_idx, findNearestSegmentIndexFromLateralDistance(left_bound, left_goal_point));
  const size_t right_goal_idx = std::max(
    goal_right_start_idx,
    findNearestSegmentIndexFromLateralDistance(right_bound, right_goal_point));

  // Insert middle points
  for (size_t i = left_start_idx + 1; i <= left_goal_idx; ++i) {
    const auto & next_point = left_bound.at(i);
    const double dist = tier4_autoware_utils::calcDistance2d(path.left_bound.back(), next_point);
    if (dist > overlap_threshold) {
      path.left_bound.push_back(next_point);
    }
  }
  for (size_t i = right_start_idx + 1; i <= right_goal_idx; ++i) {
    const auto & next_point = right_bound.at(i);
    const double dist = tier4_autoware_utils::calcDistance2d(path.right_bound.back(), next_point);
    if (dist > overlap_threshold) {
      path.right_bound.push_back(next_point);
    }
  }

  // Insert a goal point
  if (
    tier4_autoware_utils::calcDistance2d(path.left_bound.back(), left_goal_point) >
    overlap_threshold) {
    path.left_bound.push_back(left_goal_point);
  }
  if (
    tier4_autoware_utils::calcDistance2d(path.right_bound.back(), right_goal_point) >
    overlap_threshold) {
    path.right_bound.push_back(right_goal_point);
  }
  const auto & expansion_params = planner_data->drivable_area_expansion_parameters;
  if (expansion_params.enabled) {
    drivable_area_expansion::expand_drivable_area(path, planner_data);
  }

  // make bound longitudinally monotonic
  // TODO(Murooka) Fix makeBoundLongitudinallyMonotonic
  if (
    is_driving_forward &&
    (enable_expanding_hatched_road_markings || enable_expanding_intersection_areas)) {
    makeBoundLongitudinallyMonotonic(path, planner_data, true);   // for left bound
    makeBoundLongitudinallyMonotonic(path, planner_data, false);  // for right bound
  }
}

std::vector<lanelet::ConstPoint3d> getBoundWithHatchedRoadMarkings(
  const std::vector<lanelet::ConstPoint3d> & original_bound,
  const std::shared_ptr<RouteHandler> & route_handler)
{
  // a function to get polygon with a designated point id
  const auto get_corresponding_polygon_index =
    [&](const auto & polygon, const auto & target_point_id) {
      for (size_t poly_point_idx = 0; poly_point_idx < polygon.size(); ++poly_point_idx) {
        if (polygon[poly_point_idx].id() == target_point_id) {
          // NOTE: If there are duplicated points in polygon, the early one will be returned.
          return poly_point_idx;
        }
      }
      // This means calculation has some errors.
      return polygon.size() - 1;
    };

  const auto mod = [&](const int a, const int b) {
    return (a + b) % b;  // NOTE: consider negative value
  };

  std::vector<lanelet::ConstPoint3d> expanded_bound{};

  std::optional<lanelet::Polygon3d> current_polygon{std::nullopt};
  std::vector<size_t> current_polygon_border_indices;
  // expand drivable area by hatched road markings.
  for (size_t bound_point_idx = 0; bound_point_idx < original_bound.size(); ++bound_point_idx) {
    const auto & bound_point = original_bound[bound_point_idx];
    const auto polygon = getPolygonByPoint(route_handler, bound_point, "hatched_road_markings");

    bool will_close_polygon{false};
    if (!current_polygon) {
      if (!polygon) {
        expanded_bound.push_back(bound_point);
      } else {
        // There is a new additional polygon to expand
        current_polygon = polygon;
        current_polygon_border_indices.push_back(
          get_corresponding_polygon_index(*current_polygon, bound_point.id()));
      }
    } else {
      if (!polygon) {
        will_close_polygon = true;
      } else {
        current_polygon_border_indices.push_back(
          get_corresponding_polygon_index(*current_polygon, bound_point.id()));
      }
    }

    if (bound_point_idx == original_bound.size() - 1 && current_polygon) {
      // If drivable lanes ends earlier than polygon, close the polygon
      will_close_polygon = true;
    }

    if (will_close_polygon) {
      // The current additional polygon ends to expand
      if (current_polygon_border_indices.size() == 1) {
        expanded_bound.push_back((*current_polygon)[current_polygon_border_indices.front()]);
      } else {
        const size_t current_polygon_points_num = current_polygon->size();
        const bool is_polygon_opposite_direction = [&]() {
          const size_t modulo_diff = mod(
            static_cast<int>(current_polygon_border_indices[1]) -
              static_cast<int>(current_polygon_border_indices[0]),
            current_polygon_points_num);
          return modulo_diff == 1;
        }();

        const int target_points_num =
          current_polygon_points_num - current_polygon_border_indices.size() + 1;
        for (int poly_idx = 0; poly_idx <= target_points_num; ++poly_idx) {
          const int target_poly_idx = current_polygon_border_indices.front() +
                                      poly_idx * (is_polygon_opposite_direction ? -1 : 1);
          expanded_bound.push_back(
            (*current_polygon)[mod(target_poly_idx, current_polygon_points_num)]);
        }
      }
      current_polygon = std::nullopt;
      current_polygon_border_indices.clear();
    }
  }

  return expanded_bound;
}

std::vector<lanelet::ConstPoint3d> getBoundWithIntersectionAreas(
  const std::vector<lanelet::ConstPoint3d> & original_bound,
  const std::shared_ptr<RouteHandler> & route_handler,
  const std::vector<DrivableLanes> & drivable_lanes, const bool is_left)
{
  const auto extract_bound_from_polygon =
    [](const auto & polygon, const auto start_idx, const auto end_idx, const auto clockwise) {
      std::vector<lanelet::ConstPoint3d> ret{};
      for (size_t i = start_idx; i != end_idx; i = clockwise ? i + 1 : i - 1) {
        ret.push_back(polygon[i]);

        if (i + 1 == polygon.size() && clockwise) {
          if (end_idx == 0) {
            ret.push_back(polygon[end_idx]);
            return ret;
          }
          i = 0;
          continue;
        }

        if (i == 0 && !clockwise) {
          if (end_idx == polygon.size() - 1) {
            ret.push_back(polygon[end_idx]);
            return ret;
          }
          i = polygon.size() - 1;
          continue;
        }
      }

      ret.push_back(polygon[end_idx]);

      return ret;
    };

  std::vector<lanelet::ConstPoint3d> expanded_bound = original_bound;

  // expand drivable area by using intersection area.
  for (const auto & drivable_lane : drivable_lanes) {
    const auto edge_lanelet = is_left ? drivable_lane.left_lane : drivable_lane.right_lane;
    const auto lanelet_bound = is_left ? edge_lanelet.leftBound3d() : edge_lanelet.rightBound3d();

    if (lanelet_bound.size() < 2) {
      continue;
    }

    const std::string id = edge_lanelet.attributeOr("intersection_area", "else");
    if (id == "else") {
      continue;
    }

    // Step1. extract intersection partial bound.
    std::vector<lanelet::ConstPoint3d> intersection_bound{};
    {
      const auto polygon =
        route_handler->getLaneletMapPtr()->polygonLayer.get(std::atoi(id.c_str()));

      const auto is_clockwise_polygon =
        boost::geometry::is_valid(lanelet::utils::to2D(polygon.basicPolygon()));
      const auto is_clockwise_iteration = is_clockwise_polygon ? is_left : !is_left;

      const auto intersection_bound_itr_init = std::find_if(
        polygon.begin(), polygon.end(),
        [&lanelet_bound](const auto & p) { return p.id() == lanelet_bound.front().id(); });

      const auto intersection_bound_itr_last = std::find_if(
        polygon.begin(), polygon.end(),
        [&lanelet_bound](const auto & p) { return p.id() == lanelet_bound.back().id(); });

      if (
        intersection_bound_itr_init == polygon.end() ||
        intersection_bound_itr_last == polygon.end()) {
        continue;
      }

      // extract line strings between start_idx and end_idx.
      const size_t start_idx = std::distance(polygon.begin(), intersection_bound_itr_init);
      const size_t end_idx = std::distance(polygon.begin(), intersection_bound_itr_last);

      intersection_bound =
        extract_bound_from_polygon(polygon, start_idx, end_idx, is_clockwise_iteration);
    }

    // Step2. check shared bound point.
    const auto shared_point_itr_init =
      std::find_if(expanded_bound.begin(), expanded_bound.end(), [&](const auto & p) {
        return std::any_of(
          intersection_bound.begin(), intersection_bound.end(),
          [&](const auto & point) { return point.id() == p.id(); });
      });

    const auto shared_point_itr_last =
      std::find_if(expanded_bound.rbegin(), expanded_bound.rend(), [&](const auto & p) {
        return std::any_of(
          intersection_bound.rbegin(), intersection_bound.rend(),
          [&](const auto & point) { return point.id() == p.id(); });
      });

    if (
      shared_point_itr_init == expanded_bound.end() ||
      shared_point_itr_last == expanded_bound.rend()) {
      continue;
    }

    // Step3. overwrite duplicate drivable bound by intersection bound.
    {
      const auto trim_point_itr_init = std::find_if(
        intersection_bound.begin(), intersection_bound.end(),
        [&](const auto & p) { return p.id() == shared_point_itr_init->id(); });

      const auto trim_point_itr_last = std::find_if(
        intersection_bound.begin(), intersection_bound.end(),
        [&](const auto & p) { return p.id() == shared_point_itr_last->id(); });

      if (
        trim_point_itr_init == intersection_bound.end() ||
        trim_point_itr_last == intersection_bound.end()) {
        continue;
      }

      // TODO(Satoshi OTA): remove this guard.
      if (
        std::distance(intersection_bound.begin(), trim_point_itr_last) <
        std::distance(intersection_bound.begin(), trim_point_itr_init)) {
        continue;
      }

      std::vector<lanelet::ConstPoint3d> tmp_bound{};

      tmp_bound.insert(tmp_bound.end(), expanded_bound.begin(), shared_point_itr_init);
      tmp_bound.insert(tmp_bound.end(), trim_point_itr_init, trim_point_itr_last);
      tmp_bound.insert(
        tmp_bound.end(), std::next(shared_point_itr_last).base(), expanded_bound.end());

      expanded_bound = tmp_bound;
    }
  }

  return expanded_bound;
}

// calculate bounds from drivable lanes and hatched road markings
std::vector<geometry_msgs::msg::Point> calcBound(
  const std::shared_ptr<RouteHandler> route_handler,
  const std::vector<DrivableLanes> & drivable_lanes,
  const bool enable_expanding_hatched_road_markings, const bool enable_expanding_intersection_areas,
  const bool is_left)
{
  // a function to convert drivable lanes to points without duplicated points
  const auto convert_to_points = [&](const std::vector<DrivableLanes> & drivable_lanes) {
    constexpr double overlap_threshold = 0.01;

    std::vector<lanelet::ConstPoint3d> points;
    for (const auto & drivable_lane : drivable_lanes) {
      const auto bound =
        is_left ? drivable_lane.left_lane.leftBound3d() : drivable_lane.right_lane.rightBound3d();
      for (const auto & point : bound) {
        if (
          points.empty() ||
          overlap_threshold < (points.back().basicPoint2d() - point.basicPoint2d()).norm()) {
          points.push_back(point);
        }
      }
    }
    return points;
  };

  const auto to_ros_point = [](const std::vector<lanelet::ConstPoint3d> & bound) {
    std::vector<Point> ret{};
    std::for_each(bound.begin(), bound.end(), [&](const auto & p) {
      ret.push_back(lanelet::utils::conversion::toGeomMsgPt(p));
    });
    return ret;
  };

  // Step1. create drivable bound from drivable lanes.
  std::vector<lanelet::ConstPoint3d> bound_points = convert_to_points(drivable_lanes);

  // Step2. if there is no drivable area defined by polygon, return original drivable bound.
  if (!enable_expanding_hatched_road_markings && !enable_expanding_intersection_areas) {
    return motion_utils::removeOverlapPoints(to_ros_point(bound_points));
  }

  // Step3. if there are hatched road markings, expand drivable bound with the polygon.
  if (enable_expanding_hatched_road_markings) {
    bound_points = getBoundWithHatchedRoadMarkings(bound_points, route_handler);
  }

  if (!enable_expanding_intersection_areas) {
    return motion_utils::removeOverlapPoints(to_ros_point(bound_points));
  }

  // Step4. if there are intersection areas, expand drivable bound with the polygon.
  {
    bound_points =
      getBoundWithIntersectionAreas(bound_points, route_handler, drivable_lanes, is_left);
  }

  return motion_utils::removeOverlapPoints(to_ros_point(bound_points));
}

void makeBoundLongitudinallyMonotonic(
  PathWithLaneId & path, const std::shared_ptr<const PlannerData> & planner_data,
  const bool is_bound_left)
{
  using motion_utils::findNearestSegmentIndex;
  using tier4_autoware_utils::calcAzimuthAngle;
  using tier4_autoware_utils::calcDistance2d;
  using tier4_autoware_utils::calcOffsetPose;
  using tier4_autoware_utils::createQuaternionFromRPY;
  using tier4_autoware_utils::getPoint;
  using tier4_autoware_utils::getPose;
  using tier4_autoware_utils::intersect;
  using tier4_autoware_utils::normalizeRadian;

  const auto set_orientation = [](
                                 auto & bound_with_pose, const auto idx, const auto & orientation) {
    bound_with_pose.at(idx).orientation = orientation;
  };

  const auto get_intersect_idx = [](
                                   const auto & bound_with_pose, const auto start_idx,
                                   const auto & p1, const auto & p2) -> boost::optional<size_t> {
    std::vector<std::pair<size_t, Point>> intersects;
    for (size_t i = start_idx; i < bound_with_pose.size() - 1; i++) {
      const auto opt_intersect =
        intersect(p1, p2, bound_with_pose.at(i).position, bound_with_pose.at(i + 1).position);

      if (!opt_intersect) {
        continue;
      }

      intersects.emplace_back(i, *opt_intersect);
    }

    if (intersects.empty()) {
      return boost::none;
    }

    std::sort(intersects.begin(), intersects.end(), [&](const auto & a, const auto & b) {
      return calcDistance2d(p1, a.second) < calcDistance2d(p1, b.second);
    });

    return intersects.front().first;
  };

  const auto get_bound_with_pose = [&](const auto & bound_with_pose, const auto & path_points) {
    auto ret = bound_with_pose;

    const double offset = is_bound_left ? 100.0 : -100.0;

    size_t start_bound_idx = 0;

    const size_t start_path_idx =
      findNearestSegmentIndex(path_points, bound_with_pose.front().position);

    // append bound point with point
    for (size_t i = start_path_idx + 1; i < path_points.size(); i++) {
      const auto p_path_offset = calcOffsetPose(getPose(path_points.at(i)), 0.0, offset, 0.0);
      const auto intersect_idx = get_intersect_idx(
        bound_with_pose, start_bound_idx, getPoint(path_points.at(i)), getPoint(p_path_offset));

      if (!intersect_idx) {
        continue;
      }

      if (i + 1 == path_points.size()) {
        for (size_t j = intersect_idx.get(); j < bound_with_pose.size(); j++) {
          if (j + 1 == bound_with_pose.size()) {
            const auto yaw =
              calcAzimuthAngle(bound_with_pose.at(j - 1).position, bound_with_pose.at(j).position);
            set_orientation(ret, j, createQuaternionFromRPY(0.0, 0.0, yaw));
          } else {
            const auto yaw =
              calcAzimuthAngle(bound_with_pose.at(j).position, bound_with_pose.at(j + 1).position);
            set_orientation(ret, j, createQuaternionFromRPY(0.0, 0.0, yaw));
          }
        }
      } else {
        for (size_t j = intersect_idx.get() + 1; j < bound_with_pose.size(); j++) {
          set_orientation(ret, j, getPose(path_points.at(i)).orientation);
        }
      }

      constexpr size_t OVERLAP_CHECK_NUM = 3;
      start_bound_idx =
        intersect_idx.get() < OVERLAP_CHECK_NUM ? 0 : intersect_idx.get() - OVERLAP_CHECK_NUM;
    }

    return ret;
  };

  const auto is_monotonic =
    [&](const auto & p1, const auto & p2, const auto & p3, const auto is_points_left) {
      const auto p1_to_p2 = calcAzimuthAngle(p1, p2);
      const auto p2_to_p3 = calcAzimuthAngle(p2, p3);

      const auto theta = normalizeRadian(p1_to_p2 - p2_to_p3);

      return (is_points_left && 0 < theta) || (!is_points_left && theta < 0);
    };

  // define a function to remove non monotonic point on bound
  const auto remove_non_monotonic_point = [&](const auto & bound_with_pose, const bool is_reverse) {
    std::vector<Pose> monotonic_bound;

    const bool is_points_left = is_reverse ? !is_bound_left : is_bound_left;

    size_t bound_idx = 0;
    while (true) {
      monotonic_bound.push_back(bound_with_pose.at(bound_idx));

      if (bound_idx + 1 == bound_with_pose.size()) {
        break;
      }

      // NOTE: is_bound_left is used instead of is_points_left since orientation of path point is
      // opposite.
      const double lat_offset = is_bound_left ? 100.0 : -100.0;

      const auto p_bound_1 = getPoint(bound_with_pose.at(bound_idx));
      const auto p_bound_2 = getPoint(bound_with_pose.at(bound_idx + 1));

      const auto p_bound_offset =
        calcOffsetPose(getPose(bound_with_pose.at(bound_idx)), 0.0, lat_offset, 0.0);

      if (!is_monotonic(p_bound_1, p_bound_2, p_bound_offset.position, is_points_left)) {
        bound_idx++;
        continue;
      }

      // skip non monotonic points
      for (size_t i = bound_idx + 1; i < bound_with_pose.size() - 1; ++i) {
        const auto intersect_point = intersect(
          p_bound_1, p_bound_offset.position, bound_with_pose.at(i).position,
          bound_with_pose.at(i + 1).position);

        if (intersect_point) {
          Pose pose;
          pose.position = *intersect_point;
          pose.position.z = bound_with_pose.at(i).position.z;
          const auto yaw = calcAzimuthAngle(*intersect_point, bound_with_pose.at(i + 1).position);
          pose.orientation = createQuaternionFromRPY(0.0, 0.0, yaw);
          monotonic_bound.push_back(pose);
          bound_idx = i;
          break;
        }
      }

      bound_idx++;
    }

    return monotonic_bound;
  };

  const auto remove_orientation = [](const auto & bound_with_pose) {
    std::vector<Point> ret;

    ret.reserve(bound_with_pose.size());

    std::for_each(bound_with_pose.begin(), bound_with_pose.end(), [&ret](const auto & p) {
      ret.push_back(p.position);
    });

    return ret;
  };

  const auto remove_sharp_points = [](const auto & bound) {
    if (bound.size() < 2) {
      return bound;
    }

    std::vector<Point> ret = bound;
    auto itr = std::next(ret.begin());
    while (std::next(itr) != ret.end()) {
      if (itr == ret.begin()) {
        itr++;
        continue;
      }

      const auto p1 = *std::prev(itr);
      const auto p2 = *itr;
      const auto p3 = *std::next(itr);

      const std::vector vec_1to2 = {p2.x - p1.x, p2.y - p1.y, p2.z - p1.z};
      const std::vector vec_3to2 = {p2.x - p3.x, p2.y - p3.y, p2.z - p3.z};
      const auto product =
        std::inner_product(vec_1to2.begin(), vec_1to2.end(), vec_3to2.begin(), 0.0);

      const auto dist_1to2 = tier4_autoware_utils::calcDistance3d(p1, p2);
      const auto dist_3to2 = tier4_autoware_utils::calcDistance3d(p3, p2);

      constexpr double epsilon = 1e-3;

      // Remove overlapped point.
      if (dist_1to2 < epsilon || dist_3to2 < epsilon) {
        itr = std::prev(ret.erase(itr));
        continue;
      }

      // If the angle between the points is sharper than 45 degrees, remove the middle point.
      if (std::cos(M_PI_4) < product / dist_1to2 / dist_3to2 + epsilon) {
        itr = std::prev(ret.erase(itr));
        continue;
      }

      itr++;
    }

    return ret;
  };

  const auto original_bound = is_bound_left ? path.left_bound : path.right_bound;

  if (path.points.empty()) {
    return;
  }

  if (original_bound.empty()) {
    return;
  }

  if (path.points.front().lane_ids.empty()) {
    return;
  }

  // step.1 create bound with pose vector.
  std::vector<Pose> original_bound_with_pose;
  {
    original_bound_with_pose.reserve(original_bound.size());

    std::for_each(original_bound.begin(), original_bound.end(), [&](const auto & p) {
      Pose pose;
      pose.position = p;
      original_bound_with_pose.push_back(pose);
    });

    for (size_t i = 0; i < original_bound_with_pose.size(); i++) {
      if (i + 1 == original_bound_with_pose.size()) {
        const auto yaw = calcAzimuthAngle(
          original_bound_with_pose.at(i - 1).position, original_bound_with_pose.at(i).position);
        set_orientation(original_bound_with_pose, i, createQuaternionFromRPY(0.0, 0.0, yaw));
      } else {
        const auto yaw = calcAzimuthAngle(
          original_bound_with_pose.at(i).position, original_bound_with_pose.at(i + 1).position);
        set_orientation(original_bound_with_pose, i, createQuaternionFromRPY(0.0, 0.0, yaw));
      }
    }
  }

  // step.2 get base pose vector.
  std::vector<PathPointWithLaneId> clipped_points;
  {
    const auto & route_handler = planner_data->route_handler;
    const auto p = planner_data->parameters;
    const auto start_id = path.points.front().lane_ids.front();
    const auto start_lane = planner_data->route_handler->getLaneletsFromId(start_id);

    const auto lanelet_sequence =
      route_handler->getLaneletSequence(start_lane, p.backward_path_length, p.forward_path_length);
    const auto centerline_path = getCenterLinePath(
      *route_handler, lanelet_sequence, getPose(path.points.front()), p.backward_path_length,
      p.forward_path_length, p);

    if (centerline_path.points.size() < 2) {
      return;
    }

    const auto ego_idx = planner_data->findEgoIndex(centerline_path.points);
    const auto end_idx = findNearestSegmentIndex(centerline_path.points, original_bound.back());

    if (ego_idx >= end_idx) {
      return;
    }

    clipped_points.insert(
      clipped_points.end(), centerline_path.points.begin() + ego_idx,
      centerline_path.points.begin() + end_idx + 1);
  }

  if (clipped_points.empty()) {
    return;
  }

  // step.3 update bound pose by reference path pose.
  const auto updated_bound_with_pose =
    get_bound_with_pose(original_bound_with_pose, clipped_points);  // for reverse

  // step.4 create remove monotonic points by forward direction.
  auto half_monotonic_bound_with_pose =
    remove_non_monotonic_point(updated_bound_with_pose, false);  // for reverse
  std::reverse(half_monotonic_bound_with_pose.begin(), half_monotonic_bound_with_pose.end());

  // step.5 create remove monotonic points by backward direction.
  auto full_monotonic_bound_with_pose =
    remove_non_monotonic_point(half_monotonic_bound_with_pose, true);
  std::reverse(full_monotonic_bound_with_pose.begin(), full_monotonic_bound_with_pose.end());

  // step.6 remove orientation from bound with pose.
  auto full_monotonic_bound = remove_orientation(full_monotonic_bound_with_pose);

  // step.7 remove sharp bound points.
  if (is_bound_left) {
    path.left_bound = remove_sharp_points(full_monotonic_bound);
  } else {
    path.right_bound = remove_sharp_points(full_monotonic_bound);
  }
}

// generate drivable area by expanding path for freespace
void generateDrivableArea(
  PathWithLaneId & path, const double vehicle_length, const double offset,
  const bool is_driving_forward)
{
  using tier4_autoware_utils::calcOffsetPose;

  // remove path points which is close to the previous point
  PathWithLaneId resampled_path{};
  const double resample_interval = 2.0;
  for (size_t i = 0; i < path.points.size(); ++i) {
    if (i == 0) {
      resampled_path.points.push_back(path.points.at(i));
    } else {
      const auto & prev_point = resampled_path.points.back().point.pose.position;
      const auto & curr_point = path.points.at(i).point.pose.position;
      const double signed_arc_length =
        motion_utils::calcSignedArcLength(path.points, prev_point, curr_point);
      if (signed_arc_length > resample_interval) {
        resampled_path.points.push_back(path.points.at(i));
      }
    }
  }
  // add last point of path if enough far from the one of resampled path
  constexpr double th_last_point_distance = 0.3;
  if (
    tier4_autoware_utils::calcDistance2d(
      resampled_path.points.back().point.pose.position, path.points.back().point.pose.position) >
    th_last_point_distance) {
    resampled_path.points.push_back(path.points.back());
  }

  // create bound point by calculating offset point
  std::vector<Point> left_bound;
  std::vector<Point> right_bound;
  for (const auto & point : resampled_path.points) {
    const auto & pose = point.point.pose;

    const auto left_point = calcOffsetPose(pose, 0, offset, 0);
    const auto right_point = calcOffsetPose(pose, 0, -offset, 0);

    left_bound.push_back(left_point.position);
    right_bound.push_back(right_point.position);
  }

  if (is_driving_forward) {
    // add backward offset point to bound
    const Pose first_point =
      calcOffsetPose(resampled_path.points.front().point.pose, -vehicle_length, 0, 0);
    const Pose left_first_point = calcOffsetPose(first_point, 0, offset, 0);
    const Pose right_first_point = calcOffsetPose(first_point, 0, -offset, 0);
    left_bound.insert(left_bound.begin(), left_first_point.position);
    right_bound.insert(right_bound.begin(), right_first_point.position);

    // add forward offset point to bound
    const Pose last_point =
      calcOffsetPose(resampled_path.points.back().point.pose, vehicle_length, 0, 0);
    const Pose left_last_point = calcOffsetPose(last_point, 0, offset, 0);
    const Pose right_last_point = calcOffsetPose(last_point, 0, -offset, 0);
    left_bound.push_back(left_last_point.position);
    right_bound.push_back(right_last_point.position);
  } else {
    // add forward offset point to bound
    const Pose first_point =
      calcOffsetPose(resampled_path.points.front().point.pose, vehicle_length, 0, 0);
    const Pose left_first_point = calcOffsetPose(first_point, 0, offset, 0);
    const Pose right_first_point = calcOffsetPose(first_point, 0, -offset, 0);
    left_bound.insert(left_bound.begin(), left_first_point.position);
    right_bound.insert(right_bound.begin(), right_first_point.position);

    // add backward offset point to bound
    const Pose last_point =
      calcOffsetPose(resampled_path.points.back().point.pose, -vehicle_length, 0, 0);
    const Pose left_last_point = calcOffsetPose(last_point, 0, offset, 0);
    const Pose right_last_point = calcOffsetPose(last_point, 0, -offset, 0);
    left_bound.push_back(left_last_point.position);
    right_bound.push_back(right_last_point.position);
  }

  if (left_bound.empty() || right_bound.empty()) {
    return;
  }

  // fix intersected bound
  // if bound is intersected, remove them and insert intersection point
  typedef boost::geometry::model::d2::point_xy<double> BoostPoint;
  typedef boost::geometry::model::linestring<BoostPoint> LineString;
  auto modify_bound_intersection = [](const std::vector<Point> & bound) {
    const double intersection_check_distance = 10.0;
    std::vector<Point> modified_bound;
    size_t i = 0;
    while (i < bound.size() - 1) {
      BoostPoint p1(bound.at(i).x, bound.at(i).y);
      BoostPoint p2(bound.at(i + 1).x, bound.at(i + 1).y);
      LineString p_line;
      p_line.push_back(p1);
      p_line.push_back(p2);
      bool intersection_found = false;
      for (size_t j = i + 2; j < bound.size() - 1; j++) {
        const double distance = tier4_autoware_utils::calcDistance2d(bound.at(i), bound.at(j));
        if (distance > intersection_check_distance) {
          break;
        }
        LineString q_line;
        BoostPoint q1(bound.at(j).x, bound.at(j).y);
        BoostPoint q2(bound.at(j + 1).x, bound.at(j + 1).y);
        q_line.push_back(q1);
        q_line.push_back(q2);
        std::vector<BoostPoint> intersection_points;
        boost::geometry::intersection(p_line, q_line, intersection_points);
        if (intersection_points.size() > 0) {
          modified_bound.push_back(bound.at(i));
          Point intersection_point;
          intersection_point.x = intersection_points.at(0).x();
          intersection_point.y = intersection_points.at(0).y();
          modified_bound.push_back(intersection_point);
          i = j + 1;
          intersection_found = true;
          break;
        }
      }
      if (!intersection_found) {
        modified_bound.push_back(bound.at(i));
        i++;
      }
    }
    modified_bound.push_back(bound.back());
    return modified_bound;
  };
  std::vector<Point> modified_left_bound = modify_bound_intersection(left_bound);
  std::vector<Point> modified_right_bound = modify_bound_intersection(right_bound);

  // set bound to path
  path.left_bound = modified_left_bound;
  path.right_bound = modified_right_bound;
}

double getDistanceToEndOfLane(const Pose & current_pose, const lanelet::ConstLanelets & lanelets)
{
  const auto & arc_coordinates = lanelet::utils::getArcCoordinates(lanelets, current_pose);
  const double lanelet_length = lanelet::utils::getLaneletLength3d(lanelets);
  return lanelet_length - arc_coordinates.length;
}

double getDistanceToNextTrafficLight(
  const Pose & current_pose, const lanelet::ConstLanelets & lanelets)
{
  lanelet::ConstLanelet current_lanelet;
  if (!lanelet::utils::query::getClosestLanelet(lanelets, current_pose, &current_lanelet)) {
    return std::numeric_limits<double>::infinity();
  }

  const auto lanelet_point = lanelet::utils::conversion::toLaneletPoint(current_pose.position);
  const auto to_object = lanelet::geometry::toArcCoordinates(
    lanelet::utils::to2D(current_lanelet.centerline()),
    lanelet::utils::to2D(lanelet_point).basicPoint());

  for (const auto & element : current_lanelet.regulatoryElementsAs<lanelet::TrafficLight>()) {
    lanelet::ConstLineString3d lanelet_stop_lines = element->stopLine().get();

    const auto to_stop_line = lanelet::geometry::toArcCoordinates(
      lanelet::utils::to2D(current_lanelet.centerline()),
      lanelet::utils::to2D(lanelet_stop_lines).front().basicPoint());

    const auto distance_object_to_stop_line = to_stop_line.length - to_object.length;

    if (distance_object_to_stop_line > 0.0) {
      return distance_object_to_stop_line;
    }
  }

  double distance = lanelet::utils::getLaneletLength3d(current_lanelet);

  bool found_current_lane = false;
  for (const auto & llt : lanelets) {
    if (llt.id() == current_lanelet.id()) {
      found_current_lane = true;
      continue;
    }

    if (!found_current_lane) {
      continue;
    }

    for (const auto & element : llt.regulatoryElementsAs<lanelet::TrafficLight>()) {
      lanelet::ConstLineString3d lanelet_stop_lines = element->stopLine().get();

      const auto to_stop_line = lanelet::geometry::toArcCoordinates(
        lanelet::utils::to2D(llt.centerline()),
        lanelet::utils::to2D(lanelet_stop_lines).front().basicPoint());

      return distance + to_stop_line.length - to_object.length;
    }

    distance += lanelet::utils::getLaneletLength3d(llt);
  }

  return std::numeric_limits<double>::infinity();
}

double getDistanceToNextIntersection(
  const Pose & current_pose, const lanelet::ConstLanelets & lanelets)
{
  const auto & arc_coordinates = lanelet::utils::getArcCoordinates(lanelets, current_pose);

  lanelet::ConstLanelet current_lanelet;
  if (!lanelet::utils::query::getClosestLanelet(lanelets, current_pose, &current_lanelet)) {
    return std::numeric_limits<double>::max();
  }

  double distance = 0;
  bool is_after_current_lanelet = false;
  for (const auto & llt : lanelets) {
    if (llt == current_lanelet) {
      is_after_current_lanelet = true;
    }
    if (is_after_current_lanelet && llt.hasAttribute("turn_direction")) {
      bool is_lane_change_yes = false;
      const auto right_line = llt.rightBound();
      if (right_line.hasAttribute(lanelet::AttributeNamesString::LaneChange)) {
        const auto attr = right_line.attribute(lanelet::AttributeNamesString::LaneChange);
        if (attr.value() == std::string("yes")) {
          is_lane_change_yes = true;
        }
      }
      const auto left_line = llt.leftBound();
      if (left_line.hasAttribute(lanelet::AttributeNamesString::LaneChange)) {
        const auto attr = left_line.attribute(lanelet::AttributeNamesString::LaneChange);
        if (attr.value() == std::string("yes")) {
          is_lane_change_yes = true;
        }
      }
      if (!is_lane_change_yes) {
        return distance - arc_coordinates.length;
      }
    }
    distance += lanelet::utils::getLaneletLength3d(llt);
  }

  return std::numeric_limits<double>::max();
}

double getDistanceToCrosswalk(
  const Pose & current_pose, const lanelet::ConstLanelets & lanelets,
  const lanelet::routing::RoutingGraphContainer & overall_graphs)
{
  const auto & arc_coordinates = lanelet::utils::getArcCoordinates(lanelets, current_pose);

  lanelet::ConstLanelet current_lanelet;
  if (!lanelet::utils::query::getClosestLanelet(lanelets, current_pose, &current_lanelet)) {
    return std::numeric_limits<double>::infinity();
  }

  double distance = 0;
  bool is_after_current_lanelet = false;
  for (const auto & llt : lanelets) {
    if (llt == current_lanelet) {
      is_after_current_lanelet = true;
    }

    if (is_after_current_lanelet) {
      const auto conflicting_crosswalks = overall_graphs.conflictingInGraph(llt, 1);
      if (!(conflicting_crosswalks.empty())) {
        // create centerline
        const lanelet::ConstLineString2d lanelet_centerline = llt.centerline2d();
        LineString2d centerline;
        centerline.reserve(lanelet_centerline.size());
        for (const auto & point : lanelet_centerline) {
          boost::geometry::append(centerline, Point2d(point.x(), point.y()));
        }

        // create crosswalk polygon and calculate distance
        double min_distance_to_crosswalk = std::numeric_limits<double>::infinity();
        for (const auto & crosswalk : conflicting_crosswalks) {
          lanelet::CompoundPolygon2d lanelet_crosswalk_polygon = crosswalk.polygon2d();
          Polygon2d polygon;
          polygon.outer().reserve(lanelet_crosswalk_polygon.size() + 1);
          for (const auto & point : lanelet_crosswalk_polygon) {
            polygon.outer().emplace_back(point.x(), point.y());
          }
          polygon.outer().push_back(polygon.outer().front());

          std::vector<Point2d> points_intersection;
          boost::geometry::intersection(centerline, polygon, points_intersection);

          for (const auto & point : points_intersection) {
            lanelet::ConstLanelets lanelets = {llt};
            Pose pose_point;
            pose_point.position.x = point.x();
            pose_point.position.y = point.y();
            const lanelet::ArcCoordinates & arc_crosswalk =
              lanelet::utils::getArcCoordinates(lanelets, pose_point);

            const double distance_to_crosswalk = arc_crosswalk.length;
            if (distance_to_crosswalk < min_distance_to_crosswalk) {
              min_distance_to_crosswalk = distance_to_crosswalk;
            }
          }
        }
        if (distance + min_distance_to_crosswalk > arc_coordinates.length) {
          return distance + min_distance_to_crosswalk - arc_coordinates.length;
        }
      }
    }
    distance += lanelet::utils::getLaneletLength3d(llt);
  }

  return std::numeric_limits<double>::infinity();
}

double getSignedDistance(
  const Pose & current_pose, const Pose & goal_pose, const lanelet::ConstLanelets & lanelets)
{
  const auto arc_current = lanelet::utils::getArcCoordinates(lanelets, current_pose);
  const auto arc_goal = lanelet::utils::getArcCoordinates(lanelets, goal_pose);

  return arc_goal.length - arc_current.length;
}

std::vector<uint64_t> getIds(const lanelet::ConstLanelets & lanelets)
{
  std::vector<uint64_t> ids;
  ids.reserve(lanelets.size());
  for (const auto & llt : lanelets) {
    ids.push_back(llt.id());
  }
  return ids;
}

PathPointWithLaneId insertStopPoint(const double length, PathWithLaneId & path)
{
  const size_t original_size = path.points.size();

  // insert stop point
  const auto insert_idx = motion_utils::insertStopPoint(length, path.points);
  if (!insert_idx) {
    return PathPointWithLaneId();
  }

  // check if a stop point is inserted
  if (path.points.size() == original_size) {
    return path.points.at(*insert_idx);
  }

  if (*insert_idx == 0 || *insert_idx == original_size - 1) {
    return path.points.at(*insert_idx);
  }

  // check lane ids of the inserted stop point
  path.points.at(*insert_idx).lane_ids = {};
  const auto & prev_lane_ids = path.points.at(*insert_idx - 1).lane_ids;
  const auto & next_lane_ids = path.points.at(*insert_idx + 1).lane_ids;

  for (const auto target_lane_id : prev_lane_ids) {
    if (
      std::find(next_lane_ids.begin(), next_lane_ids.end(), target_lane_id) !=
      next_lane_ids.end()) {
      path.points.at(*insert_idx).lane_ids.push_back(target_lane_id);
    }
  }

  // If there is no lane ids, we are going to insert prev lane ids
  if (path.points.at(*insert_idx).lane_ids.empty()) {
    path.points.at(*insert_idx).lane_ids = prev_lane_ids;
  }

  return path.points.at(*insert_idx);
}

double getSignedDistanceFromBoundary(
  const lanelet::ConstLanelets & lanelets, const Pose & pose, bool left_side)
{
  lanelet::ConstLanelet closest_lanelet;
  lanelet::ArcCoordinates arc_coordinates;
  if (lanelet::utils::query::getClosestLanelet(lanelets, pose, &closest_lanelet)) {
    const auto lanelet_point = lanelet::utils::conversion::toLaneletPoint(pose.position);
    const auto & boundary_line_2d = left_side
                                      ? lanelet::utils::to2D(closest_lanelet.leftBound3d())
                                      : lanelet::utils::to2D(closest_lanelet.rightBound3d());
    arc_coordinates = lanelet::geometry::toArcCoordinates(
      boundary_line_2d, lanelet::utils::to2D(lanelet_point).basicPoint());
  } else {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("behavior_path_planner").get_child("utils"),
      "closest shoulder lanelet not found.");
  }

  return arc_coordinates.distance;
}

std::optional<double> getSignedDistanceFromBoundary(
  const lanelet::ConstLanelets & lanelets, const double vehicle_width, const double base_link2front,
  const double base_link2rear, const Pose & vehicle_pose, const bool left_side)
{
  // Depending on which side is selected, calculate the transformed coordinates of the front and
  // rear vehicle corners
  Point rear_corner_point, front_corner_point;
  if (left_side) {
    Point front_left, rear_left;
    rear_left.x = -base_link2rear;
    rear_left.y = vehicle_width / 2;
    front_left.x = base_link2front;
    front_left.y = vehicle_width / 2;
    rear_corner_point = tier4_autoware_utils::transformPoint(rear_left, vehicle_pose);
    front_corner_point = tier4_autoware_utils::transformPoint(front_left, vehicle_pose);
  } else {
    Point front_right, rear_right;
    rear_right.x = -base_link2rear;
    rear_right.y = -vehicle_width / 2;
    front_right.x = base_link2front;
    front_right.y = -vehicle_width / 2;
    rear_corner_point = tier4_autoware_utils::transformPoint(rear_right, vehicle_pose);
    front_corner_point = tier4_autoware_utils::transformPoint(front_right, vehicle_pose);
  }

  const auto combined_lane = lanelet::utils::combineLaneletsShape(lanelets);
  const auto & bound_line_2d = left_side ? lanelet::utils::to2D(combined_lane.leftBound3d())
                                         : lanelet::utils::to2D(combined_lane.rightBound3d());

  // Find the closest bound segment that contains the corner point in the X-direction
  // and calculate the lateral distance from that segment.
  const auto calcLateralDistanceFromBound =
    [&](const Point & vehicle_corner_point) -> boost::optional<std::pair<double, size_t>> {
    Pose vehicle_corner_pose{};
    vehicle_corner_pose.position = vehicle_corner_point;
    vehicle_corner_pose.orientation = vehicle_pose.orientation;

    boost::optional<std::pair<double, size_t>> lateral_distance_with_idx{};

    // Euclidean distance to find the closest segment containing the corner point.
    double min_distance = std::numeric_limits<double>::max();

    for (size_t i = 0; i < bound_line_2d.size() - 1; i++) {
      const Point p1 = lanelet::utils::conversion::toGeomMsgPt(bound_line_2d[i]);
      const Point p2 = lanelet::utils::conversion::toGeomMsgPt(bound_line_2d[i + 1]);

      const Point inverse_p1 = tier4_autoware_utils::inverseTransformPoint(p1, vehicle_corner_pose);
      const Point inverse_p2 = tier4_autoware_utils::inverseTransformPoint(p2, vehicle_corner_pose);
      const double dx_p1 = inverse_p1.x;
      const double dx_p2 = inverse_p2.x;
      const double dy_p1 = inverse_p1.y;
      const double dy_p2 = inverse_p2.y;

      // Calculate the Euclidean distances between vehicle's corner and the current and next points.
      const double distance1 = tier4_autoware_utils::calcDistance2d(p1, vehicle_corner_point);
      const double distance2 = tier4_autoware_utils::calcDistance2d(p2, vehicle_corner_point);

      // If one of the bound points is behind and the other is in front of the vehicle corner point
      // and any of these points is closer than the current minimum distance,
      // then update minimum distance, lateral distance and the segment index.
      if (dx_p1 < 0 && dx_p2 > 0 && (distance1 < min_distance || distance2 < min_distance)) {
        min_distance = std::min(distance1, distance2);
        // Update lateral distance using the formula derived from similar triangles in the lateral
        // cross-section view.
        lateral_distance_with_idx =
          std::make_pair(-1.0 * (dy_p1 * dx_p2 + dy_p2 * -dx_p1) / (dx_p2 - dx_p1), i);
      }
    }
    if (lateral_distance_with_idx) {
      return lateral_distance_with_idx;
    }
    return boost::optional<std::pair<double, size_t>>{};
  };

  // Calculate the lateral distance for both the rear and front corners of the vehicle.
  const boost::optional<std::pair<double, size_t>> rear_lateral_distance_with_idx =
    calcLateralDistanceFromBound(rear_corner_point);
  const boost::optional<std::pair<double, size_t>> front_lateral_distance_with_idx =
    calcLateralDistanceFromBound(front_corner_point);

  // If no closest bound segment was found for both corners, return an empty optional.
  if (!rear_lateral_distance_with_idx && !front_lateral_distance_with_idx) {
    return {};
  }
  // If only one of them found the closest bound, return the found lateral distance.
  if (!rear_lateral_distance_with_idx) {
    return front_lateral_distance_with_idx.get().first;
  } else if (!front_lateral_distance_with_idx) {
    return rear_lateral_distance_with_idx.get().first;
  }
  // If both corners found their closest bound, return the maximum (for left side) or the minimum
  // (for right side) lateral distance.
  double lateral_distance =
    left_side
      ? std::max(
          rear_lateral_distance_with_idx.get().first, front_lateral_distance_with_idx.get().first)
      : std::min(
          rear_lateral_distance_with_idx.get().first, front_lateral_distance_with_idx.get().first);

  // Iterate through all segments between the segments closest to the rear and front corners.
  // Update the lateral distance in case any of these inner segments are closer to the vehicle.
  for (size_t i = rear_lateral_distance_with_idx.get().second + 1;
       i < front_lateral_distance_with_idx.get().second; i++) {
    Pose bound_pose;
    bound_pose.position = lanelet::utils::conversion::toGeomMsgPt(bound_line_2d[i]);
    bound_pose.orientation = vehicle_pose.orientation;

    const Point inverse_rear_point =
      tier4_autoware_utils::inverseTransformPoint(rear_corner_point, bound_pose);
    const Point inverse_front_point =
      tier4_autoware_utils::inverseTransformPoint(front_corner_point, bound_pose);
    const double dx_rear = inverse_rear_point.x;
    const double dx_front = inverse_front_point.x;
    const double dy_rear = inverse_rear_point.y;
    const double dy_front = inverse_front_point.y;

    const double current_lateral_distance =
      (dy_rear * dx_front + dy_front * -dx_rear) / (dx_front - dx_rear);
    lateral_distance = left_side ? std::max(lateral_distance, current_lateral_distance)
                                 : std::min(lateral_distance, current_lateral_distance);
  }

  return lateral_distance;
}

double getArcLengthToTargetLanelet(
  const lanelet::ConstLanelets & lanelet_sequence, const lanelet::ConstLanelet & target_lane,
  const Pose & pose)
{
  const auto arc_pose = lanelet::utils::getArcCoordinates(lanelet_sequence, pose);

  const auto target_center_line = target_lane.centerline().basicLineString();

  Pose front_pose, back_pose;

  {
    const auto front_point = lanelet::utils::conversion::toGeomMsgPt(target_center_line.front());
    const double front_yaw = lanelet::utils::getLaneletAngle(target_lane, front_point);
    front_pose.position = front_point;
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(0, 0, front_yaw);
    front_pose.orientation = tf2::toMsg(tf_quat);
  }

  {
    const auto back_point = lanelet::utils::conversion::toGeomMsgPt(target_center_line.back());
    const double back_yaw = lanelet::utils::getLaneletAngle(target_lane, back_point);
    back_pose.position = back_point;
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(0, 0, back_yaw);
    back_pose.orientation = tf2::toMsg(tf_quat);
  }

  const auto arc_front = lanelet::utils::getArcCoordinates(lanelet_sequence, front_pose);
  const auto arc_back = lanelet::utils::getArcCoordinates(lanelet_sequence, back_pose);

  return std::max(
    std::min(arc_front.length - arc_pose.length, arc_back.length - arc_pose.length), 0.0);
}

Polygon2d toPolygon2d(const lanelet::ConstLanelet & lanelet)
{
  Polygon2d polygon;
  for (const auto & p : lanelet.polygon2d().basicPolygon()) {
    polygon.outer().emplace_back(p.x(), p.y());
  }
  polygon.outer().push_back(polygon.outer().front());

  return tier4_autoware_utils::isClockwise(polygon)
           ? polygon
           : tier4_autoware_utils::inverseClockwise(polygon);
}

std::vector<Polygon2d> getTargetLaneletPolygons(
  const lanelet::PolygonLayer & map_polygons, lanelet::ConstLanelets & lanelets, const Pose & pose,
  const double check_length, const std::string & target_type)
{
  std::vector<Polygon2d> polygons;

  // create lanelet polygon
  const auto arclength = lanelet::utils::getArcCoordinates(lanelets, pose);
  const auto llt_polygon = lanelet::utils::getPolygonFromArcLength(
    lanelets, arclength.length, arclength.length + check_length);
  const auto llt_polygon_2d = lanelet::utils::to2D(llt_polygon).basicPolygon();

  // If the number of vertices is not enough to create polygon, return empty polygon container
  if (llt_polygon_2d.size() < 3) {
    return polygons;
  }

  Polygon2d llt_polygon_bg;
  llt_polygon_bg.outer().reserve(llt_polygon_2d.size() + 1);
  for (const auto & llt_pt : llt_polygon_2d) {
    llt_polygon_bg.outer().emplace_back(llt_pt.x(), llt_pt.y());
  }
  llt_polygon_bg.outer().push_back(llt_polygon_bg.outer().front());

  for (const auto & map_polygon : map_polygons) {
    const std::string type = map_polygon.attributeOr(lanelet::AttributeName::Type, "");
    // If the target_type is different
    // or the number of vertices is not enough to create polygon, skip the loop
    if (type == target_type && map_polygon.size() > 2) {
      // create map polygon
      Polygon2d map_polygon_bg;
      map_polygon_bg.outer().reserve(map_polygon.size() + 1);
      for (const auto & pt : map_polygon) {
        map_polygon_bg.outer().emplace_back(pt.x(), pt.y());
      }
      map_polygon_bg.outer().push_back(map_polygon_bg.outer().front());
      if (boost::geometry::intersects(llt_polygon_bg, map_polygon_bg)) {
        polygons.push_back(map_polygon_bg);
      }
    }
  }
  return polygons;
}

// TODO(Horibe) There is a similar function in route_handler.
std::shared_ptr<PathWithLaneId> generateCenterLinePath(
  const std::shared_ptr<const PlannerData> & planner_data)
{
  auto centerline_path = std::make_shared<PathWithLaneId>();

  const auto & p = planner_data->parameters;

  const auto & route_handler = planner_data->route_handler;
  const auto & pose = planner_data->self_odometry->pose.pose;

  lanelet::ConstLanelet current_lane;
  if (!route_handler->getClosestLaneletWithinRoute(pose, &current_lane)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("behavior_path_planner").get_child("utils"),
      "failed to find closest lanelet within route!!!");
    return {};  // TODO(Horibe) What should be returned?
  }

  // For lanelet_sequence with desired length
  lanelet::ConstLanelets lanelet_sequence = route_handler->getLaneletSequence(
    current_lane, pose, p.backward_path_length, p.forward_path_length);

  std::vector<DrivableLanes> drivable_lanes(lanelet_sequence.size());
  for (size_t i = 0; i < lanelet_sequence.size(); ++i) {
    drivable_lanes.at(i).left_lane = lanelet_sequence.at(i);
    drivable_lanes.at(i).right_lane = lanelet_sequence.at(i);
  }

  *centerline_path = getCenterLinePath(
    *route_handler, lanelet_sequence, pose, p.backward_path_length, p.forward_path_length, p);

  centerline_path->header = route_handler->getRouteHeader();

  return centerline_path;
}

// TODO(Azu) Some parts of is the same with generateCenterLinePath. Therefore it might be better if
// we can refactor some of the code for better readability
lanelet::ConstLineStrings3d getMaximumDrivableArea(
  const std::shared_ptr<const PlannerData> & planner_data)
{
  const auto & p = planner_data->parameters;
  const auto & route_handler = planner_data->route_handler;
  const auto & ego_pose = planner_data->self_odometry->pose.pose;

  lanelet::ConstLanelet current_lane;
  if (!route_handler->getClosestLaneletWithinRoute(ego_pose, &current_lane)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("behavior_path_planner").get_child("utils"),
      "failed to find closest lanelet within route!!!");
    return {};
  }

  const auto current_lanes = route_handler->getLaneletSequence(
    current_lane, ego_pose, p.backward_path_length, p.forward_path_length);
  lanelet::ConstLineStrings3d linestring_shared;
  for (const auto & lane : current_lanes) {
    lanelet::ConstLineStrings3d furthest_line = route_handler->getFurthestLinestring(lane);
    linestring_shared.insert(linestring_shared.end(), furthest_line.begin(), furthest_line.end());
  }

  return linestring_shared;
}

std::vector<DrivableLanes> expandLanelets(
  const std::vector<DrivableLanes> & drivable_lanes, const double left_bound_offset,
  const double right_bound_offset, const std::vector<std::string> & types_to_skip)
{
  if (left_bound_offset == 0.0 && right_bound_offset == 0.0) return drivable_lanes;

  std::vector<DrivableLanes> expanded_drivable_lanes{};
  expanded_drivable_lanes.reserve(drivable_lanes.size());
  for (const auto & lanes : drivable_lanes) {
    const std::string l_type =
      lanes.left_lane.leftBound().attributeOr(lanelet::AttributeName::Type, "none");
    const std::string r_type =
      lanes.right_lane.rightBound().attributeOr(lanelet::AttributeName::Type, "none");

    const bool l_skip =
      std::find(types_to_skip.begin(), types_to_skip.end(), l_type) != types_to_skip.end();
    const bool r_skip =
      std::find(types_to_skip.begin(), types_to_skip.end(), r_type) != types_to_skip.end();
    const double l_offset = l_skip ? 0.0 : left_bound_offset;
    const double r_offset = r_skip ? 0.0 : -right_bound_offset;

    DrivableLanes expanded_lanes;
    if (lanes.left_lane.id() == lanes.right_lane.id()) {
      expanded_lanes.left_lane =
        lanelet::utils::getExpandedLanelet(lanes.left_lane, l_offset, r_offset);
      expanded_lanes.right_lane =
        lanelet::utils::getExpandedLanelet(lanes.right_lane, l_offset, r_offset);
    } else {
      expanded_lanes.left_lane = lanelet::utils::getExpandedLanelet(lanes.left_lane, l_offset, 0.0);
      expanded_lanes.right_lane =
        lanelet::utils::getExpandedLanelet(lanes.right_lane, 0.0, r_offset);
    }
    expanded_lanes.middle_lanes = lanes.middle_lanes;
    expanded_drivable_lanes.push_back(expanded_lanes);
  }
  return expanded_drivable_lanes;
}

PathWithLaneId getCenterLinePathFromRootLanelet(
  const lanelet::ConstLanelet & root_lanelet,
  const std::shared_ptr<const PlannerData> & planner_data)
{
  const auto & route_handler = planner_data->route_handler;
  const auto & current_pose = planner_data->self_odometry->pose.pose;
  const auto & p = planner_data->parameters;

  const auto reference_lanes = route_handler->getLaneletSequence(
    root_lanelet, current_pose, p.backward_path_length, p.forward_path_length);

  return getCenterLinePath(
    *route_handler, reference_lanes, current_pose, p.backward_path_length, p.forward_path_length,
    p);
}

PathWithLaneId getCenterLinePath(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & lanelet_sequence,
  const Pose & pose, const double backward_path_length, const double forward_path_length,
  const BehaviorPathPlannerParameters & parameter)
{
  PathWithLaneId reference_path;

  if (lanelet_sequence.empty()) {
    return reference_path;
  }

  const auto arc_coordinates = lanelet::utils::getArcCoordinates(lanelet_sequence, pose);
  const double s = arc_coordinates.length;
  const double s_backward = std::max(0., s - backward_path_length);
  double s_forward = s + forward_path_length;

  if (route_handler.isDeadEndLanelet(lanelet_sequence.back())) {
    const auto lane_length = lanelet::utils::getLaneletLength2d(lanelet_sequence);
    s_forward = std::clamp(s_forward, 0.0, lane_length);
  }

  if (route_handler.isInGoalRouteSection(lanelet_sequence.back())) {
    const auto goal_arc_coordinates =
      lanelet::utils::getArcCoordinates(lanelet_sequence, route_handler.getGoalPose());
    s_forward = std::clamp(s_forward, 0.0, goal_arc_coordinates.length);
  }

  const auto raw_path_with_lane_id =
    route_handler.getCenterLinePath(lanelet_sequence, s_backward, s_forward, true);
  auto resampled_path_with_lane_id = motion_utils::resamplePath(
    raw_path_with_lane_id, parameter.input_path_interval, parameter.enable_akima_spline_first);

  // convert centerline, which we consider as CoG center,  to rear wheel center
  if (parameter.enable_cog_on_centerline) {
    const double rear_to_cog = parameter.vehicle_length / 2 - parameter.rear_overhang;
    return motion_utils::convertToRearWheelCenter(resampled_path_with_lane_id, rear_to_cog);
  }

  return resampled_path_with_lane_id;
}

// for lane following
PathWithLaneId setDecelerationVelocity(
  const RouteHandler & route_handler, const PathWithLaneId & input,
  const lanelet::ConstLanelets & lanelet_sequence, const double lane_change_prepare_duration,
  const double lane_change_buffer)
{
  auto reference_path = input;
  if (
    route_handler.isDeadEndLanelet(lanelet_sequence.back()) &&
    lane_change_prepare_duration > std::numeric_limits<double>::epsilon()) {
    for (auto & point : reference_path.points) {
      const double lane_length = lanelet::utils::getLaneletLength2d(lanelet_sequence);
      const auto arclength = lanelet::utils::getArcCoordinates(lanelet_sequence, point.point.pose);
      const double distance_to_end =
        std::max(0.0, lane_length - std::abs(lane_change_buffer) - arclength.length);
      point.point.longitudinal_velocity_mps = std::min(
        point.point.longitudinal_velocity_mps,
        static_cast<float>(distance_to_end / lane_change_prepare_duration));
    }
  }
  return reference_path;
}

// TODO(murooka) remove calcSignedArcLength using findNearestSegmentIndex inside the
// function
PathWithLaneId setDecelerationVelocity(
  const PathWithLaneId & input, const double target_velocity, const Pose target_pose,
  const double buffer, const double deceleration_interval)
{
  auto reference_path = input;

  for (auto & point : reference_path.points) {
    const auto arclength_to_target = std::max(
      0.0, motion_utils::calcSignedArcLength(
             reference_path.points, point.point.pose.position, target_pose.position) +
             buffer);
    if (arclength_to_target > deceleration_interval) continue;
    point.point.longitudinal_velocity_mps = std::min(
      point.point.longitudinal_velocity_mps,
      static_cast<float>(
        (arclength_to_target / deceleration_interval) *
          (point.point.longitudinal_velocity_mps - target_velocity) +
        target_velocity));
  }

  const auto stop_point_length =
    motion_utils::calcSignedArcLength(reference_path.points, 0, target_pose.position) + buffer;
  constexpr double eps{0.01};
  if (std::abs(target_velocity) < eps && stop_point_length > 0.0) {
    const auto stop_point = utils::insertStopPoint(stop_point_length, reference_path);
  }

  return reference_path;
}

BehaviorModuleOutput getReferencePath(
  const lanelet::ConstLanelet & current_lane,
  const std::shared_ptr<const PlannerData> & planner_data)
{
  PathWithLaneId reference_path{};

  const auto & route_handler = planner_data->route_handler;
  const auto current_pose = planner_data->self_odometry->pose.pose;
  const auto p = planner_data->parameters;

  // Set header
  reference_path.header = route_handler->getRouteHeader();

  // calculate path with backward margin to avoid end points' instability by spline interpolation
  constexpr double extra_margin = 10.0;
  const double backward_length = p.backward_path_length + extra_margin;
  const auto current_lanes_with_backward_margin =
    route_handler->getLaneletSequence(current_lane, backward_length, p.forward_path_length);
  const auto no_shift_pose =
    lanelet::utils::getClosestCenterPose(current_lane, current_pose.position);
  reference_path = getCenterLinePath(
    *route_handler, current_lanes_with_backward_margin, no_shift_pose, backward_length,
    p.forward_path_length, p);

  // clip backward length
  // NOTE: In order to keep backward_path_length at least, resampling interval is added to the
  // backward.
  const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    reference_path.points, no_shift_pose, p.ego_nearest_dist_threshold,
    p.ego_nearest_yaw_threshold);
  reference_path.points = motion_utils::cropPoints(
    reference_path.points, no_shift_pose.position, current_seg_idx, p.forward_path_length,
    p.backward_path_length + p.input_path_interval);

  const auto drivable_lanelets = getLaneletsFromPath(reference_path, route_handler);
  const auto drivable_lanes = generateDrivableLanes(drivable_lanelets);

  const auto & dp = planner_data->drivable_area_expansion_parameters;

  const auto shorten_lanes = cutOverlappedLanes(reference_path, drivable_lanes);
  const auto expanded_lanes = expandLanelets(
    shorten_lanes, dp.drivable_area_left_bound_offset, dp.drivable_area_right_bound_offset,
    dp.drivable_area_types_to_skip);

  BehaviorModuleOutput output;
  output.path = std::make_shared<PathWithLaneId>(reference_path);
  output.reference_path = std::make_shared<PathWithLaneId>(reference_path);
  output.drivable_area_info.drivable_lanes = drivable_lanes;

  return output;
}

std::uint8_t getHighestProbLabel(const std::vector<ObjectClassification> & classification)
{
  std::uint8_t label = ObjectClassification::UNKNOWN;
  float highest_prob = 0.0;
  for (const auto & _class : classification) {
    if (highest_prob < _class.probability) {
      highest_prob = _class.probability;
      label = _class.label;
    }
  }
  return label;
}

lanelet::ConstLanelets getCurrentLanes(
  const std::shared_ptr<const PlannerData> & planner_data, const double backward_path_length,
  const double forward_path_length)
{
  const auto & route_handler = planner_data->route_handler;
  const auto current_pose = planner_data->self_odometry->pose.pose;

  lanelet::ConstLanelet current_lane;
  if (!route_handler->getClosestLaneletWithinRoute(current_pose, &current_lane)) {
    auto clock{rclcpp::Clock{RCL_ROS_TIME}};
    RCLCPP_ERROR_STREAM_THROTTLE(
      rclcpp::get_logger("behavior_path_planner").get_child("utils"), clock, 1000,
      "failed to find closest lanelet within route!!!");
    return {};  // TODO(Horibe) what should be returned?
  }

  // For current_lanes with desired length
  return route_handler->getLaneletSequence(
    current_lane, current_pose, backward_path_length, forward_path_length);
}

lanelet::ConstLanelets getCurrentLanes(const std::shared_ptr<const PlannerData> & planner_data)
{
  const auto & common_parameters = planner_data->parameters;
  return getCurrentLanes(
    planner_data, common_parameters.backward_path_length, common_parameters.forward_path_length);
}

lanelet::ConstLanelets getCurrentLanesFromPath(
  const PathWithLaneId & path, const std::shared_ptr<const PlannerData> & planner_data)
{
  const auto & route_handler = planner_data->route_handler;
  const auto & current_pose = planner_data->self_odometry->pose.pose;
  const auto & p = planner_data->parameters;

  std::set<uint64_t> lane_ids;
  for (const auto & p : path.points) {
    for (const auto & id : p.lane_ids) {
      lane_ids.insert(id);
    }
  }

  lanelet::ConstLanelets reference_lanes{};
  for (const auto & id : lane_ids) {
    reference_lanes.push_back(planner_data->route_handler->getLaneletsFromId(id));
  }

  lanelet::ConstLanelet current_lane;
  lanelet::utils::query::getClosestLanelet(reference_lanes, current_pose, &current_lane);
  auto current_lanes = route_handler->getLaneletSequence(
    current_lane, current_pose, p.backward_path_length, p.forward_path_length);

  // Extend the 'current_lanes' with previous lanes until it contains 'front_lane_ids'
  // if the extended prior lanes is in same lane sequence with current lanes
  const auto front_lane_ids = path.points.front().lane_ids;
  auto have_front_lanes = [front_lane_ids](const auto & lanes) {
    return std::any_of(lanes.begin(), lanes.end(), [&](const auto & lane) {
      return std::find(front_lane_ids.begin(), front_lane_ids.end(), lane.id()) !=
             front_lane_ids.end();
    });
  };
  auto extended_lanes = current_lanes;
  while (rclcpp::ok()) {
    const size_t pre_extension_size = extended_lanes.size();  // Get existing size before extension
    extended_lanes = extendPrevLane(route_handler, extended_lanes, true);
    if (extended_lanes.size() == pre_extension_size) break;
    if (have_front_lanes(extended_lanes)) {
      current_lanes = extended_lanes;
      break;
    }
  }

  return current_lanes;
}

lanelet::ConstLanelets extendNextLane(
  const std::shared_ptr<RouteHandler> route_handler, const lanelet::ConstLanelets & lanes,
  const bool only_in_route)
{
  if (lanes.empty()) return lanes;

  auto extended_lanes = lanes;

  // Add next lane
  const auto next_lanes = route_handler->getNextLanelets(extended_lanes.back());
  if (!next_lanes.empty()) {
    boost::optional<lanelet::ConstLanelet> target_next_lane;
    if (!only_in_route) {
      target_next_lane = next_lanes.front();
    }
    // use the next lane in route if it exists
    for (const auto & next_lane : next_lanes) {
      if (route_handler->isRouteLanelet(next_lane)) {
        target_next_lane = next_lane;
      }
    }
    if (target_next_lane) {
      extended_lanes.push_back(*target_next_lane);
    }
  }

  return extended_lanes;
}

lanelet::ConstLanelets extendPrevLane(
  const std::shared_ptr<RouteHandler> route_handler, const lanelet::ConstLanelets & lanes,
  const bool only_in_route)
{
  if (lanes.empty()) return lanes;

  auto extended_lanes = lanes;

  // Add previous lane
  const auto prev_lanes = route_handler->getPreviousLanelets(extended_lanes.front());
  if (!prev_lanes.empty()) {
    boost::optional<lanelet::ConstLanelet> target_prev_lane;
    if (!only_in_route) {
      target_prev_lane = prev_lanes.front();
    }
    // use the previous lane in route if it exists
    for (const auto & prev_lane : prev_lanes) {
      if (route_handler->isRouteLanelet(prev_lane)) {
        target_prev_lane = prev_lane;
      }
    }
    if (target_prev_lane) {
      extended_lanes.insert(extended_lanes.begin(), *target_prev_lane);
    }
  }
  return extended_lanes;
}

lanelet::ConstLanelets extendLanes(
  const std::shared_ptr<RouteHandler> route_handler, const lanelet::ConstLanelets & lanes)
{
  auto extended_lanes = extendNextLane(route_handler, lanes);
  extended_lanes = extendPrevLane(route_handler, extended_lanes);

  return extended_lanes;
}

lanelet::ConstLanelets getExtendedCurrentLanes(
  const std::shared_ptr<const PlannerData> & planner_data, const double backward_length,
  const double forward_length, const bool forward_only_in_route)
{
  auto lanes = getCurrentLanes(planner_data);
  if (lanes.empty()) return lanes;
  const auto start_lane = lanes.front();

  double forward_length_sum = 0.0;
  double backward_length_sum = 0.0;

  while (backward_length_sum < backward_length) {
    auto extended_lanes = extendPrevLane(planner_data->route_handler, lanes);
    if (extended_lanes.empty()) {
      return lanes;
    }
    // loop check
    // if current map lanes is looping and has a very large value for backward_length,
    // the extending process will not finish.
    if (extended_lanes.front().id() == start_lane.id()) {
      return lanes;
    }

    if (extended_lanes.size() > lanes.size()) {
      backward_length_sum += lanelet::utils::getLaneletLength2d(extended_lanes.front());
    } else {
      break;  // no more previous lanes to add
    }
    lanes = extended_lanes;
  }

  while (forward_length_sum < forward_length) {
    auto extended_lanes = extendNextLane(planner_data->route_handler, lanes);
    if (extended_lanes.empty()) {
      return lanes;
    }
    // loop check
    // if current map lanes is looping and has a very large value for forward_length,
    // the extending process will not finish.
    if (extended_lanes.back().id() == start_lane.id()) {
      return lanes;
    }

    if (extended_lanes.size() > lanes.size()) {
      forward_length_sum += lanelet::utils::getLaneletLength2d(extended_lanes.back());
    } else {
      break;  // no more next lanes to add
    }

    // stop extending when the lane outside of the route is reached
    // if forward_length is a very large value, set it to true,
    // as it may continue to extend forever.
    if (forward_only_in_route) {
      if (!planner_data->route_handler->isRouteLanelet(extended_lanes.back())) {
        return lanes;
      }
    }

    lanes = extended_lanes;
  }

  return lanes;
}

lanelet::ConstLanelets getExtendedCurrentLanes(
  const std::shared_ptr<const PlannerData> & planner_data)
{
  return extendLanes(planner_data->route_handler, getCurrentLanes(planner_data));
}

lanelet::ConstLanelets calcLaneAroundPose(
  const std::shared_ptr<RouteHandler> route_handler, const Pose & pose, const double forward_length,
  const double backward_length, const double dist_threshold, const double yaw_threshold)
{
  lanelet::ConstLanelet current_lane;
  if (!route_handler->getClosestLaneletWithConstrainsWithinRoute(
        pose, &current_lane, dist_threshold, yaw_threshold)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("behavior_path_planner").get_child("avoidance"),
      "failed to find closest lanelet within route!!!");
    return {};  // TODO(Horibe)
  }

  // For current_lanes with desired length
  lanelet::ConstLanelets current_lanes =
    route_handler->getLaneletSequence(current_lane, pose, backward_length, forward_length);

  return current_lanes;
}

bool checkPathRelativeAngle(const PathWithLaneId & path, const double angle_threshold)
{
  // We need at least three points to compute relative angle
  constexpr size_t relative_angle_points_num = 3;
  if (path.points.size() < relative_angle_points_num) {
    return true;
  }

  for (size_t p1_id = 0; p1_id <= path.points.size() - relative_angle_points_num; ++p1_id) {
    // Get Point1
    const auto & p1 = path.points.at(p1_id).point.pose.position;

    // Get Point2
    const auto & p2 = path.points.at(p1_id + 1).point.pose.position;

    // Get Point3
    const auto & p3 = path.points.at(p1_id + 2).point.pose.position;

    // ignore invert driving direction
    if (
      path.points.at(p1_id).point.longitudinal_velocity_mps < 0 ||
      path.points.at(p1_id + 1).point.longitudinal_velocity_mps < 0 ||
      path.points.at(p1_id + 2).point.longitudinal_velocity_mps < 0) {
      continue;
    }

    // convert to p1 coordinate
    const double x3 = p3.x - p1.x;
    const double x2 = p2.x - p1.x;
    const double y3 = p3.y - p1.y;
    const double y2 = p2.y - p1.y;

    // calculate relative angle of vector p3 based on p1p2 vector
    const double th = std::atan2(y2, x2);
    const double th2 =
      std::atan2(-x3 * std::sin(th) + y3 * std::cos(th), x3 * std::cos(th) + y3 * std::sin(th));
    if (std::abs(th2) > angle_threshold) {
      // invalid angle
      return false;
    }
  }

  return true;
}

double calcMinimumLaneChangeLength(
  const BehaviorPathPlannerParameters & common_param, const std::vector<double> & shift_intervals,
  const double length_to_intersection)
{
  if (shift_intervals.empty()) {
    return 0.0;
  }

  const double & vel = common_param.minimum_lane_changing_velocity;
  const auto lat_acc = common_param.lane_change_lat_acc_map.find(vel);
  const double & max_lateral_acc = lat_acc.second;
  const double & lateral_jerk = common_param.lane_changing_lateral_jerk;
  const double & finish_judge_buffer = common_param.lane_change_finish_judge_buffer;

  double accumulated_length = length_to_intersection;
  for (const auto & shift_interval : shift_intervals) {
    const double t =
      PathShifter::calcShiftTimeFromJerk(shift_interval, lateral_jerk, max_lateral_acc);
    accumulated_length += vel * t + finish_judge_buffer;
  }
  accumulated_length +=
    common_param.backward_length_buffer_for_end_of_lane * (shift_intervals.size() - 1.0);

  return accumulated_length;
}

lanelet::ConstLanelets getLaneletsFromPath(
  const PathWithLaneId & path, const std::shared_ptr<route_handler::RouteHandler> & route_handler)
{
  std::vector<int64_t> unique_lanelet_ids;
  for (const auto & p : path.points) {
    const auto & lane_ids = p.lane_ids;
    for (const auto & lane_id : lane_ids) {
      if (
        std::find(unique_lanelet_ids.begin(), unique_lanelet_ids.end(), lane_id) ==
        unique_lanelet_ids.end()) {
        unique_lanelet_ids.push_back(lane_id);
      }
    }
  }

  lanelet::ConstLanelets lanelets;
  for (const auto & lane_id : unique_lanelet_ids) {
    lanelets.push_back(route_handler->getLaneletsFromId(lane_id));
  }

  return lanelets;
}

std::string convertToSnakeCase(const std::string & input_str)
{
  std::string output_str = std::string{static_cast<char>(std::tolower(input_str.at(0)))};
  for (size_t i = 1; i < input_str.length(); ++i) {
    const auto input_chr = input_str.at(i);
    if (std::isupper(input_chr)) {
      output_str += "_" + std::string{static_cast<char>(std::tolower(input_chr))};
    } else {
      output_str += input_chr;
    }
  }
  return output_str;
}

std::vector<DrivableLanes> combineDrivableLanes(
  const std::vector<DrivableLanes> & original_drivable_lanes_vec,
  const std::vector<DrivableLanes> & new_drivable_lanes_vec)
{
  const auto has_same_lane =
    [](const lanelet::ConstLanelet & target_lane, const lanelet::ConstLanelets & lanes) {
      return std::find_if(lanes.begin(), lanes.end(), [&](const auto & ll) {
               return ll.id() == target_lane.id();
             }) != lanes.end();
    };

  const auto convert_to_lanes = [](const DrivableLanes & drivable_lanes) {
    auto lanes = drivable_lanes.middle_lanes;
    lanes.push_back(drivable_lanes.right_lane);
    lanes.push_back(drivable_lanes.left_lane);
    return lanes;
  };

  auto updated_drivable_lanes_vec = original_drivable_lanes_vec;
  size_t new_drivable_lanes_idx = 0;
  for (auto & updated_drivable_lanes : updated_drivable_lanes_vec) {
    // calculated corresponding index of new_drivable_lanes
    const auto opt_new_drivable_lanes_idx = [&]() -> std::optional<size_t> {
      for (size_t n_idx = 0; n_idx < new_drivable_lanes_vec.size(); ++n_idx) {
        for (const auto & ll : convert_to_lanes(updated_drivable_lanes)) {
          if (has_same_lane(ll, convert_to_lanes(new_drivable_lanes_vec.at(n_idx)))) {
            return n_idx;
          }
        }
      }
      return std::nullopt;
    }();
    if (!opt_new_drivable_lanes_idx) {
      continue;
    }
    new_drivable_lanes_idx = *opt_new_drivable_lanes_idx;
    const auto & new_drivable_lanes = new_drivable_lanes_vec.at(new_drivable_lanes_idx);

    // update left lane
    if (has_same_lane(updated_drivable_lanes.left_lane, convert_to_lanes(new_drivable_lanes))) {
      updated_drivable_lanes.left_lane = new_drivable_lanes.left_lane;
    }
    // update right lane
    if (has_same_lane(updated_drivable_lanes.right_lane, convert_to_lanes(new_drivable_lanes))) {
      updated_drivable_lanes.right_lane = new_drivable_lanes.right_lane;
    }
    // update middle lanes
    for (const auto & middle_lane : convert_to_lanes(new_drivable_lanes)) {
      if (!has_same_lane(middle_lane, convert_to_lanes(updated_drivable_lanes))) {
        updated_drivable_lanes.middle_lanes.push_back(middle_lane);
      }
    }

    // validate middle lanes
    auto & middle_lanes = updated_drivable_lanes.middle_lanes;
    if (has_same_lane(updated_drivable_lanes.right_lane, middle_lanes)) {
      middle_lanes.erase(
        std::remove(
          std::begin(middle_lanes), std::end(middle_lanes), updated_drivable_lanes.right_lane),
        std::cend(middle_lanes));
    }
    if (has_same_lane(updated_drivable_lanes.left_lane, middle_lanes)) {
      middle_lanes.erase(
        std::remove(
          std::begin(middle_lanes), std::end(middle_lanes), updated_drivable_lanes.left_lane),
        std::cend(middle_lanes));
    }
  }
  // NOTE: If original_drivable_lanes_vec is shorter than new_drivable_lanes_vec, push back remained
  // new_drivable_lanes_vec.
  if (new_drivable_lanes_idx + 1 < new_drivable_lanes_vec.size()) {
    updated_drivable_lanes_vec.insert(
      updated_drivable_lanes_vec.end(), new_drivable_lanes_vec.begin() + new_drivable_lanes_idx + 1,
      new_drivable_lanes_vec.end());
  }

  return updated_drivable_lanes_vec;
}

DrivableAreaInfo combineDrivableAreaInfo(
  const DrivableAreaInfo & drivable_area_info1, const DrivableAreaInfo & drivable_area_info2)
{
  DrivableAreaInfo combined_drivable_area_info;

  // drivable lanes
  combined_drivable_area_info.drivable_lanes =
    combineDrivableLanes(drivable_area_info1.drivable_lanes, drivable_area_info2.drivable_lanes);

  // obstacles
  for (const auto & obstacle : drivable_area_info1.obstacles) {
    combined_drivable_area_info.obstacles.push_back(obstacle);
  }
  for (const auto & obstacle : drivable_area_info2.obstacles) {
    combined_drivable_area_info.obstacles.push_back(obstacle);
  }

  // enable expanding hatched road markings
  combined_drivable_area_info.enable_expanding_hatched_road_markings =
    drivable_area_info1.enable_expanding_hatched_road_markings ||
    drivable_area_info2.enable_expanding_hatched_road_markings;

  // enable expanding intersection areas
  combined_drivable_area_info.enable_expanding_intersection_areas =
    drivable_area_info1.enable_expanding_intersection_areas ||
    drivable_area_info2.enable_expanding_intersection_areas;

  return combined_drivable_area_info;
}

// NOTE: Assuming that path.right/left_bound is already created.
void extractObstaclesFromDrivableArea(
  PathWithLaneId & path, const std::vector<DrivableAreaInfo::Obstacle> & obstacles)
{
  if (obstacles.empty()) {
    return;
  }

  std::vector<std::vector<PolygonPoint>> right_polygons;
  std::vector<std::vector<PolygonPoint>> left_polygons;
  for (const auto & obstacle : obstacles) {
    const auto & obj_pos = obstacle.pose.position;

    // get edge points of the object
    const size_t nearest_path_idx =
      motion_utils::findNearestIndex(path.points, obj_pos);  // to get z for object polygon
    std::vector<Point> edge_points;
    for (size_t i = 0; i < obstacle.poly.outer().size() - 1;
         ++i) {  // NOTE: There is a duplicated points
      edge_points.push_back(tier4_autoware_utils::createPoint(
        obstacle.poly.outer().at(i).x(), obstacle.poly.outer().at(i).y(),
        path.points.at(nearest_path_idx).point.pose.position.z));
    }

    // get a boundary that we have to change
    const bool is_object_right = !obstacle.is_left;
    const auto & bound = is_object_right ? path.right_bound : path.left_bound;

    // get polygon points inside the bounds
    const auto inside_polygon =
      drivable_area_processing::getPolygonPointsInsideBounds(bound, edge_points, is_object_right);
    if (!inside_polygon.empty()) {
      if (is_object_right) {
        right_polygons.push_back(inside_polygon);
      } else {
        left_polygons.push_back(inside_polygon);
      }
    }
  }

  for (const bool is_object_right : {true, false}) {
    const auto & polygons = is_object_right ? right_polygons : left_polygons;
    if (polygons.empty()) {
      continue;
    }

    // concatenate polygons if they are longitudinal overlapped.
    auto unique_polygons = drivable_area_processing::concatenatePolygons(polygons);

    // sort bounds longitudinally
    std::sort(
      unique_polygons.begin(), unique_polygons.end(),
      [](const std::vector<PolygonPoint> & p1, const std::vector<PolygonPoint> & p2) {
        return p2.front().is_after(p1.front());
      });

    // update boundary
    auto & bound = is_object_right ? path.right_bound : path.left_bound;
    bound = drivable_area_processing::updateBoundary(bound, unique_polygons);
  }
}

lanelet::ConstLanelets combineLanelets(
  const lanelet::ConstLanelets & base_lanes, const lanelet::ConstLanelets & added_lanes)
{
  lanelet::ConstLanelets combined_lanes = base_lanes;
  for (const auto & added_lane : added_lanes) {
    const auto it = std::find_if(
      combined_lanes.begin(), combined_lanes.end(),
      [&added_lane](const lanelet::ConstLanelet & lane) { return lane.id() == added_lane.id(); });
    if (it == combined_lanes.end()) {
      combined_lanes.push_back(added_lane);
    }
  }

  return combined_lanes;
}

}  // namespace behavior_path_planner::utils
