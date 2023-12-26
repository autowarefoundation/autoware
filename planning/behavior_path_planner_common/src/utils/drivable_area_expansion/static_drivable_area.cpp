// Copyright 2023 TIER IV, Inc.
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
#include "behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp"

#include "behavior_path_planner_common/utils/drivable_area_expansion/drivable_area_expansion.hpp"
#include "motion_utils/trajectory/trajectory.hpp"

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

namespace
{
template <class T>
size_t findNearestSegmentIndexFromLateralDistance(
  const std::vector<T> & points, const geometry_msgs::msg::Pose & target_point,
  const double yaw_threshold)
{
  using tier4_autoware_utils::calcAzimuthAngle;
  using tier4_autoware_utils::calcDistance2d;
  using tier4_autoware_utils::normalizeRadian;

  std::optional<size_t> closest_idx{std::nullopt};
  double min_lateral_dist = std::numeric_limits<double>::max();
  for (size_t seg_idx = 0; seg_idx < points.size() - 1; ++seg_idx) {
    const auto base_yaw = tf2::getYaw(target_point.orientation);
    const auto yaw =
      normalizeRadian(calcAzimuthAngle(points.at(seg_idx), points.at(seg_idx + 1)) - base_yaw);
    if (yaw_threshold < std::abs(yaw)) {
      continue;
    }
    const double lon_dist =
      motion_utils::calcLongitudinalOffsetToSegment(points, seg_idx, target_point.position);
    const double segment_length = calcDistance2d(points.at(seg_idx), points.at(seg_idx + 1));
    const double lat_dist = [&]() {
      if (lon_dist < 0.0) {
        return calcDistance2d(points.at(seg_idx), target_point.position);
      }
      if (segment_length < lon_dist) {
        return calcDistance2d(points.at(seg_idx + 1), target_point.position);
      }
      return std::abs(motion_utils::calcLateralOffset(points, target_point.position, seg_idx));
    }();
    if (lat_dist < min_lateral_dist) {
      closest_idx = seg_idx;
      min_lateral_dist = lat_dist;
    }
  }

  if (closest_idx) {
    return *closest_idx;
  }

  return motion_utils::findNearestSegmentIndex(points, target_point.position);
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

geometry_msgs::msg::Point calcLongitudinalOffsetStartPoint(
  const std::vector<geometry_msgs::msg::Point> & points, const geometry_msgs::msg::Pose & pose,
  const size_t nearest_segment_idx, const double offset)
{
  const double offset_length =
    motion_utils::calcLongitudinalOffsetToSegment(points, nearest_segment_idx, pose.position);
  const auto offset_point =
    motion_utils::calcLongitudinalOffsetPoint(points, nearest_segment_idx, offset_length + offset);

  return offset_point ? offset_point.value() : points.at(nearest_segment_idx);
}

geometry_msgs::msg::Point calcLongitudinalOffsetGoalPoint(
  const std::vector<geometry_msgs::msg::Point> & points, const geometry_msgs::msg::Pose & pose,
  const size_t nearest_segment_idx, const double offset)
{
  const double offset_length =
    motion_utils::calcLongitudinalOffsetToSegment(points, nearest_segment_idx, pose.position);
  const auto offset_point =
    motion_utils::calcLongitudinalOffsetPoint(points, nearest_segment_idx, offset_length + offset);

  return offset_point ? offset_point.value() : points.at(nearest_segment_idx + 1);
}
}  // namespace

namespace behavior_path_planner::utils::drivable_area_processing
{
std::optional<std::pair<size_t, geometry_msgs::msg::Point>> intersectBound(
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
  return std::nullopt;
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
      start_point.point = start_point_on_bound.value();
      valid_inside_polygon.insert(valid_inside_polygon.begin(), start_point);
    }
  }
  if (inside_polygon.at(end_idx).lat_dist_to_bound != 0.0) {  // not on bound
    auto end_point = inside_polygon.at(end_idx);
    const auto end_point_on_bound = motion_utils::calcLongitudinalOffsetPoint(
      bound, end_point.bound_seg_idx, end_point.lon_dist_to_segment);
    if (end_point_on_bound) {
      end_point.point = end_point_on_bound.value();
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
}  // namespace behavior_path_planner::utils::drivable_area_processing

namespace behavior_path_planner::utils
{
using tier4_autoware_utils::Point2d;

std::optional<size_t> getOverlappedLaneletId(const std::vector<DrivableLanes> & lanes)
{
  auto overlaps = [](const DrivableLanes & lanes, const DrivableLanes & target_lanes) {
    const auto lanelets = utils::transformToLanelets(lanes);
    const auto target_lanelets = utils::transformToLanelets(target_lanes);

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

  const auto is_point_in_drivable_lanes = [&has_same_id_lane, &has_same_id_lanes](
                                            const auto & lanes, const auto & p) {
    if (has_same_id_lane(lanes.right_lane, p)) {
      return true;
    }
    // check left lane
    if (has_same_id_lane(lanes.left_lane, p)) {
      return true;
    }
    // check middle lanes
    if (has_same_id_lanes(lanes.middle_lanes, p)) {
      return true;
    }
    return false;
  };

  // Step1. find first path point within drivable lanes
  size_t start_point_idx = original_points.size();

  for (size_t i = 0; i < original_points.size(); ++i) {
    const bool first_path_point_in_drivable_lane_found = std::any_of(
      shorten_lanes.begin(), shorten_lanes.end(),
      [&is_point_in_drivable_lanes, &original_points, i](const auto & lanes) {
        return is_point_in_drivable_lanes(lanes, original_points.at(i));
      });
    if (first_path_point_in_drivable_lane_found) {
      start_point_idx = i;
      break;
    }
  }

  // Step2. pick up only path points within drivable lanes
  for (const auto & lanes : shorten_lanes) {
    for (size_t i = start_point_idx; i < original_points.size(); ++i) {
      if (is_point_in_drivable_lanes(lanes, original_points.at(i))) {
        path.points.push_back(original_points.at(i));
        continue;
      }
      start_point_idx = i;
      break;
    }
  }

  return shorten_lanes;
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

    const auto right_lane = utils::getRightLanelet(current_lane, shoulder_lanes);
    const auto left_lane = utils::getLeftLanelet(current_lane, shoulder_lanes);

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
  const auto shorten_lanes = cutOverlappedLanes(path, lanes);
  return utils::expandLanelets(
    shorten_lanes, parameters.drivable_area_left_bound_offset,
    parameters.drivable_area_right_bound_offset, parameters.drivable_area_types_to_skip);
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
      findNearestSegmentIndexFromLateralDistance(left_bound, front_pose, M_PI_2);
    const size_t front_right_start_idx =
      findNearestSegmentIndexFromLateralDistance(right_bound, front_pose, M_PI_2);
    const auto left_start_point =
      calcLongitudinalOffsetStartPoint(left_bound, front_pose, front_left_start_idx, -front_length);
    const auto right_start_point = calcLongitudinalOffsetStartPoint(
      right_bound, front_pose, front_right_start_idx, -front_length);
    const size_t left_start_idx =
      motion_utils::findNearestSegmentIndex(left_bound, left_start_point);
    const size_t right_start_idx =
      motion_utils::findNearestSegmentIndex(right_bound, right_start_point);

    // Insert a start point
    path.left_bound.push_back(left_start_point);
    path.right_bound.push_back(right_start_point);

    return std::make_pair(left_start_idx, right_start_idx);
  }();

  // Get Closest segment for the goal point
  const auto goal_pose = path.points.empty() ? current_pose : path.points.back().point.pose;
  const size_t goal_left_start_idx =
    findNearestSegmentIndexFromLateralDistance(left_bound, goal_pose, M_PI_2);
  const size_t goal_right_start_idx =
    findNearestSegmentIndexFromLateralDistance(right_bound, goal_pose, M_PI_2);
  const auto left_goal_point =
    calcLongitudinalOffsetGoalPoint(left_bound, goal_pose, goal_left_start_idx, vehicle_length);
  const auto right_goal_point =
    calcLongitudinalOffsetGoalPoint(right_bound, goal_pose, goal_right_start_idx, vehicle_length);
  const size_t left_goal_idx = std::max(
    goal_left_start_idx, motion_utils::findNearestSegmentIndex(left_bound, left_goal_point));
  const size_t right_goal_idx = std::max(
    goal_right_start_idx, motion_utils::findNearestSegmentIndex(right_bound, right_goal_point));

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
    if (obstacle.poly.outer().empty()) {
      continue;
    }

    const auto & obj_pos = obstacle.pose.position;

    // get edge points of the object
    const size_t nearest_path_idx =
      motion_utils::findNearestIndex(path.points, obj_pos);  // to get z for object polygon
    std::vector<Point> edge_points;
    for (int i = 0; i < static_cast<int>(obstacle.poly.outer().size()) - 1;
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
      } else if (polygon.value().id() != current_polygon.value().id()) {
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

      if (polygon.has_value() && current_polygon.has_value()) {
        if (polygon.value().id() != current_polygon.value().id()) {
          current_polygon = polygon;
          current_polygon_border_indices.clear();
          current_polygon_border_indices.push_back(
            get_corresponding_polygon_index(current_polygon.value(), bound_point.id()));
          continue;
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
                                   const auto & p1, const auto & p2) -> std::optional<size_t> {
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
      return std::nullopt;
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
        for (size_t j = intersect_idx.value(); j < bound_with_pose.size(); j++) {
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
        for (size_t j = intersect_idx.value() + 1; j < bound_with_pose.size(); j++) {
          set_orientation(ret, j, getPose(path_points.at(i)).orientation);
        }
      }

      constexpr size_t OVERLAP_CHECK_NUM = 3;
      start_bound_idx =
        intersect_idx.value() < OVERLAP_CHECK_NUM ? 0 : intersect_idx.value() - OVERLAP_CHECK_NUM;
    }

    return ret;
  };

  const auto is_non_monotonic = [&](
                                  const auto & base_pose, const auto & point,
                                  const auto is_points_left, const auto is_reverse) {
    const auto p_transformed = tier4_autoware_utils::inverseTransformPoint(point, base_pose);
    if (p_transformed.x < 0.0 && p_transformed.y > 0.0) {
      return is_points_left && !is_reverse;
    }

    if (p_transformed.x < 0.0 && p_transformed.y < 0.0) {
      return !is_points_left && !is_reverse;
    }

    if (p_transformed.x > 0.0 && p_transformed.y > 0.0) {
      return is_points_left && is_reverse;
    }

    if (p_transformed.x > 0.0 && p_transformed.y < 0.0) {
      return !is_points_left && is_reverse;
    }

    return false;
  };

  // define a function to remove non monotonic point on bound
  const auto remove_non_monotonic_point = [&](const auto & bound_with_pose, const bool is_reverse) {
    std::vector<Pose> monotonic_bound;

    size_t bound_idx = 0;
    while (true) {
      monotonic_bound.push_back(bound_with_pose.at(bound_idx));

      if (bound_idx + 1 == bound_with_pose.size()) {
        break;
      }

      // NOTE: is_bound_left is used instead of is_points_left since orientation of path point is
      // opposite.
      const double lat_offset = is_bound_left ? 100.0 : -100.0;

      const auto p_bound_1 = getPose(bound_with_pose.at(bound_idx));
      const auto p_bound_2 = getPose(bound_with_pose.at(bound_idx + 1));

      const auto p_bound_offset = calcOffsetPose(p_bound_1, 0.0, lat_offset, 0.0);

      if (!is_non_monotonic(p_bound_1, p_bound_2.position, is_bound_left, is_reverse)) {
        bound_idx++;
        continue;
      }

      // skip non monotonic points
      for (size_t i = bound_idx + 1; i < bound_with_pose.size() - 1; ++i) {
        const auto intersect_point = intersect(
          p_bound_1.position, p_bound_offset.position, bound_with_pose.at(i).position,
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
