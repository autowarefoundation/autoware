// Copyright 2020 Tier IV, Inc.
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

#include "util.hpp"

#include "util_type.hpp"

#include <behavior_velocity_planner_common/utilization/boost_geometry_helper.hpp>
#include <behavior_velocity_planner_common/utilization/path_utilization.hpp>
#include <behavior_velocity_planner_common/utilization/trajectory_utils.hpp>
#include <behavior_velocity_planner_common/utilization/util.hpp>
#include <interpolation/spline_interpolation_points_2d.hpp>
#include <lanelet2_extension/regulatory_elements/road_marking.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/geometry/boost_polygon_utils.hpp>

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/intersects.hpp>

#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <list>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace tier4_autoware_utils
{

template <>
inline geometry_msgs::msg::Point getPoint(const lanelet::ConstPoint3d & p)
{
  geometry_msgs::msg::Point point;
  point.x = p.x();
  point.y = p.y();
  point.z = p.z();
  return point;
}

}  // namespace tier4_autoware_utils

namespace behavior_velocity_planner
{
namespace bg = boost::geometry;

namespace util
{

static std::optional<size_t> getDuplicatedPointIdx(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const geometry_msgs::msg::Point & point)
{
  for (size_t i = 0; i < path.points.size(); i++) {
    const auto & p = path.points.at(i).point.pose.position;

    constexpr double min_dist = 0.001;
    if (tier4_autoware_utils::calcDistance2d(p, point) < min_dist) {
      return i;
    }
  }

  return std::nullopt;
}

static std::optional<size_t> insertPointIndex(
  const geometry_msgs::msg::Pose & in_pose,
  autoware_auto_planning_msgs::msg::PathWithLaneId * inout_path,
  const double ego_nearest_dist_threshold, const double ego_nearest_yaw_threshold)
{
  const auto duplicate_idx_opt = getDuplicatedPointIdx(*inout_path, in_pose.position);
  if (duplicate_idx_opt) {
    return duplicate_idx_opt.value();
  }

  const size_t closest_idx = motion_utils::findFirstNearestIndexWithSoftConstraints(
    inout_path->points, in_pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
  // vector.insert(i) inserts element on the left side of v[i]
  // the velocity need to be zero order hold(from prior point)
  int insert_idx = closest_idx;
  autoware_auto_planning_msgs::msg::PathPointWithLaneId inserted_point =
    inout_path->points.at(closest_idx);
  if (planning_utils::isAheadOf(in_pose, inout_path->points.at(closest_idx).point.pose)) {
    ++insert_idx;
  } else {
    // copy with velocity from prior point
    const size_t prior_ind = closest_idx > 0 ? closest_idx - 1 : 0;
    inserted_point.point.longitudinal_velocity_mps =
      inout_path->points.at(prior_ind).point.longitudinal_velocity_mps;
  }
  inserted_point.point.pose = in_pose;

  auto it = inout_path->points.begin() + insert_idx;
  inout_path->points.insert(it, inserted_point);

  return insert_idx;
}

bool hasLaneIds(
  const autoware_auto_planning_msgs::msg::PathPointWithLaneId & p,
  const std::set<lanelet::Id> & ids)
{
  for (const auto & pid : p.lane_ids) {
    if (ids.find(pid) != ids.end()) {
      return true;
    }
  }
  return false;
}

std::optional<std::pair<size_t, size_t>> findLaneIdsInterval(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & p, const std::set<lanelet::Id> & ids)
{
  bool found = false;
  size_t start = 0;
  size_t end = p.points.size() > 0 ? p.points.size() - 1 : 0;
  if (start == end) {
    // there is only one point in the path
    return std::nullopt;
  }
  for (size_t i = 0; i < p.points.size(); ++i) {
    if (hasLaneIds(p.points.at(i), ids)) {
      if (!found) {
        // found interval for the first time
        found = true;
        start = i;
      }
    } else if (found) {
      // prior point was in the interval. interval ended
      end = i;
      break;
    }
  }
  start = start > 0 ? start - 1 : 0;  // the idx of last point before the interval
  return found ? std::make_optional(std::make_pair(start, end)) : std::nullopt;
}

/**
 * @brief Get stop point from map if exists
 * @param stop_pose stop point defined on map
 * @return true when the stop point is defined on map.
 */
static std::optional<size_t> getStopLineIndexFromMap(
  const InterpolatedPathInfo & interpolated_path_info,
  const std::shared_ptr<const PlannerData> & planner_data)
{
  const auto & path = interpolated_path_info.path;
  const auto & lane_interval = interpolated_path_info.lane_id_interval.value();

  lanelet::ConstLanelet lanelet =
    planner_data->route_handler_->getLaneletMapPtr()->laneletLayer.get(
      interpolated_path_info.lane_id);
  const auto road_markings = lanelet.regulatoryElementsAs<lanelet::autoware::RoadMarking>();
  lanelet::ConstLineStrings3d stopline;
  for (const auto & road_marking : road_markings) {
    const std::string type =
      road_marking->roadMarking().attributeOr(lanelet::AttributeName::Type, "none");
    if (type == lanelet::AttributeValueString::StopLine) {
      stopline.push_back(road_marking->roadMarking());
      break;  // only one stopline exists.
    }
  }
  if (stopline.empty()) {
    return std::nullopt;
  }

  const auto p_start = stopline.front().front();
  const auto p_end = stopline.front().back();
  const LineString2d extended_stopline =
    planning_utils::extendLine(p_start, p_end, planner_data->stop_line_extend_length);

  for (size_t i = lane_interval.first; i < lane_interval.second; i++) {
    const auto & p_front = path.points.at(i).point.pose.position;
    const auto & p_back = path.points.at(i + 1).point.pose.position;

    const LineString2d path_segment = {{p_front.x, p_front.y}, {p_back.x, p_back.y}};
    std::vector<Point2d> collision_points;
    bg::intersection(extended_stopline, path_segment, collision_points);

    if (collision_points.empty()) {
      continue;
    }

    return i;
  }

  geometry_msgs::msg::Pose stop_point_from_map;
  stop_point_from_map.position.x = 0.5 * (p_start.x() + p_end.x());
  stop_point_from_map.position.y = 0.5 * (p_start.y() + p_end.y());
  stop_point_from_map.position.z = 0.5 * (p_start.z() + p_end.z());

  return motion_utils::findFirstNearestIndexWithSoftConstraints(
    path.points, stop_point_from_map, planner_data->ego_nearest_dist_threshold,
    planner_data->ego_nearest_yaw_threshold);
}

static std::optional<size_t> getFirstPointInsidePolygonByFootprint(
  const lanelet::CompoundPolygon3d & polygon, const InterpolatedPathInfo & interpolated_path_info,
  const tier4_autoware_utils::LinearRing2d & footprint, const double vehicle_length)
{
  const auto & path_ip = interpolated_path_info.path;
  const auto [lane_start, lane_end] = interpolated_path_info.lane_id_interval.value();
  const size_t vehicle_length_idx = static_cast<size_t>(vehicle_length / interpolated_path_info.ds);
  const size_t start =
    static_cast<size_t>(std::max<int>(0, static_cast<int>(lane_start) - vehicle_length_idx));
  const auto area_2d = lanelet::utils::to2D(polygon).basicPolygon();
  for (auto i = start; i <= lane_end; ++i) {
    const auto & base_pose = path_ip.points.at(i).point.pose;
    const auto path_footprint = tier4_autoware_utils::transformVector(
      footprint, tier4_autoware_utils::pose2transform(base_pose));
    if (bg::intersects(path_footprint, area_2d)) {
      return std::make_optional<size_t>(i);
    }
  }
  return std::nullopt;
}

static std::optional<std::pair<
  size_t /* the index of interpolated PathPoint*/, size_t /* the index of corresponding Polygon */>>
getFirstPointInsidePolygonsByFootprint(
  const std::vector<lanelet::CompoundPolygon3d> & polygons,
  const InterpolatedPathInfo & interpolated_path_info,
  const tier4_autoware_utils::LinearRing2d & footprint, const double vehicle_length)
{
  const auto & path_ip = interpolated_path_info.path;
  const auto [lane_start, lane_end] = interpolated_path_info.lane_id_interval.value();
  const size_t vehicle_length_idx = static_cast<size_t>(vehicle_length / interpolated_path_info.ds);
  const size_t start =
    static_cast<size_t>(std::max<int>(0, static_cast<int>(lane_start) - vehicle_length_idx));

  for (size_t i = start; i <= lane_end; ++i) {
    const auto & pose = path_ip.points.at(i).point.pose;
    const auto path_footprint =
      tier4_autoware_utils::transformVector(footprint, tier4_autoware_utils::pose2transform(pose));
    for (size_t j = 0; j < polygons.size(); ++j) {
      const auto area_2d = lanelet::utils::to2D(polygons.at(j)).basicPolygon();
      const bool is_in_polygon = bg::intersects(area_2d, path_footprint);
      if (is_in_polygon) {
        return std::make_optional<std::pair<size_t, size_t>>(i, j);
      }
    }
  }
  return std::nullopt;
}

std::optional<IntersectionStopLines> generateIntersectionStopLines(
  const lanelet::CompoundPolygon3d & first_conflicting_area,
  const lanelet::CompoundPolygon3d & first_attention_area,
  const lanelet::ConstLineString2d & first_attention_lane_centerline,
  const std::shared_ptr<const PlannerData> & planner_data,
  const InterpolatedPathInfo & interpolated_path_info, const bool use_stuck_stopline,
  const double stopline_margin, const double max_accel, const double max_jerk,
  const double delay_response_time, const double peeking_offset,
  autoware_auto_planning_msgs::msg::PathWithLaneId * original_path)
{
  const auto & path_ip = interpolated_path_info.path;
  const double ds = interpolated_path_info.ds;
  const auto & lane_interval_ip = interpolated_path_info.lane_id_interval.value();
  const double baselink2front = planner_data->vehicle_info_.max_longitudinal_offset_m;

  const int stopline_margin_idx_dist = std::ceil(stopline_margin / ds);
  const int base2front_idx_dist =
    std::ceil(planner_data->vehicle_info_.max_longitudinal_offset_m / ds);

  // find the index of the first point whose vehicle footprint on it intersects with detection_area
  const auto local_footprint = planner_data->vehicle_info_.createFootprint(0.0, 0.0);
  std::optional<size_t> first_footprint_inside_detection_ip_opt =
    getFirstPointInsidePolygonByFootprint(
      first_attention_area, interpolated_path_info, local_footprint, baselink2front);
  if (!first_footprint_inside_detection_ip_opt) {
    return std::nullopt;
  }
  const auto first_footprint_inside_detection_ip = first_footprint_inside_detection_ip_opt.value();

  std::optional<size_t> first_footprint_attention_centerline_ip_opt = std::nullopt;
  for (auto i = std::get<0>(lane_interval_ip); i < std::get<1>(lane_interval_ip); ++i) {
    const auto & base_pose = path_ip.points.at(i).point.pose;
    const auto path_footprint = tier4_autoware_utils::transformVector(
      local_footprint, tier4_autoware_utils::pose2transform(base_pose));
    if (bg::intersects(path_footprint, first_attention_lane_centerline.basicLineString())) {
      // NOTE: maybe consideration of braking dist is necessary
      first_footprint_attention_centerline_ip_opt = i;
      break;
    }
  }
  if (!first_footprint_attention_centerline_ip_opt) {
    return std::nullopt;
  }
  const size_t first_footprint_attention_centerline_ip =
    first_footprint_attention_centerline_ip_opt.value();

  // (1) default stop line position on interpolated path
  bool default_stopline_valid = true;
  int stop_idx_ip_int = -1;
  if (const auto map_stop_idx_ip = getStopLineIndexFromMap(interpolated_path_info, planner_data);
      map_stop_idx_ip) {
    stop_idx_ip_int = static_cast<int>(map_stop_idx_ip.value()) - base2front_idx_dist;
  }
  if (stop_idx_ip_int < 0) {
    stop_idx_ip_int = first_footprint_inside_detection_ip - stopline_margin_idx_dist;
  }
  if (stop_idx_ip_int < 0) {
    default_stopline_valid = false;
  }
  const auto default_stopline_ip = stop_idx_ip_int >= 0 ? static_cast<size_t>(stop_idx_ip_int) : 0;

  // (2) ego front stop line position on interpolated path
  const geometry_msgs::msg::Pose & current_pose = planner_data->current_odometry->pose;
  const auto closest_idx_ip = motion_utils::findFirstNearestIndexWithSoftConstraints(
    path_ip.points, current_pose, planner_data->ego_nearest_dist_threshold,
    planner_data->ego_nearest_yaw_threshold);

  // (3) occlusion peeking stop line position on interpolated path
  int occlusion_peeking_line_ip_int = static_cast<int>(default_stopline_ip);
  bool occlusion_peeking_line_valid = true;
  // NOTE: if footprints[0] is already inside the detection area, invalid
  {
    const auto & base_pose0 = path_ip.points.at(default_stopline_ip).point.pose;
    const auto path_footprint0 = tier4_autoware_utils::transformVector(
      local_footprint, tier4_autoware_utils::pose2transform(base_pose0));
    if (bg::intersects(
          path_footprint0, lanelet::utils::to2D(first_attention_area).basicPolygon())) {
      occlusion_peeking_line_valid = false;
    }
  }
  if (occlusion_peeking_line_valid) {
    occlusion_peeking_line_ip_int =
      first_footprint_inside_detection_ip + std::ceil(peeking_offset / ds);
  }

  const auto occlusion_peeking_line_ip = static_cast<size_t>(
    std::clamp<int>(occlusion_peeking_line_ip_int, 0, static_cast<int>(path_ip.points.size()) - 1));
  const auto first_attention_stopline_ip = first_footprint_inside_detection_ip;
  const bool first_attention_stopline_valid = true;

  // (4) pass judge line position on interpolated path
  const double velocity = planner_data->current_velocity->twist.linear.x;
  const double acceleration = planner_data->current_acceleration->accel.accel.linear.x;
  const double braking_dist = planning_utils::calcJudgeLineDistWithJerkLimit(
    velocity, acceleration, max_accel, max_jerk, delay_response_time);
  int pass_judge_ip_int =
    static_cast<int>(first_footprint_inside_detection_ip) - std::ceil(braking_dist / ds);
  const auto pass_judge_line_ip = static_cast<size_t>(
    std::clamp<int>(pass_judge_ip_int, 0, static_cast<int>(path_ip.points.size()) - 1));
  // TODO(Mamoru Sobue): maybe braking dist should be considered
  const auto occlusion_wo_tl_pass_judge_line_ip =
    static_cast<size_t>(first_footprint_attention_centerline_ip);

  // (5) stuck vehicle stop line
  int stuck_stopline_ip_int = 0;
  bool stuck_stopline_valid = true;
  if (use_stuck_stopline) {
    // NOTE: when ego vehicle is approaching detection area and already passed
    // first_conflicting_area, this could be null.
    const auto stuck_stopline_idx_ip_opt = getFirstPointInsidePolygonByFootprint(
      first_conflicting_area, interpolated_path_info, local_footprint, baselink2front);
    if (!stuck_stopline_idx_ip_opt) {
      stuck_stopline_valid = false;
      stuck_stopline_ip_int = 0;
    } else {
      stuck_stopline_ip_int = stuck_stopline_idx_ip_opt.value() - stopline_margin_idx_dist;
    }
  } else {
    stuck_stopline_ip_int =
      std::get<0>(lane_interval_ip) - (stopline_margin_idx_dist + base2front_idx_dist);
  }
  if (stuck_stopline_ip_int < 0) {
    stuck_stopline_valid = false;
  }
  const auto stuck_stopline_ip = static_cast<size_t>(std::max(0, stuck_stopline_ip_int));

  struct IntersectionStopLinesTemp
  {
    size_t closest_idx{0};
    size_t stuck_stopline{0};
    size_t default_stopline{0};
    size_t first_attention_stopline{0};
    size_t occlusion_peeking_stopline{0};
    size_t pass_judge_line{0};
    size_t occlusion_wo_tl_pass_judge_line{0};
  };

  IntersectionStopLinesTemp intersection_stoplines_temp;
  std::list<std::pair<const size_t *, size_t *>> stoplines = {
    {&closest_idx_ip, &intersection_stoplines_temp.closest_idx},
    {&stuck_stopline_ip, &intersection_stoplines_temp.stuck_stopline},
    {&default_stopline_ip, &intersection_stoplines_temp.default_stopline},
    {&first_attention_stopline_ip, &intersection_stoplines_temp.first_attention_stopline},
    {&occlusion_peeking_line_ip, &intersection_stoplines_temp.occlusion_peeking_stopline},
    {&pass_judge_line_ip, &intersection_stoplines_temp.pass_judge_line},
    {&occlusion_wo_tl_pass_judge_line_ip,
     &intersection_stoplines_temp.occlusion_wo_tl_pass_judge_line}};
  stoplines.sort(
    [](const auto & it1, const auto & it2) { return *(std::get<0>(it1)) < *(std::get<0>(it2)); });
  for (const auto & [stop_idx_ip, stop_idx] : stoplines) {
    const auto & insert_point = path_ip.points.at(*stop_idx_ip).point.pose;
    const auto insert_idx = insertPointIndex(
      insert_point, original_path, planner_data->ego_nearest_dist_threshold,
      planner_data->ego_nearest_yaw_threshold);
    if (!insert_idx) {
      return std::nullopt;
    }
    *stop_idx = insert_idx.value();
  }
  if (
    intersection_stoplines_temp.occlusion_peeking_stopline <
    intersection_stoplines_temp.default_stopline) {
    intersection_stoplines_temp.occlusion_peeking_stopline =
      intersection_stoplines_temp.default_stopline;
  }

  IntersectionStopLines intersection_stoplines;
  intersection_stoplines.closest_idx = intersection_stoplines_temp.closest_idx;
  if (stuck_stopline_valid) {
    intersection_stoplines.stuck_stopline = intersection_stoplines_temp.stuck_stopline;
  }
  if (default_stopline_valid) {
    intersection_stoplines.default_stopline = intersection_stoplines_temp.default_stopline;
  }
  if (first_attention_stopline_valid) {
    intersection_stoplines.first_attention_stopline =
      intersection_stoplines_temp.first_attention_stopline;
  }
  if (occlusion_peeking_line_valid) {
    intersection_stoplines.occlusion_peeking_stopline =
      intersection_stoplines_temp.occlusion_peeking_stopline;
  }
  intersection_stoplines.pass_judge_line = intersection_stoplines_temp.pass_judge_line;
  intersection_stoplines.occlusion_wo_tl_pass_judge_line =
    intersection_stoplines_temp.occlusion_wo_tl_pass_judge_line;
  return intersection_stoplines;
}

std::optional<size_t> getFirstPointInsidePolygon(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const std::pair<size_t, size_t> lane_interval, const lanelet::CompoundPolygon3d & polygon,
  const bool search_forward)
{
  // NOTE: if first point is already inside the polygon, returns nullopt
  const auto polygon_2d = lanelet::utils::to2D(polygon);
  if (search_forward) {
    const auto & p0 = path.points.at(lane_interval.first).point.pose.position;
    if (bg::within(to_bg2d(p0), polygon_2d)) {
      return std::nullopt;
    }
    for (size_t i = lane_interval.first; i <= lane_interval.second; ++i) {
      const auto & p = path.points.at(i).point.pose.position;
      const auto is_in_lanelet = bg::within(to_bg2d(p), polygon_2d);
      if (is_in_lanelet) {
        return std::make_optional<size_t>(i);
      }
    }
  } else {
    const auto & p0 = path.points.at(lane_interval.second).point.pose.position;
    if (bg::within(to_bg2d(p0), polygon_2d)) {
      return std::nullopt;
    }
    for (size_t i = lane_interval.second; i >= lane_interval.first; --i) {
      const auto & p = path.points.at(i).point.pose.position;
      const auto is_in_lanelet = bg::within(to_bg2d(p), polygon_2d);
      if (is_in_lanelet) {
        return std::make_optional<size_t>(i);
      }
      if (i == 0) {
        break;
      }
    }
  }
  return std::nullopt;
}

static std::optional<std::pair<size_t, const lanelet::CompoundPolygon3d &>>
getFirstPointInsidePolygons(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const std::pair<size_t, size_t> lane_interval,
  const std::vector<lanelet::CompoundPolygon3d> & polygons, const bool search_forward = true)
{
  if (search_forward) {
    for (size_t i = lane_interval.first; i <= lane_interval.second; ++i) {
      bool is_in_lanelet = false;
      const auto & p = path.points.at(i).point.pose.position;
      for (const auto & polygon : polygons) {
        const auto polygon_2d = lanelet::utils::to2D(polygon);
        is_in_lanelet = bg::within(to_bg2d(p), polygon_2d);
        if (is_in_lanelet) {
          return std::make_optional<std::pair<size_t, const lanelet::CompoundPolygon3d &>>(
            i, polygon);
        }
      }
      if (is_in_lanelet) {
        break;
      }
    }
  } else {
    for (size_t i = lane_interval.second; i >= lane_interval.first; --i) {
      bool is_in_lanelet = false;
      const auto & p = path.points.at(i).point.pose.position;
      for (const auto & polygon : polygons) {
        const auto polygon_2d = lanelet::utils::to2D(polygon);
        is_in_lanelet = bg::within(to_bg2d(p), polygon_2d);
        if (is_in_lanelet) {
          return std::make_optional<std::pair<size_t, const lanelet::CompoundPolygon3d &>>(
            i, polygon);
        }
      }
      if (is_in_lanelet) {
        break;
      }
      if (i == 0) {
        break;
      }
    }
  }
  return std::nullopt;
}

std::optional<size_t> generateStuckStopLine(
  const lanelet::CompoundPolygon3d & conflicting_area,
  const std::shared_ptr<const PlannerData> & planner_data,
  const InterpolatedPathInfo & interpolated_path_info, const double stopline_margin,
  const bool use_stuck_stopline, autoware_auto_planning_msgs::msg::PathWithLaneId * original_path)
{
  const auto & path_ip = interpolated_path_info.path;
  const double ds = interpolated_path_info.ds;
  const auto & lane_interval_ip = interpolated_path_info.lane_id_interval.value();
  const auto lane_interval_ip_start = std::get<0>(lane_interval_ip);
  size_t stuck_stopline_idx_ip = 0;
  if (use_stuck_stopline) {
    stuck_stopline_idx_ip = lane_interval_ip_start;
  } else {
    const auto stuck_stopline_idx_ip_opt =
      getFirstPointInsidePolygon(path_ip, lane_interval_ip, conflicting_area);
    if (!stuck_stopline_idx_ip_opt) {
      return std::nullopt;
    }
    stuck_stopline_idx_ip = stuck_stopline_idx_ip_opt.value();
  }

  const int stopline_margin_idx_dist = std::ceil(stopline_margin / ds);
  const int base2front_idx_dist =
    std::ceil(planner_data->vehicle_info_.max_longitudinal_offset_m / ds);
  const size_t insert_idx_ip = static_cast<size_t>(std::max(
    static_cast<int>(stuck_stopline_idx_ip) - 1 - stopline_margin_idx_dist - base2front_idx_dist,
    0));
  const auto & insert_point = path_ip.points.at(insert_idx_ip).point.pose;
  return insertPointIndex(
    insert_point, original_path, planner_data->ego_nearest_dist_threshold,
    planner_data->ego_nearest_yaw_threshold);
}

static std::vector<lanelet::CompoundPolygon3d> getPolygon3dFromLanelets(
  const lanelet::ConstLanelets & ll_vec)
{
  std::vector<lanelet::CompoundPolygon3d> polys;
  for (auto && ll : ll_vec) {
    polys.push_back(ll.polygon3d());
  }
  return polys;
}

static std::string getTurnDirection(lanelet::ConstLanelet lane)
{
  return lane.attributeOr("turn_direction", "else");
}

/**
 * @param pair lanelets and the vector of original lanelets in topological order (not reversed as
 *in generateDetectionLaneDivisions())
 **/
static std::pair<lanelet::ConstLanelets, std::vector<lanelet::ConstLanelets>>
mergeLaneletsByTopologicalSort(
  const lanelet::ConstLanelets & lanelets,
  const lanelet::routing::RoutingGraphPtr routing_graph_ptr)
{
  const int n_node = lanelets.size();
  std::vector<std::vector<int>> adjacency(n_node);
  for (int dst = 0; dst < n_node; ++dst) {
    adjacency[dst].resize(n_node);
    for (int src = 0; src < n_node; ++src) {
      adjacency[dst][src] = false;
    }
  }
  std::set<lanelet::Id> lanelet_ids;
  std::unordered_map<lanelet::Id, int> id2ind;
  std::unordered_map<int, lanelet::Id> ind2id;
  std::unordered_map<lanelet::Id, lanelet::ConstLanelet> id2lanelet;
  int ind = 0;
  for (const auto & lanelet : lanelets) {
    lanelet_ids.insert(lanelet.id());
    const auto id = lanelet.id();
    id2ind[id] = ind;
    ind2id[ind] = id;
    id2lanelet[id] = lanelet;
    ind++;
  }
  // NOTE: this function aims to traverse the detection lanelet backward from ego side to farthest
  // side, so if lane B follows lane A on the routing_graph, adj[B][A] = true
  for (const auto & lanelet : lanelets) {
    const auto & followings = routing_graph_ptr->following(lanelet);
    const int dst = lanelet.id();
    for (const auto & following : followings) {
      if (const int src = following.id(); lanelet_ids.find(src) != lanelet_ids.end()) {
        adjacency[(id2ind[src])][(id2ind[dst])] = true;
      }
    }
  }
  // terminal node
  std::map<lanelet::Id, std::vector<lanelet::Id>> branches;
  auto has_no_previous = [&](const int node) {
    for (int dst = 0; dst < n_node; dst++) {
      if (adjacency[dst][node]) {
        return false;
      }
    }
    return true;
  };
  for (int src = 0; src < n_node; src++) {
    if (!has_no_previous(src)) {
      continue;
    }
    // So `src` has no previous lanelets
    branches[(ind2id[src])] = std::vector<lanelet::Id>{};
    auto & branch = branches[(ind2id[src])];
    lanelet::Id node_iter = ind2id[src];
    std::set<lanelet::Id> visited_ids;
    while (true) {
      const auto & destinations = adjacency[(id2ind[node_iter])];
      // NOTE: assuming detection lanelets have only one "previous"(on the routing_graph) lanelet
      const auto next = std::find(destinations.begin(), destinations.end(), true);
      if (next == destinations.end()) {
        branch.push_back(node_iter);
        break;
      }
      if (visited_ids.find(node_iter) != visited_ids.end()) {
        // loop detected
        break;
      }
      branch.push_back(node_iter);
      visited_ids.insert(node_iter);
      node_iter = ind2id[std::distance(destinations.begin(), next)];
    }
  }
  for (decltype(branches)::iterator it = branches.begin(); it != branches.end(); it++) {
    auto & branch = it->second;
    std::reverse(branch.begin(), branch.end());
  }
  lanelet::ConstLanelets merged;
  std::vector<lanelet::ConstLanelets> originals;
  for (const auto & [id, sub_ids] : branches) {
    if (sub_ids.size() == 0) {
      continue;
    }
    lanelet::ConstLanelets merge;
    originals.push_back(lanelet::ConstLanelets({}));
    auto & original = originals.back();
    for (const auto sub_id : sub_ids) {
      merge.push_back(id2lanelet[sub_id]);
      original.push_back(id2lanelet[sub_id]);
    }
    merged.push_back(lanelet::utils::combineLaneletsShape(merge));
  }
  return {merged, originals};
}

IntersectionLanelets getObjectiveLanelets(
  lanelet::LaneletMapConstPtr lanelet_map_ptr, lanelet::routing::RoutingGraphPtr routing_graph_ptr,
  const lanelet::ConstLanelet assigned_lanelet, const lanelet::ConstLanelets & lanelets_on_path,
  const std::set<lanelet::Id> & associative_ids, const double detection_area_length,
  const double occlusion_detection_area_length, const bool consider_wrong_direction_vehicle)
{
  const auto turn_direction = assigned_lanelet.attributeOr("turn_direction", "else");

  // retrieve a stopline associated with a traffic light
  bool has_traffic_light = false;
  if (const auto tl_reg_elems = assigned_lanelet.regulatoryElementsAs<lanelet::TrafficLight>();
      tl_reg_elems.size() != 0) {
    const auto tl_reg_elem = tl_reg_elems.front();
    const auto stopline_opt = tl_reg_elem->stopLine();
    if (!!stopline_opt) has_traffic_light = true;
  }

  // for low priority lane
  // If ego_lane has right of way (i.e. is high priority),
  // ignore yieldLanelets (i.e. low priority lanes)
  lanelet::ConstLanelets yield_lanelets{};
  const auto right_of_ways = assigned_lanelet.regulatoryElementsAs<lanelet::RightOfWay>();
  for (const auto & right_of_way : right_of_ways) {
    if (lanelet::utils::contains(right_of_way->rightOfWayLanelets(), assigned_lanelet)) {
      for (const auto & yield_lanelet : right_of_way->yieldLanelets()) {
        yield_lanelets.push_back(yield_lanelet);
        for (const auto & previous_lanelet : routing_graph_ptr->previous(yield_lanelet)) {
          yield_lanelets.push_back(previous_lanelet);
        }
      }
    }
  }

  // get all following lanes of previous lane
  lanelet::ConstLanelets ego_lanelets = lanelets_on_path;
  for (const auto & previous_lanelet : routing_graph_ptr->previous(assigned_lanelet)) {
    ego_lanelets.push_back(previous_lanelet);
    for (const auto & following_lanelet : routing_graph_ptr->following(previous_lanelet)) {
      if (lanelet::utils::contains(ego_lanelets, following_lanelet)) {
        continue;
      }
      ego_lanelets.push_back(following_lanelet);
    }
  }

  // get conflicting lanes on assigned lanelet
  const auto & conflicting_lanelets =
    lanelet::utils::getConflictingLanelets(routing_graph_ptr, assigned_lanelet);
  std::vector<lanelet::ConstLanelet> adjacent_followings;

  for (const auto & conflicting_lanelet : conflicting_lanelets) {
    for (const auto & following_lanelet : routing_graph_ptr->following(conflicting_lanelet)) {
      adjacent_followings.push_back(following_lanelet);
    }
    for (const auto & following_lanelet : routing_graph_ptr->previous(conflicting_lanelet)) {
      adjacent_followings.push_back(following_lanelet);
    }
  }

  // final objective lanelets
  lanelet::ConstLanelets detection_lanelets;
  lanelet::ConstLanelets conflicting_ex_ego_lanelets;
  // conflicting lanes is necessary to get stopline for stuck vehicle
  for (auto && conflicting_lanelet : conflicting_lanelets) {
    if (!lanelet::utils::contains(ego_lanelets, conflicting_lanelet))
      conflicting_ex_ego_lanelets.push_back(conflicting_lanelet);
  }

  // exclude yield lanelets and ego lanelets from detection_lanelets
  if (turn_direction == std::string("straight") && has_traffic_light) {
    // if assigned lanelet is "straight" with traffic light, detection area is not necessary
  } else {
    if (consider_wrong_direction_vehicle) {
      for (const auto & conflicting_lanelet : conflicting_lanelets) {
        if (lanelet::utils::contains(yield_lanelets, conflicting_lanelet)) {
          continue;
        }
        detection_lanelets.push_back(conflicting_lanelet);
      }
      for (const auto & adjacent_following : adjacent_followings) {
        detection_lanelets.push_back(adjacent_following);
      }
    } else {
      // otherwise we need to know the priority from RightOfWay
      for (const auto & conflicting_lanelet : conflicting_lanelets) {
        if (
          lanelet::utils::contains(yield_lanelets, conflicting_lanelet) ||
          lanelet::utils::contains(ego_lanelets, conflicting_lanelet)) {
          continue;
        }
        detection_lanelets.push_back(conflicting_lanelet);
      }
    }
  }

  // get possible lanelet path that reaches conflicting_lane longer than given length
  lanelet::ConstLanelets detection_and_preceding_lanelets;
  {
    const double length = detection_area_length;
    std::set<lanelet::Id> detection_ids;
    for (const auto & ll : detection_lanelets) {
      // Preceding lanes does not include detection_lane so add them at the end
      const auto & inserted = detection_ids.insert(ll.id());
      if (inserted.second) detection_and_preceding_lanelets.push_back(ll);
      // get preceding lanelets without ego_lanelets
      // to prevent the detection area from including the ego lanes and its' preceding lanes.
      const auto lanelet_sequences = lanelet::utils::query::getPrecedingLaneletSequences(
        routing_graph_ptr, ll, length, ego_lanelets);
      for (const auto & ls : lanelet_sequences) {
        for (const auto & l : ls) {
          const auto & inserted = detection_ids.insert(l.id());
          if (inserted.second) detection_and_preceding_lanelets.push_back(l);
        }
      }
    }
  }

  lanelet::ConstLanelets occlusion_detection_and_preceding_lanelets;
  {
    const double length = occlusion_detection_area_length;
    std::set<lanelet::Id> detection_ids;
    for (const auto & ll : detection_lanelets) {
      // Preceding lanes does not include detection_lane so add them at the end
      const auto & inserted = detection_ids.insert(ll.id());
      if (inserted.second) occlusion_detection_and_preceding_lanelets.push_back(ll);
      // get preceding lanelets without ego_lanelets
      // to prevent the detection area from including the ego lanes and its' preceding lanes.
      const auto lanelet_sequences = lanelet::utils::query::getPrecedingLaneletSequences(
        routing_graph_ptr, ll, length, ego_lanelets);
      for (const auto & ls : lanelet_sequences) {
        for (const auto & l : ls) {
          const auto & inserted = detection_ids.insert(l.id());
          if (inserted.second) occlusion_detection_and_preceding_lanelets.push_back(l);
        }
      }
    }
  }
  lanelet::ConstLanelets occlusion_detection_and_preceding_lanelets_wo_turn_direction;
  for (const auto & ll : occlusion_detection_and_preceding_lanelets) {
    const auto turn_direction = getTurnDirection(ll);
    if (turn_direction == "left" || turn_direction == "right") {
      continue;
    }
    occlusion_detection_and_preceding_lanelets_wo_turn_direction.push_back(ll);
  }

  auto [attention_lanelets, original_attention_lanelet_sequences] =
    mergeLaneletsByTopologicalSort(detection_and_preceding_lanelets, routing_graph_ptr);

  IntersectionLanelets result;
  result.attention_ = std::move(attention_lanelets);
  for (const auto & original_attention_lanelet_seq : original_attention_lanelet_sequences) {
    // NOTE: in mergeLaneletsByTopologicalSort(), sub_ids are empty checked, so it is ensured that
    // back() exists.
    std::optional<lanelet::ConstLineString3d> stopline{std::nullopt};
    for (auto it = original_attention_lanelet_seq.rbegin();
         it != original_attention_lanelet_seq.rend(); ++it) {
      const auto traffic_lights = it->regulatoryElementsAs<lanelet::TrafficLight>();
      for (const auto & traffic_light : traffic_lights) {
        const auto stopline_opt = traffic_light->stopLine();
        if (!stopline_opt) continue;
        stopline = stopline_opt.get();
        break;
      }
      if (stopline) break;
    }
    result.attention_stoplines_.push_back(stopline);
  }
  result.attention_non_preceding_ = std::move(detection_lanelets);
  for (unsigned i = 0; i < result.attention_non_preceding_.size(); ++i) {
    std::optional<lanelet::ConstLineString3d> stopline = std::nullopt;
    const auto & ll = result.attention_non_preceding_.at(i);
    const auto traffic_lights = ll.regulatoryElementsAs<lanelet::TrafficLight>();
    for (const auto & traffic_light : traffic_lights) {
      const auto stopline_opt = traffic_light->stopLine();
      if (!stopline_opt) continue;
      stopline = stopline_opt.get();
    }
    result.attention_non_preceding_stoplines_.push_back(stopline);
  }
  result.conflicting_ = std::move(conflicting_ex_ego_lanelets);
  result.adjacent_ = planning_utils::getConstLaneletsFromIds(lanelet_map_ptr, associative_ids);
  // NOTE: occlusion_attention is not inverted here
  // TODO(Mamoru Sobue): apply mergeLaneletsByTopologicalSort for occlusion lanelets as well and
  // then trim part of them based on curvature threshold
  result.occlusion_attention_ =
    std::move(occlusion_detection_and_preceding_lanelets_wo_turn_direction);

  // NOTE: to properly update(), each element in conflicting_/conflicting_area_,
  // attention_non_preceding_/attention_non_preceding_area_ need to be matched
  result.attention_area_ = getPolygon3dFromLanelets(result.attention_);
  result.attention_non_preceding_area_ = getPolygon3dFromLanelets(result.attention_non_preceding_);
  result.conflicting_area_ = getPolygon3dFromLanelets(result.conflicting_);
  result.adjacent_area_ = getPolygon3dFromLanelets(result.adjacent_);
  result.occlusion_attention_area_ = getPolygon3dFromLanelets(result.occlusion_attention_);
  return result;
}

bool isOverTargetIndex(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const int closest_idx,
  const geometry_msgs::msg::Pose & current_pose, const int target_idx)
{
  if (closest_idx == target_idx) {
    const geometry_msgs::msg::Pose target_pose = path.points.at(target_idx).point.pose;
    return planning_utils::isAheadOf(current_pose, target_pose);
  }
  return static_cast<bool>(closest_idx > target_idx);
}

bool isBeforeTargetIndex(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const int closest_idx,
  const geometry_msgs::msg::Pose & current_pose, const int target_idx)
{
  if (closest_idx == target_idx) {
    const geometry_msgs::msg::Pose target_pose = path.points.at(target_idx).point.pose;
    return planning_utils::isAheadOf(target_pose, current_pose);
  }
  return static_cast<bool>(target_idx > closest_idx);
}

/*
static std::vector<int> getAllAdjacentLanelets(
  const lanelet::routing::RoutingGraphPtr routing_graph, lanelet::ConstLanelet lane)
{
  std::set<int> results;

  results.insert(lane.id());

  auto it = routing_graph->adjacentRight(lane);
  // take all lane on the right side
  while (!!it) {
    results.insert(it.get().id());
    it = routing_graph->adjacentRight(it.get());
  }
  // take all lane on the left side
  it = routing_graph->adjacentLeft(lane);
  while (!!it) {
    results.insert(it.get().id());
    it = routing_graph->adjacentLeft(it.get());

  }
  return std::vector<int>(results.begin(), results.end());
}
*/

/*
lanelet::ConstLanelets extendedAdjacentDirectionLanes(
  lanelet::LaneletMapConstPtr map, const lanelet::routing::RoutingGraphPtr routing_graph,
  lanelet::ConstLanelet lane)
{
  // some of the intersections are not well-formed, and "adjacent" turning
  // lanelets are not sharing the LineStrings
  const std::string turn_direction = getTurnDirection(lane);
  if (turn_direction != "left" && turn_direction != "right" && turn_direction != "straight")
    return {};

  std::set<int> previous_lanelet_ids;
  for (auto && previous_lanelet : routing_graph->previous(lane)) {
    previous_lanelet_ids.insert(previous_lanelet.id());
  }

  std::set<int> besides_previous_lanelet_ids;
  for (auto && previous_lanelet_id : previous_lanelet_ids) {
    lanelet::ConstLanelet previous_lanelet = map->laneletLayer.get(previous_lanelet_id);
    for (auto && beside_lanelet : getAllAdjacentLanelets(routing_graph, previous_lanelet)) {
      besides_previous_lanelet_ids.insert(beside_lanelet);
    }
  }

  std::set<int> following_turning_lanelets;
  following_turning_lanelets.insert(lane.id());
  for (auto && besides_previous_lanelet_id : besides_previous_lanelet_ids) {
    lanelet::ConstLanelet besides_previous_lanelet =
      map->laneletLayer.get(besides_previous_lanelet_id);
    for (auto && following_lanelet : routing_graph->following(besides_previous_lanelet)) {
      // if this has {"turn_direction", "${turn_direction}"}, take this
      if (getTurnDirection(following_lanelet) == turn_direction)
        following_turning_lanelets.insert(following_lanelet.id());
    }
  }
  lanelet::ConstLanelets ret{};
  for (auto && id : following_turning_lanelets) {
    ret.push_back(map->laneletLayer.get(id));
  }
  return ret;
}
*/

std::optional<Polygon2d> getIntersectionArea(
  lanelet::ConstLanelet assigned_lane, lanelet::LaneletMapConstPtr lanelet_map_ptr)
{
  const std::string area_id_str = assigned_lane.attributeOr("intersection_area", "else");
  if (area_id_str == "else") return std::nullopt;

  const int area_id = std::atoi(area_id_str.c_str());
  const auto poly_3d = lanelet_map_ptr->polygonLayer.get(area_id);
  Polygon2d poly{};
  for (const auto & p : poly_3d) poly.outer().emplace_back(p.x(), p.y());
  return std::make_optional(poly);
}

bool hasAssociatedTrafficLight(lanelet::ConstLanelet lane)
{
  std::optional<int> tl_id = std::nullopt;
  for (auto && tl_reg_elem : lane.regulatoryElementsAs<lanelet::TrafficLight>()) {
    tl_id = tl_reg_elem->id();
    break;
  }
  return tl_id.has_value();
}

TrafficPrioritizedLevel getTrafficPrioritizedLevel(
  lanelet::ConstLanelet lane, const std::map<int, TrafficSignalStamped> & tl_infos)
{
  using TrafficSignalElement = autoware_perception_msgs::msg::TrafficSignalElement;

  std::optional<int> tl_id = std::nullopt;
  for (auto && tl_reg_elem : lane.regulatoryElementsAs<lanelet::TrafficLight>()) {
    tl_id = tl_reg_elem->id();
    break;
  }
  if (!tl_id) {
    // this lane has no traffic light
    return TrafficPrioritizedLevel::NOT_PRIORITIZED;
  }
  const auto tl_info_it = tl_infos.find(tl_id.value());
  if (tl_info_it == tl_infos.end()) {
    // the info of this traffic light is not available
    return TrafficPrioritizedLevel::NOT_PRIORITIZED;
  }
  const auto & tl_info = tl_info_it->second;
  bool has_amber_signal{false};
  for (auto && tl_light : tl_info.signal.elements) {
    if (tl_light.color == TrafficSignalElement::AMBER) {
      has_amber_signal = true;
    }
    if (tl_light.color == TrafficSignalElement::RED) {
      // NOTE: Return here since the red signal has the highest priority.
      return TrafficPrioritizedLevel::FULLY_PRIORITIZED;
    }
  }
  if (has_amber_signal) {
    return TrafficPrioritizedLevel::PARTIALLY_PRIORITIZED;
  }
  return TrafficPrioritizedLevel::NOT_PRIORITIZED;
}

double getHighestCurvature(const lanelet::ConstLineString3d & centerline)
{
  std::vector<lanelet::ConstPoint3d> points;
  for (auto point = centerline.begin(); point != centerline.end(); point++) {
    points.push_back(*point);
  }

  SplineInterpolationPoints2d interpolation(points);
  const std::vector<double> curvatures = interpolation.getSplineInterpolatedCurvatures();
  std::vector<double> curvatures_positive;
  for (const auto & curvature : curvatures) {
    curvatures_positive.push_back(std::fabs(curvature));
  }
  return *std::max_element(curvatures_positive.begin(), curvatures_positive.end());
}

std::vector<lanelet::ConstLineString3d> generateDetectionLaneDivisions(
  lanelet::ConstLanelets detection_lanelets_all,
  const lanelet::routing::RoutingGraphPtr routing_graph_ptr, const double resolution,
  const double curvature_threshold, const double curvature_calculation_ds)
{
  using lanelet::utils::getCenterlineWithOffset;

  // (0) remove left/right lanelet
  lanelet::ConstLanelets detection_lanelets;
  for (const auto & detection_lanelet : detection_lanelets_all) {
    // TODO(Mamoru Sobue): instead of ignoring, only trim straight part of lanelet
    const auto fine_centerline =
      lanelet::utils::generateFineCenterline(detection_lanelet, curvature_calculation_ds);
    const double highest_curvature = getHighestCurvature(fine_centerline);
    if (highest_curvature > curvature_threshold) {
      continue;
    }
    detection_lanelets.push_back(detection_lanelet);
  }

  // (1) tsort detection_lanelets
  const auto [merged_detection_lanelets, originals] =
    mergeLaneletsByTopologicalSort(detection_lanelets, routing_graph_ptr);

  // (2) merge each branch to one lanelet
  // NOTE: somehow bg::area() for merged lanelet does not work, so calculate it here
  std::vector<std::pair<lanelet::ConstLanelet, double>> merged_lanelet_with_area;
  for (unsigned i = 0; i < merged_detection_lanelets.size(); ++i) {
    const auto & merged_detection_lanelet = merged_detection_lanelets.at(i);
    const auto & original = originals.at(i);
    double area = 0;
    for (const auto & partition : original) {
      area += bg::area(partition.polygon2d().basicPolygon());
    }
    merged_lanelet_with_area.emplace_back(merged_detection_lanelet, area);
  }

  // (3) discretize each merged lanelet
  std::vector<lanelet::ConstLineString3d> detection_divisions;
  for (const auto & [merged_lanelet, area] : merged_lanelet_with_area) {
    const double length = bg::length(merged_lanelet.centerline());
    const double width = area / length;
    for (int i = 0; i < static_cast<int>(width / resolution); ++i) {
      const double offset = resolution * i - width / 2;
      detection_divisions.push_back(
        getCenterlineWithOffset(merged_lanelet, offset, resolution).invert());
    }
    detection_divisions.push_back(
      getCenterlineWithOffset(merged_lanelet, width / 2, resolution).invert());
  }
  return detection_divisions;
}

std::optional<InterpolatedPathInfo> generateInterpolatedPath(
  const lanelet::Id lane_id, const std::set<lanelet::Id> & associative_lane_ids,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & input_path, const double ds,
  const rclcpp::Logger logger)
{
  InterpolatedPathInfo interpolated_path_info;
  if (!splineInterpolate(input_path, ds, interpolated_path_info.path, logger)) {
    return std::nullopt;
  }
  interpolated_path_info.ds = ds;
  interpolated_path_info.lane_id = lane_id;
  interpolated_path_info.associative_lane_ids = associative_lane_ids;
  interpolated_path_info.lane_id_interval =
    findLaneIdsInterval(interpolated_path_info.path, associative_lane_ids);
  return interpolated_path_info;
}

// from here
geometry_msgs::msg::Pose getObjectPoseWithVelocityDirection(
  const autoware_auto_perception_msgs::msg::PredictedObjectKinematics & obj_state)
{
  if (obj_state.initial_twist_with_covariance.twist.linear.x >= 0) {
    return obj_state.initial_pose_with_covariance.pose;
  }

  // When the object velocity is negative, invert orientation (yaw)
  auto obj_pose = obj_state.initial_pose_with_covariance.pose;
  double yaw, pitch, roll;
  tf2::getEulerYPR(obj_pose.orientation, yaw, pitch, roll);
  tf2::Quaternion inv_q;
  inv_q.setRPY(roll, pitch, yaw + M_PI);
  obj_pose.orientation = tf2::toMsg(inv_q);
  return obj_pose;
}

static bool isTargetStuckVehicleType(
  const autoware_auto_perception_msgs::msg::PredictedObject & object)
{
  if (
    object.classification.at(0).label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::CAR ||
    object.classification.at(0).label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::BUS ||
    object.classification.at(0).label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::TRUCK ||
    object.classification.at(0).label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::TRAILER ||
    object.classification.at(0).label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::MOTORCYCLE) {
    return true;
  }
  return false;
}

bool checkStuckVehicleInIntersection(
  const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr objects_ptr,
  const Polygon2d & stuck_vehicle_detect_area, const double stuck_vehicle_vel_thr,
  DebugData * debug_data)
{
  for (const auto & object : objects_ptr->objects) {
    if (!isTargetStuckVehicleType(object)) {
      continue;  // not target vehicle type
    }
    const auto obj_v_norm = std::hypot(
      object.kinematics.initial_twist_with_covariance.twist.linear.x,
      object.kinematics.initial_twist_with_covariance.twist.linear.y);
    if (obj_v_norm > stuck_vehicle_vel_thr) {
      continue;  // not stop vehicle
    }

    // check if the footprint is in the stuck detect area
    const auto obj_footprint = tier4_autoware_utils::toPolygon2d(object);
    const bool is_in_stuck_area = !bg::disjoint(obj_footprint, stuck_vehicle_detect_area);
    if (is_in_stuck_area && debug_data) {
      debug_data->stuck_targets.objects.push_back(object);
      return true;
    }
  }
  return false;
}

static lanelet::LineString3d getLineStringFromArcLength(
  const lanelet::ConstLineString3d & linestring, const double s1, const double s2)
{
  lanelet::Points3d points;
  double accumulated_length = 0;
  size_t start_index = linestring.size();
  for (size_t i = 0; i < linestring.size() - 1; i++) {
    const auto & p1 = linestring[i];
    const auto & p2 = linestring[i + 1];
    const double length = boost::geometry::distance(p1.basicPoint(), p2.basicPoint());
    if (accumulated_length + length > s1) {
      start_index = i;
      break;
    }
    accumulated_length += length;
  }
  if (start_index < linestring.size() - 1) {
    const auto & p1 = linestring[start_index];
    const auto & p2 = linestring[start_index + 1];
    const double residue = s1 - accumulated_length;
    const auto direction_vector = (p2.basicPoint() - p1.basicPoint()).normalized();
    const auto start_basic_point = p1.basicPoint() + residue * direction_vector;
    const auto start_point = lanelet::Point3d(lanelet::InvalId, start_basic_point);
    points.push_back(start_point);
  }

  accumulated_length = 0;
  size_t end_index = linestring.size();
  for (size_t i = 0; i < linestring.size() - 1; i++) {
    const auto & p1 = linestring[i];
    const auto & p2 = linestring[i + 1];
    const double length = boost::geometry::distance(p1.basicPoint(), p2.basicPoint());
    if (accumulated_length + length > s2) {
      end_index = i;
      break;
    }
    accumulated_length += length;
  }

  for (size_t i = start_index + 1; i < end_index; i++) {
    const auto p = lanelet::Point3d(linestring[i]);
    points.push_back(p);
  }
  if (end_index < linestring.size() - 1) {
    const auto & p1 = linestring[end_index];
    const auto & p2 = linestring[end_index + 1];
    const double residue = s2 - accumulated_length;
    const auto direction_vector = (p2.basicPoint() - p1.basicPoint()).normalized();
    const auto end_basic_point = p1.basicPoint() + residue * direction_vector;
    const auto end_point = lanelet::Point3d(lanelet::InvalId, end_basic_point);
    points.push_back(end_point);
  }
  return lanelet::LineString3d{lanelet::InvalId, points};
}

static lanelet::ConstLanelet createLaneletFromArcLength(
  const lanelet::ConstLanelet & lanelet, const double s1, const double s2)
{
  const double total_length = boost::geometry::length(lanelet.centerline2d().basicLineString());
  // make sure that s1, and s2 are between [0, lane_length]
  const auto s1_saturated = std::max(0.0, std::min(s1, total_length));
  const auto s2_saturated = std::max(0.0, std::min(s2, total_length));

  const auto ratio_s1 = s1_saturated / total_length;
  const auto ratio_s2 = s2_saturated / total_length;

  const auto s1_left =
    static_cast<double>(ratio_s1 * boost::geometry::length(lanelet.leftBound().basicLineString()));
  const auto s2_left =
    static_cast<double>(ratio_s2 * boost::geometry::length(lanelet.leftBound().basicLineString()));
  const auto s1_right =
    static_cast<double>(ratio_s1 * boost::geometry::length(lanelet.rightBound().basicLineString()));
  const auto s2_right =
    static_cast<double>(ratio_s2 * boost::geometry::length(lanelet.rightBound().basicLineString()));

  const auto left_bound = getLineStringFromArcLength(lanelet.leftBound(), s1_left, s2_left);
  const auto right_bound = getLineStringFromArcLength(lanelet.rightBound(), s1_right, s2_right);

  return lanelet::Lanelet(lanelet::InvalId, left_bound, right_bound);
}

bool checkYieldStuckVehicleInIntersection(
  const util::TargetObjects & target_objects,
  const util::InterpolatedPathInfo & interpolated_path_info,
  const lanelet::ConstLanelets & attention_lanelets, const std::string & turn_direction,
  const double width, const double stuck_vehicle_vel_thr, const double yield_stuck_distance_thr,
  DebugData * debug_data)
{
  LineString2d sparse_intersection_path;
  const auto [start, end] = interpolated_path_info.lane_id_interval.value();
  for (unsigned i = start; i < end; ++i) {
    const auto & point = interpolated_path_info.path.points.at(i).point.pose.position;
    const auto yaw = tf2::getYaw(interpolated_path_info.path.points.at(i).point.pose.orientation);
    if (turn_direction == "right") {
      const double right_x = point.x - width / 2 * std::sin(yaw);
      const double right_y = point.y + width / 2 * std::cos(yaw);
      sparse_intersection_path.emplace_back(right_x, right_y);
    } else if (turn_direction == "left") {
      const double left_x = point.x + width / 2 * std::sin(yaw);
      const double left_y = point.y - width / 2 * std::cos(yaw);
      sparse_intersection_path.emplace_back(left_x, left_y);
    } else {
      // straight
      sparse_intersection_path.emplace_back(point.x, point.y);
    }
  }
  lanelet::ConstLanelets yield_stuck_detect_lanelets;
  for (const auto & attention_lanelet : attention_lanelets) {
    const auto centerline = attention_lanelet.centerline2d().basicLineString();
    std::vector<Point2d> intersects;
    bg::intersection(sparse_intersection_path, centerline, intersects);
    if (intersects.empty()) {
      continue;
    }
    const auto intersect = intersects.front();
    const auto intersect_arc_coords = lanelet::geometry::toArcCoordinates(
      centerline, lanelet::BasicPoint2d(intersect.x(), intersect.y()));
    const double yield_stuck_start =
      std::max(0.0, intersect_arc_coords.length - yield_stuck_distance_thr);
    const double yield_stuck_end = intersect_arc_coords.length;
    yield_stuck_detect_lanelets.push_back(
      createLaneletFromArcLength(attention_lanelet, yield_stuck_start, yield_stuck_end));
  }
  debug_data->yield_stuck_detect_area = getPolygon3dFromLanelets(yield_stuck_detect_lanelets);
  for (const auto & object : target_objects.all_attention_objects) {
    const auto obj_v_norm = std::hypot(
      object.object.kinematics.initial_twist_with_covariance.twist.linear.x,
      object.object.kinematics.initial_twist_with_covariance.twist.linear.y);

    if (obj_v_norm > stuck_vehicle_vel_thr) {
      continue;
    }
    for (const auto & yield_stuck_detect_lanelet : yield_stuck_detect_lanelets) {
      const bool is_in_lanelet = lanelet::utils::isInLanelet(
        object.object.kinematics.initial_pose_with_covariance.pose, yield_stuck_detect_lanelet);
      if (is_in_lanelet) {
        debug_data->yield_stuck_targets.objects.push_back(object.object);
        return true;
      }
    }
  }
  return false;
}

Polygon2d generateStuckVehicleDetectAreaPolygon(
  const util::PathLanelets & path_lanelets, const double stuck_vehicle_detect_dist)
{
  using lanelet::utils::getArcCoordinates;
  using lanelet::utils::getLaneletLength3d;
  using lanelet::utils::getPolygonFromArcLength;
  using lanelet::utils::to2D;

  Polygon2d polygon{};
  if (path_lanelets.conflicting_interval_and_remaining.size() == 0) {
    return polygon;
  }

  double target_polygon_length =
    getLaneletLength3d(path_lanelets.conflicting_interval_and_remaining);
  lanelet::ConstLanelets targets = path_lanelets.conflicting_interval_and_remaining;
  if (path_lanelets.next) {
    targets.push_back(path_lanelets.next.value());
    const double next_arc_length =
      std::min(stuck_vehicle_detect_dist, getLaneletLength3d(path_lanelets.next.value()));
    target_polygon_length += next_arc_length;
  }
  const auto target_polygon =
    to2D(getPolygonFromArcLength(targets, 0, target_polygon_length)).basicPolygon();

  if (target_polygon.empty()) {
    return polygon;
  }

  for (const auto & p : target_polygon) {
    polygon.outer().emplace_back(p.x(), p.y());
  }

  polygon.outer().emplace_back(polygon.outer().front());
  bg::correct(polygon);

  return polygon;
}

std::optional<size_t> checkAngleForTargetLanelets(
  const geometry_msgs::msg::Pose & pose, const lanelet::ConstLanelets & target_lanelets,
  const double detection_area_angle_thr, const bool consider_wrong_direction_vehicle,
  const double dist_margin, const bool is_parked_vehicle)
{
  for (unsigned i = 0; i < target_lanelets.size(); ++i) {
    const auto & ll = target_lanelets.at(i);
    if (!lanelet::utils::isInLanelet(pose, ll, dist_margin)) {
      continue;
    }
    const double ll_angle = lanelet::utils::getLaneletAngle(ll, pose.position);
    const double pose_angle = tf2::getYaw(pose.orientation);
    const double angle_diff = tier4_autoware_utils::normalizeRadian(ll_angle - pose_angle, -M_PI);
    if (consider_wrong_direction_vehicle) {
      if (std::fabs(angle_diff) > 1.57 || std::fabs(angle_diff) < detection_area_angle_thr) {
        return std::make_optional<size_t>(i);
      }
    } else {
      if (std::fabs(angle_diff) < detection_area_angle_thr) {
        return std::make_optional<size_t>(i);
      }
      // NOTE: sometimes parked vehicle direction is reversed even if its longitudinal velocity is
      // positive
      if (
        is_parked_vehicle && (std::fabs(angle_diff) < detection_area_angle_thr ||
                              (std::fabs(angle_diff + M_PI) < detection_area_angle_thr))) {
        return std::make_optional<size_t>(i);
      }
    }
  }
  return std::nullopt;
}

void cutPredictPathWithDuration(
  util::TargetObjects * target_objects, const rclcpp::Clock::SharedPtr clock, const double time_thr)
{
  const rclcpp::Time current_time = clock->now();
  for (auto & target_object : target_objects->all_attention_objects) {  // each objects
    for (auto & predicted_path :
         target_object.object.kinematics.predicted_paths) {  // each predicted paths
      const auto origin_path = predicted_path;
      predicted_path.path.clear();

      for (size_t k = 0; k < origin_path.path.size(); ++k) {  // each path points
        const auto & predicted_pose = origin_path.path.at(k);
        const auto predicted_time =
          rclcpp::Time(target_objects->header.stamp) +
          rclcpp::Duration(origin_path.time_step) * static_cast<double>(k);
        if ((predicted_time - current_time).seconds() < time_thr) {
          predicted_path.path.push_back(predicted_pose);
        }
      }
    }
  }
}

TimeDistanceArray calcIntersectionPassingTime(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const std::shared_ptr<const PlannerData> & planner_data, const lanelet::Id lane_id,
  const std::set<lanelet::Id> & associative_ids, const size_t closest_idx,
  const size_t last_intersection_stopline_candidate_idx, const double time_delay,
  const double intersection_velocity, const double minimum_ego_velocity,
  const bool use_upstream_velocity, const double minimum_upstream_velocity,
  tier4_debug_msgs::msg::Float64MultiArrayStamped * debug_ttc_array)
{
  const double current_velocity = planner_data->current_velocity->twist.linear.x;

  int assigned_lane_found = false;

  // crop intersection part of the path, and set the reference velocity to intersection_velocity
  // for ego's ttc
  PathWithLaneId reference_path;
  std::optional<size_t> upstream_stopline{std::nullopt};
  for (size_t i = 0; i < path.points.size() - 1; ++i) {
    auto reference_point = path.points.at(i);
    // assume backward velocity is current ego velocity
    if (i < closest_idx) {
      reference_point.point.longitudinal_velocity_mps = current_velocity;
    }
    if (
      i > last_intersection_stopline_candidate_idx &&
      std::fabs(reference_point.point.longitudinal_velocity_mps) <
        std::numeric_limits<double>::epsilon() &&
      !upstream_stopline) {
      upstream_stopline = i;
    }
    if (!use_upstream_velocity) {
      reference_point.point.longitudinal_velocity_mps = intersection_velocity;
    }
    reference_path.points.push_back(reference_point);
    bool has_objective_lane_id = hasLaneIds(path.points.at(i), associative_ids);
    if (assigned_lane_found && !has_objective_lane_id) {
      break;
    }
    assigned_lane_found = has_objective_lane_id;
  }
  if (!assigned_lane_found) {
    return {{0.0, 0.0}};  // has already passed the intersection.
  }

  std::vector<std::pair<double, double>> original_path_xy;
  for (size_t i = 0; i < reference_path.points.size(); ++i) {
    const auto & p = reference_path.points.at(i).point.pose.position;
    original_path_xy.emplace_back(p.x, p.y);
  }

  // apply smoother to reference velocity
  PathWithLaneId smoothed_reference_path = reference_path;
  if (!smoothPath(reference_path, smoothed_reference_path, planner_data)) {
    smoothed_reference_path = reference_path;
  }

  // calculate when ego is going to reach each (interpolated) points on the path
  TimeDistanceArray time_distance_array{};
  double dist_sum = 0.0;
  double passing_time = time_delay;
  time_distance_array.emplace_back(passing_time, dist_sum);

  // NOTE: `reference_path` is resampled in `reference_smoothed_path`, so
  // `last_intersection_stopline_candidate_idx` makes no sense
  const auto smoothed_path_closest_idx = motion_utils::findFirstNearestIndexWithSoftConstraints(
    smoothed_reference_path.points, path.points.at(closest_idx).point.pose,
    planner_data->ego_nearest_dist_threshold, planner_data->ego_nearest_yaw_threshold);

  const std::optional<size_t> upstream_stopline_idx_opt = [&]() -> std::optional<size_t> {
    if (upstream_stopline) {
      const auto upstream_stopline_point = path.points.at(upstream_stopline.value()).point.pose;
      return motion_utils::findFirstNearestIndexWithSoftConstraints(
        smoothed_reference_path.points, upstream_stopline_point,
        planner_data->ego_nearest_dist_threshold, planner_data->ego_nearest_yaw_threshold);
    } else {
      return std::nullopt;
    }
  }();
  const bool has_upstream_stopline = upstream_stopline_idx_opt.has_value();
  const size_t upstream_stopline_ind = upstream_stopline_idx_opt.value_or(0);

  for (size_t i = smoothed_path_closest_idx; i < smoothed_reference_path.points.size() - 1; ++i) {
    const auto & p1 = smoothed_reference_path.points.at(i);
    const auto & p2 = smoothed_reference_path.points.at(i + 1);

    const double dist = tier4_autoware_utils::calcDistance2d(p1, p2);
    dist_sum += dist;

    // use average velocity between p1 and p2
    const double average_velocity =
      (p1.point.longitudinal_velocity_mps + p2.point.longitudinal_velocity_mps) / 2.0;
    const double passing_velocity = [=]() {
      if (use_upstream_velocity) {
        if (has_upstream_stopline && i > upstream_stopline_ind) {
          return minimum_upstream_velocity;
        }
        return std::max<double>(average_velocity, minimum_ego_velocity);
      } else {
        return std::max<double>(average_velocity, minimum_ego_velocity);
      }
    }();
    passing_time += (dist / passing_velocity);

    time_distance_array.emplace_back(passing_time, dist_sum);
  }
  debug_ttc_array->layout.dim.resize(3);
  debug_ttc_array->layout.dim.at(0).label = "lane_id_@[0][0], ttc_time, ttc_dist, path_x, path_y";
  debug_ttc_array->layout.dim.at(0).size = 5;
  debug_ttc_array->layout.dim.at(1).label = "values";
  debug_ttc_array->layout.dim.at(1).size = time_distance_array.size();
  debug_ttc_array->data.reserve(
    time_distance_array.size() * debug_ttc_array->layout.dim.at(0).size);
  for (unsigned i = 0; i < time_distance_array.size(); ++i) {
    debug_ttc_array->data.push_back(lane_id);
  }
  for (const auto & [t, d] : time_distance_array) {
    debug_ttc_array->data.push_back(t);
  }
  for (const auto & [t, d] : time_distance_array) {
    debug_ttc_array->data.push_back(d);
  }
  for (size_t i = smoothed_path_closest_idx; i < smoothed_reference_path.points.size(); ++i) {
    const auto & p = smoothed_reference_path.points.at(i).point.pose.position;
    debug_ttc_array->data.push_back(p.x);
  }
  for (size_t i = smoothed_path_closest_idx; i < smoothed_reference_path.points.size(); ++i) {
    const auto & p = smoothed_reference_path.points.at(i).point.pose.position;
    debug_ttc_array->data.push_back(p.y);
  }
  return time_distance_array;
}

double calcDistanceUntilIntersectionLanelet(
  const lanelet::ConstLanelet & assigned_lanelet,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const size_t closest_idx)
{
  const auto lane_id = assigned_lanelet.id();
  const auto intersection_first_itr =
    std::find_if(path.points.cbegin(), path.points.cend(), [&](const auto & p) {
      return std::find(p.lane_ids.begin(), p.lane_ids.end(), lane_id) != p.lane_ids.end();
    });
  if (
    intersection_first_itr == path.points.begin() || intersection_first_itr == path.points.end()) {
    return 0.0;
  }
  const auto dst_idx = std::distance(path.points.begin(), intersection_first_itr) - 1;

  if (closest_idx > static_cast<size_t>(dst_idx)) {
    return 0.0;
  }

  double distance = std::abs(motion_utils::calcSignedArcLength(path.points, closest_idx, dst_idx));
  const auto & lane_first_point = assigned_lanelet.centerline2d().front();
  distance += std::hypot(
    path.points.at(dst_idx).point.pose.position.x - lane_first_point.x(),
    path.points.at(dst_idx).point.pose.position.y - lane_first_point.y());
  return distance;
}

void IntersectionLanelets::update(
  const bool is_prioritized, const InterpolatedPathInfo & interpolated_path_info,
  const tier4_autoware_utils::LinearRing2d & footprint, const double vehicle_length)
{
  is_prioritized_ = is_prioritized;
  // find the first conflicting/detection area polygon intersecting the path
  if (!first_conflicting_area_) {
    auto first = getFirstPointInsidePolygonsByFootprint(
      conflicting_area_, interpolated_path_info, footprint, vehicle_length);
    if (first) {
      first_conflicting_lane_ = conflicting_.at(first.value().second);
      first_conflicting_area_ = conflicting_area_.at(first.value().second);
    }
  }
  if (!first_attention_area_) {
    auto first = getFirstPointInsidePolygonsByFootprint(
      attention_non_preceding_area_, interpolated_path_info, footprint, vehicle_length);
    if (first) {
      first_attention_lane_ = attention_non_preceding_.at(first.value().second);
      first_attention_area_ = attention_non_preceding_area_.at(first.value().second);
    }
  }
}

static lanelet::ConstLanelets getPrevLanelets(
  const lanelet::ConstLanelets & lanelets_on_path, const std::set<lanelet::Id> & associative_ids)
{
  lanelet::ConstLanelets previous_lanelets;
  for (const auto & ll : lanelets_on_path) {
    if (associative_ids.find(ll.id()) != associative_ids.end()) {
      return previous_lanelets;
    }
    previous_lanelets.push_back(ll);
  }
  return previous_lanelets;
}

// end inclusive
lanelet::ConstLanelet generatePathLanelet(
  const PathWithLaneId & path, const size_t start_idx, const size_t end_idx, const double width,
  const double interval)
{
  lanelet::Points3d lefts;
  lanelet::Points3d rights;
  size_t prev_idx = start_idx;
  for (size_t i = start_idx; i <= end_idx; ++i) {
    const auto & p = path.points.at(i).point.pose;
    const auto & p_prev = path.points.at(prev_idx).point.pose;
    if (i != start_idx && tier4_autoware_utils::calcDistance2d(p_prev, p) < interval) {
      continue;
    }
    prev_idx = i;
    const double yaw = tf2::getYaw(p.orientation);
    const double x = p.position.x;
    const double y = p.position.y;
    // NOTE: maybe this is opposite
    const double left_x = x + width / 2 * std::sin(yaw);
    const double left_y = y - width / 2 * std::cos(yaw);
    const double right_x = x - width / 2 * std::sin(yaw);
    const double right_y = y + width / 2 * std::cos(yaw);
    lefts.emplace_back(lanelet::InvalId, left_x, left_y, p.position.z);
    rights.emplace_back(lanelet::InvalId, right_x, right_y, p.position.z);
  }
  lanelet::LineString3d left = lanelet::LineString3d(lanelet::InvalId, lefts);
  lanelet::LineString3d right = lanelet::LineString3d(lanelet::InvalId, rights);

  return lanelet::Lanelet(lanelet::InvalId, left, right);
}

std::optional<PathLanelets> generatePathLanelets(
  const lanelet::ConstLanelets & lanelets_on_path,
  const util::InterpolatedPathInfo & interpolated_path_info,
  const std::set<lanelet::Id> & associative_ids,
  const lanelet::CompoundPolygon3d & first_conflicting_area,
  const std::vector<lanelet::CompoundPolygon3d> & conflicting_areas,
  const std::optional<lanelet::CompoundPolygon3d> & first_attention_area,
  const std::vector<lanelet::CompoundPolygon3d> & attention_areas, const size_t closest_idx,
  const double width)
{
  static constexpr double path_lanelet_interval = 1.5;

  const auto & assigned_lane_interval_opt = interpolated_path_info.lane_id_interval;
  if (!assigned_lane_interval_opt) {
    return std::nullopt;
  }
  const auto assigned_lane_interval = assigned_lane_interval_opt.value();
  const auto & path = interpolated_path_info.path;

  PathLanelets path_lanelets;
  // prev
  path_lanelets.prev = getPrevLanelets(lanelets_on_path, associative_ids);
  path_lanelets.all = path_lanelets.prev;

  // entry2ego if exist
  const auto [assigned_lane_start, assigned_lane_end] = assigned_lane_interval;
  if (closest_idx > assigned_lane_start) {
    path_lanelets.all.push_back(
      generatePathLanelet(path, assigned_lane_start, closest_idx, width, path_lanelet_interval));
  }

  // ego_or_entry2exit
  const auto ego_or_entry_start = std::max(closest_idx, assigned_lane_start);
  path_lanelets.ego_or_entry2exit =
    generatePathLanelet(path, ego_or_entry_start, assigned_lane_end, width, path_lanelet_interval);
  path_lanelets.all.push_back(path_lanelets.ego_or_entry2exit);

  // next
  if (assigned_lane_end < path.points.size() - 1) {
    const int next_id = path.points.at(assigned_lane_end).lane_ids.at(0);
    const auto next_lane_interval_opt = findLaneIdsInterval(path, {next_id});
    if (next_lane_interval_opt) {
      const auto [next_start, next_end] = next_lane_interval_opt.value();
      path_lanelets.next =
        generatePathLanelet(path, next_start, next_end, width, path_lanelet_interval);
      path_lanelets.all.push_back(path_lanelets.next.value());
    }
  }

  const auto first_inside_conflicting_idx_opt =
    first_attention_area.has_value()
      ? getFirstPointInsidePolygon(path, assigned_lane_interval, first_attention_area.value())
      : getFirstPointInsidePolygon(path, assigned_lane_interval, first_conflicting_area);
  const auto last_inside_conflicting_idx_opt =
    first_attention_area.has_value()
      ? getFirstPointInsidePolygons(path, assigned_lane_interval, attention_areas, false)
      : getFirstPointInsidePolygons(path, assigned_lane_interval, conflicting_areas, false);
  if (first_inside_conflicting_idx_opt && last_inside_conflicting_idx_opt) {
    const auto first_inside_conflicting_idx = first_inside_conflicting_idx_opt.value();
    const auto last_inside_conflicting_idx = last_inside_conflicting_idx_opt.value().first;
    lanelet::ConstLanelet conflicting_interval = generatePathLanelet(
      path, first_inside_conflicting_idx, last_inside_conflicting_idx, width,
      path_lanelet_interval);
    path_lanelets.conflicting_interval_and_remaining.push_back(std::move(conflicting_interval));
    if (last_inside_conflicting_idx < assigned_lane_end) {
      lanelet::ConstLanelet remaining_interval = generatePathLanelet(
        path, last_inside_conflicting_idx, assigned_lane_end, width, path_lanelet_interval);
      path_lanelets.conflicting_interval_and_remaining.push_back(std::move(remaining_interval));
    }
  }
  return path_lanelets;
}

void TargetObject::calc_dist_to_stopline()
{
  if (!attention_lanelet || !stopline) {
    return;
  }
  const auto attention_lanelet_val = attention_lanelet.value();
  const auto object_arc_coords = lanelet::utils::getArcCoordinates(
    {attention_lanelet_val}, object.kinematics.initial_pose_with_covariance.pose);
  const auto stopline_val = stopline.value();
  geometry_msgs::msg::Pose stopline_center;
  stopline_center.position.x = (stopline_val.front().x() + stopline_val.back().x()) / 2.0;
  stopline_center.position.y = (stopline_val.front().y() + stopline_val.back().y()) / 2.0;
  stopline_center.position.z = (stopline_val.front().z() + stopline_val.back().z()) / 2.0;
  const auto stopline_arc_coords =
    lanelet::utils::getArcCoordinates({attention_lanelet_val}, stopline_center);
  dist_to_stopline = (stopline_arc_coords.length - object_arc_coords.length);
}

}  // namespace util
}  // namespace behavior_velocity_planner
