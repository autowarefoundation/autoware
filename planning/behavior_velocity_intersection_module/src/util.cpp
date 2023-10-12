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

#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <algorithm>
#include <cmath>
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
  autoware_auto_planning_msgs::msg::PathWithLaneId * inout_path)
{
  const auto duplicate_idx_opt = getDuplicatedPointIdx(*inout_path, in_pose.position);
  if (duplicate_idx_opt) {
    return duplicate_idx_opt.value();
  }

  static constexpr double dist_thr = 10.0;
  static constexpr double angle_thr = M_PI / 1.5;
  const auto closest_idx_opt =
    motion_utils::findNearestIndex(inout_path->points, in_pose, dist_thr, angle_thr);
  if (!closest_idx_opt) {
    return std::nullopt;
  }
  const size_t closest_idx = closest_idx_opt.get();
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
  const autoware_auto_planning_msgs::msg::PathPointWithLaneId & p, const std::set<int> & ids)
{
  for (const auto & pid : p.lane_ids) {
    if (ids.find(pid) != ids.end()) {
      return true;
    }
  }
  return false;
}

std::optional<std::pair<size_t, size_t>> findLaneIdsInterval(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & p, const std::set<int> & ids)
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
  const std::shared_ptr<const PlannerData> & planner_data, const double dist_thr)
{
  const auto & path = interpolated_path_info.path;
  const auto & lane_interval = interpolated_path_info.lane_id_interval.value();

  lanelet::ConstLanelet lanelet =
    planner_data->route_handler_->getLaneletMapPtr()->laneletLayer.get(
      interpolated_path_info.lane_id);
  const auto road_markings = lanelet.regulatoryElementsAs<lanelet::autoware::RoadMarking>();
  lanelet::ConstLineStrings3d stop_line;
  for (const auto & road_marking : road_markings) {
    const std::string type =
      road_marking->roadMarking().attributeOr(lanelet::AttributeName::Type, "none");
    if (type == lanelet::AttributeValueString::StopLine) {
      stop_line.push_back(road_marking->roadMarking());
      break;  // only one stop_line exists.
    }
  }
  if (stop_line.empty()) {
    return std::nullopt;
  }

  const auto p_start = stop_line.front().front();
  const auto p_end = stop_line.front().back();
  const LineString2d extended_stop_line =
    planning_utils::extendLine(p_start, p_end, planner_data->stop_line_extend_length);

  for (size_t i = lane_interval.first; i < lane_interval.second; i++) {
    const auto & p_front = path.points.at(i).point.pose.position;
    const auto & p_back = path.points.at(i + 1).point.pose.position;

    const LineString2d path_segment = {{p_front.x, p_front.y}, {p_back.x, p_back.y}};
    std::vector<Point2d> collision_points;
    bg::intersection(extended_stop_line, path_segment, collision_points);

    if (collision_points.empty()) {
      continue;
    }

    return i;
  }

  geometry_msgs::msg::Pose stop_point_from_map;
  stop_point_from_map.position.x = 0.5 * (p_start.x() + p_end.x());
  stop_point_from_map.position.y = 0.5 * (p_start.y() + p_end.y());
  stop_point_from_map.position.z = 0.5 * (p_start.z() + p_end.z());

  const auto stop_idx_ip_opt =
    motion_utils::findNearestIndex(path.points, stop_point_from_map, static_cast<double>(dist_thr));
  if (!stop_idx_ip_opt) {
    return std::nullopt;
  }
  return stop_idx_ip_opt.get();
}

static std::optional<size_t> getFirstPointInsidePolygonByFootprint(
  const lanelet::CompoundPolygon3d & polygon, const InterpolatedPathInfo & interpolated_path_info,
  const tier4_autoware_utils::LinearRing2d & footprint)
{
  const auto & path_ip = interpolated_path_info.path;
  const auto [lane_start, lane_end] = interpolated_path_info.lane_id_interval.value();

  const auto area_2d = lanelet::utils::to2D(polygon).basicPolygon();
  for (auto i = lane_start; i <= lane_end; ++i) {
    const auto & base_pose = path_ip.points.at(i).point.pose;
    const auto path_footprint = tier4_autoware_utils::transformVector(
      footprint, tier4_autoware_utils::pose2transform(base_pose));
    if (bg::intersects(path_footprint, area_2d)) {
      return std::make_optional<size_t>(i);
    }
  }
  return std::nullopt;
}

static std::optional<std::pair<size_t, const lanelet::CompoundPolygon3d &>>
getFirstPointInsidePolygonsByFootprint(
  const std::vector<lanelet::CompoundPolygon3d> & polygons,
  const InterpolatedPathInfo & interpolated_path_info,
  const tier4_autoware_utils::LinearRing2d & footprint)
{
  const auto & path_ip = interpolated_path_info.path;
  const auto [lane_start, lane_end] = interpolated_path_info.lane_id_interval.value();

  for (size_t i = lane_start; i <= lane_end; ++i) {
    const auto & pose = path_ip.points.at(i).point.pose;
    const auto path_footprint =
      tier4_autoware_utils::transformVector(footprint, tier4_autoware_utils::pose2transform(pose));
    for (const auto & polygon : polygons) {
      const auto area_2d = lanelet::utils::to2D(polygon).basicPolygon();
      const bool is_in_polygon = bg::intersects(area_2d, path_footprint);
      if (is_in_polygon) {
        return std::make_optional<std::pair<size_t, const lanelet::CompoundPolygon3d &>>(
          i, polygon);
      }
    }
  }
  return std::nullopt;
}

std::optional<IntersectionStopLines> generateIntersectionStopLines(
  const lanelet::CompoundPolygon3d & first_conflicting_area,
  const lanelet::CompoundPolygon3d & first_detection_area,
  const std::shared_ptr<const PlannerData> & planner_data,
  const InterpolatedPathInfo & interpolated_path_info, const bool use_stuck_stopline,
  const double stop_line_margin, const double peeking_offset,
  autoware_auto_planning_msgs::msg::PathWithLaneId * original_path)
{
  const auto & path_ip = interpolated_path_info.path;
  const double ds = interpolated_path_info.ds;
  const auto & lane_interval_ip = interpolated_path_info.lane_id_interval.value();

  const int stop_line_margin_idx_dist = std::ceil(stop_line_margin / ds);
  const int base2front_idx_dist =
    std::ceil(planner_data->vehicle_info_.max_longitudinal_offset_m / ds);

  /*
  // find the index of the first point that intersects with detection_areas
  const auto first_inside_detection_idx_ip_opt =
    getFirstPointInsidePolygon(path_ip, lane_interval_ip, first_detection_area);
  // if path is not intersecting with detection_area, error
  if (!first_inside_detection_idx_ip_opt) {
    return std::nullopt;
  }
  const auto first_inside_detection_ip = first_inside_detection_idx_ip_opt.value();
  */
  // find the index of the first point whose vehicle footprint on it intersects with detection_area
  const auto local_footprint = planner_data->vehicle_info_.createFootprint(0.0, 0.0);
  std::optional<size_t> first_footprint_inside_detection_ip_opt =
    getFirstPointInsidePolygonByFootprint(
      first_detection_area, interpolated_path_info, local_footprint);
  if (!first_footprint_inside_detection_ip_opt) {
    return std::nullopt;
  }
  const auto first_footprint_inside_detection_ip = first_footprint_inside_detection_ip_opt.value();

  // (1) default stop line position on interpolated path
  bool default_stop_line_valid = true;
  int stop_idx_ip_int = -1;
  if (const auto map_stop_idx_ip =
        getStopLineIndexFromMap(interpolated_path_info, planner_data, 10.0);
      map_stop_idx_ip) {
    stop_idx_ip_int = static_cast<int>(map_stop_idx_ip.value()) - base2front_idx_dist;
  }
  if (stop_idx_ip_int < 0) {
    stop_idx_ip_int = first_footprint_inside_detection_ip - stop_line_margin_idx_dist;
  }
  if (stop_idx_ip_int < 0) {
    default_stop_line_valid = false;
  }
  const auto default_stop_line_ip = stop_idx_ip_int >= 0 ? static_cast<size_t>(stop_idx_ip_int) : 0;

  // (2) ego front stop line position on interpolated path
  const geometry_msgs::msg::Pose & current_pose = planner_data->current_odometry->pose;
  const auto closest_idx_ip_opt =
    motion_utils::findNearestIndex(path_ip.points, current_pose, 3.0, M_PI_4);
  if (!closest_idx_ip_opt) {
    return std::nullopt;
  }
  const auto closest_idx_ip = closest_idx_ip_opt.value();

  // (3) occlusion peeking stop line position on interpolated path
  int occlusion_peeking_line_ip_int = static_cast<int>(default_stop_line_ip);
  bool occlusion_peeking_line_valid = true;
  {
    // NOTE: if footprints[0] is already inside the detection area, invalid
    const auto & base_pose0 = path_ip.points.at(default_stop_line_ip).point.pose;
    const auto path_footprint0 = tier4_autoware_utils::transformVector(
      local_footprint, tier4_autoware_utils::pose2transform(base_pose0));
    if (bg::intersects(
          path_footprint0, lanelet::utils::to2D(first_detection_area).basicPolygon())) {
      occlusion_peeking_line_valid = false;
    }
  }
  if (occlusion_peeking_line_valid) {
    occlusion_peeking_line_ip_int = first_footprint_inside_detection_ip;
  }
  const auto first_attention_stop_line_ip = static_cast<size_t>(occlusion_peeking_line_ip_int);
  const bool first_attention_stop_line_valid = occlusion_peeking_line_valid;
  occlusion_peeking_line_ip_int += std::ceil(peeking_offset / ds);
  const auto occlusion_peeking_line_ip = static_cast<size_t>(
    std::clamp<int>(occlusion_peeking_line_ip_int, 0, static_cast<int>(path_ip.points.size()) - 1));

  // (4) pass judge line position on interpolated path
  const double velocity = planner_data->current_velocity->twist.linear.x;
  const double acceleration = planner_data->current_acceleration->accel.accel.linear.x;
  const double max_stop_acceleration = planner_data->max_stop_acceleration_threshold;
  const double max_stop_jerk = planner_data->max_stop_jerk_threshold;
  const double delay_response_time = planner_data->delay_response_time;
  const double braking_dist = planning_utils::calcJudgeLineDistWithJerkLimit(
    velocity, acceleration, max_stop_acceleration, max_stop_jerk, delay_response_time);
  int pass_judge_ip_int =
    static_cast<int>(first_footprint_inside_detection_ip) - std::ceil(braking_dist / ds);
  const auto pass_judge_line_ip = static_cast<size_t>(
    std::clamp<int>(pass_judge_ip_int, 0, static_cast<int>(path_ip.points.size()) - 1));

  // (5) stuck vehicle stop line
  int stuck_stop_line_ip_int = 0;
  bool stuck_stop_line_valid = true;
  if (use_stuck_stopline) {
    // NOTE: when ego vehicle is approaching detection area and already passed
    // first_conflicting_area, this could be null.
    const auto stuck_stop_line_idx_ip_opt = getFirstPointInsidePolygonByFootprint(
      first_conflicting_area, interpolated_path_info, local_footprint);
    if (!stuck_stop_line_idx_ip_opt) {
      stuck_stop_line_valid = false;
      stuck_stop_line_ip_int = 0;
    } else {
      stuck_stop_line_ip_int = stuck_stop_line_idx_ip_opt.value() - stop_line_margin_idx_dist;
    }
  } else {
    stuck_stop_line_ip_int =
      std::get<0>(lane_interval_ip) - (stop_line_margin_idx_dist + base2front_idx_dist);
  }
  if (stuck_stop_line_ip_int < 0) {
    stuck_stop_line_valid = false;
  }
  const auto stuck_stop_line_ip = static_cast<size_t>(std::max(0, stuck_stop_line_ip_int));

  struct IntersectionStopLinesTemp
  {
    size_t closest_idx{0};
    size_t stuck_stop_line{0};
    size_t default_stop_line{0};
    size_t first_attention_stop_line{0};
    size_t occlusion_peeking_stop_line{0};
    size_t pass_judge_line{0};
  };

  IntersectionStopLinesTemp intersection_stop_lines_temp;
  std::list<std::pair<const size_t *, size_t *>> stop_lines = {
    {&closest_idx_ip, &intersection_stop_lines_temp.closest_idx},
    {&stuck_stop_line_ip, &intersection_stop_lines_temp.stuck_stop_line},
    {&default_stop_line_ip, &intersection_stop_lines_temp.default_stop_line},
    {&first_attention_stop_line_ip, &intersection_stop_lines_temp.first_attention_stop_line},
    {&occlusion_peeking_line_ip, &intersection_stop_lines_temp.occlusion_peeking_stop_line},
    {&pass_judge_line_ip, &intersection_stop_lines_temp.pass_judge_line},
  };
  stop_lines.sort(
    [](const auto & it1, const auto & it2) { return *(std::get<0>(it1)) < *(std::get<0>(it2)); });
  for (const auto & [stop_idx_ip, stop_idx] : stop_lines) {
    const auto & insert_point = path_ip.points.at(*stop_idx_ip).point.pose;
    const auto insert_idx = insertPointIndex(insert_point, original_path);
    if (!insert_idx) {
      return std::nullopt;
    }
    *stop_idx = insert_idx.value();
  }
  if (
    intersection_stop_lines_temp.occlusion_peeking_stop_line <
    intersection_stop_lines_temp.default_stop_line) {
    intersection_stop_lines_temp.occlusion_peeking_stop_line =
      intersection_stop_lines_temp.default_stop_line;
  }
  if (
    intersection_stop_lines_temp.occlusion_peeking_stop_line >
    intersection_stop_lines_temp.pass_judge_line) {
    intersection_stop_lines_temp.pass_judge_line =
      intersection_stop_lines_temp.occlusion_peeking_stop_line;
  }

  IntersectionStopLines intersection_stop_lines;
  intersection_stop_lines.closest_idx = intersection_stop_lines_temp.closest_idx;
  if (stuck_stop_line_valid) {
    intersection_stop_lines.stuck_stop_line = intersection_stop_lines_temp.stuck_stop_line;
  }
  if (default_stop_line_valid) {
    intersection_stop_lines.default_stop_line = intersection_stop_lines_temp.default_stop_line;
  }
  if (first_attention_stop_line_valid) {
    intersection_stop_lines.first_attention_stop_line =
      intersection_stop_lines_temp.first_attention_stop_line;
  }
  if (occlusion_peeking_line_valid) {
    intersection_stop_lines.occlusion_peeking_stop_line =
      intersection_stop_lines_temp.occlusion_peeking_stop_line;
  }
  intersection_stop_lines.pass_judge_line = intersection_stop_lines_temp.pass_judge_line;
  return intersection_stop_lines;
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
  const InterpolatedPathInfo & interpolated_path_info, const double stop_line_margin,
  const bool use_stuck_stopline, autoware_auto_planning_msgs::msg::PathWithLaneId * original_path)
{
  const auto & path_ip = interpolated_path_info.path;
  const double ds = interpolated_path_info.ds;
  const auto & lane_interval_ip = interpolated_path_info.lane_id_interval.value();
  const auto lane_interval_ip_start = std::get<0>(lane_interval_ip);
  size_t stuck_stop_line_idx_ip = 0;
  if (use_stuck_stopline) {
    stuck_stop_line_idx_ip = lane_interval_ip_start;
  } else {
    const auto stuck_stop_line_idx_ip_opt =
      getFirstPointInsidePolygon(path_ip, lane_interval_ip, conflicting_area);
    if (!stuck_stop_line_idx_ip_opt) {
      return std::nullopt;
    }
    stuck_stop_line_idx_ip = stuck_stop_line_idx_ip_opt.value();
  }

  const int stop_line_margin_idx_dist = std::ceil(stop_line_margin / ds);
  const int base2front_idx_dist =
    std::ceil(planner_data->vehicle_info_.max_longitudinal_offset_m / ds);
  const size_t insert_idx_ip = static_cast<size_t>(std::max(
    static_cast<int>(stuck_stop_line_idx_ip) - 1 - stop_line_margin_idx_dist - base2front_idx_dist,
    0));
  const auto & insert_point = path_ip.points.at(insert_idx_ip).point.pose;
  return insertPointIndex(insert_point, original_path);
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

IntersectionLanelets getObjectiveLanelets(
  lanelet::LaneletMapConstPtr lanelet_map_ptr, lanelet::routing::RoutingGraphPtr routing_graph_ptr,
  const lanelet::ConstLanelet assigned_lanelet, const lanelet::ConstLanelets & lanelets_on_path,
  const std::set<int> & associative_ids, const double detection_area_length,
  const double occlusion_detection_area_length, const bool consider_wrong_direction_vehicle)
{
  const auto turn_direction = assigned_lanelet.attributeOr("turn_direction", "else");

  // retrieve a stopline associated with a traffic light
  bool has_traffic_light = false;
  if (const auto tl_reg_elems = assigned_lanelet.regulatoryElementsAs<lanelet::TrafficLight>();
      tl_reg_elems.size() != 0) {
    const auto tl_reg_elem = tl_reg_elems.front();
    const auto stop_line_opt = tl_reg_elem->stopLine();
    if (!!stop_line_opt) has_traffic_light = true;
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
  // conflicting lanes is necessary to get stop_line for stuck vehicle
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

  IntersectionLanelets result;
  result.attention_ = std::move(detection_and_preceding_lanelets);
  result.attention_non_preceding_ = std::move(detection_lanelets);
  result.conflicting_ = std::move(conflicting_ex_ego_lanelets);
  result.adjacent_ = planning_utils::getConstLaneletsFromIds(lanelet_map_ptr, associative_ids);
  result.occlusion_attention_ = std::move(occlusion_detection_and_preceding_lanelets);
  // compoundPolygon3d
  result.attention_area_ = getPolygon3dFromLanelets(result.attention_);
  result.attention_non_preceding_area_ = getPolygon3dFromLanelets(result.attention_non_preceding_);
  result.conflicting_area_ = getPolygon3dFromLanelets(result.conflicting_);
  result.adjacent_area_ = getPolygon3dFromLanelets(result.adjacent_);
  result.occlusion_attention_area_ = getPolygon3dFromLanelets(result.occlusion_attention_);
  return result;
}

static std::string getTurnDirection(lanelet::ConstLanelet lane)
{
  return lane.attributeOr("turn_direction", "else");
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

std::vector<DiscretizedLane> generateDetectionLaneDivisions(
  lanelet::ConstLanelets detection_lanelets_all,
  const lanelet::routing::RoutingGraphPtr routing_graph_ptr, const double resolution,
  const double curvature_threshold, const double curvature_calculation_ds)
{
  using lanelet::utils::getCenterlineWithOffset;
  using lanelet::utils::to2D;

  // (0) remove left/right lanelet
  lanelet::ConstLanelets detection_lanelets;
  for (const auto & detection_lanelet : detection_lanelets_all) {
    const auto turn_direction = getTurnDirection(detection_lanelet);
    if (turn_direction.compare("left") == 0 || turn_direction.compare("right") == 0) {
      continue;
    }
    const auto fine_centerline =
      lanelet::utils::generateFineCenterline(detection_lanelet, curvature_calculation_ds);
    const double highest_curvature = getHighestCurvature(fine_centerline);
    if (highest_curvature > curvature_threshold) {
      continue;
    }
    detection_lanelets.push_back(detection_lanelet);
  }

  // (1) tsort detection_lanelets
  // generate adjacency matrix
  // if lanelet1 => lanelet2; then adjacency[lanelet2][lanelet1] = true
  const int n_node = detection_lanelets.size();
  std::vector<std::vector<int>> adjacency(n_node);
  for (int dst = 0; dst < n_node; ++dst) {
    adjacency[dst].resize(n_node);
    for (int src = 0; src < n_node; ++src) {
      adjacency[dst][src] = false;
    }
  }
  std::set<int> detection_lanelet_ids;
  std::unordered_map<int, int> id2ind, ind2id;
  std::unordered_map<int, lanelet::ConstLanelet> id2lanelet;
  int ind = 0;
  for (const auto & detection_lanelet : detection_lanelets) {
    detection_lanelet_ids.insert(detection_lanelet.id());
    const int id = detection_lanelet.id();
    id2ind[id] = ind;
    ind2id[ind] = id;
    id2lanelet[id] = detection_lanelet;
    ind++;
  }
  for (const auto & detection_lanelet : detection_lanelets) {
    const auto & followings = routing_graph_ptr->following(detection_lanelet);
    const int dst = detection_lanelet.id();
    for (const auto & following : followings) {
      if (const int src = following.id();
          detection_lanelet_ids.find(src) != detection_lanelet_ids.end()) {
        adjacency[(id2ind[src])][(id2ind[dst])] = true;
      }
    }
  }
  // terminal node
  std::map<int, std::vector<int>> branches;
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
    branches[(ind2id[src])] = std::vector<int>{};
    auto & branch = branches[(ind2id[src])];
    int node_iter = ind2id[src];
    while (true) {
      const auto & destinations = adjacency[(id2ind[node_iter])];
      // NOTE: assuming detection lanelets have only one previous lanelet
      const auto next = std::find(destinations.begin(), destinations.end(), true);
      if (next == destinations.end()) {
        branch.push_back(node_iter);
        break;
      }
      branch.push_back(node_iter);
      node_iter = ind2id[std::distance(destinations.begin(), next)];
    }
  }
  for (decltype(branches)::iterator it = branches.begin(); it != branches.end(); it++) {
    auto & branch = it->second;
    std::reverse(branch.begin(), branch.end());
  }

  // (2) merge each branch to one lanelet
  // NOTE: somehow bg::area() for merged lanelet does not work, so calculate it here
  std::unordered_map<int, std::pair<lanelet::ConstLanelet, double>> merged_branches;
  for (const auto & [src, branch] : branches) {
    lanelet::Points3d lefts;
    lanelet::Points3d rights;
    double area = 0;
    for (const auto & lane_id : branch) {
      const auto lane = id2lanelet[lane_id];
      for (const auto & left_point : lane.leftBound()) {
        lefts.push_back(lanelet::Point3d(left_point));
      }
      for (const auto & right_point : lane.rightBound()) {
        rights.push_back(lanelet::Point3d(right_point));
      }
      area += bg::area(lane.polygon2d().basicPolygon());
    }
    lanelet::LineString3d right = lanelet::LineString3d(lanelet::InvalId, lefts).invert();
    lanelet::LineString3d left = lanelet::LineString3d(lanelet::InvalId, rights).invert();
    lanelet::Lanelet merged = lanelet::Lanelet(lanelet::InvalId, left, right);
    merged_branches[src] = std::make_pair(merged, area);
  }

  // (3) discretize each merged lanelet
  std::vector<DiscretizedLane> detection_divisions;
  for (const auto & [last_lane_id, branch] : merged_branches) {
    DiscretizedLane detection_division;
    detection_division.lane_id = last_lane_id;
    const auto detection_lanelet = branch.first;
    const double area = branch.second;
    const double length = bg::length(detection_lanelet.centerline());
    const double width = area / length;
    auto & divisions = detection_division.divisions;
    for (int i = 0; i < static_cast<int>(width / resolution); ++i) {
      const double offset = resolution * i - width / 2;
      divisions.push_back(to2D(getCenterlineWithOffset(detection_lanelet, offset, resolution)));
    }
    divisions.push_back(to2D(getCenterlineWithOffset(detection_lanelet, width / 2, resolution)));
    detection_divisions.push_back(detection_division);
  }
  return detection_divisions;
}

std::optional<InterpolatedPathInfo> generateInterpolatedPath(
  const int lane_id, const std::set<int> & associative_lane_ids,
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

bool checkAngleForTargetLanelets(
  const geometry_msgs::msg::Pose & pose, const double longitudinal_velocity,
  const lanelet::ConstLanelets & target_lanelets, const double detection_area_angle_thr,
  const bool consider_wrong_direction_vehicle, const double dist_margin,
  const double parked_vehicle_speed_threshold)
{
  for (const auto & ll : target_lanelets) {
    if (!lanelet::utils::isInLanelet(pose, ll, dist_margin)) {
      continue;
    }
    const double ll_angle = lanelet::utils::getLaneletAngle(ll, pose.position);
    const double pose_angle = tf2::getYaw(pose.orientation);
    const double angle_diff = tier4_autoware_utils::normalizeRadian(ll_angle - pose_angle, -M_PI);
    if (consider_wrong_direction_vehicle) {
      if (std::fabs(angle_diff) > 1.57 || std::fabs(angle_diff) < detection_area_angle_thr) {
        return true;
      }
    } else {
      if (std::fabs(angle_diff) < detection_area_angle_thr) {
        return true;
      }
      // NOTE: sometimes parked vehicle direction is reversed even if its longitudinal velocity is
      // positive
      if (
        std::fabs(longitudinal_velocity) < parked_vehicle_speed_threshold &&
        (std::fabs(angle_diff) < detection_area_angle_thr ||
         (std::fabs(angle_diff + M_PI) < detection_area_angle_thr))) {
        return true;
      }
    }
  }
  return false;
}

void cutPredictPathWithDuration(
  autoware_auto_perception_msgs::msg::PredictedObjects * objects_ptr,
  const rclcpp::Clock::SharedPtr clock, const double time_thr)
{
  const rclcpp::Time current_time = clock->now();
  for (auto & object : objects_ptr->objects) {                         // each objects
    for (auto & predicted_path : object.kinematics.predicted_paths) {  // each predicted paths
      const auto origin_path = predicted_path;
      predicted_path.path.clear();

      for (size_t k = 0; k < origin_path.path.size(); ++k) {  // each path points
        const auto & predicted_pose = origin_path.path.at(k);
        const auto predicted_time =
          rclcpp::Time(objects_ptr->header.stamp) +
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
  const std::shared_ptr<const PlannerData> & planner_data, const std::set<int> & associative_ids,
  const size_t closest_idx, const size_t last_intersection_stop_line_candidate_idx,
  const double time_delay, const double intersection_velocity, const double minimum_ego_velocity,
  const bool use_upstream_velocity, const double minimum_upstream_velocity)
{
  double dist_sum = 0.0;
  int assigned_lane_found = false;

  // crop intersection part of the path, and set the reference velocity to intersection_velocity
  // for ego's ttc
  PathWithLaneId reference_path;
  for (size_t i = closest_idx; i < path.points.size(); ++i) {
    auto reference_point = path.points.at(i);
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

  // apply smoother to reference velocity
  PathWithLaneId smoothed_reference_path = reference_path;
  smoothPath(reference_path, smoothed_reference_path, planner_data);

  // calculate when ego is going to reach each (interpolated) points on the path
  TimeDistanceArray time_distance_array{};
  dist_sum = 0.0;
  double passing_time = time_delay;
  time_distance_array.emplace_back(passing_time, dist_sum);
  for (size_t i = 1; i < smoothed_reference_path.points.size(); ++i) {
    const auto & p1 = smoothed_reference_path.points.at(i - 1);
    const auto & p2 = smoothed_reference_path.points.at(i);

    const double dist = tier4_autoware_utils::calcDistance2d(p1, p2);
    dist_sum += dist;

    // use average velocity between p1 and p2
    const double average_velocity =
      (p1.point.longitudinal_velocity_mps + p2.point.longitudinal_velocity_mps) / 2.0;
    const double minimum_ego_velocity_division =
      (use_upstream_velocity && i > last_intersection_stop_line_candidate_idx)
        ? minimum_upstream_velocity /* to avoid null division */
        : minimum_ego_velocity;
    const double passing_velocity =
      std::max<double>(minimum_ego_velocity_division, average_velocity);
    passing_time += (dist / passing_velocity);

    time_distance_array.emplace_back(passing_time, dist_sum);
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
  const tier4_autoware_utils::LinearRing2d & footprint)
{
  is_prioritized_ = is_prioritized;
  // find the first conflicting/detection area polygon intersecting the path
  if (!first_conflicting_area_) {
    auto first =
      getFirstPointInsidePolygonsByFootprint(conflicting_area_, interpolated_path_info, footprint);
    if (first) {
      first_conflicting_area_ = first.value().second;
    }
  }
  if (!first_attention_area_) {
    auto first =
      getFirstPointInsidePolygonsByFootprint(attention_area_, interpolated_path_info, footprint);
    if (first) {
      first_attention_area_ = first.value().second;
    }
  }
}

static lanelet::ConstLanelets getPrevLanelets(
  const lanelet::ConstLanelets & lanelets_on_path, const std::set<int> & associative_ids)
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

std::optional<PathLanelets> generatePathLanelets(
  const lanelet::ConstLanelets & lanelets_on_path,
  const util::InterpolatedPathInfo & interpolated_path_info, const std::set<int> & associative_ids,
  const lanelet::CompoundPolygon3d & first_conflicting_area,
  const std::vector<lanelet::CompoundPolygon3d> & conflicting_areas,
  const std::optional<lanelet::CompoundPolygon3d> & first_attention_area,
  const std::vector<lanelet::CompoundPolygon3d> & attention_areas, const size_t closest_idx,
  const double width)
{
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
      planning_utils::generatePathLanelet(path, assigned_lane_start, closest_idx, width));
  }

  // ego_or_entry2exit
  const auto ego_or_entry_start = std::max(closest_idx, assigned_lane_start);
  path_lanelets.ego_or_entry2exit =
    planning_utils::generatePathLanelet(path, ego_or_entry_start, assigned_lane_end, width);
  path_lanelets.all.push_back(path_lanelets.ego_or_entry2exit);

  // next
  if (assigned_lane_end < path.points.size() - 1) {
    const int next_id = path.points.at(assigned_lane_end).lane_ids.at(0);
    const auto next_lane_interval_opt = findLaneIdsInterval(path, {next_id});
    if (next_lane_interval_opt) {
      const auto [next_start, next_end] = next_lane_interval_opt.value();
      path_lanelets.next = planning_utils::generatePathLanelet(path, next_start, next_end, width);
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
    lanelet::ConstLanelet conflicting_interval = planning_utils::generatePathLanelet(
      path, first_inside_conflicting_idx, last_inside_conflicting_idx, width);
    path_lanelets.conflicting_interval_and_remaining.push_back(std::move(conflicting_interval));
    if (last_inside_conflicting_idx < assigned_lane_end) {
      lanelet::ConstLanelet remaining_interval = planning_utils::generatePathLanelet(
        path, last_inside_conflicting_idx, assigned_lane_end, width);
      path_lanelets.conflicting_interval_and_remaining.push_back(std::move(remaining_interval));
    }
  }
  return path_lanelets;
}

}  // namespace util
}  // namespace behavior_velocity_planner
