// Copyright 2024 Tier IV, Inc.
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

#include "scene_intersection.hpp"
#include "util.hpp"

#include <behavior_velocity_planner_common/utilization/boost_geometry_helper.hpp>  // for to_bg2d
#include <behavior_velocity_planner_common/utilization/util.hpp>  // for planning_utils::
#include <interpolation/spline_interpolation_points_2d.hpp>
#include <lanelet2_extension/regulatory_elements/road_marking.hpp>  // for lanelet::autoware::RoadMarking
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <motion_utils/trajectory/trajectory.hpp>

#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/within.hpp>

#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/Point.h>

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

namespace
{
namespace bg = boost::geometry;

lanelet::ConstLanelets getPrevLanelets(
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
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const size_t start_idx,
  const size_t end_idx, const double width, const double interval)
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

std::optional<std::pair<size_t, const lanelet::CompoundPolygon3d &>> getFirstPointInsidePolygons(
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
        is_in_lanelet = bg::within(behavior_velocity_planner::to_bg2d(p), polygon_2d);
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
        is_in_lanelet = bg::within(behavior_velocity_planner::to_bg2d(p), polygon_2d);
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

}  // namespace

namespace behavior_velocity_planner
{
namespace bg = boost::geometry;

intersection::Result<IntersectionModule::BasicData, intersection::Indecisive>
IntersectionModule::prepareIntersectionData(const bool is_prioritized, PathWithLaneId * path)
{
  using intersection::Result;

  const auto lanelet_map_ptr = planner_data_->route_handler_->getLaneletMapPtr();
  const auto routing_graph_ptr = planner_data_->route_handler_->getRoutingGraphPtr();
  const auto & assigned_lanelet = lanelet_map_ptr->laneletLayer.get(lane_id_);
  const double baselink2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;
  const auto footprint = planner_data_->vehicle_info_.createFootprint(0.0, 0.0);
  const auto & current_pose = planner_data_->current_odometry->pose;

  // spline interpolation
  const auto interpolated_path_info_opt = util::generateInterpolatedPath(
    lane_id_, associative_ids_, *path, planner_param_.common.path_interpolation_ds, logger_);
  if (!interpolated_path_info_opt) {
    return Result<IntersectionModule::BasicData, intersection::Indecisive>::make_err(
      intersection::Indecisive{"splineInterpolate failed"});
  }

  const auto & interpolated_path_info = interpolated_path_info_opt.value();
  if (!interpolated_path_info.lane_id_interval) {
    return Result<IntersectionModule::BasicData, intersection::Indecisive>::make_err(
      intersection::Indecisive{
        "Path has no interval on intersection lane " + std::to_string(lane_id_)});
  }

  // cache intersection lane information because it is invariant
  if (!intersection_lanelets_) {
    intersection_lanelets_ =
      generateObjectiveLanelets(lanelet_map_ptr, routing_graph_ptr, assigned_lanelet);
  }
  auto & intersection_lanelets = intersection_lanelets_.value();

  // at the very first time of regisTration of this module, the path may not be conflicting with the
  // attention area, so update() is called to update the internal data as well as traffic light info
  intersection_lanelets.update(
    is_prioritized, interpolated_path_info, footprint, baselink2front, routing_graph_ptr);

  const auto & conflicting_lanelets = intersection_lanelets.conflicting();
  const auto & first_conflicting_area_opt = intersection_lanelets.first_conflicting_area();
  const auto & first_conflicting_lane_opt = intersection_lanelets.first_conflicting_lane();
  if (conflicting_lanelets.empty() || !first_conflicting_area_opt || !first_conflicting_lane_opt) {
    // this is abnormal
    return Result<IntersectionModule::BasicData, intersection::Indecisive>::make_err(
      intersection::Indecisive{"conflicting area is empty"});
  }
  const auto & first_conflicting_lane = first_conflicting_lane_opt.value();
  const auto & first_conflicting_area = first_conflicting_area_opt.value();
  const auto & second_attention_area_opt = intersection_lanelets.second_attention_area();

  // generate all stop line candidates
  // see the doc for struct IntersectionStopLines
  /// even if the attention area is null, stuck vehicle stop line needs to be generated from
  /// conflicting lanes
  const auto & dummy_first_attention_lane = intersection_lanelets.first_attention_lane()
                                              ? intersection_lanelets.first_attention_lane().value()
                                              : first_conflicting_lane;

  const auto intersection_stoplines_opt = generateIntersectionStopLines(
    assigned_lanelet, first_conflicting_area, dummy_first_attention_lane, second_attention_area_opt,
    interpolated_path_info, path);
  if (!intersection_stoplines_opt) {
    return Result<IntersectionModule::BasicData, intersection::Indecisive>::make_err(
      intersection::Indecisive{"failed to generate intersection_stoplines"});
  }
  const auto & intersection_stoplines = intersection_stoplines_opt.value();
  const auto closest_idx = intersection_stoplines.closest_idx;

  const auto & first_attention_area_opt = intersection_lanelets.first_attention_area();
  const auto & conflicting_area = intersection_lanelets.conflicting_area();
  const auto lanelets_on_path =
    planning_utils::getLaneletsOnPath(*path, lanelet_map_ptr, current_pose);
  // see the doc for struct PathLanelets
  const auto path_lanelets_opt = generatePathLanelets(
    lanelets_on_path, interpolated_path_info, first_conflicting_area, conflicting_area,
    first_attention_area_opt, intersection_lanelets.attention_area(), closest_idx);
  if (!path_lanelets_opt.has_value()) {
    return Result<IntersectionModule::BasicData, intersection::Indecisive>::make_err(
      intersection::Indecisive{"failed to generate PathLanelets"});
  }
  const auto & path_lanelets = path_lanelets_opt.value();

  if (!occlusion_attention_divisions_) {
    occlusion_attention_divisions_ = generateDetectionLaneDivisions(
      intersection_lanelets.occlusion_attention(), routing_graph_ptr,
      planner_data_->occupancy_grid->info.resolution);
  }

  return Result<IntersectionModule::BasicData, intersection::Indecisive>::make_ok(
    BasicData{interpolated_path_info, intersection_stoplines, path_lanelets});
}

std::optional<size_t> IntersectionModule::getStopLineIndexFromMap(
  const intersection::InterpolatedPathInfo & interpolated_path_info,
  lanelet::ConstLanelet assigned_lanelet)
{
  const auto & path = interpolated_path_info.path;
  const auto & lane_interval = interpolated_path_info.lane_id_interval.value();

  const auto road_markings =
    assigned_lanelet.regulatoryElementsAs<lanelet::autoware::RoadMarking>();
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
    planning_utils::extendLine(p_start, p_end, planner_data_->stop_line_extend_length);

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
    path.points, stop_point_from_map, planner_data_->ego_nearest_dist_threshold,
    planner_data_->ego_nearest_yaw_threshold);
}

std::optional<intersection::IntersectionStopLines>
IntersectionModule::generateIntersectionStopLines(
  lanelet::ConstLanelet assigned_lanelet, const lanelet::CompoundPolygon3d & first_conflicting_area,
  const lanelet::ConstLanelet & first_attention_lane,
  const std::optional<lanelet::CompoundPolygon3d> & second_attention_area_opt,
  const intersection::InterpolatedPathInfo & interpolated_path_info,
  autoware_auto_planning_msgs::msg::PathWithLaneId * original_path)
{
  const bool use_stuck_stopline = planner_param_.stuck_vehicle.use_stuck_stopline;
  const double stopline_margin = planner_param_.common.default_stopline_margin;
  const double max_accel = planner_param_.common.max_accel;
  const double max_jerk = planner_param_.common.max_jerk;
  const double delay_response_time = planner_param_.common.delay_response_time;
  const double peeking_offset = planner_param_.occlusion.peeking_offset;

  const auto first_attention_area = first_attention_lane.polygon3d();
  const auto first_attention_lane_centerline = first_attention_lane.centerline2d();
  const auto & path_ip = interpolated_path_info.path;
  const double ds = interpolated_path_info.ds;
  const auto & lane_interval_ip = interpolated_path_info.lane_id_interval.value();
  const double baselink2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;

  const int stopline_margin_idx_dist = std::ceil(stopline_margin / ds);
  const int base2front_idx_dist = std::ceil(baselink2front / ds);

  // find the index of the first point whose vehicle footprint on it intersects with attention_area
  const auto local_footprint = planner_data_->vehicle_info_.createFootprint(0.0, 0.0);
  const std::optional<size_t> first_footprint_inside_1st_attention_ip_opt =
    util::getFirstPointInsidePolygonByFootprint(
      first_attention_area, interpolated_path_info, local_footprint, baselink2front);
  if (!first_footprint_inside_1st_attention_ip_opt) {
    return std::nullopt;
  }
  const auto first_footprint_inside_1st_attention_ip =
    first_footprint_inside_1st_attention_ip_opt.value();

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
  if (const auto map_stop_idx_ip =
        getStopLineIndexFromMap(interpolated_path_info, assigned_lanelet);
      map_stop_idx_ip) {
    stop_idx_ip_int = static_cast<int>(map_stop_idx_ip.value()) - base2front_idx_dist;
  }
  if (stop_idx_ip_int < 0) {
    stop_idx_ip_int = first_footprint_inside_1st_attention_ip - stopline_margin_idx_dist;
  }
  if (stop_idx_ip_int < 0) {
    default_stopline_valid = false;
  }
  const auto default_stopline_ip = stop_idx_ip_int >= 0 ? static_cast<size_t>(stop_idx_ip_int) : 0;

  // (2) ego front stop line position on interpolated path
  const geometry_msgs::msg::Pose & current_pose = planner_data_->current_odometry->pose;
  const auto closest_idx_ip = motion_utils::findFirstNearestIndexWithSoftConstraints(
    path_ip.points, current_pose, planner_data_->ego_nearest_dist_threshold,
    planner_data_->ego_nearest_yaw_threshold);

  // (3) occlusion peeking stop line position on interpolated path
  int occlusion_peeking_line_ip_int = static_cast<int>(default_stopline_ip);
  bool occlusion_peeking_line_valid = true;
  // NOTE: if footprints[0] is already inside the attention area, invalid
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
      first_footprint_inside_1st_attention_ip + std::ceil(peeking_offset / ds);
  }
  const auto occlusion_peeking_line_ip = static_cast<size_t>(
    std::clamp<int>(occlusion_peeking_line_ip_int, 0, static_cast<int>(path_ip.points.size()) - 1));

  // (4) first attention stopline position on interpolated path
  const auto first_attention_stopline_ip = first_footprint_inside_1st_attention_ip;
  const bool first_attention_stopline_valid = true;

  // (5) 1st pass judge line position on interpolated path
  const double velocity = planner_data_->current_velocity->twist.linear.x;
  const double acceleration = planner_data_->current_acceleration->accel.accel.linear.x;
  const double braking_dist = planning_utils::calcJudgeLineDistWithJerkLimit(
    velocity, acceleration, max_accel, max_jerk, delay_response_time);
  int first_pass_judge_ip_int =
    static_cast<int>(first_footprint_inside_1st_attention_ip) - std::ceil(braking_dist / ds);
  const auto first_pass_judge_line_ip = static_cast<size_t>(
    std::clamp<int>(first_pass_judge_ip_int, 0, static_cast<int>(path_ip.points.size()) - 1));
  const auto occlusion_wo_tl_pass_judge_line_ip = static_cast<size_t>(std::max<int>(
    0, static_cast<int>(first_footprint_attention_centerline_ip) - std::ceil(braking_dist / ds)));

  // (6) stuck vehicle stopline position on interpolated path
  int stuck_stopline_ip_int = 0;
  bool stuck_stopline_valid = true;
  if (use_stuck_stopline) {
    // NOTE: when ego vehicle is approaching attention area and already passed
    // first_conflicting_area, this could be null.
    const auto stuck_stopline_idx_ip_opt = util::getFirstPointInsidePolygonByFootprint(
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

  // (7) second attention stopline position on interpolated path
  int second_attention_stopline_ip_int = -1;
  bool second_attention_stopline_valid = false;
  if (second_attention_area_opt) {
    const auto & second_attention_area = second_attention_area_opt.value();
    std::optional<size_t> first_footprint_inside_2nd_attention_ip_opt =
      util::getFirstPointInsidePolygonByFootprint(
        second_attention_area, interpolated_path_info, local_footprint, baselink2front);
    if (first_footprint_inside_2nd_attention_ip_opt) {
      second_attention_stopline_ip_int = first_footprint_inside_2nd_attention_ip_opt.value();
      second_attention_stopline_valid = true;
    }
  }
  const auto second_attention_stopline_ip =
    second_attention_stopline_ip_int >= 0 ? static_cast<size_t>(second_attention_stopline_ip_int)
                                          : 0;

  // (8) second pass judge line position on interpolated path. It is the same as first pass judge
  // line if second_attention_lane is null
  int second_pass_judge_ip_int = occlusion_wo_tl_pass_judge_line_ip;
  const auto second_pass_judge_line_ip =
    second_attention_area_opt ? static_cast<size_t>(std::max<int>(second_pass_judge_ip_int, 0))
                              : first_pass_judge_line_ip;

  struct IntersectionStopLinesTemp
  {
    size_t closest_idx{0};
    size_t stuck_stopline{0};
    size_t default_stopline{0};
    size_t first_attention_stopline{0};
    size_t second_attention_stopline{0};
    size_t occlusion_peeking_stopline{0};
    size_t first_pass_judge_line{0};
    size_t second_pass_judge_line{0};
    size_t occlusion_wo_tl_pass_judge_line{0};
  };

  IntersectionStopLinesTemp intersection_stoplines_temp;
  std::list<std::pair<const size_t *, size_t *>> stoplines = {
    {&closest_idx_ip, &intersection_stoplines_temp.closest_idx},
    {&stuck_stopline_ip, &intersection_stoplines_temp.stuck_stopline},
    {&default_stopline_ip, &intersection_stoplines_temp.default_stopline},
    {&first_attention_stopline_ip, &intersection_stoplines_temp.first_attention_stopline},
    {&second_attention_stopline_ip, &intersection_stoplines_temp.second_attention_stopline},
    {&occlusion_peeking_line_ip, &intersection_stoplines_temp.occlusion_peeking_stopline},
    {&first_pass_judge_line_ip, &intersection_stoplines_temp.first_pass_judge_line},
    {&second_pass_judge_line_ip, &intersection_stoplines_temp.second_pass_judge_line},
    {&occlusion_wo_tl_pass_judge_line_ip,
     &intersection_stoplines_temp.occlusion_wo_tl_pass_judge_line}};
  stoplines.sort(
    [](const auto & it1, const auto & it2) { return *(std::get<0>(it1)) < *(std::get<0>(it2)); });
  for (const auto & [stop_idx_ip, stop_idx] : stoplines) {
    const auto & insert_point = path_ip.points.at(*stop_idx_ip).point.pose;
    const auto insert_idx = util::insertPointIndex(
      insert_point, original_path, planner_data_->ego_nearest_dist_threshold,
      planner_data_->ego_nearest_yaw_threshold);
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

  intersection::IntersectionStopLines intersection_stoplines;
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
  if (second_attention_stopline_valid) {
    intersection_stoplines.second_attention_stopline =
      intersection_stoplines_temp.second_attention_stopline;
  }
  if (occlusion_peeking_line_valid) {
    intersection_stoplines.occlusion_peeking_stopline =
      intersection_stoplines_temp.occlusion_peeking_stopline;
  }
  intersection_stoplines.first_pass_judge_line = intersection_stoplines_temp.first_pass_judge_line;
  intersection_stoplines.second_pass_judge_line =
    intersection_stoplines_temp.second_pass_judge_line;
  intersection_stoplines.occlusion_wo_tl_pass_judge_line =
    intersection_stoplines_temp.occlusion_wo_tl_pass_judge_line;
  return intersection_stoplines;
}

intersection::IntersectionLanelets IntersectionModule::generateObjectiveLanelets(
  lanelet::LaneletMapConstPtr lanelet_map_ptr, lanelet::routing::RoutingGraphPtr routing_graph_ptr,
  const lanelet::ConstLanelet assigned_lanelet)
{
  const double detection_area_length = planner_param_.common.attention_area_length;
  const double occlusion_detection_area_length =
    planner_param_.occlusion.occlusion_attention_area_length;
  const bool consider_wrong_direction_vehicle =
    planner_param_.collision_detection.consider_wrong_direction_vehicle;

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
  lanelet::ConstLanelets ego_lanelets{};
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
  if (turn_direction_ == std::string("straight") && has_traffic_light) {
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
    const std::string turn_direction = ll.attributeOr("turn_direction", "else");
    if (turn_direction == "left" || turn_direction == "right") {
      continue;
    }
    occlusion_detection_and_preceding_lanelets_wo_turn_direction.push_back(ll);
  }

  auto [attention_lanelets, original_attention_lanelet_sequences] =
    util::mergeLaneletsByTopologicalSort(detection_and_preceding_lanelets, routing_graph_ptr);

  intersection::IntersectionLanelets result;
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
  result.adjacent_ = planning_utils::getConstLaneletsFromIds(lanelet_map_ptr, associative_ids_);
  // NOTE: occlusion_attention is not inverted here
  // TODO(Mamoru Sobue): apply mergeLaneletsByTopologicalSort for occlusion lanelets as well and
  // then trim part of them based on curvature threshold
  result.occlusion_attention_ =
    std::move(occlusion_detection_and_preceding_lanelets_wo_turn_direction);

  // NOTE: to properly update(), each element in conflicting_/conflicting_area_,
  // attention_non_preceding_/attention_non_preceding_area_ need to be matched
  result.attention_area_ = util::getPolygon3dFromLanelets(result.attention_);
  result.attention_non_preceding_area_ =
    util::getPolygon3dFromLanelets(result.attention_non_preceding_);
  result.conflicting_area_ = util::getPolygon3dFromLanelets(result.conflicting_);
  result.adjacent_area_ = util::getPolygon3dFromLanelets(result.adjacent_);
  result.occlusion_attention_area_ = util::getPolygon3dFromLanelets(result.occlusion_attention_);
  return result;
}

std::optional<intersection::PathLanelets> IntersectionModule::generatePathLanelets(
  const lanelet::ConstLanelets & lanelets_on_path,
  const intersection::InterpolatedPathInfo & interpolated_path_info,
  const lanelet::CompoundPolygon3d & first_conflicting_area,
  const std::vector<lanelet::CompoundPolygon3d> & conflicting_areas,
  const std::optional<lanelet::CompoundPolygon3d> & first_attention_area,
  const std::vector<lanelet::CompoundPolygon3d> & attention_areas, const size_t closest_idx)
{
  const double width = planner_data_->vehicle_info_.vehicle_width_m;
  static constexpr double path_lanelet_interval = 1.5;

  const auto & assigned_lane_interval_opt = interpolated_path_info.lane_id_interval;
  if (!assigned_lane_interval_opt) {
    return std::nullopt;
  }
  const auto assigned_lane_interval = assigned_lane_interval_opt.value();
  const auto & path = interpolated_path_info.path;

  intersection::PathLanelets path_lanelets;
  // prev
  path_lanelets.prev = ::getPrevLanelets(lanelets_on_path, associative_ids_);
  path_lanelets.all = path_lanelets.prev;

  // entry2ego if exist
  const auto [assigned_lane_start, assigned_lane_end] = assigned_lane_interval;
  if (closest_idx > assigned_lane_start) {
    path_lanelets.all.push_back(
      ::generatePathLanelet(path, assigned_lane_start, closest_idx, width, path_lanelet_interval));
  }

  // ego_or_entry2exit
  const auto ego_or_entry_start = std::max(closest_idx, assigned_lane_start);
  path_lanelets.ego_or_entry2exit =
    generatePathLanelet(path, ego_or_entry_start, assigned_lane_end, width, path_lanelet_interval);
  path_lanelets.all.push_back(path_lanelets.ego_or_entry2exit);

  // next
  if (assigned_lane_end < path.points.size() - 1) {
    const int next_id = path.points.at(assigned_lane_end).lane_ids.at(0);
    const auto next_lane_interval_opt = util::findLaneIdsInterval(path, {next_id});
    if (next_lane_interval_opt) {
      const auto [next_start, next_end] = next_lane_interval_opt.value();
      path_lanelets.next =
        generatePathLanelet(path, next_start, next_end, width, path_lanelet_interval);
      path_lanelets.all.push_back(path_lanelets.next.value());
    }
  }

  const auto first_inside_conflicting_idx_opt =
    first_attention_area.has_value()
      ? util::getFirstPointInsidePolygon(path, assigned_lane_interval, first_attention_area.value())
      : util::getFirstPointInsidePolygon(path, assigned_lane_interval, first_conflicting_area);
  const auto last_inside_conflicting_idx_opt =
    first_attention_area.has_value()
      ? ::getFirstPointInsidePolygons(path, assigned_lane_interval, attention_areas, false)
      : ::getFirstPointInsidePolygons(path, assigned_lane_interval, conflicting_areas, false);
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

std::vector<lanelet::ConstLineString3d> IntersectionModule::generateDetectionLaneDivisions(
  lanelet::ConstLanelets detection_lanelets_all,
  const lanelet::routing::RoutingGraphPtr routing_graph_ptr, const double resolution)
{
  const double curvature_threshold =
    planner_param_.occlusion.attention_lane_crop_curvature_threshold;
  const double curvature_calculation_ds =
    planner_param_.occlusion.attention_lane_curvature_calculation_ds;

  using lanelet::utils::getCenterlineWithOffset;

  // (0) remove left/right lanelet
  lanelet::ConstLanelets detection_lanelets;
  for (const auto & detection_lanelet : detection_lanelets_all) {
    // TODO(Mamoru Sobue): instead of ignoring, only trim straight part of lanelet
    const auto fine_centerline =
      lanelet::utils::generateFineCenterline(detection_lanelet, curvature_calculation_ds);
    const double highest_curvature = ::getHighestCurvature(fine_centerline);
    if (highest_curvature > curvature_threshold) {
      continue;
    }
    detection_lanelets.push_back(detection_lanelet);
  }

  // (1) tsort detection_lanelets
  const auto [merged_detection_lanelets, originals] =
    util::mergeLaneletsByTopologicalSort(detection_lanelets, routing_graph_ptr);

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

}  // namespace behavior_velocity_planner
