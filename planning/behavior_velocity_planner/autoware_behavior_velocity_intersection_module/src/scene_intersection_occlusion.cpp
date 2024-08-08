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

#include <autoware/universe_utils/geometry/boost_polygon_utils.hpp>  // for toPolygon2d
#include <opencv2/imgproc.hpp>

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/intersection.hpp>

#include <lanelet2_core/geometry/Polygon.h>

#include <tuple>

namespace autoware::behavior_velocity_planner
{
namespace bg = boost::geometry;

std::tuple<
  IntersectionModule::OcclusionType, bool /* module detection with margin */,
  bool /* reconciled occlusion disapproval */>
IntersectionModule::getOcclusionStatus(
  const TrafficPrioritizedLevel & traffic_prioritized_level,
  const InterpolatedPathInfo & interpolated_path_info)
{
  const auto & intersection_lanelets = intersection_lanelets_.value();
  const auto & occlusion_attention_lanelets = intersection_lanelets.occlusion_attention();

  // ==========================================================================================
  // for the convenience of Psim user, this module ignores occlusion if there has not been any
  // information published for the associated traffic light even if occlusion.enable is true,
  // and only runs collision checking on that intersection lane.
  //
  // this is because Psim-users/scenario-files do not set traffic light information perfectly
  // most of the times, and they just set bare minimum traffic information only for traffic lights
  // they are interested in or want to test.
  //
  // no_tl_info_ever variable is defined for that purpose. if there has been any
  // information published for the associated traffic light in the real world through perception/V2I
  // or in the simulation, then it should be kept in last_tl_valid_observation_ and this variable
  // becomes false
  // ==========================================================================================
  const bool no_tl_info_ever = (has_traffic_light_ && !last_tl_valid_observation_.has_value());
  const bool is_amber_or_red_or_no_tl_info_ever =
    (traffic_prioritized_level == TrafficPrioritizedLevel::PARTIALLY_PRIORITIZED) ||
    (traffic_prioritized_level == TrafficPrioritizedLevel::FULLY_PRIORITIZED) || no_tl_info_ever;
  // check occlusion on detection lane
  auto occlusion_status =
    (planner_param_.occlusion.enable && !occlusion_attention_lanelets.empty() &&
     !is_amber_or_red_or_no_tl_info_ever)
      ? detectOcclusion(interpolated_path_info)
      : NotOccluded{};

  // ==========================================================================================
  // if the traffic light changed from green to yellow/red, hysteresis time for occlusion is
  // unnecessary
  // ==========================================================================================
  const auto transition_to_prioritized =
    (previous_prioritized_level_ == TrafficPrioritizedLevel::NOT_PRIORITIZED &&
     traffic_prioritized_level != TrafficPrioritizedLevel::NOT_PRIORITIZED);
  if (transition_to_prioritized) {
    occlusion_stop_state_machine_.setState(StateMachine::State::GO);
  } else {
    occlusion_stop_state_machine_.setStateWithMarginTime(
      std::holds_alternative<NotOccluded>(occlusion_status) ? StateMachine::State::GO
                                                            : StateMachine::STOP,
      logger_.get_child("occlusion_stop"), *clock_);
  }

  const bool is_occlusion_cleared_with_margin =
    (occlusion_stop_state_machine_.getState() == StateMachine::State::GO);  // module's detection
  // distinguish if ego detected occlusion or RTC detects occlusion
  const bool ext_occlusion_requested =
    (is_occlusion_cleared_with_margin && !occlusion_activated_);  // RTC's detection
  if (ext_occlusion_requested) {
    occlusion_status = RTCOccluded{};
  }
  const bool is_occlusion_state =
    (!is_occlusion_cleared_with_margin || ext_occlusion_requested);  // including approval
  if (is_occlusion_state && std::holds_alternative<NotOccluded>(occlusion_status)) {
    occlusion_status = prev_occlusion_status_;
  } else {
    prev_occlusion_status_ = occlusion_status;
  }
  return {occlusion_status, is_occlusion_cleared_with_margin, is_occlusion_state};
}

IntersectionModule::OcclusionType IntersectionModule::detectOcclusion(
  const InterpolatedPathInfo & interpolated_path_info) const
{
  const auto & intersection_lanelets = intersection_lanelets_.value();
  const auto & adjacent_lanelets = intersection_lanelets.adjacent();
  const auto & attention_areas = intersection_lanelets.occlusion_attention_area();
  const auto first_attention_area = intersection_lanelets.first_attention_area().value();
  const auto & lane_divisions = occlusion_attention_divisions_.value();

  const auto & occ_grid = *planner_data_->occupancy_grid;
  const auto & current_pose = planner_data_->current_odometry->pose;
  const double occlusion_dist_thr = planner_param_.occlusion.occlusion_required_clearance_distance;

  const auto & path_ip = interpolated_path_info.path;
  const auto & lane_interval_ip = interpolated_path_info.lane_id_interval.value();

  const auto first_attention_area_idx =
    util::getFirstPointInsidePolygon(path_ip, lane_interval_ip, first_attention_area);
  if (!first_attention_area_idx) {
    return NotOccluded{};
  }

  const auto first_inside_attention_idx_ip_opt =
    util::getFirstPointInsidePolygon(path_ip, lane_interval_ip, first_attention_area);
  const std::pair<size_t, size_t> lane_attention_interval_ip =
    first_inside_attention_idx_ip_opt
      ? std::make_pair(first_inside_attention_idx_ip_opt.value(), std::get<1>(lane_interval_ip))
      : lane_interval_ip;
  const auto [lane_start_idx, lane_end_idx] = lane_attention_interval_ip;

  const int width = occ_grid.info.width;
  const int height = occ_grid.info.height;
  const double resolution = occ_grid.info.resolution;
  const auto & origin = occ_grid.info.origin.position;
  auto coord2index = [&](const double x, const double y) {
    const int idx_x = (x - origin.x) / resolution;
    const int idx_y = (y - origin.y) / resolution;
    if (idx_x < 0 || idx_x >= width) return std::make_tuple(false, -1, -1);
    if (idx_y < 0 || idx_y >= height) return std::make_tuple(false, -1, -1);
    return std::make_tuple(true, idx_x, idx_y);
  };

  Polygon2d grid_poly;
  grid_poly.outer().emplace_back(origin.x, origin.y);
  grid_poly.outer().emplace_back(origin.x + (width - 1) * resolution, origin.y);
  grid_poly.outer().emplace_back(
    origin.x + (width - 1) * resolution, origin.y + (height - 1) * resolution);
  grid_poly.outer().emplace_back(origin.x, origin.y + (height - 1) * resolution);
  grid_poly.outer().emplace_back(origin.x, origin.y);
  bg::correct(grid_poly);

  auto findCommonCvPolygons =
    [&](const auto & area2d, std::vector<std::vector<cv::Point>> & cv_polygons) -> void {
    autoware::universe_utils::Polygon2d area2d_poly;
    for (const auto & p : area2d) {
      area2d_poly.outer().emplace_back(p.x(), p.y());
    }
    area2d_poly.outer().push_back(area2d_poly.outer().front());
    bg::correct(area2d_poly);
    std::vector<Polygon2d> common_areas;
    bg::intersection(area2d_poly, grid_poly, common_areas);
    if (common_areas.empty()) {
      return;
    }
    for (size_t i = 0; i < common_areas.size(); ++i) {
      common_areas[i].outer().push_back(common_areas[i].outer().front());
      bg::correct(common_areas[i]);
    }
    for (const auto & common_area : common_areas) {
      std::vector<cv::Point> cv_polygon;
      for (const auto & p : common_area.outer()) {
        const int idx_x = static_cast<int>((p.x() - origin.x) / resolution);
        const int idx_y = static_cast<int>((p.y() - origin.y) / resolution);
        cv_polygon.emplace_back(idx_x, height - 1 - idx_y);
      }
      cv_polygons.push_back(cv_polygon);
    }
  };

  // (1) prepare detection area mask
  // attention: 255
  // non-attention: 0
  // NOTE: interesting area is set to 255 for later masking
  cv::Mat attention_mask(width, height, CV_8UC1, cv::Scalar(0));
  std::vector<std::vector<cv::Point>> attention_area_cv_polygons;
  for (const auto & attention_area : attention_areas) {
    const auto area2d = lanelet::utils::to2D(attention_area);
    findCommonCvPolygons(area2d, attention_area_cv_polygons);
  }
  for (const auto & poly : attention_area_cv_polygons) {
    cv::fillPoly(attention_mask, poly, cv::Scalar(255), cv::LINE_AA);
  }
  // (1.1)
  // reset adjacent_lanelets area to 0 on attention_mask
  std::vector<std::vector<cv::Point>> adjacent_lane_cv_polygons;
  for (const auto & adjacent_lanelet : adjacent_lanelets) {
    const auto area2d = adjacent_lanelet.polygon2d().basicPolygon();
    findCommonCvPolygons(area2d, adjacent_lane_cv_polygons);
  }
  for (const auto & poly : adjacent_lane_cv_polygons) {
    cv::fillPoly(attention_mask, poly, cv::Scalar(0), cv::LINE_AA);
  }

  // (2) prepare unknown mask
  // In OpenCV the pixel at (X=x, Y=y) (with left-upper origin) is accessed by img[y, x]
  // unknown: 255
  // not-unknown: 0
  cv::Mat unknown_mask_raw(width, height, CV_8UC1, cv::Scalar(0));
  cv::Mat unknown_mask(width, height, CV_8UC1, cv::Scalar(0));
  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      const int idx = y * width + x;
      const unsigned char intensity = occ_grid.data.at(idx);
      if (
        planner_param_.occlusion.free_space_max <= intensity &&
        intensity < planner_param_.occlusion.occupied_min) {
        unknown_mask_raw.at<unsigned char>(height - 1 - y, x) = 255;
      }
    }
  }
  // (2.1) apply morphologyEx
  const int morph_size = static_cast<int>(planner_param_.occlusion.denoise_kernel / resolution);
  cv::morphologyEx(
    unknown_mask_raw, unknown_mask, cv::MORPH_OPEN,
    cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morph_size, morph_size)));

  // (3) occlusion mask
  static constexpr unsigned char OCCLUDED = 255;
  static constexpr unsigned char BLOCKED = 127;
  cv::Mat occlusion_mask(width, height, CV_8UC1, cv::Scalar(0));
  cv::bitwise_and(attention_mask, unknown_mask, occlusion_mask);
  // re-use attention_mask
  attention_mask = cv::Mat(width, height, CV_8UC1, cv::Scalar(0));
  // (3.1) draw all cells on attention_mask behind blocking vehicles as not occluded
  const auto & blocking_attention_objects = object_info_manager_.parkedObjects();
  for (const auto & blocking_attention_object_info : blocking_attention_objects) {
    debug_data_.parked_targets.objects.push_back(
      blocking_attention_object_info->predicted_object());
  }
  std::vector<std::vector<cv::Point>> blocking_polygons;
  for (const auto & blocking_attention_object_info : blocking_attention_objects) {
    const Polygon2d obj_poly =
      autoware::universe_utils::toPolygon2d(blocking_attention_object_info->predicted_object());
    findCommonCvPolygons(obj_poly.outer(), blocking_polygons);
  }
  for (const auto & blocking_polygon : blocking_polygons) {
    cv::fillPoly(attention_mask, blocking_polygon, cv::Scalar(BLOCKED), cv::LINE_AA);
  }
  for (const auto & division : lane_divisions) {
    bool blocking_vehicle_found = false;
    for (const auto & point_it : division) {
      const auto [valid, idx_x, idx_y] = coord2index(point_it.x(), point_it.y());
      if (!valid) continue;
      if (blocking_vehicle_found) {
        occlusion_mask.at<unsigned char>(height - 1 - idx_y, idx_x) = 0;
        continue;
      }
      if (attention_mask.at<unsigned char>(height - 1 - idx_y, idx_x) == BLOCKED) {
        blocking_vehicle_found = true;
        occlusion_mask.at<unsigned char>(height - 1 - idx_y, idx_x) = 0;
      }
    }
  }

  // (4) extract occlusion polygons
  const auto & possible_object_bbox = planner_param_.occlusion.possible_object_bbox;
  const double possible_object_bbox_x = possible_object_bbox.at(0) / resolution;
  const double possible_object_bbox_y = possible_object_bbox.at(1) / resolution;
  const double possible_object_area = possible_object_bbox_x * possible_object_bbox_y;
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(occlusion_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  std::vector<std::vector<cv::Point>> valid_contours;
  for (const auto & contour : contours) {
    if (contour.size() <= 2) {
      continue;
    }
    std::vector<cv::Point> approx_contour;
    cv::approxPolyDP(
      contour, approx_contour,
      std::round(std::min(possible_object_bbox_x, possible_object_bbox_y) / std::sqrt(2.0)), true);
    if (approx_contour.size() <= 2) continue;
    // check area
    const double poly_area = cv::contourArea(approx_contour);
    if (poly_area < possible_object_area) continue;
    // check bounding box size
    const auto bbox = cv::minAreaRect(approx_contour);
    if (const auto size = bbox.size; std::min(size.height, size.width) <
                                       std::min(possible_object_bbox_x, possible_object_bbox_y) ||
                                     std::max(size.height, size.width) <
                                       std::max(possible_object_bbox_x, possible_object_bbox_y)) {
      continue;
    }
    valid_contours.push_back(approx_contour);
    geometry_msgs::msg::Polygon polygon_msg;
    geometry_msgs::msg::Point32 point_msg;
    for (const auto & p : approx_contour) {
      const double glob_x = (p.x + 0.5) * resolution + origin.x;
      const double glob_y = (height - 0.5 - p.y) * resolution + origin.y;
      point_msg.x = glob_x;
      point_msg.y = glob_y;
      point_msg.z = origin.z;
      polygon_msg.points.push_back(point_msg);
    }
    debug_data_.occlusion_polygons.push_back(polygon_msg);
  }
  // (4.1) re-draw occluded cells using valid_contours
  occlusion_mask = cv::Mat(width, height, CV_8UC1, cv::Scalar(0));
  for (const auto & valid_contour : valid_contours) {
    // NOTE: drawContour does not work well
    cv::fillPoly(occlusion_mask, valid_contour, cv::Scalar(OCCLUDED), cv::LINE_AA);
  }

  // (5) find distance
  // (5.1) discretize path_ip with resolution for computational cost
  LineString2d path_linestring;
  path_linestring.emplace_back(
    path_ip.points.at(lane_start_idx).point.pose.position.x,
    path_ip.points.at(lane_start_idx).point.pose.position.y);
  {
    auto prev_path_linestring_point = path_ip.points.at(lane_start_idx).point.pose.position;
    for (auto i = lane_start_idx + 1; i <= lane_end_idx; i++) {
      const auto path_linestring_point = path_ip.points.at(i).point.pose.position;
      if (
        autoware::universe_utils::calcDistance2d(
          prev_path_linestring_point, path_linestring_point) <
        1.0 /* rough tick for computational cost */) {
        continue;
      }
      path_linestring.emplace_back(path_linestring_point.x, path_linestring_point.y);
      prev_path_linestring_point = path_linestring_point;
    }
  }

  auto findNearestPointToProjection = [](
                                        const lanelet::ConstLineString3d & division,
                                        const Point2d & projection, const double dist_thresh) {
    double min_dist = std::numeric_limits<double>::infinity();
    auto nearest = division.end();
    for (auto it = division.begin(); it != division.end(); it++) {
      const double dist = std::hypot(it->x() - projection.x(), it->y() - projection.y());
      if (dist < min_dist) {
        min_dist = dist;
        nearest = it;
      }
      if (dist < dist_thresh) {
        break;
      }
    }
    return nearest;
  };
  struct NearestOcclusionInterval
  {
    int64 division_index{0};
    int64 point_index{0};
    geometry_msgs::msg::Point point;
    geometry_msgs::msg::Point projection;
    geometry_msgs::msg::Point visible_end;
  } nearest_occlusion_point;
  double min_dist = std::numeric_limits<double>::infinity();
  for (unsigned division_index = 0; division_index < lane_divisions.size(); ++division_index) {
    const auto & division = lane_divisions.at(division_index);
    LineString2d division_linestring;
    auto division_point_it = division.begin();
    division_linestring.emplace_back(division_point_it->x(), division_point_it->y());
    for (auto point_it = division.begin(); point_it != division.end(); point_it++) {
      if (
        std::hypot(point_it->x() - division_point_it->x(), point_it->y() - division_point_it->y()) <
        3.0 /* rough tick for computational cost */) {
        continue;
      }
      division_linestring.emplace_back(point_it->x(), point_it->y());
      division_point_it = point_it;
    }

    // find the intersection point of lane_line and path
    std::vector<Point2d> intersection_points;
    boost::geometry::intersection(division_linestring, path_linestring, intersection_points);
    if (intersection_points.empty()) {
      continue;
    }
    const auto & projection_point = intersection_points.at(0);
    const auto projection_it = findNearestPointToProjection(division, projection_point, resolution);
    if (projection_it == division.end()) {
      continue;
    }
    double acc_dist = 0.0;
    bool found_min_dist_for_this_division = false;
    bool is_prev_occluded = false;
    auto acc_dist_it = projection_it;
    for (auto point_it = projection_it; point_it != division.end(); point_it++) {
      const double dist =
        std::hypot(point_it->x() - acc_dist_it->x(), point_it->y() - acc_dist_it->y());
      acc_dist += dist;
      acc_dist_it = point_it;
      const auto [valid, idx_x, idx_y] = coord2index(point_it->x(), point_it->y());
      if (!valid) continue;
      const auto pixel = occlusion_mask.at<unsigned char>(height - 1 - idx_y, idx_x);
      if (pixel == BLOCKED) {
        break;
      }
      if (pixel == OCCLUDED) {
        if (acc_dist < min_dist) {
          min_dist = acc_dist;
          nearest_occlusion_point = {
            division_index, std::distance(division.begin(), point_it),
            autoware::universe_utils::createPoint(point_it->x(), point_it->y(), origin.z),
            autoware::universe_utils::createPoint(projection_it->x(), projection_it->y(), origin.z),
            autoware::universe_utils::createPoint(
              projection_it->x(), projection_it->y(),
              origin.z) /* initialize with projection point at first*/};
          found_min_dist_for_this_division = true;
        } else if (found_min_dist_for_this_division && is_prev_occluded) {
          // although this cell is not "nearest" cell, we have found the "nearest" cell on this
          // division previously in this iteration, and the iterated cells are still OCCLUDED since
          // then
          nearest_occlusion_point.visible_end =
            autoware::universe_utils::createPoint(point_it->x(), point_it->y(), origin.z);
        }
      }
      is_prev_occluded = (pixel == OCCLUDED);
    }
  }

  if (min_dist == std::numeric_limits<double>::infinity() || min_dist > occlusion_dist_thr) {
    return NotOccluded{min_dist};
  }

  debug_data_.nearest_occlusion_projection =
    std::make_pair(nearest_occlusion_point.point, nearest_occlusion_point.projection);
  debug_data_.nearest_occlusion_triangle = std::make_tuple(
    current_pose.position, nearest_occlusion_point.point, nearest_occlusion_point.visible_end);
  Polygon2d ego_occlusion_triangle;
  ego_occlusion_triangle.outer().emplace_back(current_pose.position.x, current_pose.position.y);
  ego_occlusion_triangle.outer().emplace_back(
    nearest_occlusion_point.point.x, nearest_occlusion_point.point.y);
  ego_occlusion_triangle.outer().emplace_back(
    nearest_occlusion_point.visible_end.x, nearest_occlusion_point.visible_end.y);
  bg::correct(ego_occlusion_triangle);
  for (const auto & attention_object_info : object_info_manager_.allObjects()) {
    const auto obj_poly =
      autoware::universe_utils::toPolygon2d(attention_object_info->predicted_object());
    if (bg::intersects(obj_poly, ego_occlusion_triangle)) {
      debug_data_.static_occlusion = false;
      return DynamicallyOccluded{min_dist};
    }
  }
  debug_data_.static_occlusion = true;
  return StaticallyOccluded{min_dist};
}
}  // namespace autoware::behavior_velocity_planner
