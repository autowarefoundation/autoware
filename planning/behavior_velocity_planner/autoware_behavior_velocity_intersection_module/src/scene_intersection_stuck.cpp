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

#include <autoware/behavior_velocity_planner_common/utilization/boost_geometry_helper.hpp>  // for toGeomPoly
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/length.hpp>

#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Point.h>

namespace
{
lanelet::LineString3d getLineStringFromArcLength(
  const lanelet::ConstLineString3d & linestring, const double s1, const double s2)
{
  lanelet::Points3d points;
  double accumulated_length = 0;
  size_t start_index = linestring.size();
  if (start_index == 0) {
    return lanelet::LineString3d{lanelet::InvalId, points};
  }
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

lanelet::ConstLanelet createLaneletFromArcLength(
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

}  // namespace

namespace autoware::behavior_velocity_planner
{
namespace bg = boost::geometry;

std::optional<StuckStop> IntersectionModule::isStuckStatus(
  const tier4_planning_msgs::msg::PathWithLaneId & path,
  const IntersectionStopLines & intersection_stoplines, const PathLanelets & path_lanelets) const
{
  const auto closest_idx = intersection_stoplines.closest_idx;
  auto fromEgoDist = [&](const size_t index) {
    return autoware::motion_utils::calcSignedArcLength(path.points, closest_idx, index);
  };

  const auto & intersection_lanelets = intersection_lanelets_.value();  // this is OK
  const bool stuck_detected = checkStuckVehicleInIntersection(path_lanelets);
  const auto first_conflicting_lane =
    intersection_lanelets.first_conflicting_lane().value();  // this is OK
  const bool is_first_conflicting_lane_private =
    (std::string(first_conflicting_lane.attributeOr("location", "else")).compare("private") == 0);
  const auto stuck_stopline_idx_opt = intersection_stoplines.stuck_stopline;
  const auto default_stopline_idx_opt = intersection_stoplines.default_stopline;
  const auto first_attention_stopline_idx_opt = intersection_stoplines.first_attention_stopline;
  const auto occlusion_peeking_stopline_idx_opt = intersection_stoplines.occlusion_peeking_stopline;
  if (stuck_detected) {
    if (
      is_first_conflicting_lane_private &&
      planner_param_.stuck_vehicle.disable_against_private_lane) {
      // do nothing
    } else {
      std::optional<size_t> stopline_idx = std::nullopt;
      if (stuck_stopline_idx_opt) {
        const bool is_over_stuck_stopline = fromEgoDist(stuck_stopline_idx_opt.value()) <
                                            -planner_param_.common.stopline_overshoot_margin;
        if (!is_over_stuck_stopline) {
          stopline_idx = stuck_stopline_idx_opt.value();
        }
      }
      if (!stopline_idx) {
        if (default_stopline_idx_opt && fromEgoDist(default_stopline_idx_opt.value()) >= 0.0) {
          stopline_idx = default_stopline_idx_opt.value();
        } else if (
          first_attention_stopline_idx_opt &&
          fromEgoDist(first_attention_stopline_idx_opt.value()) >= 0.0) {
          stopline_idx = closest_idx;
        }
      }
      if (stopline_idx) {
        return StuckStop{closest_idx, stopline_idx.value(), occlusion_peeking_stopline_idx_opt};
      }
    }
  }
  return std::nullopt;
}

bool IntersectionModule::isTargetStuckVehicleType(
  const autoware_perception_msgs::msg::PredictedObject & object) const
{
  const auto label = object.classification.at(0).label;
  const auto & p = planner_param_.stuck_vehicle.target_type;

  if (label == autoware_perception_msgs::msg::ObjectClassification::CAR && p.car) {
    return true;
  }
  if (label == autoware_perception_msgs::msg::ObjectClassification::BUS && p.bus) {
    return true;
  }
  if (label == autoware_perception_msgs::msg::ObjectClassification::TRUCK && p.truck) {
    return true;
  }
  if (label == autoware_perception_msgs::msg::ObjectClassification::TRAILER && p.trailer) {
    return true;
  }
  if (label == autoware_perception_msgs::msg::ObjectClassification::MOTORCYCLE && p.motorcycle) {
    return true;
  }
  if (label == autoware_perception_msgs::msg::ObjectClassification::BICYCLE && p.bicycle) {
    return true;
  }
  if (label == autoware_perception_msgs::msg::ObjectClassification::UNKNOWN && p.unknown) {
    return true;
  }
  return false;
}

bool IntersectionModule::isTargetYieldStuckVehicleType(
  const autoware_perception_msgs::msg::PredictedObject & object) const
{
  const auto label = object.classification.at(0).label;
  const auto & p = planner_param_.yield_stuck.target_type;

  if (label == autoware_perception_msgs::msg::ObjectClassification::CAR && p.car) {
    return true;
  }
  if (label == autoware_perception_msgs::msg::ObjectClassification::BUS && p.bus) {
    return true;
  }
  if (label == autoware_perception_msgs::msg::ObjectClassification::TRUCK && p.truck) {
    return true;
  }
  if (label == autoware_perception_msgs::msg::ObjectClassification::TRAILER && p.trailer) {
    return true;
  }
  if (label == autoware_perception_msgs::msg::ObjectClassification::MOTORCYCLE && p.motorcycle) {
    return true;
  }
  if (label == autoware_perception_msgs::msg::ObjectClassification::BICYCLE && p.bicycle) {
    return true;
  }
  if (label == autoware_perception_msgs::msg::ObjectClassification::UNKNOWN && p.unknown) {
    return true;
  }
  return false;
}

bool IntersectionModule::checkStuckVehicleInIntersection(const PathLanelets & path_lanelets) const
{
  using lanelet::utils::getArcCoordinates;
  using lanelet::utils::getLaneletLength3d;
  using lanelet::utils::getPolygonFromArcLength;
  using lanelet::utils::to2D;

  const bool stuck_detection_direction = [&]() {
    return (turn_direction_ == "left" && planner_param_.stuck_vehicle.turn_direction.left) ||
           (turn_direction_ == "right" && planner_param_.stuck_vehicle.turn_direction.right) ||
           (turn_direction_ == "straight" && planner_param_.stuck_vehicle.turn_direction.straight);
  }();
  if (!stuck_detection_direction) {
    return false;
  }

  const auto & objects_ptr = planner_data_->predicted_objects;

  // considering lane change in the intersection, these lanelets are generated from the path
  const double stuck_vehicle_detect_dist = planner_param_.stuck_vehicle.stuck_vehicle_detect_dist;
  Polygon2d stuck_vehicle_detect_area{};
  if (path_lanelets.conflicting_interval_and_remaining.size() == 0) {
    return false;
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
    return false;
  }

  for (const auto & p : target_polygon) {
    stuck_vehicle_detect_area.outer().emplace_back(p.x(), p.y());
  }

  stuck_vehicle_detect_area.outer().emplace_back(stuck_vehicle_detect_area.outer().front());
  bg::correct(stuck_vehicle_detect_area);

  debug_data_.stuck_vehicle_detect_area = toGeomPoly(stuck_vehicle_detect_area);

  for (const auto & object : objects_ptr->objects) {
    if (!isTargetStuckVehicleType(object)) {
      continue;  // not target vehicle type
    }
    const auto obj_v_norm = std::hypot(
      object.kinematics.initial_twist_with_covariance.twist.linear.x,
      object.kinematics.initial_twist_with_covariance.twist.linear.y);
    if (obj_v_norm > planner_param_.stuck_vehicle.stuck_vehicle_velocity_threshold) {
      continue;  // not stop vehicle
    }

    // check if the footprint is in the stuck detect area
    const auto obj_footprint = autoware::universe_utils::toPolygon2d(object);
    // NOTE: in order not to stop too much
    const bool is_in_stuck_area = bg::within(
      to_bg2d(object.kinematics.initial_pose_with_covariance.pose.position),
      stuck_vehicle_detect_area);
    if (is_in_stuck_area) {
      debug_data_.stuck_targets.objects.push_back(object);
      return true;
    }
  }
  return false;
}

std::optional<YieldStuckStop> IntersectionModule::isYieldStuckStatus(
  const tier4_planning_msgs::msg::PathWithLaneId & path,
  const InterpolatedPathInfo & interpolated_path_info,
  const IntersectionStopLines & intersection_stoplines) const
{
  const auto closest_idx = intersection_stoplines.closest_idx;
  auto fromEgoDist = [&](const size_t index) {
    return autoware::motion_utils::calcSignedArcLength(path.points, closest_idx, index);
  };
  const auto & intersection_lanelets = intersection_lanelets_.value();
  const auto default_stopline_idx = intersection_stoplines.default_stopline.value();
  const auto first_attention_stopline_idx = intersection_stoplines.first_attention_stopline.value();
  const auto stuck_stopline_idx_opt = intersection_stoplines.stuck_stopline;

  const bool yield_stuck_detected = checkYieldStuckVehicleInIntersection(
    interpolated_path_info, intersection_lanelets.attention_non_preceding());
  if (yield_stuck_detected) {
    std::optional<size_t> stopline_idx = std::nullopt;
    const bool is_before_default_stopline = fromEgoDist(default_stopline_idx) >= 0.0;
    const bool is_before_first_attention_stopline =
      fromEgoDist(first_attention_stopline_idx) >= 0.0;
    if (stuck_stopline_idx_opt) {
      const bool is_over_stuck_stopline = fromEgoDist(stuck_stopline_idx_opt.value()) <
                                          -planner_param_.common.stopline_overshoot_margin;
      if (!is_over_stuck_stopline) {
        stopline_idx = stuck_stopline_idx_opt.value();
      }
    }
    if (!stopline_idx) {
      if (is_before_default_stopline) {
        stopline_idx = default_stopline_idx;
      } else if (is_before_first_attention_stopline) {
        stopline_idx = closest_idx;
      }
    }
    if (stopline_idx) {
      return YieldStuckStop{closest_idx, stopline_idx.value(), std::string("")};
    }
  }
  return std::nullopt;
}

bool IntersectionModule::checkYieldStuckVehicleInIntersection(
  const InterpolatedPathInfo & interpolated_path_info,
  const lanelet::ConstLanelets & attention_lanelets) const
{
  const bool yield_stuck_detection_direction = [&]() {
    return (turn_direction_ == "left" && planner_param_.yield_stuck.turn_direction.left) ||
           (turn_direction_ == "right" && planner_param_.yield_stuck.turn_direction.right) ||
           (turn_direction_ == "straight" && planner_param_.yield_stuck.turn_direction.straight);
  }();
  if (!yield_stuck_detection_direction) {
    return false;
  }

  const double width = planner_data_->vehicle_info_.vehicle_width_m;
  const double stuck_vehicle_vel_thr =
    planner_param_.stuck_vehicle.stuck_vehicle_velocity_threshold;
  const double yield_stuck_distance_thr = planner_param_.yield_stuck.distance_threshold;

  LineString2d sparse_intersection_path;
  const auto [start, end] = interpolated_path_info.lane_id_interval.value();
  for (unsigned i = start; i < end; ++i) {
    const auto & point = interpolated_path_info.path.points.at(i).point.pose.position;
    const auto yaw = tf2::getYaw(interpolated_path_info.path.points.at(i).point.pose.orientation);
    if (turn_direction_ == "right") {
      const double right_x = point.x - width / 2 * std::sin(yaw);
      const double right_y = point.y + width / 2 * std::cos(yaw);
      sparse_intersection_path.emplace_back(right_x, right_y);
    } else if (turn_direction_ == "left") {
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
      ::createLaneletFromArcLength(attention_lanelet, yield_stuck_start, yield_stuck_end));
  }
  debug_data_.yield_stuck_detect_area = util::getPolygon3dFromLanelets(yield_stuck_detect_lanelets);
  for (const auto & object_info : object_info_manager_.attentionObjects()) {
    const auto & object = object_info->predicted_object();
    if (!isTargetYieldStuckVehicleType(object)) {
      continue;
    }
    const auto obj_v_norm = std::hypot(
      object.kinematics.initial_twist_with_covariance.twist.linear.x,
      object.kinematics.initial_twist_with_covariance.twist.linear.y);

    if (obj_v_norm > stuck_vehicle_vel_thr) {
      continue;
    }
    for (const auto & yield_stuck_detect_lanelet : yield_stuck_detect_lanelets) {
      const bool is_in_lanelet = lanelet::utils::isInLanelet(
        object.kinematics.initial_pose_with_covariance.pose, yield_stuck_detect_lanelet);
      if (is_in_lanelet) {
        debug_data_.yield_stuck_targets.objects.push_back(object);
        return true;
      }
    }
  }
  return false;
}
}  // namespace autoware::behavior_velocity_planner
