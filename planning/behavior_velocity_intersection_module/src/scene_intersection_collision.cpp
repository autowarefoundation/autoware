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

#include <behavior_velocity_planner_common/utilization/boost_geometry_helper.hpp>  // for toGeomPoly
#include <behavior_velocity_planner_common/utilization/trajectory_utils.hpp>       // for smoothPath
#include <lanelet2_extension/utility/utilities.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <tier4_autoware_utils/geometry/boost_polygon_utils.hpp>  // for toPolygon2d
#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/correct.hpp>

#include <lanelet2_core/geometry/Polygon.h>

namespace
{
tier4_autoware_utils::Polygon2d createOneStepPolygon(
  const geometry_msgs::msg::Pose & prev_pose, const geometry_msgs::msg::Pose & next_pose,
  const autoware_auto_perception_msgs::msg::Shape & shape)
{
  namespace bg = boost::geometry;
  const auto prev_poly = tier4_autoware_utils::toPolygon2d(prev_pose, shape);
  const auto next_poly = tier4_autoware_utils::toPolygon2d(next_pose, shape);

  tier4_autoware_utils::Polygon2d one_step_poly;
  for (const auto & point : prev_poly.outer()) {
    one_step_poly.outer().push_back(point);
  }
  for (const auto & point : next_poly.outer()) {
    one_step_poly.outer().push_back(point);
  }

  bg::correct(one_step_poly);

  tier4_autoware_utils::Polygon2d convex_one_step_poly;
  bg::convex_hull(one_step_poly, convex_one_step_poly);

  return convex_one_step_poly;
}
}  // namespace

namespace behavior_velocity_planner
{
namespace bg = boost::geometry;

bool IntersectionModule::isTargetCollisionVehicleType(
  const autoware_auto_perception_msgs::msg::PredictedObject & object) const
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
      autoware_auto_perception_msgs::msg::ObjectClassification::MOTORCYCLE ||
    object.classification.at(0).label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::BICYCLE) {
    return true;
  }
  return false;
}

std::optional<intersection::NonOccludedCollisionStop>
IntersectionModule::isGreenPseudoCollisionStatus(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const size_t collision_stopline_idx,
  const intersection::IntersectionStopLines & intersection_stoplines,
  const TargetObjects & target_objects)
{
  const auto lanelet_map_ptr = planner_data_->route_handler_->getLaneletMapPtr();
  const auto & assigned_lanelet = lanelet_map_ptr->laneletLayer.get(lane_id_);
  // If there are any vehicles on the attention area when ego entered the intersection on green
  // light, do pseudo collision detection because the vehicles are very slow and no collisions may
  // be detected. check if ego vehicle entered assigned lanelet
  const bool is_green_solid_on = isGreenSolidOn();
  if (!is_green_solid_on) {
    return std::nullopt;
  }
  const auto closest_idx = intersection_stoplines.closest_idx;
  const auto occlusion_stopline_idx = intersection_stoplines.occlusion_peeking_stopline.value();
  if (!initial_green_light_observed_time_) {
    const auto assigned_lane_begin_point = assigned_lanelet.centerline().front();
    const bool approached_assigned_lane =
      motion_utils::calcSignedArcLength(
        path.points, closest_idx,
        tier4_autoware_utils::createPoint(
          assigned_lane_begin_point.x(), assigned_lane_begin_point.y(),
          assigned_lane_begin_point.z())) <
      planner_param_.collision_detection.yield_on_green_traffic_light
        .distance_to_assigned_lanelet_start;
    if (approached_assigned_lane) {
      initial_green_light_observed_time_ = clock_->now();
    }
  }
  if (initial_green_light_observed_time_) {
    const auto now = clock_->now();
    const bool still_wait =
      (rclcpp::Duration((now - initial_green_light_observed_time_.value())).seconds() <
       planner_param_.collision_detection.yield_on_green_traffic_light.duration);
    if (!still_wait) {
      return std::nullopt;
    }
    const bool exist_close_vehicles = std::any_of(
      target_objects.all_attention_objects.begin(), target_objects.all_attention_objects.end(),
      [&](const auto & object) {
        return object.dist_to_stopline.has_value() &&
               object.dist_to_stopline.value() <
                 planner_param_.collision_detection.yield_on_green_traffic_light
                   .object_dist_to_stopline;
      });
    if (exist_close_vehicles) {
      return intersection::NonOccludedCollisionStop{
        closest_idx, collision_stopline_idx, occlusion_stopline_idx};
    }
  }
  return std::nullopt;
}

bool IntersectionModule::checkCollision(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, TargetObjects * target_objects,
  const intersection::PathLanelets & path_lanelets, const size_t closest_idx,
  const size_t last_intersection_stopline_candidate_idx, const double time_delay,
  const TrafficPrioritizedLevel & traffic_prioritized_level)
{
  using lanelet::utils::getArcCoordinates;
  using lanelet::utils::getPolygonFromArcLength;

  // check collision between target_objects predicted path and ego lane
  // cut the predicted path at passing_time
  tier4_debug_msgs::msg::Float64MultiArrayStamped ego_ttc_time_array;
  const auto time_distance_array = calcIntersectionPassingTime(
    path, closest_idx, last_intersection_stopline_candidate_idx, time_delay, &ego_ttc_time_array);

  if (
    std::find(planner_param_.debug.ttc.begin(), planner_param_.debug.ttc.end(), lane_id_) !=
    planner_param_.debug.ttc.end()) {
    ego_ttc_time_array.stamp = path.header.stamp;
    ego_ttc_pub_->publish(ego_ttc_time_array);
  }

  const double passing_time = time_distance_array.back().first;
  cutPredictPathWithDuration(target_objects, passing_time);

  const auto & concat_lanelets = path_lanelets.all;
  const auto closest_arc_coords = getArcCoordinates(
    concat_lanelets, tier4_autoware_utils::getPose(path.points.at(closest_idx).point));
  const auto & ego_lane = path_lanelets.ego_or_entry2exit;
  debug_data_.ego_lane = ego_lane.polygon3d();
  const auto ego_poly = ego_lane.polygon2d().basicPolygon();

  // change TTC margin based on ego traffic light color
  const auto [collision_start_margin_time, collision_end_margin_time] = [&]() {
    if (traffic_prioritized_level == TrafficPrioritizedLevel::FULLY_PRIORITIZED) {
      return std::make_pair(
        planner_param_.collision_detection.fully_prioritized.collision_start_margin_time,
        planner_param_.collision_detection.fully_prioritized.collision_end_margin_time);
    }
    if (traffic_prioritized_level == TrafficPrioritizedLevel::PARTIALLY_PRIORITIZED) {
      return std::make_pair(
        planner_param_.collision_detection.partially_prioritized.collision_start_margin_time,
        planner_param_.collision_detection.partially_prioritized.collision_end_margin_time);
    }
    return std::make_pair(
      planner_param_.collision_detection.not_prioritized.collision_start_margin_time,
      planner_param_.collision_detection.not_prioritized.collision_end_margin_time);
  }();
  const auto expectedToStopBeforeStopLine = [&](const TargetObject & target_object) {
    if (!target_object.dist_to_stopline) {
      return false;
    }
    const double dist_to_stopline = target_object.dist_to_stopline.value();
    if (dist_to_stopline < 0) {
      return false;
    }
    const double v = target_object.object.kinematics.initial_twist_with_covariance.twist.linear.x;
    const double braking_distance =
      v * v /
      (2.0 * std::fabs(planner_param_.collision_detection.ignore_on_amber_traffic_light
                         .object_expected_deceleration));
    return dist_to_stopline > braking_distance;
  };
  const auto isTolerableOvershoot = [&](const TargetObject & target_object) {
    if (
      !target_object.attention_lanelet || !target_object.dist_to_stopline ||
      !target_object.stopline) {
      return false;
    }
    const double dist_to_stopline = target_object.dist_to_stopline.value();
    const double v = target_object.object.kinematics.initial_twist_with_covariance.twist.linear.x;
    const double braking_distance =
      v * v /
      (2.0 * std::fabs(planner_param_.collision_detection.ignore_on_amber_traffic_light
                         .object_expected_deceleration));
    if (dist_to_stopline > braking_distance) {
      return false;
    }
    const auto stopline_front = target_object.stopline.value().front();
    const auto stopline_back = target_object.stopline.value().back();
    tier4_autoware_utils::LineString2d object_line;
    object_line.emplace_back(
      (stopline_front.x() + stopline_back.x()) / 2.0,
      (stopline_front.y() + stopline_back.y()) / 2.0);
    const auto stopline_mid = object_line.front();
    const auto endpoint = target_object.attention_lanelet.value().centerline().back();
    object_line.emplace_back(endpoint.x(), endpoint.y());
    std::vector<tier4_autoware_utils::Point2d> intersections;
    bg::intersection(object_line, ego_lane.centerline2d().basicLineString(), intersections);
    if (intersections.empty()) {
      return false;
    }
    const auto collision_point = intersections.front();
    // distance from object expected stop position to collision point
    const double stopline_to_object = -1.0 * dist_to_stopline + braking_distance;
    const double stopline_to_collision =
      std::hypot(collision_point.x() - stopline_mid.x(), collision_point.y() - stopline_mid.y());
    const double object2collision = stopline_to_collision - stopline_to_object;
    const double margin =
      planner_param_.collision_detection.ignore_on_red_traffic_light.object_margin_to_path;
    return (object2collision > margin) || (object2collision < 0);
  };
  // check collision between predicted_path and ego_area
  tier4_debug_msgs::msg::Float64MultiArrayStamped object_ttc_time_array;
  object_ttc_time_array.layout.dim.resize(3);
  object_ttc_time_array.layout.dim.at(0).label = "objects";
  object_ttc_time_array.layout.dim.at(0).size = 1;  // incremented in the loop, first row is lane_id
  object_ttc_time_array.layout.dim.at(1).label =
    "[x, y, th, length, width, speed, dangerous, ref_obj_enter_time, ref_obj_exit_time, "
    "start_time, start_dist, "
    "end_time, end_dist, first_collision_x, first_collision_y, last_collision_x, last_collision_y, "
    "prd_x[0], ... pred_x[19], pred_y[0], ... pred_y[19]]";
  object_ttc_time_array.layout.dim.at(1).size = 57;
  for (unsigned i = 0; i < object_ttc_time_array.layout.dim.at(1).size; ++i) {
    object_ttc_time_array.data.push_back(lane_id_);
  }
  bool collision_detected = false;
  for (const auto & target_object : target_objects->all_attention_objects) {
    const auto & object = target_object.object;
    // If the vehicle is expected to stop before their stopline, ignore
    const bool expected_to_stop_before_stopline = expectedToStopBeforeStopLine(target_object);
    if (
      traffic_prioritized_level == TrafficPrioritizedLevel::PARTIALLY_PRIORITIZED &&
      expected_to_stop_before_stopline) {
      debug_data_.amber_ignore_targets.objects.push_back(object);
      continue;
    }
    const bool is_tolerable_overshoot = isTolerableOvershoot(target_object);
    if (
      traffic_prioritized_level == TrafficPrioritizedLevel::FULLY_PRIORITIZED &&
      !expected_to_stop_before_stopline && is_tolerable_overshoot) {
      debug_data_.red_overshoot_ignore_targets.objects.push_back(object);
      continue;
    }
    for (const auto & predicted_path : object.kinematics.predicted_paths) {
      if (
        predicted_path.confidence <
        planner_param_.collision_detection.min_predicted_path_confidence) {
        // ignore the predicted path with too low confidence
        continue;
      }

      // collision point
      const auto first_itr = std::adjacent_find(
        predicted_path.path.cbegin(), predicted_path.path.cend(),
        [&ego_poly, &object](const auto & a, const auto & b) {
          return bg::intersects(ego_poly, ::createOneStepPolygon(a, b, object.shape));
        });
      if (first_itr == predicted_path.path.cend()) continue;
      const auto last_itr = std::adjacent_find(
        predicted_path.path.crbegin(), predicted_path.path.crend(),
        [&ego_poly, &object](const auto & a, const auto & b) {
          return bg::intersects(ego_poly, ::createOneStepPolygon(a, b, object.shape));
        });
      if (last_itr == predicted_path.path.crend()) continue;

      // possible collision time interval
      const double ref_object_enter_time =
        static_cast<double>(first_itr - predicted_path.path.begin()) *
        rclcpp::Duration(predicted_path.time_step).seconds();
      auto start_time_distance_itr = time_distance_array.begin();
      if (ref_object_enter_time - collision_start_margin_time > 0) {
        // start of possible ego position in the intersection
        start_time_distance_itr = std::lower_bound(
          time_distance_array.begin(), time_distance_array.end(),
          ref_object_enter_time - collision_start_margin_time,
          [](const auto & a, const double b) { return a.first < b; });
        if (start_time_distance_itr == time_distance_array.end()) {
          // ego is already at the exit of intersection when npc is at collision point even if npc
          // accelerates so ego's position interval is empty
          continue;
        }
      }
      const double ref_object_exit_time =
        static_cast<double>(last_itr.base() - predicted_path.path.begin()) *
        rclcpp::Duration(predicted_path.time_step).seconds();
      auto end_time_distance_itr = std::lower_bound(
        time_distance_array.begin(), time_distance_array.end(),
        ref_object_exit_time + collision_end_margin_time,
        [](const auto & a, const double b) { return a.first < b; });
      if (end_time_distance_itr == time_distance_array.end()) {
        // ego is already passing the intersection, when npc is is at collision point
        // so ego's position interval is up to the end of intersection lane
        end_time_distance_itr = time_distance_array.end() - 1;
      }
      const double start_arc_length = std::max(
        0.0, closest_arc_coords.length + (*start_time_distance_itr).second -
               planner_data_->vehicle_info_.rear_overhang_m);
      const double end_arc_length = std::min(
        closest_arc_coords.length + (*end_time_distance_itr).second +
          planner_data_->vehicle_info_.max_longitudinal_offset_m,
        lanelet::utils::getLaneletLength2d(concat_lanelets));

      const auto trimmed_ego_polygon =
        getPolygonFromArcLength(concat_lanelets, start_arc_length, end_arc_length);

      if (trimmed_ego_polygon.empty()) {
        continue;
      }

      Polygon2d polygon{};
      for (const auto & p : trimmed_ego_polygon) {
        polygon.outer().emplace_back(p.x(), p.y());
      }
      bg::correct(polygon);
      debug_data_.candidate_collision_ego_lane_polygon = toGeomPoly(polygon);

      for (auto itr = first_itr; itr != last_itr.base(); ++itr) {
        const auto footprint_polygon = tier4_autoware_utils::toPolygon2d(*itr, object.shape);
        if (bg::intersects(polygon, footprint_polygon)) {
          collision_detected = true;
          break;
        }
      }
      object_ttc_time_array.layout.dim.at(0).size++;
      const auto & pos = object.kinematics.initial_pose_with_covariance.pose.position;
      const auto & shape = object.shape;
      object_ttc_time_array.data.insert(
        object_ttc_time_array.data.end(),
        {pos.x, pos.y, tf2::getYaw(object.kinematics.initial_pose_with_covariance.pose.orientation),
         shape.dimensions.x, shape.dimensions.y,
         object.kinematics.initial_twist_with_covariance.twist.linear.x,
         1.0 * static_cast<int>(collision_detected), ref_object_enter_time, ref_object_exit_time,
         start_time_distance_itr->first, start_time_distance_itr->second,
         end_time_distance_itr->first, end_time_distance_itr->second, first_itr->position.x,
         first_itr->position.y, last_itr->position.x, last_itr->position.y});
      for (unsigned i = 0; i < 20; i++) {
        const auto & pos =
          predicted_path.path.at(std::min<size_t>(i, predicted_path.path.size() - 1)).position;
        object_ttc_time_array.data.push_back(pos.x);
        object_ttc_time_array.data.push_back(pos.y);
      }
      if (collision_detected) {
        debug_data_.conflicting_targets.objects.push_back(object);
        break;
      }
    }
  }

  if (
    std::find(planner_param_.debug.ttc.begin(), planner_param_.debug.ttc.end(), lane_id_) !=
    planner_param_.debug.ttc.end()) {
    object_ttc_time_array.stamp = path.header.stamp;
    object_ttc_pub_->publish(object_ttc_time_array);
  }

  return collision_detected;
}

std::optional<size_t> IntersectionModule::checkAngleForTargetLanelets(

  const geometry_msgs::msg::Pose & pose, const lanelet::ConstLanelets & target_lanelets,
  const bool is_parked_vehicle) const
{
  const double detection_area_angle_thr = planner_param_.common.attention_area_angle_threshold;
  const bool consider_wrong_direction_vehicle =
    planner_param_.common.attention_area_angle_threshold;
  const double dist_margin = planner_param_.common.attention_area_margin;

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

void IntersectionModule::cutPredictPathWithDuration(
  TargetObjects * target_objects, const double time_thr)
{
  const rclcpp::Time current_time = clock_->now();
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

IntersectionModule::TimeDistanceArray IntersectionModule::calcIntersectionPassingTime(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const size_t closest_idx,
  const size_t last_intersection_stopline_candidate_idx, const double time_delay,
  tier4_debug_msgs::msg::Float64MultiArrayStamped * debug_ttc_array)
{
  const double intersection_velocity =
    planner_param_.collision_detection.velocity_profile.default_velocity;
  const double minimum_ego_velocity =
    planner_param_.collision_detection.velocity_profile.minimum_default_velocity;
  const bool use_upstream_velocity =
    planner_param_.collision_detection.velocity_profile.use_upstream;
  const double minimum_upstream_velocity =
    planner_param_.collision_detection.velocity_profile.minimum_upstream_velocity;
  const double current_velocity = planner_data_->current_velocity->twist.linear.x;

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
    bool has_objective_lane_id = util::hasLaneIds(path.points.at(i), associative_ids_);
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
  if (!smoothPath(reference_path, smoothed_reference_path, planner_data_)) {
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
    planner_data_->ego_nearest_dist_threshold, planner_data_->ego_nearest_yaw_threshold);

  const std::optional<size_t> upstream_stopline_idx_opt = [&]() -> std::optional<size_t> {
    if (upstream_stopline) {
      const auto upstream_stopline_point = path.points.at(upstream_stopline.value()).point.pose;
      return motion_utils::findFirstNearestIndexWithSoftConstraints(
        smoothed_reference_path.points, upstream_stopline_point,
        planner_data_->ego_nearest_dist_threshold, planner_data_->ego_nearest_yaw_threshold);
    } else {
      return std::nullopt;
    }
  }();

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
        if (upstream_stopline_idx_opt && i > upstream_stopline_idx_opt.value()) {
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
    debug_ttc_array->data.push_back(lane_id_);
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

}  // namespace behavior_velocity_planner
