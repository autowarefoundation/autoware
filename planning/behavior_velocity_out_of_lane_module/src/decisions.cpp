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

#include "decisions.hpp"

#include <lanelet2_extension/utility/utilities.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <tier4_autoware_utils/ros/marker_helper.hpp>
#include <tier4_autoware_utils/ros/uuid_helper.hpp>

#include <boost/geometry/algorithms/within.hpp>

#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>
namespace behavior_velocity_planner::out_of_lane
{
double distance_along_path(const EgoData & ego_data, const size_t target_idx)
{
  return motion_utils::calcSignedArcLength(
    ego_data.path.points, ego_data.pose.position, ego_data.first_path_idx + target_idx);
}

double time_along_path(const EgoData & ego_data, const size_t target_idx, const double min_velocity)
{
  const auto dist = distance_along_path(ego_data, target_idx);
  const auto v = std::max(
    std::max(ego_data.velocity, min_velocity),
    ego_data.path.points[ego_data.first_path_idx + target_idx].point.longitudinal_velocity_mps *
      0.5);
  return dist / v;
}

bool object_is_incoming(
  const lanelet::BasicPoint2d & object_position,
  const std::shared_ptr<route_handler::RouteHandler> route_handler,
  const lanelet::ConstLanelet & lane)
{
  const auto lanelets = route_handler->getPrecedingLaneletSequence(lane, 50.0);
  if (boost::geometry::within(object_position, lane.polygon2d().basicPolygon())) return true;
  for (const auto & lls : lanelets)
    for (const auto & ll : lls)
      if (boost::geometry::within(object_position, ll.polygon2d().basicPolygon())) return true;
  return false;
}

std::optional<std::pair<double, double>> object_time_to_range(
  const autoware_auto_perception_msgs::msg::PredictedObject & object, const OverlapRange & range,
  const std::shared_ptr<route_handler::RouteHandler> route_handler, const double dist_buffer,
  const rclcpp::Logger & logger)
{
  // skip the dynamic object if it is not in a lane preceding the overlapped lane
  // lane changes are intentionally not considered
  const auto object_point = lanelet::BasicPoint2d(
    object.kinematics.initial_pose_with_covariance.pose.position.x,
    object.kinematics.initial_pose_with_covariance.pose.position.y);
  if (!object_is_incoming(object_point, route_handler, range.lane)) return {};

  const auto max_deviation = object.shape.dimensions.y + range.inside_distance + dist_buffer;
  auto worst_enter_time = std::optional<double>();
  auto worst_exit_time = std::optional<double>();

  for (const auto & predicted_path : object.kinematics.predicted_paths) {
    const auto unique_path_points = motion_utils::removeOverlapPoints(predicted_path.path);
    const auto time_step = rclcpp::Duration(predicted_path.time_step).seconds();
    const auto enter_point =
      geometry_msgs::msg::Point().set__x(range.entering_point.x()).set__y(range.entering_point.y());
    const auto enter_segment_idx =
      motion_utils::findNearestSegmentIndex(unique_path_points, enter_point);
    const auto enter_offset = motion_utils::calcLongitudinalOffsetToSegment(
      unique_path_points, enter_segment_idx, enter_point);
    const auto enter_lat_dist =
      std::abs(motion_utils::calcLateralOffset(unique_path_points, enter_point, enter_segment_idx));
    const auto enter_segment_length = tier4_autoware_utils::calcDistance2d(
      unique_path_points[enter_segment_idx], unique_path_points[enter_segment_idx + 1]);
    const auto enter_offset_ratio = enter_offset / enter_segment_length;
    const auto enter_time =
      static_cast<double>(enter_segment_idx) * time_step + enter_offset_ratio * time_step;

    const auto exit_point =
      geometry_msgs::msg::Point().set__x(range.exiting_point.x()).set__y(range.exiting_point.y());
    const auto exit_segment_idx =
      motion_utils::findNearestSegmentIndex(unique_path_points, exit_point);
    const auto exit_offset = motion_utils::calcLongitudinalOffsetToSegment(
      unique_path_points, exit_segment_idx, exit_point);
    const auto exit_lat_dist =
      std::abs(motion_utils::calcLateralOffset(unique_path_points, exit_point, exit_segment_idx));
    const auto exit_segment_length = tier4_autoware_utils::calcDistance2d(
      unique_path_points[exit_segment_idx], unique_path_points[exit_segment_idx + 1]);
    const auto exit_offset_ratio = exit_offset / static_cast<double>(exit_segment_length);
    const auto exit_time =
      static_cast<double>(exit_segment_idx) * time_step + exit_offset_ratio * time_step;

    RCLCPP_DEBUG(
      logger, "\t\t\tPredicted path (time step = %2.2fs): enter @ %2.2fs, exit @ %2.2fs", time_step,
      enter_time, exit_time);
    // predicted path is too far from the overlapping range to be relevant
    const auto is_far_from_entering_point = enter_lat_dist > max_deviation;
    const auto is_far_from_exiting_point = exit_lat_dist > max_deviation;
    if (is_far_from_entering_point && is_far_from_exiting_point) {
      RCLCPP_DEBUG(
        logger,
        " * far_from_enter (%d) = %2.2fm | far_from_exit (%d) = %2.2fm | max_dev = %2.2fm\n",
        is_far_from_entering_point, enter_lat_dist, is_far_from_exiting_point, exit_lat_dist,
        max_deviation);
      continue;
    }
    // else we rely on the interpolation to estimate beyond the end of the predicted path

    const auto same_driving_direction_as_ego = enter_time < exit_time;
    if (same_driving_direction_as_ego) {
      worst_enter_time = worst_enter_time ? std::min(*worst_enter_time, enter_time) : enter_time;
      worst_exit_time = worst_exit_time ? std::max(*worst_exit_time, exit_time) : exit_time;
    } else {
      worst_enter_time = worst_enter_time ? std::max(*worst_enter_time, enter_time) : enter_time;
      worst_exit_time = worst_exit_time ? std::min(*worst_exit_time, exit_time) : exit_time;
    }
  }
  if (worst_enter_time && worst_exit_time) {
    RCLCPP_DEBUG(
      logger, "\t\t\t * found enter/exit time [%2.2f, %2.2f]\n", *worst_enter_time,
      *worst_exit_time);
    return std::make_pair(*worst_enter_time, *worst_exit_time);
  }
  RCLCPP_DEBUG(logger, "\t\t\t * enter/exit time not found\n");
  return {};
}

std::optional<std::pair<double, double>> object_time_to_range(
  const autoware_auto_perception_msgs::msg::PredictedObject & object, const OverlapRange & range,
  const DecisionInputs & inputs, const rclcpp::Logger & logger)
{
  const auto & p = object.kinematics.initial_pose_with_covariance.pose.position;
  const auto object_point = lanelet::BasicPoint2d(p.x, p.y);
  const auto half_size = object.shape.dimensions.x / 2.0;
  lanelet::ConstLanelets object_lanelets;
  for (const auto & ll : inputs.lanelets)
    if (boost::geometry::within(object_point, ll.polygon2d().basicPolygon()))
      object_lanelets.push_back(ll);

  geometry_msgs::msg::Pose pose;
  pose.position.set__x(range.entering_point.x()).set__y(range.entering_point.y());
  const auto range_enter_length = lanelet::utils::getArcCoordinates({range.lane}, pose).length;
  pose.position.set__x(range.exiting_point.x()).set__y(range.exiting_point.y());
  const auto range_exit_length = lanelet::utils::getArcCoordinates({range.lane}, pose).length;
  const auto range_size = std::abs(range_enter_length - range_exit_length);
  auto worst_enter_dist = std::optional<double>();
  auto worst_exit_dist = std::optional<double>();
  for (const auto & lane : object_lanelets) {
    const auto path = inputs.route_handler->getRoutingGraphPtr()->shortestPath(lane, range.lane);
    RCLCPP_DEBUG(
      logger, "\t\t\tPath ? %d [from %ld to %ld]\n", path.has_value(), lane.id(), range.lane.id());
    if (path) {
      lanelet::ConstLanelets lls;
      for (const auto & ll : *path) lls.push_back(ll);
      pose.position.set__x(object_point.x()).set__y(object_point.y());
      const auto object_curr_length = lanelet::utils::getArcCoordinates(lls, pose).length;
      pose.position.set__x(range.entering_point.x()).set__y(range.entering_point.y());
      const auto enter_dist =
        lanelet::utils::getArcCoordinates(lls, pose).length - object_curr_length;
      pose.position.set__x(range.exiting_point.x()).set__y(range.exiting_point.y());
      const auto exit_dist =
        lanelet::utils::getArcCoordinates(lls, pose).length - object_curr_length;
      RCLCPP_DEBUG(
        logger, "\t\t\t%2.2f -> [%2.2f(%2.2f, %2.2f) - %2.2f(%2.2f, %2.2f)]\n", object_curr_length,
        enter_dist, range.entering_point.x(), range.entering_point.y(), exit_dist,
        range.exiting_point.x(), range.exiting_point.y());
      const auto already_entered_range = std::abs(enter_dist - exit_dist) > range_size * 2.0;
      if (already_entered_range) continue;
      // multiple paths to the overlap -> be conservative and use the "worst" case
      // "worst" = min/max arc length depending on if the lane is running opposite to the ego path
      const auto is_opposite = enter_dist > exit_dist;
      if (!worst_enter_dist)
        worst_enter_dist = enter_dist;
      else if (is_opposite)
        worst_enter_dist = std::max(*worst_enter_dist, enter_dist);
      else
        worst_enter_dist = std::min(*worst_enter_dist, enter_dist);
      if (!worst_exit_dist)
        worst_exit_dist = exit_dist;
      else if (is_opposite)
        worst_exit_dist = std::max(*worst_exit_dist, exit_dist);
      else
        worst_exit_dist = std::min(*worst_exit_dist, exit_dist);
    }
  }
  if (worst_enter_dist && worst_exit_dist) {
    const auto v = object.kinematics.initial_twist_with_covariance.twist.linear.x;
    return std::make_pair((*worst_enter_dist - half_size) / v, (*worst_exit_dist + half_size) / v);
  }
  return {};
}

bool threshold_condition(const RangeTimes & range_times, const PlannerParam & params)
{
  const auto enter_within_threshold =
    range_times.object.enter_time > 0.0 && range_times.object.enter_time < params.time_threshold;
  const auto exit_within_threshold =
    range_times.object.exit_time > 0.0 && range_times.object.exit_time < params.time_threshold;
  return enter_within_threshold || exit_within_threshold;
}

bool intervals_condition(
  const RangeTimes & range_times, const PlannerParam & params, const rclcpp::Logger & logger)
{
  const auto opposite_way_condition = [&]() -> bool {
    const auto ego_exits_before_object_enters =
      range_times.ego.exit_time + params.intervals_ego_buffer <
      range_times.object.enter_time + params.intervals_obj_buffer;
    RCLCPP_DEBUG(
      logger,
      "\t\t\t[Intervals] (opposite way) ego exit %2.2fs < obj enter %2.2fs ? -> should not "
      "enter = %d\n",
      range_times.ego.exit_time + params.intervals_ego_buffer,
      range_times.object.enter_time + params.intervals_obj_buffer, ego_exits_before_object_enters);
    return ego_exits_before_object_enters;
  };
  const auto same_way_condition = [&]() -> bool {
    const auto object_enters_during_overlap =
      range_times.ego.enter_time - params.intervals_ego_buffer <
        range_times.object.enter_time + params.intervals_obj_buffer &&
      range_times.object.enter_time - params.intervals_obj_buffer - range_times.ego.exit_time <
        range_times.ego.exit_time + params.intervals_ego_buffer;
    const auto object_exits_during_overlap =
      range_times.ego.enter_time - params.intervals_ego_buffer <
        range_times.object.exit_time + params.intervals_obj_buffer &&
      range_times.object.exit_time - params.intervals_obj_buffer - range_times.ego.exit_time <
        range_times.ego.exit_time + params.intervals_ego_buffer;
    RCLCPP_DEBUG(
      logger,
      "\t\t\t[Intervals] obj enters during overlap ? %d OR obj exits during overlap %d ? -> should "
      "not "
      "enter = %d\n",
      object_enters_during_overlap, object_exits_during_overlap,
      object_enters_during_overlap || object_exits_during_overlap);
    return object_enters_during_overlap || object_exits_during_overlap;
  };

  const auto object_is_going_same_way =
    range_times.object.enter_time < range_times.object.exit_time;
  return (object_is_going_same_way && same_way_condition()) ||
         (!object_is_going_same_way && opposite_way_condition());
}

bool ttc_condition(
  const RangeTimes & range_times, const PlannerParam & params, const rclcpp::Logger & logger)
{
  const auto ttc_at_enter = range_times.ego.enter_time - range_times.object.enter_time;
  const auto ttc_at_exit = range_times.ego.exit_time - range_times.object.exit_time;
  const auto collision_during_overlap = (ttc_at_enter < 0.0) != (ttc_at_exit < 0.0);
  const auto ttc_is_bellow_threshold =
    std::min(std::abs(ttc_at_enter), std::abs(ttc_at_exit)) <= params.ttc_threshold;
  RCLCPP_DEBUG(
    logger, "\t\t\t[TTC] (%2.2fs - %2.2fs) -> %d\n", ttc_at_enter, ttc_at_exit,
    (collision_during_overlap || ttc_is_bellow_threshold));
  return collision_during_overlap || ttc_is_bellow_threshold;
}

bool will_collide_on_range(
  const RangeTimes & range_times, const PlannerParam & params, const rclcpp::Logger & logger)
{
  RCLCPP_DEBUG(
    logger, " enter at %2.2fs, exits at %2.2fs\n", range_times.object.enter_time,
    range_times.object.exit_time);
  return (params.mode == "threshold" && threshold_condition(range_times, params)) ||
         (params.mode == "intervals" && intervals_condition(range_times, params, logger)) ||
         (params.mode == "ttc" && ttc_condition(range_times, params, logger));
}

bool should_not_enter(
  const OverlapRange & range, const DecisionInputs & inputs, const PlannerParam & params,
  const rclcpp::Logger & logger)
{
  RangeTimes range_times{};
  range_times.ego.enter_time =
    time_along_path(inputs.ego_data, range.entering_path_idx, params.ego_min_velocity);
  range_times.ego.exit_time =
    time_along_path(inputs.ego_data, range.exiting_path_idx, params.ego_min_velocity);
  RCLCPP_DEBUG(
    logger, "\t[%lu -> %lu] %ld | ego enters at %2.2f, exits at %2.2f\n", range.entering_path_idx,
    range.exiting_path_idx, range.lane.id(), range_times.ego.enter_time, range_times.ego.exit_time);
  for (const auto & object : inputs.objects.objects) {
    RCLCPP_DEBUG(
      logger, "\t\t[%s] going at %2.2fm/s",
      tier4_autoware_utils::toHexString(object.object_id).c_str(),
      object.kinematics.initial_twist_with_covariance.twist.linear.x);
    if (object.kinematics.initial_twist_with_covariance.twist.linear.x < params.objects_min_vel) {
      RCLCPP_DEBUG(logger, " SKIP (velocity bellow threshold %2.2fm/s)\n", params.objects_min_vel);
      continue;  // skip objects with velocity bellow a threshold
    }
    // skip objects that are already on the interval
    const auto enter_exit_time =
      params.objects_use_predicted_paths
        ? object_time_to_range(
            object, range, inputs.route_handler, params.objects_dist_buffer, logger)
        : object_time_to_range(object, range, inputs, logger);
    if (!enter_exit_time) {
      RCLCPP_DEBUG(logger, " SKIP (no enter/exit times found)\n");
      continue;  // skip objects that are not driving towards the overlapping range
    }

    range_times.object.enter_time = enter_exit_time->first;
    range_times.object.exit_time = enter_exit_time->second;
    if (will_collide_on_range(range_times, params, logger)) {
      range.debug.times = range_times;
      range.debug.object = object;
      return true;
    }
  }
  range.debug.times = range_times;
  return false;
}

void set_decision_velocity(
  std::optional<Slowdown> & decision, const double distance, const PlannerParam & params)
{
  if (distance < params.stop_dist_threshold) {
    decision->velocity = 0.0;
  } else if (distance < params.slow_dist_threshold) {
    decision->velocity = params.slow_velocity;
  } else {
    decision.reset();
  }
}

std::optional<Slowdown> calculate_decision(
  const OverlapRange & range, const DecisionInputs & inputs, const PlannerParam & params,
  const rclcpp::Logger & logger)
{
  std::optional<Slowdown> decision;
  if (should_not_enter(range, inputs, params, logger)) {
    decision.emplace();
    decision->target_path_idx =
      inputs.ego_data.first_path_idx + range.entering_path_idx;  // add offset from curr pose
    decision->lane_to_avoid = range.lane;
    const auto ego_dist_to_range = distance_along_path(inputs.ego_data, range.entering_path_idx);
    set_decision_velocity(decision, ego_dist_to_range, params);
  }
  return decision;
}

std::vector<Slowdown> calculate_decisions(
  const DecisionInputs & inputs, const PlannerParam & params, const rclcpp::Logger & logger)
{
  std::vector<Slowdown> decisions;
  for (const auto & range : inputs.ranges) {
    if (range.entering_path_idx == 0UL) continue;  // skip if we already entered the range
    const auto optional_decision = calculate_decision(range, inputs, params, logger);
    range.debug.decision = optional_decision;
    if (optional_decision) decisions.push_back(*optional_decision);
  }
  return decisions;
}

}  // namespace behavior_velocity_planner::out_of_lane
