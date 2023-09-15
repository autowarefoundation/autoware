// Copyright 2022 TIER IV, Inc.
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

#include "behavior_path_planner/utils/geometric_parallel_parking/geometric_parallel_parking.hpp"

#include "behavior_path_planner/utils/path_utils.hpp"
#include "behavior_path_planner/utils/start_planner/util.hpp"
#include "behavior_path_planner/utils/utils.hpp"
#include "motion_utils/trajectory/path_with_lane_id.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"
#include "tier4_autoware_utils/math/unit_conversion.hpp"

#include <interpolation/spline_interpolation.hpp>
#include <lanelet2_extension/utility/utilities.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <lanelet2_core/geometry/Polygon.h>
#endif

#include <boost/geometry/algorithms/within.hpp>

#include <limits>
#include <string>
#include <utility>
#include <vector>

using autoware_auto_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseArray;
using geometry_msgs::msg::Transform;
using geometry_msgs::msg::TransformStamped;
using lanelet::utils::getArcCoordinates;
using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::calcOffsetPose;
using tier4_autoware_utils::deg2rad;
using tier4_autoware_utils::inverseTransformPoint;
using tier4_autoware_utils::inverseTransformPose;
using tier4_autoware_utils::normalizeRadian;
using tier4_autoware_utils::toMsg;
using tier4_autoware_utils::transformPose;

namespace behavior_path_planner
{
void GeometricParallelParking::incrementPathIndex()
{
  current_path_idx_ = std::min(current_path_idx_ + 1, paths_.size() - 1);
}

PathWithLaneId GeometricParallelParking::getPathByIdx(size_t const idx) const
{
  if (paths_.empty() || paths_.size() <= idx) {
    return PathWithLaneId{};
  }

  return paths_.at(idx);
}

PathWithLaneId GeometricParallelParking::getCurrentPath() const
{
  return paths_.at(current_path_idx_);
}

PathWithLaneId GeometricParallelParking::getFullPath() const
{
  PathWithLaneId path{};
  for (const auto & partial_path : paths_) {
    path.points.insert(path.points.end(), partial_path.points.begin(), partial_path.points.end());
  }

  PathWithLaneId filtered_path = path;
  filtered_path.points = motion_utils::removeOverlapPoints(filtered_path.points);
  return filtered_path;
}

PathWithLaneId GeometricParallelParking::getArcPath() const
{
  PathWithLaneId path{};
  for (const auto & arc_path : arc_paths_) {
    path.points.insert(path.points.end(), arc_path.points.begin(), arc_path.points.end());
  }
  return path;
}

bool GeometricParallelParking::isParking() const
{
  return current_path_idx_ > 0;
}

void GeometricParallelParking::setVelocityToArcPaths(
  std::vector<PathWithLaneId> & arc_paths, const double velocity, const bool set_stop_end)
{
  for (auto & path : arc_paths) {
    for (size_t i = 0; i < path.points.size(); i++) {
      if (i == path.points.size() - 1 && set_stop_end) {
        // stop point at the end of the path
        path.points.at(i).point.longitudinal_velocity_mps = 0.0;
      } else {
        path.points.at(i).point.longitudinal_velocity_mps = velocity;
      }
    }
  }
}

std::vector<PathWithLaneId> GeometricParallelParking::generatePullOverPaths(
  const Pose & start_pose, const Pose & goal_pose, const double R_E_r,
  const lanelet::ConstLanelets & road_lanes, const lanelet::ConstLanelets & shoulder_lanes,
  const bool is_forward, const double end_pose_offset, const double velocity)
{
  const double lane_departure_margin = is_forward
                                         ? parameters_.forward_parking_lane_departure_margin
                                         : parameters_.backward_parking_lane_departure_margin;
  const double arc_path_interval = is_forward ? parameters_.forward_parking_path_interval
                                              : parameters_.backward_parking_path_interval;
  auto arc_paths = planOneTrial(
    start_pose, goal_pose, R_E_r, road_lanes, shoulder_lanes, is_forward, end_pose_offset,
    lane_departure_margin, arc_path_interval);
  if (arc_paths.empty()) {
    return std::vector<PathWithLaneId>{};
  }
  arc_paths_ = arc_paths;

  // set parking velocity and stop velocity at the end of the path
  constexpr bool set_stop_end = true;
  setVelocityToArcPaths(arc_paths, velocity, set_stop_end);

  // straight path from current to parking start
  const auto straight_path = generateStraightPath(start_pose, road_lanes);

  // check the continuity of straight path and arc path
  const Pose & road_path_last_pose = straight_path.points.back().point.pose;
  const Pose & arc_path_first_pose = arc_paths.front().points.front().point.pose;
  const double yaw_diff = std::abs(tier4_autoware_utils::normalizeRadian(
    tf2::getYaw(road_path_last_pose.orientation) - tf2::getYaw(arc_path_first_pose.orientation)));
  const double distance = calcDistance2d(road_path_last_pose, arc_path_first_pose);
  if (yaw_diff > tier4_autoware_utils::deg2rad(5.0) || distance > 0.1) {
    return std::vector<PathWithLaneId>{};
  }

  // combine straight_path -> arc_path*2
  auto paths = arc_paths;
  paths.insert(paths.begin(), straight_path);

  return paths;
}

void GeometricParallelParking::clearPaths()
{
  current_path_idx_ = 0;
  arc_paths_.clear();
  paths_.clear();
  pairs_terminal_velocity_and_accel_.clear();
}

bool GeometricParallelParking::planPullOver(
  const Pose & goal_pose, const lanelet::ConstLanelets & road_lanes,
  const lanelet::ConstLanelets & shoulder_lanes, const bool is_forward)
{
  const auto & common_params = planner_data_->parameters;
  const double end_pose_offset = is_forward ? -parameters_.after_forward_parking_straight_distance
                                            : parameters_.after_backward_parking_straight_distance;
  const Pose arc_end_pose = calcOffsetPose(goal_pose, end_pose_offset, 0, 0);

  // not plan after parking has started
  if (isParking()) {
    return false;
  }

  if (is_forward) {
    // When turning forward to the right, the front left goes out,
    // so reduce the steer angle at that time for seach no lane departure path.
    // TODO(Sugahara): define in the config
    constexpr double start_pose_offset = 0.0;
    constexpr double min_steer_rad = 0.05;
    constexpr double steer_interval = 0.1;
    for (double steer = parameters_.forward_parking_max_steer_angle; steer > min_steer_rad;
         steer -= steer_interval) {
      const double R_E_r = common_params.wheel_base / std::tan(steer);
      const auto start_pose =
        calcStartPose(arc_end_pose, road_lanes, start_pose_offset, R_E_r, is_forward);
      if (!start_pose) {
        continue;
      }

      const auto paths = generatePullOverPaths(
        *start_pose, goal_pose, R_E_r, road_lanes, shoulder_lanes, is_forward, end_pose_offset,
        parameters_.forward_parking_velocity);
      if (!paths.empty()) {
        paths_ = paths;
        return true;
      }
    }
  } else {  // backward
    // When turning backward to the left, the front right goes out,
    // so make the parking start point in front for seach no lane departure path
    // (same to reducing the steer angle)
    constexpr double max_offset = 5.0;
    constexpr double offset_interval = 1.0;
    for (double start_pose_offset = 0; start_pose_offset < max_offset;
         start_pose_offset += offset_interval) {
      const auto start_pose =
        calcStartPose(arc_end_pose, road_lanes, start_pose_offset, R_E_min_, is_forward);
      if (!start_pose) {
        continue;
      }

      const auto paths = generatePullOverPaths(
        *start_pose, goal_pose, R_E_min_, road_lanes, shoulder_lanes, is_forward, end_pose_offset,
        parameters_.backward_parking_velocity);
      if (!paths.empty()) {
        paths_ = paths;
        return true;
      }
    }
  }

  return false;
}

bool GeometricParallelParking::planPullOut(
  const Pose & start_pose, const Pose & goal_pose, const lanelet::ConstLanelets & road_lanes,
  const lanelet::ConstLanelets & shoulder_lanes)
{
  constexpr bool is_forward = false;         // parking backward means pull_out forward
  constexpr double start_pose_offset = 0.0;  // start_pose is current_pose
  constexpr double max_offset = 10.0;
  constexpr double offset_interval = 1.0;

  for (double end_pose_offset = 0; end_pose_offset < max_offset;
       end_pose_offset += offset_interval) {
    // pull_out end pose which is the second arc path end
    const auto end_pose =
      calcStartPose(start_pose, road_lanes, end_pose_offset, R_E_min_, is_forward);
    if (!end_pose) {
      continue;
    }

    // plan reverse path of parking. end_pose <-> start_pose
    auto arc_paths = planOneTrial(
      *end_pose, start_pose, R_E_min_, road_lanes, shoulder_lanes, is_forward, start_pose_offset,
      parameters_.pull_out_lane_departure_margin, parameters_.pull_out_path_interval);
    if (arc_paths.empty()) {
      // not found path
      continue;
    }

    // reverse to turn_right -> turn_left
    std::reverse(arc_paths.begin(), arc_paths.end());

    // reverse path points order
    for (auto & path : arc_paths) {
      std::reverse(path.points.begin(), path.points.end());
    }

    // reverse lane_ids. shoulder, lane order
    for (auto & path : arc_paths) {
      for (auto & p : path.points) {
        std::reverse(p.lane_ids.begin(), p.lane_ids.end());
      }
    }

    // get road center line path from pull_out end to goal, and combine after the second arc path
    const double s_start = getArcCoordinates(road_lanes, *end_pose).length;
    const auto path_end_info = start_planner_utils::calcEndArcLength(
      s_start, planner_data_->parameters.forward_path_length, road_lanes, goal_pose);
    const double s_end = path_end_info.first;
    const bool path_terminal_is_goal = path_end_info.second;
    PathWithLaneId road_center_line_path =
      planner_data_->route_handler->getCenterLinePath(road_lanes, s_start, s_end, true);

    // check the continuity of straight path and arc path
    const Pose & road_path_first_pose = road_center_line_path.points.front().point.pose;
    const Pose & arc_path_last_pose = arc_paths.back().points.back().point.pose;
    const double yaw_diff = std::abs(tier4_autoware_utils::normalizeRadian(
      tf2::getYaw(road_path_first_pose.orientation) - tf2::getYaw(arc_path_last_pose.orientation)));
    const double distance = calcDistance2d(road_path_first_pose, arc_path_last_pose);
    if (yaw_diff > tier4_autoware_utils::deg2rad(5.0) || distance > 0.1) {
      continue;
    }

    // set pull_out velocity to arc paths and 0 velocity to end point
    constexpr bool set_stop_end = false;
    setVelocityToArcPaths(arc_paths, parameters_.pull_out_velocity, set_stop_end);

    // combine the road center line path with the second arc path
    auto paths = arc_paths;
    paths.back().points.insert(
      paths.back().points.end(),
      road_center_line_path.points.begin() + 1,  // to avoid overlapped point
      road_center_line_path.points.end());
    paths.back().points = motion_utils::removeOverlapPoints(paths.back().points);

    // if the end point is the goal, set the velocity to 0
    if (path_terminal_is_goal) {
      paths.back().points.back().point.longitudinal_velocity_mps = 0.0;
    }

    arc_paths_ = arc_paths;
    paths_ = paths;

    return true;
  }
  return false;
}

boost::optional<Pose> GeometricParallelParking::calcStartPose(
  const Pose & goal_pose, const lanelet::ConstLanelets & road_lanes, const double start_pose_offset,
  const double R_E_r, const bool is_forward)
{
  const auto arc_coordinates = lanelet::utils::getArcCoordinates(road_lanes, goal_pose);

  // todo
  // When forwarding, the turning radius of the right and left will be the same.
  // But the left turn should also have a minimum turning radius.
  // see https://www.sciencedirect.com/science/article/pii/S1474667016436852 for the dx detail
  const double squared_distance_to_arc_connect =
    std::pow(R_E_r, 2) - std::pow(-arc_coordinates.distance / 2 + R_E_r, 2);
  if (squared_distance_to_arc_connect < 0) {
    // may be current_pose is behind the lane
    return boost::none;
  }
  const double dx_sign = is_forward ? -1 : 1;
  const double dx = 2 * std::sqrt(squared_distance_to_arc_connect) * dx_sign;
  const Pose start_pose =
    calcOffsetPose(goal_pose, dx + start_pose_offset, -arc_coordinates.distance, 0);

  return start_pose;
}

PathWithLaneId GeometricParallelParking::generateStraightPath(
  const Pose & start_pose, const lanelet::ConstLanelets & road_lanes)
{
  // get straight path before parking.
  const auto start_arc_position = lanelet::utils::getArcCoordinates(road_lanes, start_pose);

  const Pose current_pose = planner_data_->self_odometry->pose.pose;
  const auto current_arc_position = lanelet::utils::getArcCoordinates(road_lanes, current_pose);

  auto path = planner_data_->route_handler->getCenterLinePath(
    road_lanes, current_arc_position.length, start_arc_position.length, true);
  path.header = planner_data_->route_handler->getRouteHeader();
  if (!path.points.empty()) {
    path.points.back().point.longitudinal_velocity_mps = 0;
  }

  return path;
}

std::vector<PathWithLaneId> GeometricParallelParking::planOneTrial(
  const Pose & start_pose, const Pose & goal_pose, const double R_E_r,
  const lanelet::ConstLanelets & road_lanes, const lanelet::ConstLanelets & shoulder_lanes,
  const bool is_forward, const double end_pose_offset, const double lane_departure_margin,
  const double arc_path_interval)
{
  clearPaths();

  const auto & common_params = planner_data_->parameters;
  const auto & route_handler = planner_data_->route_handler;

  const Pose arc_end_pose = calcOffsetPose(goal_pose, end_pose_offset, 0, 0);
  const double self_yaw = tf2::getYaw(start_pose.orientation);
  const double goal_yaw = tf2::getYaw(arc_end_pose.orientation);
  const double psi = normalizeRadian(self_yaw - goal_yaw);

  const Pose Cr = calcOffsetPose(arc_end_pose, 0, -R_E_r, 0);
  const double d_Cr_Einit = calcDistance2d(Cr, start_pose);

  const Point Cr_goal_coords = inverseTransformPoint(Cr.position, arc_end_pose);
  const Point self_point_goal_coords = inverseTransformPoint(start_pose.position, arc_end_pose);

  const double alpha =
    M_PI_2 - psi + std::asin((self_point_goal_coords.y - Cr_goal_coords.y) / d_Cr_Einit);

  const double R_E_l =
    (std::pow(d_Cr_Einit, 2) - std::pow(R_E_r, 2)) / (2 * (R_E_r + d_Cr_Einit * std::cos(alpha)));
  if (R_E_l <= 0) {
    return std::vector<PathWithLaneId>{};
  }

  // combine road and shoulder lanes
  // cut the road lanes up to start_pose to prevent unintended processing for overlapped lane
  lanelet::ConstLanelets lanes{};
  tier4_autoware_utils::Point2d start_point2d(start_pose.position.x, start_pose.position.y);
  for (const auto & lane : road_lanes) {
    if (boost::geometry::within(start_point2d, lane.polygon2d().basicPolygon())) {
      lanes.push_back(lane);
      break;
    }
    lanes.push_back(lane);
  }
  lanes.insert(lanes.end(), shoulder_lanes.begin(), shoulder_lanes.end());

  // If start_pose is parallel to goal_pose, we can know lateral deviation of edges of vehicle,
  // and detect lane departure.
  if (is_forward) {  // Check left bound
    const double R_front_left =
      std::hypot(R_E_r + common_params.vehicle_width / 2, common_params.base_link2front);
    const double distance_to_left_bound =
      utils::getSignedDistanceFromBoundary(shoulder_lanes, arc_end_pose, true);
    const double left_deviation = R_front_left - R_E_r;
    if (std::abs(distance_to_left_bound) - left_deviation < lane_departure_margin) {
      return std::vector<PathWithLaneId>{};
    }
  } else {  // Check right bound
    const double R_front_right =
      std::hypot(R_E_l + common_params.vehicle_width / 2, common_params.base_link2front);
    const double right_deviation = R_front_right - R_E_l;
    const double distance_to_right_bound =
      utils::getSignedDistanceFromBoundary(lanes, start_pose, false);
    if (distance_to_right_bound - right_deviation < lane_departure_margin) {
      return std::vector<PathWithLaneId>{};
    }
  }

  // Generate arc path(left turn -> right turn)
  const Pose Cl = calcOffsetPose(start_pose, 0, R_E_l, 0);
  double theta_l = std::acos(
    (std::pow(R_E_l, 2) + std::pow(R_E_l + R_E_r, 2) - std::pow(d_Cr_Einit, 2)) /
    (2 * R_E_l * (R_E_l + R_E_r)));
  theta_l = is_forward ? theta_l : -theta_l;

  PathWithLaneId path_turn_left = generateArcPath(
    Cl, R_E_l, -M_PI_2, normalizeRadian(-M_PI_2 + theta_l), arc_path_interval, is_forward,
    is_forward);
  path_turn_left.header = route_handler->getRouteHeader();

  PathWithLaneId path_turn_right = generateArcPath(
    Cr, R_E_r, normalizeRadian(psi + M_PI_2 + theta_l), M_PI_2, arc_path_interval, !is_forward,
    is_forward);
  path_turn_right.header = route_handler->getRouteHeader();

  // Need to add straight path to last right_turning for parking in parallel
  if (std::abs(end_pose_offset) > 0) {
    PathPointWithLaneId straight_point{};
    straight_point.point.pose = goal_pose;
    path_turn_right.points.push_back(straight_point);
  }

  // Populate lane ids for a given path.
  // It checks if each point in the path is within a lane
  // and if its ID hasn't been added yet, it appends the ID to the container.
  std::vector<lanelet::Id> path_lane_ids;
  const auto populateLaneIds = [&](const auto & path) {
    for (const auto & p : path.points) {
      for (const auto & lane : lanes) {
        if (
          lanelet::utils::isInLanelet(p.point.pose, lane) &&
          std::find(path_lane_ids.begin(), path_lane_ids.end(), lane.id()) == path_lane_ids.end()) {
          path_lane_ids.push_back(lane.id());
        }
      }
    }
  };
  populateLaneIds(path_turn_left);
  populateLaneIds(path_turn_right);

  // Set lane ids to each point in a given path.
  // It assigns the accumulated lane ids from path_lane_ids to each point's lane_ids member.
  const auto setLaneIdsToPath = [&](PathWithLaneId & path) {
    for (auto & p : path.points) {
      p.lane_ids = path_lane_ids;
    }
  };
  setLaneIdsToPath(path_turn_left);
  setLaneIdsToPath(path_turn_right);

  // generate arc path vector
  paths_.push_back(path_turn_left);
  paths_.push_back(path_turn_right);

  // set terminal velocity and acceleration(temporary implementation)
  if (is_forward) {
    pairs_terminal_velocity_and_accel_.push_back(
      std::make_pair(parameters_.forward_parking_velocity, 0.0));
    pairs_terminal_velocity_and_accel_.push_back(
      std::make_pair(parameters_.forward_parking_velocity, 0.0));
  } else {
    pairs_terminal_velocity_and_accel_.push_back(
      std::make_pair(parameters_.backward_parking_velocity, 0.0));
    pairs_terminal_velocity_and_accel_.push_back(
      std::make_pair(parameters_.backward_parking_velocity, 0.0));
  }

  // set pull_over start and end pose
  // todo: make start and end pose for pull_out
  start_pose_ = start_pose;
  arc_end_pose_ = arc_end_pose;

  // debug
  Cr_ = Cr;
  Cl_ = Cl;

  return paths_;
}

PathWithLaneId GeometricParallelParking::generateArcPath(
  const Pose & center, const double radius, const double start_yaw, double end_yaw,
  const double arc_path_interval,
  const bool is_left_turn,  // is_left_turn means clockwise around center.
  const bool is_forward)
{
  PathWithLaneId path;
  const double yaw_interval = arc_path_interval / radius;
  double yaw = start_yaw;
  if (is_left_turn) {
    if (end_yaw < start_yaw) end_yaw += M_PI_2;
    while (yaw < end_yaw) {
      const auto p = generateArcPathPoint(center, radius, yaw, is_left_turn, is_forward);
      path.points.push_back(p);
      yaw += yaw_interval;
    }
  } else {  // right_turn
    if (end_yaw > start_yaw) end_yaw -= M_PI_2;
    while (yaw > end_yaw) {
      const auto p = generateArcPathPoint(center, radius, yaw, is_left_turn, is_forward);
      path.points.push_back(p);
      yaw -= yaw_interval;
    }
  }

  // insert the last point exactly
  const auto p = generateArcPathPoint(center, radius, end_yaw, is_left_turn, is_forward);
  constexpr double min_dist = 0.01;
  if (path.points.empty() || calcDistance2d(path.points.back(), p) > min_dist) {
    path.points.push_back(p);
  }

  return path;
}

PathPointWithLaneId GeometricParallelParking::generateArcPathPoint(
  const Pose & center, const double radius, const double yaw, const bool is_left_turn,
  const bool is_forward)
{
  // get pose in center_pose coords
  Pose pose_center_coords;
  pose_center_coords.position.x = radius * std::cos(yaw);
  pose_center_coords.position.y = radius * std::sin(yaw);

  // set orientation
  tf2::Quaternion quat;
  if ((is_left_turn && !is_forward) || (!is_left_turn && is_forward)) {
    quat.setRPY(0, 0, normalizeRadian(yaw - M_PI_2));
  } else {
    quat.setRPY(0, 0, normalizeRadian(yaw + M_PI_2));
  }
  pose_center_coords.orientation = tf2::toMsg(quat);

  // get pose in map coords
  PathPointWithLaneId p{};
  p.point.pose = transformPose(pose_center_coords, center);

  return p;
}

void GeometricParallelParking::setTurningRadius(
  const BehaviorPathPlannerParameters & common_params, const double max_steer_angle)
{
  R_E_min_ = common_params.wheel_base / std::tan(max_steer_angle);
  R_Bl_min_ = std::hypot(
    R_E_min_ + common_params.wheel_tread / 2 + common_params.left_over_hang,
    common_params.wheel_base + common_params.front_overhang);
}

}  // namespace behavior_path_planner
