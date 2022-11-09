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

#include "behavior_path_planner/scene_module/utils/geometric_parallel_parking.hpp"

#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/utilities.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"

#include <interpolation/spline_interpolation.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <limits>
#include <string>
#include <utility>
#include <vector>

using autoware_auto_planning_msgs::msg::PathWithLaneId;
using behavior_path_planner::util::convertToGeometryPoseArray;
using behavior_path_planner::util::removeOverlappingPoints;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseArray;
using geometry_msgs::msg::PoseStamped;
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

  return removeOverlappingPoints(path);
}

PathWithLaneId GeometricParallelParking::getArcPath() const
{
  PathWithLaneId path{};
  for (const auto & arc_path : arc_paths_) {
    path.points.insert(path.points.end(), arc_path.points.begin(), arc_path.points.end());
  }
  return path;
}

bool GeometricParallelParking::isParking() const { return current_path_idx_ > 0; }

bool GeometricParallelParking::isEnoughDistanceToStart(const Pose & start_pose) const
{
  const Pose current_pose = planner_data_->self_pose->pose;
  const Pose current_to_start =
    inverseTransformPose(start_pose, current_pose);  // todo: arc length is better

  // not enough to stop with max deceleration
  const double current_vel = util::l2Norm(planner_data_->self_odometry->twist.twist.linear);
  const double stop_distance = std::pow(current_vel, 2) / parameters_.maximum_deceleration / 2;
  if (current_to_start.position.x < stop_distance) {
    return false;
  }

  // not enough to restart from stopped
  constexpr double min_restart_distance = 3.0;
  if (
    current_vel < parameters_.th_stopped_velocity &&
    current_to_start.position.x > parameters_.th_arrived_distance &&
    current_to_start.position.x < min_restart_distance) {
    return false;
  }

  return true;
}

void GeometricParallelParking::setVelocityToArcPaths(
  std::vector<PathWithLaneId> & arc_paths, const double velocity)
{
  for (auto & path : arc_paths) {
    for (size_t i = 0; i < path.points.size(); i++) {
      if (i == path.points.size() - 1) {
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
  if (!isEnoughDistanceToStart(start_pose)) {
    return std::vector<PathWithLaneId>{};
  }
  const double lane_departure_margin = is_forward
                                         ? parameters_.forward_parking_lane_departure_margin
                                         : parameters_.backward_parking_lane_departure_margin;
  auto arc_paths = planOneTrial(
    start_pose, goal_pose, R_E_r, road_lanes, shoulder_lanes, is_forward, end_pose_offset,
    lane_departure_margin);
  if (arc_paths.empty()) {
    return std::vector<PathWithLaneId>{};
  }
  arc_paths_ = arc_paths;

  // set parking velocity and stop velocity at the end of the path
  setVelocityToArcPaths(arc_paths, velocity);

  // straight path from current to parking start
  const auto straight_path = generateStraightPath(start_pose);

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
}

bool GeometricParallelParking::planPullOver(
  const Pose & goal_pose, const lanelet::ConstLanelets & road_lanes,
  const lanelet::ConstLanelets & shoulder_lanes, const bool is_forward)
{
  clearPaths();

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
    constexpr double start_pose_offset = 0.0;
    constexpr double min_steer_rad = 0.05;
    constexpr double steer_interval = 0.1;
    for (double steer = parameters_.max_steer_angle; steer > min_steer_rad;
         steer -= steer_interval) {
      const double R_E_r = common_params.wheel_base / std::tan(steer);
      const auto start_pose = calcStartPose(arc_end_pose, start_pose_offset, R_E_r, is_forward);
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
      const auto start_pose = calcStartPose(arc_end_pose, start_pose_offset, R_E_min_, is_forward);
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
  clearPaths();

  constexpr bool is_forward = false;         // parking backward means departing forward
  constexpr double start_pose_offset = 0.0;  // start_pose is current_pose
  constexpr double max_offset = 10.0;
  constexpr double offset_interval = 1.0;

  for (double end_pose_offset = 0; end_pose_offset < max_offset;
       end_pose_offset += offset_interval) {
    // departing end pose which is the second arc path end
    const auto end_pose = calcStartPose(start_pose, end_pose_offset, R_E_min_, is_forward);
    if (!end_pose) {
      continue;
    }

    // plan reverse path of parking. end_pose <-> start_pose
    auto arc_paths = planOneTrial(
      *end_pose, start_pose, R_E_min_, road_lanes, shoulder_lanes, is_forward, start_pose_offset,
      parameters_.departing_lane_departure_margin);
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

    arc_paths_ = arc_paths;

    // get road center line path from departing end to goal, and combine after the second arc path
    PathWithLaneId road_center_line_path;
    {
      const double s_start = getArcCoordinates(road_lanes, *end_pose).length + 1.0;  // need buffer?
      const double s_end = getArcCoordinates(road_lanes, goal_pose).length;
      road_center_line_path =
        planner_data_->route_handler->getCenterLinePath(road_lanes, s_start, s_end, true);
    }
    auto paths = arc_paths;
    paths.back().points.insert(
      paths.back().points.end(), road_center_line_path.points.begin(),
      road_center_line_path.points.end());
    removeOverlappingPoints(paths.back());

    // set departing velocity and stop velocity at the end of the path
    setVelocityToArcPaths(paths, parameters_.departing_velocity);
    paths_ = paths;

    return true;
  }
  return false;
}

boost::optional<Pose> GeometricParallelParking::calcStartPose(
  const Pose & goal_pose, const double start_pose_offset, const double R_E_r, const bool is_forward)
{
  // Not use shoulder lanes.
  const auto current_lanes = util::getExtendedCurrentLanes(planner_data_);
  const auto arc_coordinates = lanelet::utils::getArcCoordinates(current_lanes, goal_pose);

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

PathWithLaneId GeometricParallelParking::generateStraightPath(const Pose & start_pose)
{
  // get straight path before parking.
  const auto current_lanes = util::getExtendedCurrentLanes(planner_data_);
  const auto start_arc_position = lanelet::utils::getArcCoordinates(current_lanes, start_pose);

  const Pose current_pose = planner_data_->self_pose->pose;
  const auto current_arc_position = lanelet::utils::getArcCoordinates(current_lanes, current_pose);

  auto path = planner_data_->route_handler->getCenterLinePath(
    current_lanes, current_arc_position.length, start_arc_position.length, true);
  path.header = planner_data_->route_handler->getRouteHeader();
  path.points.back().point.longitudinal_velocity_mps = 0;

  return path;
}

std::vector<PathWithLaneId> GeometricParallelParking::planOneTrial(
  const Pose & start_pose, const Pose & goal_pose, const double R_E_r,
  const lanelet::ConstLanelets & road_lanes, const lanelet::ConstLanelets & shoulder_lanes,
  const bool is_forward, const double end_pose_offset, const double lane_departure_margin)
{
  const auto common_params = planner_data_->parameters;

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
  lanelet::ConstLanelets lanes = road_lanes;
  lanes.insert(lanes.end(), shoulder_lanes.begin(), shoulder_lanes.end());

  // If start_pose is parallel to goal_pose, we can know lateral deviation of edges of vehicle,
  // and detect lane departure.
  if (is_forward) {  // Check left bound
    const double R_front_left =
      std::hypot(R_E_r + common_params.vehicle_width / 2, common_params.base_link2front);
    const double distance_to_left_bound =
      util::getSignedDistanceFromShoulderLeftBoundary(shoulder_lanes, arc_end_pose);
    const double left_deviation = R_front_left - R_E_r;
    if (std::abs(distance_to_left_bound) - left_deviation < lane_departure_margin) {
      return std::vector<PathWithLaneId>{};
    }
  } else {  // Check right bound
    const double R_front_right =
      std::hypot(R_E_l + common_params.vehicle_width / 2, common_params.base_link2front);
    const double right_deviation = R_front_right - R_E_l;
    const double distance_to_right_bound =
      util::getSignedDistanceFromRightBoundary(lanes, start_pose);
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

  PathWithLaneId path_turn_left =
    generateArcPath(Cl, R_E_l, -M_PI_2, normalizeRadian(-M_PI_2 + theta_l), is_forward, is_forward);
  path_turn_left.header = planner_data_->route_handler->getRouteHeader();

  PathWithLaneId path_turn_right = generateArcPath(
    Cr, R_E_r, normalizeRadian(psi + M_PI_2 + theta_l), M_PI_2, !is_forward, is_forward);
  path_turn_right.header = planner_data_->route_handler->getRouteHeader();

  auto setLaneIds = [lanes](PathPointWithLaneId & p) {
    for (const auto & lane : lanes) {
      p.lane_ids.push_back(lane.id());
    }
  };
  auto setLaneIdsToPath = [setLaneIds](PathWithLaneId & path) {
    for (auto & p : path.points) {
      setLaneIds(p);
    }
  };
  setLaneIdsToPath(path_turn_left);
  setLaneIdsToPath(path_turn_right);

  // Need to add straight path to last right_turning for parking in parallel
  if (std::abs(end_pose_offset) > 0) {
    PathPointWithLaneId straight_point{};
    straight_point.point.pose = goal_pose;
    setLaneIds(straight_point);
    path_turn_right.points.push_back(straight_point);
  }

  // generate arc path vector
  paths_.push_back(path_turn_left);
  paths_.push_back(path_turn_right);

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
  const bool is_left_turn,  // is_left_turn means clockwise around center.
  const bool is_forward)
{
  PathWithLaneId path;
  const double yaw_interval = parameters_.arc_path_interval / radius;
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

void GeometricParallelParking::setData(
  const std::shared_ptr<const PlannerData> & planner_data,
  const ParallelParkingParameters & parameters)
{
  planner_data_ = planner_data;
  parameters_ = parameters;

  auto common_params = planner_data_->parameters;

  R_E_min_ = common_params.wheel_base / std::tan(parameters_.max_steer_angle);
  R_Bl_min_ = std::hypot(
    R_E_min_ + common_params.wheel_tread / 2 + common_params.left_over_hang,
    common_params.wheel_base + common_params.front_overhang);
}
}  // namespace behavior_path_planner
