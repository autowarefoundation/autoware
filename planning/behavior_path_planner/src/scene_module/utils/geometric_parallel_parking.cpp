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
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseArray;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Transform;
using geometry_msgs::msg::TransformStamped;
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

PathWithLaneId GeometricParallelParking::getCurrentPath() const
{
  return paths_.at(current_path_idx_);
}

PathWithLaneId GeometricParallelParking::getFullPath() const
{
  PathWithLaneId path{};
  for (const auto & p : paths_) {
    path.points.insert(path.points.end(), p.points.begin(), p.points.end());
  }
  return path;
}

PathWithLaneId GeometricParallelParking::getArcPath() const
{
  PathWithLaneId path{};
  for (size_t i = 1; i < paths_.size(); i++) {
    const auto p = paths_.at(i);
    path.points.insert(path.points.end(), p.points.begin(), p.points.end());
  }
  return path;
}

void GeometricParallelParking::clear()
{
  current_path_idx_ = 0;
  paths_.clear();
}

bool GeometricParallelParking::isParking() const { return current_path_idx_ > 0; }

bool GeometricParallelParking::plan(
  const Pose goal_pose, const lanelet::ConstLanelets lanes, const bool is_forward)
{
  const auto common_params = planner_data_->parameters;
  // plan path only when parking has not started
  if (!isParking()) {
    if (is_forward) {
      // When turning forward to the right, the front left goes out,
      // so reduce the steer angle at that time for seach no lane departure path.
      for (double steer = max_steer_rad_; steer > 0.05; steer -= 0.1) {
        const double R_E_r = common_params.wheel_base / std::tan(steer);
        if (planOneTraial(goal_pose, 0, R_E_r, lanes, is_forward)) return true;
      }
    } else {
      // When turning backward to the left, the front right goes out,
      // so make the parking start point in front for seach no lane departure path
      // (same to reducing the steer angle)
      for (double dx = 0; dx < 5; dx += 1.0) {
        if (planOneTraial(goal_pose, dx, R_E_min_, lanes, is_forward)) return true;
      }
    }
  }
  return false;
}

Pose GeometricParallelParking::calcStartPose(
  const Pose goal_pose, const double start_pose_offset, const double R_E_r, const bool is_forward)
{
  // Not use shoulder lanes.
  const auto current_lanes = util::getExtendedCurrentLanes(planner_data_);
  const auto arc_coordinates = lanelet::utils::getArcCoordinates(current_lanes, goal_pose);

  // todo
  // When forwarding, the turning radius of the right and left are the same.
  // But the left turn should also have a minimum turning radius.
  float dx = 2 * std::sqrt(std::pow(R_E_r, 2) - std::pow(-arc_coordinates.distance / 2 + R_E_r, 2));
  dx = is_forward ? -dx : dx;
  Pose start_pose = calcOffsetPose(goal_pose, dx + start_pose_offset, -arc_coordinates.distance, 0);

  return start_pose;
}

void GeometricParallelParking::generateStraightPath(const Pose start_pose)
{
  // get stright path before parking.
  const auto current_lanes = util::getExtendedCurrentLanes(planner_data_);
  const auto start_arc_position = lanelet::utils::getArcCoordinates(current_lanes, start_pose);

  const Pose current_pose = planner_data_->self_pose->pose;
  const auto crrent_arc_position = lanelet::utils::getArcCoordinates(current_lanes, current_pose);

  auto path = planner_data_->route_handler->getCenterLinePath(
    current_lanes, crrent_arc_position.length, start_arc_position.length, true);
  path.header = planner_data_->route_handler->getRouteHeader();

  const auto common_params = planner_data_->parameters;
  path.drivable_area = util::generateDrivableArea(
    current_lanes, common_params.drivable_area_resolution, common_params.vehicle_length,
    planner_data_);

  path.points.back().point.longitudinal_velocity_mps = 0;

  paths_.push_back(path);
}

bool GeometricParallelParking::planOneTraial(
  const Pose goal_pose, const double start_pose_offset, const double R_E_r,
  const lanelet::ConstLanelets lanes, const bool is_forward)
{
  path_pose_array_.poses.clear();
  paths_.clear();

  const double after_parking_straight_distance =
    is_forward ? -parameters_.after_forward_parking_straight_distance
               : parameters_.after_backward_parking_straight_distance;
  const Pose arc_end_pose = calcOffsetPose(goal_pose, after_parking_straight_distance, 0, 0);
  const Pose start_pose = calcStartPose(arc_end_pose, start_pose_offset, R_E_r, is_forward);
  const Pose current_pose = planner_data_->self_pose->pose;
  const Pose current_to_start = inverseTransformPose(start_pose, current_pose);

  const double current_vel = util::l2Norm(planner_data_->self_odometry->twist.twist.linear);
  const double stop_distance = std::pow(current_vel, 2) / std::abs(parameters_.min_acc) / 2;
  if (current_to_start.position.x < stop_distance) {
    return false;
  }
  // not enogh to restart from stopped
  if (
    current_vel < parameters_.th_stopped_velocity_mps &&
    current_to_start.position.x > parameters_.th_arrived_distance_m &&
    current_to_start.position.x < 3.0) {
    return false;
  }

  const float self_yaw = tf2::getYaw(start_pose.orientation);
  const float goal_yaw = tf2::getYaw(arc_end_pose.orientation);
  const float psi = normalizeRadian(self_yaw - goal_yaw);
  const auto common_params = planner_data_->parameters;

  Pose Cr = calcOffsetPose(arc_end_pose, 0, -R_E_r, 0);
  const float d_Cr_Einit = calcDistance2d(Cr, start_pose);

  geometry_msgs::msg::Point Cr_goalcoords = inverseTransformPoint(Cr.position, arc_end_pose);
  geometry_msgs::msg::Point self_point_goalcoords =
    inverseTransformPoint(start_pose.position, arc_end_pose);

  const float alpha =
    M_PI_2 - psi + std::asin((self_point_goalcoords.y - Cr_goalcoords.y) / d_Cr_Einit);

  const float R_E_l =
    (std::pow(d_Cr_Einit, 2) - std::pow(R_E_r, 2)) / (2 * (R_E_r + d_Cr_Einit * std::cos(alpha)));
  if (R_E_l <= 0) {
    return false;
  }

  // If start_pose is prallel to goal_pose, we can know lateral deviation of eges of vehicle,
  // and detect lane departure.
  // Check left bound
  if (is_forward) {
    const float R_front_left =
      std::hypot(R_E_r + common_params.vehicle_width / 2, common_params.base_link2front);
    const double distance_to_left_bound = util::getDistanceToShoulderBoundary(lanes, arc_end_pose);
    const float left_deviation = R_front_left - R_E_r;
    if (std::abs(distance_to_left_bound) < left_deviation) {
      return false;
    }
  } else {  // Check right bound
    const float R_front_right =
      std::hypot(R_E_l + common_params.vehicle_width / 2, common_params.base_link2front);
    const float right_deviation = R_front_right - R_E_l;
    const double distance_to_right_bound = util::getDistanceToRightBoundary(lanes, start_pose);
    if (distance_to_right_bound < right_deviation) {
      return false;
    }
  }

  generateStraightPath(start_pose);

  // Generate arc path(left turn -> right turn)
  Pose Cl = calcOffsetPose(start_pose, 0, R_E_l, 0);
  float theta_l = std::acos(
    (std::pow(R_E_l, 2) + std::pow(R_E_l + R_E_r, 2) - std::pow(d_Cr_Einit, 2)) /
    (2 * R_E_l * (R_E_l + R_E_r)));
  theta_l = is_forward ? theta_l : -theta_l;

  PathWithLaneId path_turn_left =
    generateArcPath(Cl, R_E_l, -M_PI_2, normalizeRadian(-M_PI_2 + theta_l), is_forward, is_forward);

  PathWithLaneId path_turn_right = generateArcPath(
    Cr, R_E_r, normalizeRadian(psi + M_PI_2 + theta_l), M_PI_2, !is_forward, is_forward);
  // Need to add straight path to last right_turning for parking in parallel
  PathPointWithLaneId straight_point{};
  lanelet::ConstLanelet goal_lane;
  lanelet::utils::query::getClosestLanelet(lanes, goal_pose, &goal_lane);
  straight_point.point.pose = goal_pose;
  // Use z of previous point because z of goal is 0.
  // https://github.com/autowarefoundation/autoware.universe/issues/711
  straight_point.point.pose.position.z = path_turn_right.points.back().point.pose.position.z;
  straight_point.point.longitudinal_velocity_mps = 0.0;
  straight_point.lane_ids.push_back(goal_lane.id());
  path_turn_right.points.push_back(straight_point);

  path_turn_left.header = planner_data_->route_handler->getRouteHeader();
  path_turn_left.drivable_area = util::generateDrivableArea(
    lanes, common_params.drivable_area_resolution, common_params.vehicle_length, planner_data_);
  paths_.push_back(path_turn_left);

  path_turn_right.header = planner_data_->route_handler->getRouteHeader();
  path_turn_right.drivable_area = util::generateDrivableArea(
    lanes, common_params.drivable_area_resolution, common_params.vehicle_length, planner_data_);
  paths_.push_back(path_turn_right);

  Cr_.pose = Cr;
  Cr_.header = planner_data_->route_handler->getRouteHeader();
  Cl_.pose = Cl;
  Cl_.header = planner_data_->route_handler->getRouteHeader();
  start_pose_.pose = start_pose;
  start_pose_.header = planner_data_->route_handler->getRouteHeader();
  arc_end_pose_.pose = arc_end_pose;
  arc_end_pose_.header = planner_data_->route_handler->getRouteHeader();
  path_pose_array_ = convertToGeometryPoseArray(getFullPath());
  path_pose_array_.header = planner_data_->route_handler->getRouteHeader();

  return true;
}

PathWithLaneId GeometricParallelParking::generateArcPath(
  const Pose & center, const float radius, const float start_yaw, float end_yaw,
  const bool is_left_turn,  // is_left_turn means clockwise around center.
  const bool is_forward)
{
  PathWithLaneId path;

  const float velocity =
    is_forward ? parameters_.forward_parking_velocity : parameters_.backward_parking_velocity;
  const float yaw_interval = parameters_.arc_path_interval / radius;
  float yaw = start_yaw;
  if (is_left_turn) {
    if (end_yaw < start_yaw) end_yaw += M_PI_2;
    while (yaw < end_yaw) {
      PathPointWithLaneId p = generateArcPathPoint(center, radius, yaw, is_left_turn, is_forward);
      p.point.longitudinal_velocity_mps = velocity;
      path.points.push_back(p);
      yaw += yaw_interval;
    }
  } else {  // right_turn
    if (end_yaw > start_yaw) end_yaw -= M_PI_2;
    while (yaw > end_yaw) {
      PathPointWithLaneId p = generateArcPathPoint(center, radius, yaw, is_left_turn, is_forward);
      p.point.longitudinal_velocity_mps = velocity;
      path.points.push_back(p);
      yaw -= yaw_interval;
    }
  }

  PathPointWithLaneId p = generateArcPathPoint(center, radius, end_yaw, is_left_turn, is_forward);
  // If last parking turn, going straight is needed for parking in parallel.
  const bool is_final_turn = (is_left_turn && !is_forward) || (!is_left_turn && is_forward);
  p.point.longitudinal_velocity_mps = is_final_turn ? velocity : 0;
  path.points.push_back(p);

  return path;
}

PathPointWithLaneId GeometricParallelParking::generateArcPathPoint(
  const Pose & center, const float radius, const float yaw, const bool is_left_turn,
  const bool is_forward)
{
  Pose pose_centercoords;
  pose_centercoords.position.x = radius * std::cos(yaw);
  pose_centercoords.position.y = radius * std::sin(yaw);

  tf2::Quaternion quat;
  if ((is_left_turn && !is_forward) || (!is_left_turn && is_forward)) {
    quat.setRPY(0, 0, normalizeRadian(yaw - M_PI_2));
  } else {
    quat.setRPY(0, 0, normalizeRadian(yaw + M_PI_2));
  }
  pose_centercoords.orientation = tf2::toMsg(quat);

  PathPointWithLaneId p{};
  p.point.pose = transformPose(pose_centercoords, center);
  lanelet::ConstLanelet current_lane;
  planner_data_->route_handler->getClosestLaneletWithinRoute(p.point.pose, &current_lane);

  // Use z of lanelet closest point because z of goal is 0.
  // https://github.com/autowarefoundation/autoware.universe/issues/711
  double min_distance = std::numeric_limits<double>::max();
  for (const auto & pt : current_lane.centerline3d()) {
    const double distance =
      calcDistance2d(p.point.pose, lanelet::utils::conversion::toGeomMsgPt(pt));
    if (distance < min_distance) {
      min_distance = distance;
      p.point.pose.position.z = pt.z();
    }
  }

  p.lane_ids.push_back(current_lane.id());

  return p;
}

void GeometricParallelParking::setParams(
  const std::shared_ptr<const PlannerData> & planner_data, ParallelParkingParameters parameters)
{
  planner_data_ = planner_data;
  parameters_ = parameters;

  auto common_params = planner_data_->parameters;
  max_steer_rad_ = deg2rad(max_steer_deg_);

  R_E_min_ = common_params.wheel_base / std::tan(max_steer_rad_);
  R_Bl_min_ = std::hypot(
    R_E_min_ + common_params.wheel_tread / 2 + common_params.left_over_hang,
    common_params.wheel_base + common_params.front_overhang);
}
}  // namespace behavior_path_planner
