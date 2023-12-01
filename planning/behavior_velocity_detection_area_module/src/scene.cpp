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

#include "scene.hpp"

#include <behavior_velocity_planner_common/utilization/arc_lane_util.hpp>
#include <behavior_velocity_planner_common/utilization/util.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <lanelet2_core/geometry/Polygon.h>

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
namespace bg = boost::geometry;
using motion_utils::calcLongitudinalOffsetPose;
using motion_utils::calcSignedArcLength;

DetectionAreaModule::DetectionAreaModule(
  const int64_t module_id, const int64_t lane_id,
  const lanelet::autoware::DetectionArea & detection_area_reg_elem,
  const PlannerParam & planner_param, const rclcpp::Logger logger,
  const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock),
  lane_id_(lane_id),
  detection_area_reg_elem_(detection_area_reg_elem),
  state_(State::GO),
  planner_param_(planner_param)
{
  velocity_factor_.init(PlanningBehavior::USER_DEFINED_DETECTION_AREA);
}

LineString2d DetectionAreaModule::getStopLineGeometry2d() const
{
  const lanelet::ConstLineString3d stop_line = detection_area_reg_elem_.stopLine();
  return planning_utils::extendLine(
    stop_line[0], stop_line[1], planner_data_->stop_line_extend_length);
}

bool DetectionAreaModule::modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason)
{
  // Store original path
  const auto original_path = *path;

  // Reset data
  debug_data_ = DebugData();
  debug_data_.base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;
  *stop_reason = planning_utils::initializeStopReason(StopReason::DETECTION_AREA);

  // Find obstacles in detection area
  const auto obstacle_points = getObstaclePoints();
  debug_data_.obstacle_points = obstacle_points;
  if (!obstacle_points.empty()) {
    last_obstacle_found_time_ = std::make_shared<const rclcpp::Time>(clock_->now());
  }

  // Get stop line geometry
  const auto stop_line = getStopLineGeometry2d();

  // Get self pose
  const auto & self_pose = planner_data_->current_odometry->pose;
  const size_t current_seg_idx = findEgoSegmentIndex(path->points);

  // Get stop point
  const auto stop_point = arc_lane_utils::createTargetPoint(
    original_path, stop_line, lane_id_, planner_param_.stop_margin,
    planner_data_->vehicle_info_.max_longitudinal_offset_m);
  if (!stop_point) {
    return true;
  }

  const auto & stop_point_idx = stop_point->first;
  const auto & stop_pose = stop_point->second;
  const size_t stop_line_seg_idx = planning_utils::calcSegmentIndexFromPointIndex(
    path->points, stop_pose.position, stop_point_idx);

  auto modified_stop_pose = stop_pose;
  size_t modified_stop_line_seg_idx = stop_line_seg_idx;

  const auto is_stopped = planner_data_->isVehicleStopped(0.0);
  const auto stop_dist = calcSignedArcLength(
    path->points, self_pose.position, current_seg_idx, stop_pose.position, stop_line_seg_idx);

  // Don't re-approach when the ego stops closer to the stop point than hold_stop_margin_distance
  if (is_stopped && stop_dist < planner_param_.hold_stop_margin_distance) {
    const auto ego_pos_on_path =
      calcLongitudinalOffsetPose(original_path.points, self_pose.position, 0.0);

    if (!ego_pos_on_path) {
      return false;
    }

    modified_stop_pose = ego_pos_on_path.value();
    modified_stop_line_seg_idx = current_seg_idx;
  }

  setDistance(stop_dist);

  // Check state
  setSafe(canClearStopState());
  if (isActivated()) {
    state_ = State::GO;
    last_obstacle_found_time_ = {};
    return true;
  }

  // Force ignore objects after dead_line
  if (planner_param_.use_dead_line) {
    // Use '-' for margin because it's the backward distance from stop line
    const auto dead_line_point = arc_lane_utils::createTargetPoint(
      original_path, stop_line, lane_id_, -planner_param_.dead_line_margin,
      planner_data_->vehicle_info_.max_longitudinal_offset_m);

    if (dead_line_point) {
      const size_t dead_line_point_idx = dead_line_point->first;
      const auto & dead_line_pose = dead_line_point->second;

      const size_t dead_line_seg_idx = planning_utils::calcSegmentIndexFromPointIndex(
        path->points, dead_line_pose.position, dead_line_point_idx);

      debug_data_.dead_line_poses.push_back(dead_line_pose);

      const double dist_from_ego_to_dead_line = calcSignedArcLength(
        original_path.points, self_pose.position, current_seg_idx, dead_line_pose.position,
        dead_line_seg_idx);
      if (dist_from_ego_to_dead_line < 0.0) {
        RCLCPP_WARN(logger_, "[detection_area] vehicle is over dead line");
        setSafe(true);
        return true;
      }
    }
  }

  // Ignore objects detected after stop_line if not in STOP state
  const double dist_from_ego_to_stop = calcSignedArcLength(
    original_path.points, self_pose.position, current_seg_idx, stop_pose.position,
    stop_line_seg_idx);
  if (
    state_ != State::STOP &&
    dist_from_ego_to_stop < -planner_param_.distance_to_judge_over_stop_line) {
    setSafe(true);
    return true;
  }

  // Ignore objects if braking distance is not enough
  if (planner_param_.use_pass_judge_line) {
    if (state_ != State::STOP && !hasEnoughBrakingDistance(self_pose, stop_point->second)) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, std::chrono::milliseconds(1000).count(),
        "[detection_area] vehicle is over stop border");
      setSafe(true);
      return true;
    }
  }

  // Insert stop point
  state_ = State::STOP;
  planning_utils::insertStopPoint(modified_stop_pose.position, modified_stop_line_seg_idx, *path);

  // For virtual wall
  debug_data_.stop_poses.push_back(stop_point->second);

  // Create StopReason
  {
    StopFactor stop_factor{};
    stop_factor.stop_pose = stop_point->second;
    stop_factor.stop_factor_points = obstacle_points;
    planning_utils::appendStopReason(stop_factor, stop_reason);
    velocity_factor_.set(
      path->points, planner_data_->current_odometry->pose, stop_point->second,
      VelocityFactor::UNKNOWN);
  }

  // Create legacy StopReason
  {
    const auto insert_idx = stop_point->first + 1;

    if (
      !first_stop_path_point_index_ ||
      static_cast<int>(insert_idx) < first_stop_path_point_index_) {
      debug_data_.first_stop_pose = stop_point->second;
      first_stop_path_point_index_ = static_cast<int>(insert_idx);
    }
  }

  return true;
}

// calc smallest enclosing circle with average O(N) algorithm
// reference:
// https://erickimphotography.com/blog/wp-content/uploads/2018/09/Computational-Geometry-Algorithms-and-Applications-3rd-Ed.pdf
std::pair<lanelet::BasicPoint2d, double> calcSmallestEnclosingCircle(
  const lanelet::ConstPolygon2d & poly)
{
  // The `eps` is used to avoid precision bugs in circle inclusion checks.
  // If the value of `eps` is too small, this function doesn't work well. More than 1e-10 is
  // recommended.
  const double eps = 1e-5;
  lanelet::BasicPoint2d center(0.0, 0.0);
  double radius_squared = 0.0;

  auto cross = [](const lanelet::BasicPoint2d & p1, const lanelet::BasicPoint2d & p2) -> double {
    return p1.x() * p2.y() - p1.y() * p2.x();
  };

  auto make_circle_3 = [&](
                         const lanelet::BasicPoint2d & p1, const lanelet::BasicPoint2d & p2,
                         const lanelet::BasicPoint2d & p3) -> void {
    // reference for circumcenter vector https://en.wikipedia.org/wiki/Circumscribed_circle
    const double A = (p2 - p3).squaredNorm();
    const double B = (p3 - p1).squaredNorm();
    const double C = (p1 - p2).squaredNorm();
    const double S = cross(p2 - p1, p3 - p1);
    if (std::abs(S) < eps) return;
    center = (A * (B + C - A) * p1 + B * (C + A - B) * p2 + C * (A + B - C) * p3) / (4 * S * S);
    radius_squared = (center - p1).squaredNorm() + eps;
  };

  auto make_circle_2 =
    [&](const lanelet::BasicPoint2d & p1, const lanelet::BasicPoint2d & p2) -> void {
    center = (p1 + p2) * 0.5;
    radius_squared = (center - p1).squaredNorm() + eps;
  };

  auto in_circle = [&](const lanelet::BasicPoint2d & p) -> bool {
    return (center - p).squaredNorm() <= radius_squared;
  };

  // mini disc
  for (size_t i = 1; i < poly.size(); i++) {
    const auto p1 = poly[i].basicPoint2d();
    if (in_circle(p1)) continue;

    // mini disc with point
    const auto p0 = poly[0].basicPoint2d();
    make_circle_2(p0, p1);
    for (size_t j = 0; j < i; j++) {
      const auto p2 = poly[j].basicPoint2d();
      if (in_circle(p2)) continue;

      // mini disc with two points
      make_circle_2(p1, p2);
      for (size_t k = 0; k < j; k++) {
        const auto p3 = poly[k].basicPoint2d();
        if (in_circle(p3)) continue;

        // mini disc with tree points
        make_circle_3(p1, p2, p3);
      }
    }
  }

  return std::make_pair(center, radius_squared);
}

std::vector<geometry_msgs::msg::Point> DetectionAreaModule::getObstaclePoints() const
{
  std::vector<geometry_msgs::msg::Point> obstacle_points;

  const auto detection_areas = detection_area_reg_elem_.detectionAreas();
  const auto & points = *(planner_data_->no_ground_pointcloud);

  for (const auto & detection_area : detection_areas) {
    const auto poly = lanelet::utils::to2D(detection_area);
    const auto circle = calcSmallestEnclosingCircle(poly);
    for (const auto p : points) {
      const double squared_dist = (circle.first.x() - p.x) * (circle.first.x() - p.x) +
                                  (circle.first.y() - p.y) * (circle.first.y() - p.y);
      if (squared_dist <= circle.second) {
        if (bg::within(Point2d{p.x, p.y}, poly.basicPolygon())) {
          obstacle_points.push_back(tier4_autoware_utils::createPoint(p.x, p.y, p.z));
          // get all obstacle point becomes high computation cost so skip if any point is found
          break;
        }
      }
    }
  }

  return obstacle_points;
}

bool DetectionAreaModule::canClearStopState() const
{
  // vehicle can clear stop state if the obstacle has never appeared in detection area
  if (!last_obstacle_found_time_) {
    return true;
  }

  // vehicle can clear stop state if the certain time has passed since the obstacle disappeared
  const auto elapsed_time = clock_->now() - *last_obstacle_found_time_;
  if (elapsed_time.seconds() >= planner_param_.state_clear_time) {
    return true;
  }

  // rollback in simulation mode
  if (elapsed_time.seconds() < 0.0) {
    return true;
  }

  return false;
}

bool DetectionAreaModule::hasEnoughBrakingDistance(
  const geometry_msgs::msg::Pose & self_pose, const geometry_msgs::msg::Pose & line_pose) const
{
  // get vehicle info and compute pass_judge_line_distance
  const auto current_velocity = planner_data_->current_velocity->twist.linear.x;
  const double max_acc = planner_data_->max_stop_acceleration_threshold;
  const double delay_response_time = planner_data_->delay_response_time;
  const double pass_judge_line_distance =
    planning_utils::calcJudgeLineDistWithAccLimit(current_velocity, max_acc, delay_response_time);

  // prevent from being judged as not having enough distance when the current velocity is zero
  // and the vehicle crosses the stop line
  if (current_velocity < 1e-3) {
    return true;
  }

  return arc_lane_utils::calcSignedDistance(self_pose, line_pose.position) >
         pass_judge_line_distance;
}
}  // namespace behavior_velocity_planner
