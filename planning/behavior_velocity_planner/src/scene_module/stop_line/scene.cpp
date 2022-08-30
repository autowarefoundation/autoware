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

#include <motion_utils/trajectory/trajectory.hpp>
#include <scene_module/stop_line/scene.hpp>
#include <utilization/arc_lane_util.hpp>
#include <utilization/util.hpp>

#include <algorithm>
#include <vector>

namespace behavior_velocity_planner
{
namespace bg = boost::geometry;

StopLineModule::StopLineModule(
  const int64_t module_id, const size_t lane_id, const lanelet::ConstLineString3d & stop_line,
  const PlannerParam & planner_param, const rclcpp::Logger logger,
  const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock),
  lane_id_(lane_id),
  stop_line_(stop_line),
  state_(State::APPROACH)
{
  planner_param_ = planner_param;
}

bool StopLineModule::modifyPathVelocity(
  autoware_auto_planning_msgs::msg::PathWithLaneId * path,
  tier4_planning_msgs::msg::StopReason * stop_reason)
{
  debug_data_ = DebugData();
  if (path->points.empty()) return true;
  const auto base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;
  debug_data_.base_link2front = base_link2front;
  first_stop_path_point_index_ = static_cast<int>(path->points.size()) - 1;
  *stop_reason =
    planning_utils::initializeStopReason(tier4_planning_msgs::msg::StopReason::STOP_LINE);

  const LineString2d stop_line = planning_utils::extendLine(
    stop_line_[0], stop_line_[1], planner_data_->stop_line_extend_length);

  // Calculate stop pose and insert index
  const auto stop_point = arc_lane_utils::createTargetPoint(
    *path, stop_line, lane_id_, planner_param_.stop_margin,
    planner_data_->vehicle_info_.max_longitudinal_offset_m);

  // If no collision found, do nothing
  if (!stop_point) {
    RCLCPP_DEBUG_THROTTLE(logger_, *clock_, 5000 /* ms */, "is no collision");
    return true;
  }

  const auto stop_point_idx = stop_point->first;
  auto stop_pose = stop_point->second;
  stop_pose.position.z = (stop_line_[0].z() + stop_line_[1].z()) / 2.0;

  /**
   * @brief : calculate signed arc length consider stop margin from stop line
   *
   * |----------------------------|
   * s---ego----------x--|--------g
   */
  const size_t stop_line_seg_idx = planning_utils::calcSegmentIndexFromPointIndex(
    path->points, stop_pose.position, stop_point_idx);
  const double stop_line_margin = base_link2front + planner_param_.stop_margin;
  const size_t current_seg_idx = findEgoSegmentIndex(path->points);
  const double signed_arc_dist_to_stop_point =
    motion_utils::calcSignedArcLength(
      path->points, planner_data_->current_pose.pose.position, current_seg_idx, stop_pose.position,
      stop_line_seg_idx) -
    stop_line_margin;
  if (state_ == State::APPROACH) {
    // Insert stop pose
    planning_utils::insertStopPoint(stop_pose.position, stop_line_seg_idx, *path);

    // Update first stop index
    first_stop_path_point_index_ = static_cast<int>(stop_point_idx);
    debug_data_.stop_pose = stop_pose;

    // Get stop point and stop factor
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = stop_pose;
      stop_factor.stop_factor_points.push_back(getCenterOfStopLine(stop_line_));
      planning_utils::appendStopReason(stop_factor, stop_reason);
    }

    // Move to stopped state if stopped
    if (
      signed_arc_dist_to_stop_point < planner_param_.stop_check_dist &&
      planner_data_->isVehicleStopped(planner_param_.stop_duration_sec)) {
      RCLCPP_INFO(logger_, "APPROACH -> STOPPED");
      state_ = State::STOPPED;
      if (signed_arc_dist_to_stop_point < -planner_param_.stop_check_dist) {
        RCLCPP_ERROR(
          logger_, "Failed to stop near stop line but ego stopped. Change state to STOPPED");
      }
    }
  } else if (state_ == State::STOPPED) {
    // Change state after vehicle departure
    if (!planner_data_->isVehicleStopped()) {
      RCLCPP_INFO(logger_, "STOPPED -> START");
      state_ = State::START;
    }
  } else if (state_ == State::START) {
    // Initialize if vehicle is far from stop_line
    if (planner_param_.use_initialization_stop_line_state) {
      if (signed_arc_dist_to_stop_point > planner_param_.stop_check_dist) {
        RCLCPP_INFO(logger_, "START -> APPROACH");
        state_ = State::APPROACH;
      }
    }
  }

  return true;
}

geometry_msgs::msg::Point StopLineModule::getCenterOfStopLine(
  const lanelet::ConstLineString3d & stop_line)
{
  geometry_msgs::msg::Point center_point;
  center_point.x = (stop_line[0].x() + stop_line[1].x()) / 2.0;
  center_point.y = (stop_line[0].y() + stop_line[1].y()) / 2.0;
  center_point.z = (stop_line[0].z() + stop_line[1].z()) / 2.0;
  return center_point;
}
}  // namespace behavior_velocity_planner
