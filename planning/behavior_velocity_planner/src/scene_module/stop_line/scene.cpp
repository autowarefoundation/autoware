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
#include <utilization/util.hpp>

#include <algorithm>
#include <vector>

namespace behavior_velocity_planner
{

namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Line = bg::model::linestring<Point>;
using motion_utils::calcLongitudinalOffsetPoint;
using motion_utils::calcLongitudinalOffsetPose;
using motion_utils::calcSignedArcLength;
using motion_utils::insertTargetPoint;
using tier4_autoware_utils::createPoint;
using tier4_autoware_utils::getPoint;
using tier4_autoware_utils::getPose;

namespace
{
std::vector<Point> getLinestringIntersects(
  const PathWithLaneId & ego_path, const Line & linestring,
  const geometry_msgs::msg::Point & ego_pos,
  const size_t max_num = std::numeric_limits<size_t>::max())
{
  std::vector<Point> intersects{};

  bool found_max_num = false;
  for (size_t i = 0; i < ego_path.points.size() - 1; ++i) {
    const auto & p_back = ego_path.points.at(i).point.pose.position;
    const auto & p_front = ego_path.points.at(i + 1).point.pose.position;
    const Line segment{{p_back.x, p_back.y}, {p_front.x, p_front.y}};

    std::vector<Point> tmp_intersects{};
    bg::intersection(segment, linestring, tmp_intersects);

    for (const auto & p : tmp_intersects) {
      intersects.push_back(p);
      if (intersects.size() == max_num) {
        found_max_num = true;
        break;
      }
    }

    if (found_max_num) {
      break;
    }
  }

  const auto compare = [&](const Point & p1, const Point & p2) {
    const auto dist_l1 =
      calcSignedArcLength(ego_path.points, size_t(0), createPoint(p1.x(), p1.y(), ego_pos.z));

    const auto dist_l2 =
      calcSignedArcLength(ego_path.points, size_t(0), createPoint(p2.x(), p2.y(), ego_pos.z));

    return dist_l1 < dist_l2;
  };

  std::sort(intersects.begin(), intersects.end(), compare);

  return intersects;
}
}  // namespace

StopLineModule::StopLineModule(
  const int64_t module_id, const size_t lane_id, const lanelet::ConstLineString3d & stop_line,
  const PlannerParam & planner_param, const rclcpp::Logger logger,
  const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock),
  module_id_(module_id),
  stop_line_(stop_line),
  lane_id_(lane_id),
  state_(State::APPROACH)
{
  planner_param_ = planner_param;
}

bool StopLineModule::modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason)
{
  const auto & base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;
  debug_data_ = DebugData();
  debug_data_.base_link2front = base_link2front;
  *stop_reason = planning_utils::initializeStopReason(StopReason::STOP_LINE);

  const auto ego_path = *path;
  const auto & ego_pos = planner_data_->current_pose.pose.position;

  const auto stop_line = planning_utils::extendLine(
    stop_line_[0], stop_line_[1], planner_data_->stop_line_extend_length);

  debug_data_.search_stopline = stop_line;

  const auto intersects = getLinestringIntersects(ego_path, stop_line, ego_pos, 1);

  if (intersects.empty()) {
    return false;
  }

  const auto p_stop_line = createPoint(intersects.front().x(), intersects.front().y(), ego_pos.z);
  const auto margin = planner_param_.stop_margin + base_link2front;
  const auto stop_pose = calcLongitudinalOffsetPose(ego_path.points, p_stop_line, -margin);

  if (!stop_pose) {
    return false;
  }

  StopFactor stop_factor;
  stop_factor.stop_pose = stop_pose.get();
  stop_factor.stop_factor_points.push_back(p_stop_line);

  /**
   * @brief : calculate signed arc length consider stop margin from stop line
   *
   * |----------------------------|
   * s---ego----------x--|--------g
   */
  const auto signed_arc_dist_to_stop_point =
    calcSignedArcLength(ego_path.points, ego_pos, stop_pose.get().position);

  switch (state_) {
    case State::APPROACH: {
      planning_utils::insertStopPoint(stop_pose.get().position, *path);
      planning_utils::appendStopReason(stop_factor, stop_reason);

      debug_data_.stop_pose = stop_pose.get();

      if (
        signed_arc_dist_to_stop_point < planner_param_.hold_stop_margin_distance &&
        planner_data_->isVehicleStopped()) {
        RCLCPP_INFO(logger_, "APPROACH -> STOPPED");

        state_ = State::STOPPED;
        stopped_time_ = std::make_shared<const rclcpp::Time>(clock_->now());

        if (signed_arc_dist_to_stop_point < -planner_param_.hold_stop_margin_distance) {
          RCLCPP_ERROR(
            logger_, "Failed to stop near stop line but ego stopped. Change state to STOPPED");
        }
      }

      break;
    }

    case State::STOPPED: {
      const auto ego_pos_on_path = calcLongitudinalOffsetPoint(ego_path.points, ego_pos, 0.0);

      if (!ego_pos_on_path) {
        break;
      }

      planning_utils::insertStopPoint(ego_pos_on_path.get(), *path);
      planning_utils::appendStopReason(stop_factor, stop_reason);

      debug_data_.stop_pose = stop_pose.get();

      const auto elapsed_time = (clock_->now() - *stopped_time_).seconds();

      if (planner_param_.stop_duration_sec < elapsed_time) {
        RCLCPP_INFO(logger_, "STOPPED -> START");
        state_ = State::START;
      }

      break;
    }

    case State::START: {
      // Initialize if vehicle is far from stop_line
      if (planner_param_.use_initialization_stop_line_state) {
        if (signed_arc_dist_to_stop_point > planner_param_.hold_stop_margin_distance) {
          RCLCPP_INFO(logger_, "START -> APPROACH");
          state_ = State::APPROACH;
        }
      }

      break;
    }

    default:
      RCLCPP_ERROR(logger_, "Unknown state.");
  }

  return true;
}
}  // namespace behavior_velocity_planner
