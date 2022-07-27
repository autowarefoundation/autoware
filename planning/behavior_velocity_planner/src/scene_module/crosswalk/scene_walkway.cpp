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
#include <scene_module/crosswalk/scene_walkway.hpp>
#include <utilization/util.hpp>

#include <cmath>

namespace behavior_velocity_planner
{
namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point>;
using Line = bg::model::linestring<Point>;
using motion_utils::calcLongitudinalOffsetPose;
using motion_utils::calcSignedArcLength;
using motion_utils::findNearestSegmentIndex;
using motion_utils::insertTargetPoint;
using tier4_autoware_utils::createPoint;
using tier4_autoware_utils::getPose;

WalkwayModule::WalkwayModule(
  const int64_t module_id, const lanelet::ConstLanelet & walkway,
  const PlannerParam & planner_param, const rclcpp::Logger logger,
  const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock),
  module_id_(module_id),
  walkway_(walkway),
  state_(State::APPROACH)
{
  planner_param_ = planner_param;
}

boost::optional<std::pair<double, geometry_msgs::msg::Point>> WalkwayModule::getStopLine(
  const PathWithLaneId & ego_path) const
{
  const auto & ego_pos = planner_data_->current_pose.pose.position;

  const auto stop_line = getStopLineFromMap(module_id_, planner_data_, "crosswalk_id");
  if (stop_line) {
    const auto intersects = getLinestringIntersects(
      ego_path, lanelet::utils::to2D(stop_line.get()).basicLineString(), ego_pos, 2);
    if (!intersects.empty()) {
      const auto p_stop_line =
        createPoint(intersects.front().x(), intersects.front().y(), ego_pos.z);
      const auto dist_ego_to_stop = calcSignedArcLength(ego_path.points, ego_pos, p_stop_line);
      return std::make_pair(dist_ego_to_stop, p_stop_line);
    }
  }

  {
    if (!path_intersects_.empty()) {
      const auto p_stop_line = path_intersects_.front();
      const auto dist_ego_to_stop = calcSignedArcLength(ego_path.points, ego_pos, p_stop_line) -
                                    planner_param_.stop_line_distance;
      return std::make_pair(dist_ego_to_stop, p_stop_line);
    }
  }

  return {};
}

bool WalkwayModule::modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason)
{
  const auto & base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;

  debug_data_ = DebugData();
  debug_data_.base_link2front = base_link2front;
  *stop_reason = planning_utils::initializeStopReason(StopReason::WALKWAY);

  const auto input = *path;

  path_intersects_.clear();

  const auto & ego_pos = planner_data_->current_pose.pose.position;
  const auto intersects =
    getPolygonIntersects(input, walkway_.polygon2d().basicPolygon(), ego_pos, 2);

  for (const auto & p : intersects) {
    path_intersects_.push_back(createPoint(p.x(), p.y(), ego_pos.z));
  }

  if (path_intersects_.empty()) {
    return false;
  }

  if (state_ == State::APPROACH) {
    const auto p_stop_line = getStopLine(input);
    if (!p_stop_line) {
      return false;
    }

    const auto & p_stop = p_stop_line.get().second;
    const auto margin = planner_param_.stop_line_distance + base_link2front;
    const auto stop_pose = calcLongitudinalOffsetPose(input.points, p_stop, -margin);

    if (!stop_pose) {
      return false;
    }

    insertStopPoint(stop_pose.get().position, *path);

    /* get stop point and stop factor */
    StopFactor stop_factor;
    stop_factor.stop_pose = stop_pose.get();
    stop_factor.stop_factor_points.push_back(path_intersects_.front());
    planning_utils::appendStopReason(stop_factor, stop_reason);

    // use arc length to identify if ego vehicle is in front of walkway stop or not.
    const double signed_arc_dist_to_stop_point =
      calcSignedArcLength(input.points, ego_pos, stop_pose.get().position);

    const double distance_threshold = 1.0;
    debug_data_.stop_judge_range = distance_threshold;

    const auto stop_at_stop_point =
      signed_arc_dist_to_stop_point < distance_threshold &&
      planner_data_->isVehicleStopped(planner_param_.stop_duration_sec);

    if (stop_at_stop_point) {
      // If ego vehicle is after walkway stop and stopped then move to stop state
      state_ = State::STOP;
      if (signed_arc_dist_to_stop_point < -distance_threshold) {
        RCLCPP_ERROR(
          logger_, "Failed to stop near walkway but ego stopped. Change state to STOPPED");
      }
    }

    return true;

  } else if (state_ == State::STOP) {
    if (planner_data_->isVehicleStopped()) {
      state_ = State::SURPASSED;
    }
  } else if (state_ == State::SURPASSED) {
  }

  return true;
}

void WalkwayModule::insertStopPoint(
  const geometry_msgs::msg::Point & stop_point, PathWithLaneId & output)
{
  const size_t base_idx = findNearestSegmentIndex(output.points, stop_point);
  const auto insert_idx = insertTargetPoint(base_idx, stop_point, output.points);

  if (!insert_idx) {
    return;
  }

  for (size_t i = insert_idx.get(); i < output.points.size(); ++i) {
    output.points.at(i).point.longitudinal_velocity_mps = 0.0;
  }

  debug_data_.stop_poses.push_back(getPose(output.points.at(insert_idx.get())));
}
}  // namespace behavior_velocity_planner
