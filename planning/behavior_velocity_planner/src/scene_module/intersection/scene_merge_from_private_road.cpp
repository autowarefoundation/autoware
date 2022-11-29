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

#include <lanelet2_extension/regulatory_elements/road_marking.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <scene_module/intersection/scene_merge_from_private_road.hpp>
#include <scene_module/intersection/util.hpp>
#include <utilization/boost_geometry_helper.hpp>
#include <utilization/path_utilization.hpp>
#include <utilization/util.hpp>

#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <memory>
#include <string>
#include <vector>

namespace behavior_velocity_planner
{
namespace bg = boost::geometry;

MergeFromPrivateRoadModule::MergeFromPrivateRoadModule(
  const int64_t module_id, const int64_t lane_id,
  [[maybe_unused]] std::shared_ptr<const PlannerData> planner_data,
  const PlannerParam & planner_param, const rclcpp::Logger logger,
  const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock), lane_id_(lane_id)
{
  planner_param_ = planner_param;
  state_machine_.setState(StateMachine::State::STOP);
}

bool MergeFromPrivateRoadModule::modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason)
{
  debug_data_ = DebugData();
  *stop_reason = planning_utils::initializeStopReason(StopReason::MERGE_FROM_PRIVATE_ROAD);

  const auto input_path = *path;
  debug_data_.path_raw = input_path;

  StateMachine::State current_state = state_machine_.getState();
  RCLCPP_DEBUG(
    logger_, "lane_id = %ld, state = %s", lane_id_, StateMachine::toString(current_state).c_str());

  /* get current pose */
  geometry_msgs::msg::PoseStamped current_pose = planner_data_->current_pose;

  /* get lanelet map */
  const auto lanelet_map_ptr = planner_data_->route_handler_->getLaneletMapPtr();
  const auto routing_graph_ptr = planner_data_->route_handler_->getRoutingGraphPtr();

  /* get detection area */
  auto && [detection_lanelets, conflicting_lanelets] = util::getObjectiveLanelets(
    lanelet_map_ptr, routing_graph_ptr, lane_id_, planner_param_.detection_area_length,
    false /* tl_arrow_solid on does not matter here*/);
  if (detection_lanelets.empty()) {
    RCLCPP_DEBUG(logger_, "no detection area. skip computation.");
    return true;
  }
  const auto detection_area =
    util::getPolygon3dFromLanelets(detection_lanelets, planner_param_.detection_area_length);
  const std::vector<lanelet::CompoundPolygon3d> conflicting_area =
    util::getPolygon3dFromLanelets(conflicting_lanelets);
  debug_data_.detection_area = detection_area;

  /* set stop-line and stop-judgement-line for base_link */
  const auto private_path =
    extractPathNearExitOfPrivateRoad(*path, planner_data_->vehicle_info_.vehicle_length_m);
  const auto [stuck_line_idx_opt, stop_lines_idx_opt] = util::generateStopLine(
    lane_id_, detection_area, conflicting_area, planner_data_, planner_param_.stop_line_margin,
    0.0 /* unnecessary in merge_from_private */, false /* same */, path, *path,
    logger_.get_child("util"), clock_);
  if (!stop_lines_idx_opt.has_value()) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(logger_, *clock_, 1000 /* ms */, "setStopLineIdx fail");
    return false;
  }

  const auto & stop_lines_idx = stop_lines_idx_opt.value();
  const size_t stop_line_idx = stop_lines_idx.stop_line;
  if (stop_line_idx == 0) {
    RCLCPP_DEBUG(logger_, "stop line is at path[0], ignore planning.");
    return true;
  }

  debug_data_.virtual_wall_pose = planning_utils::getAheadPose(
    stop_line_idx, planner_data_->vehicle_info_.max_longitudinal_offset_m, *path);
  debug_data_.stop_point_pose = path->points.at(stop_line_idx).point.pose;
  const size_t first_inside_lane_idx = stop_lines_idx.first_inside_lane;
  debug_data_.first_collision_point = path->points.at(first_inside_lane_idx).point.pose.position;

  /* set stop speed */
  if (state_machine_.getState() == StateMachine::State::STOP) {
    constexpr double v = 0.0;
    planning_utils::setVelocityFromIndex(stop_line_idx, v, path);

    /* get stop point and stop factor */
    tier4_planning_msgs::msg::StopFactor stop_factor;
    stop_factor.stop_pose = debug_data_.stop_point_pose;
    stop_factor.stop_factor_points.emplace_back(debug_data_.first_collision_point);
    planning_utils::appendStopReason(stop_factor, stop_reason);

    const double signed_arc_dist_to_stop_point = motion_utils::calcSignedArcLength(
      path->points, current_pose.pose.position, path->points.at(stop_line_idx).point.pose.position);

    constexpr double distance_threshold = 2.0;
    if (
      signed_arc_dist_to_stop_point < distance_threshold &&
      planner_data_->isVehicleStopped(planner_param_.stop_duration_sec)) {
      state_machine_.setState(StateMachine::State::GO);
      if (signed_arc_dist_to_stop_point < -distance_threshold) {
        RCLCPP_ERROR(logger_, "Failed to stop near stop line but ego stopped. Change state to GO");
      }
    }

    return true;
  }

  return true;
}

autoware_auto_planning_msgs::msg::PathWithLaneId
MergeFromPrivateRoadModule::extractPathNearExitOfPrivateRoad(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const double extend_length)
{
  if (path.points.size() < 2) {
    return path;
  }

  autoware_auto_planning_msgs::msg::PathWithLaneId private_path = path;
  private_path.points.clear();

  double sum_dist = 0.0;
  bool prev_has_target_lane_id = false;
  for (int i = static_cast<int>(path.points.size()) - 2; i >= 0; i--) {
    bool has_target_lane_id = false;
    for (const auto path_id : path.points.at(i).lane_ids) {
      if (path_id == lane_id_) {
        has_target_lane_id = true;
      }
    }
    if (has_target_lane_id) {
      // add path point with target lane id
      // (lanelet with target lane id is exit of private road)
      private_path.points.emplace_back(path.points.at(i));
      prev_has_target_lane_id = true;
      continue;
    }
    if (prev_has_target_lane_id) {
      // extend path to the front
      private_path.points.emplace_back(path.points.at(i));
      sum_dist += tier4_autoware_utils::calcDistance2d(
        path.points.at(i).point.pose, path.points.at(i + 1).point.pose);
      if (sum_dist > extend_length) {
        break;
      }
    }
  }

  std::reverse(private_path.points.begin(), private_path.points.end());
  return private_path;
}
}  // namespace behavior_velocity_planner
