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
#include <utilization/interpolate.hpp>
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
  state_machine_.setState(State::STOP);
}

bool MergeFromPrivateRoadModule::modifyPathVelocity(
  autoware_auto_planning_msgs::msg::PathWithLaneId * path,
  tier4_planning_msgs::msg::StopReason * stop_reason)
{
  debug_data_ = DebugData();
  *stop_reason = planning_utils::initializeStopReason(
    tier4_planning_msgs::msg::StopReason::MERGE_FROM_PRIVATE_ROAD);

  const auto input_path = *path;
  debug_data_.path_raw = input_path;

  State current_state = state_machine_.getState();
  RCLCPP_DEBUG(logger_, "lane_id = %ld, state = %s", lane_id_, toString(current_state).c_str());

  /* get current pose */
  geometry_msgs::msg::PoseStamped current_pose = planner_data_->current_pose;

  /* get lanelet map */
  const auto lanelet_map_ptr = planner_data_->lanelet_map;
  const auto routing_graph_ptr = planner_data_->routing_graph;

  /* get detection area */
  std::vector<lanelet::ConstLanelets> detection_area_lanelets;
  std::vector<lanelet::ConstLanelets> conflicting_area_lanelets;

  util::getObjectiveLanelets(
    lanelet_map_ptr, routing_graph_ptr, lane_id_, planner_param_.intersection_param,
    &conflicting_area_lanelets, &detection_area_lanelets, logger_);
  std::vector<lanelet::CompoundPolygon3d> conflicting_areas = util::getPolygon3dFromLaneletsVec(
    conflicting_area_lanelets, planner_param_.intersection_param.detection_area_length);
  if (conflicting_areas.empty()) {
    RCLCPP_DEBUG(logger_, "no detection area. skip computation.");
    return true;
  }
  debug_data_.detection_area = conflicting_areas;

  /* set stop-line and stop-judgement-line for base_link */
  int stop_line_idx = -1;
  int judge_line_idx = -1;
  int first_idx_inside_lane = -1;
  const auto private_path =
    extractPathNearExitOfPrivateRoad(*path, planner_data_->vehicle_info_.vehicle_length_m);
  if (!util::generateStopLine(
        lane_id_, conflicting_areas, planner_data_, planner_param_.intersection_param, path,
        private_path, &stop_line_idx, &judge_line_idx, &first_idx_inside_lane,
        logger_.get_child("util"))) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(logger_, *clock_, 1000 /* ms */, "setStopLineIdx fail");
    return false;
  }

  if (stop_line_idx <= 0 || judge_line_idx <= 0) {
    RCLCPP_DEBUG(logger_, "stop line or judge line is at path[0], ignore planning.");
    return true;
  }

  debug_data_.virtual_wall_pose = util::getAheadPose(
    stop_line_idx, planner_data_->vehicle_info_.max_longitudinal_offset_m, *path);
  debug_data_.stop_point_pose = path->points.at(stop_line_idx).point.pose;
  if (first_idx_inside_lane != -1) {
    debug_data_.first_collision_point = path->points.at(first_idx_inside_lane).point.pose.position;
  }

  /* set stop speed */
  if (state_machine_.getState() == State::STOP) {
    constexpr double stop_vel = 0.0;
    const double decel_vel = planner_param_.intersection_param.decel_velocity;
    double v = (has_traffic_light_ && turn_direction_ == "straight") ? decel_vel : stop_vel;
    util::setVelocityFrom(stop_line_idx, v, path);

    /* get stop point and stop factor */
    if (v == stop_vel) {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = debug_data_.stop_point_pose;
      stop_factor.stop_factor_points.emplace_back(debug_data_.first_collision_point);
      planning_utils::appendStopReason(stop_factor, stop_reason);
    }

    const double signed_arc_dist_to_stop_point = tier4_autoware_utils::calcSignedArcLength(
      path->points, current_pose.pose.position, path->points.at(stop_line_idx).point.pose.position);

    constexpr double distance_threshold = 2.0;
    if (
      signed_arc_dist_to_stop_point < distance_threshold &&
      planner_data_->isVehicleStopped(planner_param_.stop_duration_sec)) {
      state_machine_.setState(State::GO);
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

void MergeFromPrivateRoadModule::StateMachine::setState(State state) { state_ = state; }

void MergeFromPrivateRoadModule::StateMachine::setMarginTime(const double t) { margin_time_ = t; }

MergeFromPrivateRoadModule::State MergeFromPrivateRoadModule::StateMachine::getState()
{
  return state_;
}
}  // namespace behavior_velocity_planner
