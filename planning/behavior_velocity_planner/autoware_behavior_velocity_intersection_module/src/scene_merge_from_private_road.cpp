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

#include "scene_merge_from_private_road.hpp"

#include "util.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/path_utilization.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/road_marking.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner
{
namespace bg = boost::geometry;

MergeFromPrivateRoadModule::MergeFromPrivateRoadModule(
  const int64_t module_id, const int64_t lane_id,
  [[maybe_unused]] std::shared_ptr<const PlannerData> planner_data,
  const PlannerParam & planner_param, const std::set<lanelet::Id> & associative_ids,
  const rclcpp::Logger logger, const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock),
  lane_id_(lane_id),
  associative_ids_(associative_ids)
{
  velocity_factor_.init(PlanningBehavior::MERGE);
  planner_param_ = planner_param;
  state_machine_.setState(StateMachine::State::STOP);
}

static std::optional<lanelet::ConstLanelet> getFirstConflictingLanelet(
  const lanelet::ConstLanelets & conflicting_lanelets,
  const InterpolatedPathInfo & interpolated_path_info,
  const autoware::universe_utils::LinearRing2d & footprint, const double vehicle_length)
{
  const auto & path_ip = interpolated_path_info.path;
  const auto [lane_start, end] = interpolated_path_info.lane_id_interval.value();
  const size_t vehicle_length_idx = static_cast<size_t>(vehicle_length / interpolated_path_info.ds);
  const size_t start =
    static_cast<size_t>(std::max<int>(0, static_cast<int>(lane_start) - vehicle_length_idx));

  for (size_t i = start; i <= end; ++i) {
    const auto & pose = path_ip.points.at(i).point.pose;
    const auto path_footprint = autoware::universe_utils::transformVector(
      footprint, autoware::universe_utils::pose2transform(pose));
    for (const auto & conflicting_lanelet : conflicting_lanelets) {
      const auto polygon_2d = conflicting_lanelet.polygon2d().basicPolygon();
      const bool intersects = bg::intersects(polygon_2d, path_footprint);
      if (intersects) {
        return std::make_optional(conflicting_lanelet);
      }
    }
  }
  return std::nullopt;
}

bool MergeFromPrivateRoadModule::modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason)
{
  debug_data_ = DebugData();
  *stop_reason = planning_utils::initializeStopReason(StopReason::MERGE_FROM_PRIVATE_ROAD);

  const auto input_path = *path;

  StateMachine::State current_state = state_machine_.getState();
  RCLCPP_DEBUG(
    logger_, "lane_id = %ld, state = %s", lane_id_, StateMachine::toString(current_state).c_str());

  /* get current pose */
  geometry_msgs::msg::Pose current_pose = planner_data_->current_odometry->pose;

  /* get lanelet map */
  const auto lanelet_map_ptr = planner_data_->route_handler_->getLaneletMapPtr();
  const auto routing_graph_ptr = planner_data_->route_handler_->getRoutingGraphPtr();

  /* spline interpolation */
  const auto interpolated_path_info_opt = util::generateInterpolatedPath(
    lane_id_, associative_ids_, *path, planner_param_.path_interpolation_ds, logger_);
  if (!interpolated_path_info_opt) {
    RCLCPP_DEBUG_SKIPFIRST_THROTTLE(logger_, *clock_, 1000 /* ms */, "splineInterpolate failed");
    RCLCPP_DEBUG(logger_, "===== plan end =====");
    return false;
  }
  const auto & interpolated_path_info = interpolated_path_info_opt.value();
  if (!interpolated_path_info.lane_id_interval) {
    RCLCPP_WARN(logger_, "Path has no interval on intersection lane %ld", lane_id_);
    RCLCPP_DEBUG(logger_, "===== plan end =====");
    return false;
  }

  const double baselink2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;
  const auto local_footprint = planner_data_->vehicle_info_.createFootprint(0.0, 0.0);
  if (!first_conflicting_lanelet_) {
    const auto conflicting_lanelets = getAttentionLanelets();
    first_conflicting_lanelet_ = getFirstConflictingLanelet(
      conflicting_lanelets, interpolated_path_info, local_footprint, baselink2front);
  }
  if (!first_conflicting_lanelet_) {
    return false;
  }
  const auto first_conflicting_lanelet = first_conflicting_lanelet_.value();

  const auto first_conflicting_idx_opt = util::getFirstPointInsidePolygonByFootprint(
    first_conflicting_lanelet.polygon3d(), interpolated_path_info, local_footprint, baselink2front);
  if (!first_conflicting_idx_opt) {
    return false;
  }
  // ==========================================================================================
  // first_conflicting_idx is calculated considering baselink2front already, so there is no need
  // to subtract baselink2front/ds here
  // ==========================================================================================
  const auto stopline_idx_ip = static_cast<size_t>(std::max<int>(
    0, static_cast<int>(first_conflicting_idx_opt.value()) -
         static_cast<int>(planner_param_.stopline_margin / planner_param_.path_interpolation_ds)));

  const auto stopline_idx_opt = util::insertPointIndex(
    interpolated_path_info.path.points.at(stopline_idx_ip).point.pose, path,
    planner_data_->ego_nearest_dist_threshold, planner_data_->ego_nearest_yaw_threshold);
  if (!stopline_idx_opt) {
    RCLCPP_DEBUG(logger_, "failed to insert stopline, ignore planning.");
    return true;
  }
  const auto stopline_idx = stopline_idx_opt.value();

  debug_data_.virtual_wall_pose = planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
  debug_data_.stop_point_pose = path->points.at(stopline_idx).point.pose;

  /* set stop speed */
  if (state_machine_.getState() == StateMachine::State::STOP) {
    constexpr double v = 0.0;
    planning_utils::setVelocityFromIndex(stopline_idx, v, path);

    /* get stop point and stop factor */
    tier4_planning_msgs::msg::StopFactor stop_factor;
    stop_factor.stop_pose = debug_data_.stop_point_pose;
    planning_utils::appendStopReason(stop_factor, stop_reason);
    const auto & stop_pose = path->points.at(stopline_idx).point.pose;
    velocity_factor_.set(
      path->points, planner_data_->current_odometry->pose, stop_pose, VelocityFactor::UNKNOWN);

    const double signed_arc_dist_to_stop_point = autoware::motion_utils::calcSignedArcLength(
      path->points, current_pose.position, path->points.at(stopline_idx).point.pose.position);

    if (
      signed_arc_dist_to_stop_point < planner_param_.stop_distance_threshold &&
      planner_data_->isVehicleStopped(planner_param_.stop_duration_sec)) {
      state_machine_.setState(StateMachine::State::GO);
      if (signed_arc_dist_to_stop_point < -planner_param_.stop_distance_threshold) {
        RCLCPP_ERROR(logger_, "Failed to stop near stop line but ego stopped. Change state to GO");
      }
    }

    return true;
  }

  return true;
}

lanelet::ConstLanelets MergeFromPrivateRoadModule::getAttentionLanelets() const
{
  const auto lanelet_map_ptr = planner_data_->route_handler_->getLaneletMapPtr();
  const auto routing_graph_ptr = planner_data_->route_handler_->getRoutingGraphPtr();

  const auto & assigned_lanelet = lanelet_map_ptr->laneletLayer.get(lane_id_);
  const auto conflicting_lanelets =
    lanelet::utils::getConflictingLanelets(routing_graph_ptr, assigned_lanelet);
  lanelet::ConstLanelets sibling_lanelets;
  for (const auto & previous_lanelet : routing_graph_ptr->previous(assigned_lanelet)) {
    sibling_lanelets.push_back(previous_lanelet);
    for (const auto & following_lanelet : routing_graph_ptr->following(previous_lanelet)) {
      if (lanelet::utils::contains(sibling_lanelets, following_lanelet)) {
        continue;
      }
      sibling_lanelets.push_back(following_lanelet);
    }
  }

  lanelet::ConstLanelets attention_lanelets;
  for (const auto & conflicting_lanelet : conflicting_lanelets) {
    if (lanelet::utils::contains(sibling_lanelets, conflicting_lanelet)) {
      continue;
    }
    attention_lanelets.push_back(conflicting_lanelet);
  }
  return attention_lanelets;
}

}  // namespace autoware::behavior_velocity_planner
