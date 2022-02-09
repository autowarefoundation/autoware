// Copyright 2021 Tier IV, Inc.
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

#include <lanelet2_extension/utility/utilities.hpp>
#include <scene_module/occlusion_spot/occlusion_spot_utils.hpp>
#include <scene_module/occlusion_spot/risk_predictive_braking.hpp>
#include <scene_module/occlusion_spot/scene_occlusion_spot_in_public_road.hpp>
#include <utilization/util.hpp>

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <memory>
#include <set>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
using occlusion_spot_utils::PossibleCollisionInfo;
using occlusion_spot_utils::ROAD_TYPE::PUBLIC;
namespace utils = occlusion_spot_utils;

OcclusionSpotInPublicModule::OcclusionSpotInPublicModule(
  const int64_t module_id, [[maybe_unused]] std::shared_ptr<const PlannerData> planner_data,
  const PlannerParam & planner_param, const rclcpp::Logger logger,
  const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock)
{
  param_ = planner_param;
}

bool OcclusionSpotInPublicModule::modifyPathVelocity(
  autoware_auto_planning_msgs::msg::PathWithLaneId * path,
  [[maybe_unused]] tier4_planning_msgs::msg::StopReason * stop_reason)
{
  debug_data_ = DebugData();
  if (path->points.size() < 2) {
    return true;
  }
  // set planner data
  {
    param_.v.max_stop_jerk = planner_data_->max_stop_jerk_threshold;
    param_.v.max_stop_accel = planner_data_->max_stop_acceleration_threshold;
    param_.v.v_ego = planner_data_->current_velocity->twist.linear.x;
    param_.v.a_ego = planner_data_->current_accel.get();
  }
  const geometry_msgs::msg::Pose ego_pose = planner_data_->current_pose.pose;
  const auto & lanelet_map_ptr = planner_data_->lanelet_map;
  const auto & routing_graph_ptr = planner_data_->routing_graph;
  const auto & traffic_rules_ptr = planner_data_->traffic_rules;
  const auto & dynamic_obj_arr_ptr = planner_data_->predicted_objects;

  if (!lanelet_map_ptr || !traffic_rules_ptr || !dynamic_obj_arr_ptr || !routing_graph_ptr) {
    return true;
  }
  PathWithLaneId clipped_path;
  utils::clipPathByLength(*path, clipped_path, param_.detection_area_length);
  PathWithLaneId interp_path;
  utils::splineInterpolate(clipped_path, 1.0, &interp_path, logger_);
  int closest_idx = -1;
  if (!planning_utils::calcClosestIndex<PathWithLaneId>(
        interp_path, ego_pose, closest_idx, param_.dist_thr, param_.angle_thr)) {
    return true;
  }
  // return if ego is final point of interpolated path
  if (closest_idx == static_cast<int>(interp_path.points.size()) - 1) return true;
  DetectionAreaIdx focus_area =
    extractTargetRoadArcLength(lanelet_map_ptr, param_.detection_area_length, *path, PUBLIC);
  if (!focus_area) return true;
  std::vector<PredictedObject> obj =
    utils::getParkedVehicles(*dynamic_obj_arr_ptr, param_, debug_data_.parked_vehicle_point);
  double offset_from_start_to_ego = utils::offsetFromStartToEgo(interp_path, ego_pose, closest_idx);
  // Note: Don't consider offset from path start to ego here
  std::vector<PossibleCollisionInfo> possible_collisions =
    utils::generatePossibleCollisionBehindParkedVehicle(
      interp_path, param_, offset_from_start_to_ego, obj);
  utils::filterCollisionByRoadType(possible_collisions, focus_area);
  utils::calcSlowDownPointsForPossibleCollision(0, interp_path, 0.0, possible_collisions);
  // Note: Consider offset from path start to ego here
  utils::handleCollisionOffset(possible_collisions, offset_from_start_to_ego, 0.0);
  // apply safe velocity using ebs and pbs deceleration
  utils::applySafeVelocityConsideringPossibleCollision(path, possible_collisions, param_);

  debug_data_.possible_collisions = possible_collisions;
  return true;
}

}  // namespace behavior_velocity_planner
