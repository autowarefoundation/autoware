// Copyright 2020 Tier IV, Inc. All rights reserved.
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

#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <scene_module/occlusion_spot/occlusion_spot_utils.hpp>
#include <scene_module/occlusion_spot/risk_predictive_braking.hpp>
#include <scene_module/occlusion_spot/scene_occlusion_spot.hpp>
#include <utilization/boost_geometry_helper.hpp>
#include <utilization/path_utilization.hpp>
#include <utilization/util.hpp>

#include <boost/optional.hpp>

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <memory>
#include <vector>

namespace behavior_velocity_planner
{
namespace utils = occlusion_spot_utils;

OcclusionSpotModule::OcclusionSpotModule(
  const int64_t module_id, [[maybe_unused]] std::shared_ptr<const PlannerData> planner_data,
  const PlannerParam & planner_param, const rclcpp::Logger logger,
  const rclcpp::Clock::SharedPtr clock,
  const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher)
: SceneModuleInterface(module_id, logger, clock), publisher_(publisher), param_(planner_param)
{
  if (param_.detection_method == utils::DETECTION_METHOD::OCCUPANCY_GRID) {
    debug_data_.detection_type = "occupancy";
    //! occupancy grid limitation( 100 * 100 )
    param_.detection_area_length = std::min(50.0, param_.detection_area_length);
  } else if (param_.detection_method == utils::DETECTION_METHOD::PREDICTED_OBJECT) {
    debug_data_.detection_type = "object";
  }
  if (param_.use_partition_lanelet) {
    const lanelet::LaneletMapConstPtr & ll = planner_data->route_handler_->getLaneletMapPtr();
    planning_utils::getAllPartitionLanelets(ll, debug_data_.partition_lanelets);
  }
}

bool OcclusionSpotModule::modifyPathVelocity(
  autoware_auto_planning_msgs::msg::PathWithLaneId * path,
  [[maybe_unused]] tier4_planning_msgs::msg::StopReason * stop_reason)
{
  debug_data_.resetData();
  if (path->points.size() < 2) {
    return true;
  }
  // set planner data
  {
    param_.v.max_stop_jerk = planner_data_->max_stop_jerk_threshold;
    param_.v.max_stop_accel = planner_data_->max_stop_acceleration_threshold;
    param_.v.v_ego = planner_data_->current_velocity->twist.linear.x;
    param_.v.a_ego = planner_data_->current_accel.get();
    const double detection_area_offset = 5.0;  // for visualization and stability
    param_.detection_area_max_length =
      planning_utils::calcJudgeLineDistWithJerkLimit(
        param_.v.v_ego, param_.v.a_ego, param_.v.non_effective_accel, param_.v.non_effective_jerk,
        planner_data_->delay_response_time) +
      detection_area_offset;
  }
  const geometry_msgs::msg::Pose ego_pose = planner_data_->current_pose.pose;
  PathWithLaneId clipped_path;
  utils::clipPathByLength(*path, clipped_path, param_.detection_area_length);
  PathWithLaneId interp_path;
  //! never change this interpolation interval(will affect module accuracy)
  splineInterpolate(clipped_path, 1.0, &interp_path, logger_);
  if (param_.pass_judge == utils::PASS_JUDGE::CURRENT_VELOCITY) {
    interp_path = utils::applyVelocityToPath(interp_path, param_.v.v_ego);
  } else if (param_.pass_judge == utils::PASS_JUDGE::SMOOTH_VELOCITY) {
  }
  debug_data_.interp_path = interp_path;
  const geometry_msgs::msg::Point start_point = interp_path.points.at(0).point.pose.position;
  const auto offset = tier4_autoware_utils::calcSignedArcLength(
    interp_path.points, ego_pose, start_point, param_.dist_thr, param_.angle_thr);
  if (offset == boost::none) return true;
  const double offset_from_start_to_ego = -offset.get();
  auto & detection_area_polygons = debug_data_.detection_area_polygons;
  if (!utils::buildDetectionAreaPolygon(
        detection_area_polygons, interp_path, offset_from_start_to_ego, param_)) {
    return true;  // path point is not enough
  }
  std::vector<utils::PossibleCollisionInfo> possible_collisions;
  if (param_.detection_method == utils::DETECTION_METHOD::OCCUPANCY_GRID) {
    const auto & occ_grid_ptr = planner_data_->occupancy_grid;
    if (!occ_grid_ptr) return true;  // mo data
    nav_msgs::msg::OccupancyGrid occ_grid = *occ_grid_ptr;
    grid_map::GridMap grid_map;
    grid_utils::denoiseOccupancyGridCV(occ_grid, grid_map, param_.grid);
    if (param_.debug) publisher_->publish(occ_grid);  //
    // Note: Don't consider offset from path start to ego here
    if (!utils::createPossibleCollisionsInDetectionArea(
          possible_collisions, grid_map, interp_path, offset_from_start_to_ego, param_,
          debug_data_)) {
      // no occlusion spot
      return true;
    }
  } else if (param_.detection_method == utils::DETECTION_METHOD::PREDICTED_OBJECT) {
    const auto & dynamic_obj_arr_ptr = planner_data_->predicted_objects;
    if (!dynamic_obj_arr_ptr) return true;  // mo data
    std::vector<PredictedObject> obj =
      utils::getParkedVehicles(*dynamic_obj_arr_ptr, param_, debug_data_.parked_vehicle_point);
    const auto filtered_obj =
      utils::filterDynamicObjectByDetectionArea(obj, detection_area_polygons);
    // Note: Don't consider offset from path start to ego here
    if (!utils::generatePossibleCollisionBehindParkedVehicle(
          possible_collisions, interp_path, param_, offset_from_start_to_ego, filtered_obj)) {
      // no occlusion spot
      return true;
    }
  }
  RCLCPP_DEBUG_STREAM_THROTTLE(
    logger_, *clock_, 3000, "num possible collision:" << possible_collisions.size());
  utils::calcSlowDownPointsForPossibleCollision(0, interp_path, 0.0, possible_collisions);
  // Note: Consider offset from path start to ego here
  utils::handleCollisionOffset(possible_collisions, offset_from_start_to_ego);
  // apply safe velocity using ebs and pbs deceleration
  utils::applySafeVelocityConsideringPossibleCollision(path, possible_collisions, param_);
  // these debug topics needs computation resource

  debug_data_.z = path->points.front().point.pose.position.z;
  debug_data_.possible_collisions = possible_collisions;
  debug_data_.path_raw = *path;
  return true;
}

}  // namespace behavior_velocity_planner
