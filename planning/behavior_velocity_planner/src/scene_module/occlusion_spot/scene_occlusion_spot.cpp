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
#include <utilization/trajectory_utils.hpp>
#include <utilization/util.hpp>

#include <boost/optional.hpp>

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <memory>
#include <vector>

// turn on only when debugging.
#define DEBUG_PRINT(enable, n, x)                                  \
  if (enable) {                                                    \
    const std::string time_msg = n + std::to_string(x);            \
    RCLCPP_INFO_STREAM_THROTTLE(logger_, *clock_, 3000, time_msg); \
  }

namespace
{
namespace utils = behavior_velocity_planner::occlusion_spot_utils;
using autoware_auto_perception_msgs::msg::PredictedObject;
std::vector<PredictedObject> extractStuckVehicle(
  const std::vector<PredictedObject> & vehicles, const double stop_velocity)
{
  std::vector<PredictedObject> stuck_vehicles;
  for (const auto & obj : vehicles) {
    if (utils::isStuckVehicle(obj, stop_velocity)) {
      stuck_vehicles.emplace_back(obj);
    }
  }
  return stuck_vehicles;
}
}  // namespace

namespace behavior_velocity_planner
{
namespace utils = occlusion_spot_utils;

OcclusionSpotModule::OcclusionSpotModule(
  const int64_t module_id, const std::shared_ptr<const PlannerData> & planner_data,
  const PlannerParam & planner_param, const rclcpp::Logger & logger,
  const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock), param_(planner_param)
{
  if (param_.detection_method == utils::DETECTION_METHOD::OCCUPANCY_GRID) {
    debug_data_.detection_type = "occupancy";
    //! occupancy grid limitation( 100 * 100 )
    const double max_length = 35.0;  // current available length
    param_.detection_area_length = std::min(max_length, param_.detection_area_length);
  } else if (param_.detection_method == utils::DETECTION_METHOD::PREDICTED_OBJECT) {
    debug_data_.detection_type = "object";
  }
  if (param_.use_partition_lanelet) {
    const lanelet::LaneletMapConstPtr & ll = planner_data->route_handler_->getLaneletMapPtr();
    planning_utils::getAllPartitionLanelets(ll, partition_lanelets_);
  }
}

bool OcclusionSpotModule::modifyPathVelocity(
  PathWithLaneId * path, [[maybe_unused]] StopReason * stop_reason)
{
  if (param_.is_show_processing_time) stop_watch_.tic("total_processing_time");
  debug_data_.resetData();
  if (path->points.size() < 2) {
    return true;
  }
  // set planner data
  {
    param_.v.max_stop_jerk = planner_data_->max_stop_jerk_threshold;
    param_.v.max_stop_accel = planner_data_->max_stop_acceleration_threshold;
    param_.v.v_ego = planner_data_->current_velocity->twist.linear.x;
    param_.v.a_ego = planner_data_->current_acceleration->accel.accel.linear.x;
    param_.v.delay_time = planner_data_->system_delay;
    param_.detection_area_max_length =
      planning_utils::calcJudgeLineDistWithJerkLimit(
        param_.v.v_ego, param_.v.a_ego, param_.v.non_effective_accel, param_.v.non_effective_jerk,
        planner_data_->delay_response_time) +
      param_.detection_area_offset;  // To fill difference between planned and measured acc
  }
  const geometry_msgs::msg::Pose ego_pose = planner_data_->current_pose.pose;
  PathWithLaneId clipped_path;
  utils::clipPathByLength(*path, clipped_path, param_.detection_area_length);
  PathWithLaneId path_interpolated;
  //! never change this interpolation interval(will affect module accuracy)
  splineInterpolate(clipped_path, 1.0, path_interpolated, logger_);
  const geometry_msgs::msg::Point start_point = path_interpolated.points.at(0).point.pose.position;
  const auto offset = motion_utils::calcSignedArcLength(
    path_interpolated.points, ego_pose, start_point, param_.dist_thr, param_.angle_thr);
  if (offset == boost::none) return true;
  const double offset_from_start_to_ego = -offset.get();
  const bool show_time = param_.is_show_processing_time;
  if (show_time) stop_watch_.tic("processing_time");
  PathWithLaneId predicted_path;
  if (param_.pass_judge == utils::PASS_JUDGE::CURRENT_VELOCITY) {
    predicted_path = utils::applyVelocityToPath(path_interpolated, param_.v.v_ego);
  } else if (param_.pass_judge == utils::PASS_JUDGE::SMOOTH_VELOCITY) {
    if (!smoothPath(path_interpolated, predicted_path, planner_data_)) {
      predicted_path = utils::applyVelocityToPath(path_interpolated, param_.v.v_ego);
      // use current ego velocity in path if optimization failure
    }
  }
  DEBUG_PRINT(show_time, "apply velocity [ms]: ", stop_watch_.toc("processing_time", true));
  const size_t ego_seg_idx = findEgoSegmentIndex(predicted_path.points);
  if (!utils::buildDetectionAreaPolygon(
        debug_data_.detection_area_polygons, predicted_path, ego_pose, ego_seg_idx, param_)) {
    return true;  // path point is not enough
  }
  DEBUG_PRINT(show_time, "generate poly[ms]: ", stop_watch_.toc("processing_time", true));
  std::vector<utils::PossibleCollisionInfo> possible_collisions;
  // extract only close lanelet
  if (param_.use_partition_lanelet) {
    planning_utils::extractClosePartition(
      ego_pose.position, partition_lanelets_, debug_data_.close_partition);
  }
  DEBUG_PRINT(show_time, "extract[ms]: ", stop_watch_.toc("processing_time", true));
  const auto objects_ptr = planner_data_->predicted_objects;
  const auto vehicles =
    utils::extractVehicles(objects_ptr, ego_pose.position, param_.detection_area_length);
  const std::vector<PredictedObject> filtered_vehicles =
    utils::filterVehiclesByDetectionArea(vehicles, debug_data_.detection_area_polygons);
  DEBUG_PRINT(show_time, "filter obj[ms]: ", stop_watch_.toc("processing_time", true));
  if (param_.detection_method == utils::DETECTION_METHOD::OCCUPANCY_GRID) {
    const auto & occ_grid_ptr = planner_data_->occupancy_grid;
    if (!occ_grid_ptr) return true;  // no data
    grid_map::GridMap grid_map;
    Polygons2d stuck_vehicle_foot_prints;
    Polygons2d moving_vehicle_foot_prints;
    utils::categorizeVehicles(
      filtered_vehicles, stuck_vehicle_foot_prints, moving_vehicle_foot_prints,
      param_.stuck_vehicle_vel);
    // occ -> image
    // find out occlusion from erode occlusion candidate num iter is strength of filter
    const int num_iter = static_cast<int>(
      (param_.detection_area.min_occlusion_spot_size / occ_grid_ptr->info.resolution) - 1);
    grid_utils::denoiseOccupancyGridCV(
      occ_grid_ptr, stuck_vehicle_foot_prints, moving_vehicle_foot_prints, grid_map, param_.grid,
      param_.is_show_cv_window, num_iter, param_.use_object_info,
      param_.use_moving_object_ray_cast);
    DEBUG_PRINT(show_time, "grid [ms]: ", stop_watch_.toc("processing_time", true));
    // Note: Don't consider offset from path start to ego here
    if (!utils::generatePossibleCollisionsFromGridMap(
          possible_collisions, grid_map, path_interpolated, offset_from_start_to_ego, param_,
          debug_data_)) {
      // no occlusion spot
      return true;
    }
  } else if (param_.detection_method == utils::DETECTION_METHOD::PREDICTED_OBJECT) {
    const auto stuck_vehicles = extractStuckVehicle(filtered_vehicles, param_.stuck_vehicle_vel);
    // Note: Don't consider offset from path start to ego here
    if (!utils::generatePossibleCollisionsFromObjects(
          possible_collisions, path_interpolated, param_, offset_from_start_to_ego,
          stuck_vehicles)) {
      // no occlusion spot
      return true;
    }
  }
  DEBUG_PRINT(show_time, "occlusion [ms]: ", stop_watch_.toc("processing_time", true));
  DEBUG_PRINT(show_time, "num collision:", possible_collisions.size());
  utils::calcSlowDownPointsForPossibleCollision(0, path_interpolated, 0.0, possible_collisions);
  // Note: Consider offset from path start to ego here
  utils::handleCollisionOffset(possible_collisions, offset_from_start_to_ego);
  // apply safe velocity using ebs and pbs deceleration
  utils::applySafeVelocityConsideringPossibleCollision(path, possible_collisions, param_);
  // these debug topics needs computation resource
  debug_data_.z = path->points.front().point.pose.position.z;
  debug_data_.possible_collisions = possible_collisions;
  if (param_.is_show_occlusion) {
    debug_data_.path_interpolated = path_interpolated;
    debug_data_.path_raw.points = clipped_path.points;
  }
  DEBUG_PRINT(show_time, "total [ms]: ", stop_watch_.toc("total_processing_time", true));
  return true;
}

}  // namespace behavior_velocity_planner
