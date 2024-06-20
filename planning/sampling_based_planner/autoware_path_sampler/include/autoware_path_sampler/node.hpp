// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTOWARE_PATH_SAMPLER__NODE_HPP_
#define AUTOWARE_PATH_SAMPLER__NODE_HPP_

#include "autoware_path_sampler/common_structs.hpp"
#include "autoware_path_sampler/parameters.hpp"
#include "autoware_path_sampler/type_alias.hpp"
#include "autoware_sampler_common/transform/spline_transform.hpp"
#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"
#include "rclcpp/rclcpp.hpp"

#include <autoware/universe_utils/ros/polling_subscriber.hpp>
#include <autoware_sampler_common/structures.hpp>

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::path_sampler
{
class PathSampler : public rclcpp::Node
{
public:
  explicit PathSampler(const rclcpp::NodeOptions & node_options);

protected:  // for the static_centerline_generator package
  // argument variables
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_{};
  mutable DebugData debug_data_{};
  mutable std::shared_ptr<TimeKeeper> time_keeper_ptr_{nullptr};

  // parameters
  TrajectoryParam traj_param_{};
  EgoNearestParam ego_nearest_param_{};
  Parameters params_;
  size_t debug_id_ = 0;

  // variables for subscribers
  Odometry::SharedPtr ego_state_ptr_;

  // variables for previous information
  std::optional<autoware::sampler_common::Path> prev_path_;

  // interface publisher
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr virtual_wall_pub_;

  // interface subscriber
  rclcpp::Subscription<Path>::SharedPtr path_sub_;
  autoware::universe_utils::InterProcessPollingSubscriber<Odometry> odom_sub_{
    this, "~/input/odometry"};
  autoware::universe_utils::InterProcessPollingSubscriber<PredictedObjects> objects_sub_{
    this, "~/input/objects"};

  // debug publisher
  rclcpp::Publisher<MarkerArray>::SharedPtr debug_markers_pub_;
  rclcpp::Publisher<StringStamped>::SharedPtr debug_calculation_time_pub_;

  // parameter callback
  rcl_interfaces::msg::SetParametersResult onParam(
    const std::vector<rclcpp::Parameter> & parameters);
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  // subscriber callback function
  void objectsCallback(const PredictedObjects::SharedPtr msg);
  void onPath(const Path::SharedPtr);

  // main functions
  bool isDataReady(
    const Path & path, const Odometry::ConstSharedPtr ego_state_ptr, rclcpp::Clock clock);
  PlannerData createPlannerData(const Path & path, const Odometry & ego_state) const;
  std::vector<TrajectoryPoint> generateTrajectory(const PlannerData & planner_data);
  std::vector<TrajectoryPoint> extendTrajectory(
    const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<TrajectoryPoint> & optimized_points) const;
  void resetPreviousData();
  autoware::sampler_common::State getPlanningState(
    autoware::sampler_common::State & state,
    const autoware::sampler_common::transform::Spline2D & path_spline) const;

  // sub-functions of generateTrajectory
  void copyZ(
    const std::vector<TrajectoryPoint> & from_traj, std::vector<TrajectoryPoint> & to_traj);
  void copyVelocity(
    const std::vector<TrajectoryPoint> & from_traj, std::vector<TrajectoryPoint> & to_traj,
    const geometry_msgs::msg::Pose & ego_pose);
  autoware::sampler_common::Path generatePath(const PlannerData & planner_data);
  std::vector<autoware::sampler_common::Path> generateCandidatesFromPreviousPath(
    const PlannerData & planner_data,
    const autoware::sampler_common::transform::Spline2D & path_spline);
  std::vector<TrajectoryPoint> generateTrajectoryPoints(const PlannerData & planner_data);
  void publishVirtualWall(const geometry_msgs::msg::Pose & stop_pose) const;
  void publishDebugMarker(const std::vector<TrajectoryPoint> & traj_points) const;
};
}  // namespace autoware::path_sampler

#endif  // AUTOWARE_PATH_SAMPLER__NODE_HPP_
