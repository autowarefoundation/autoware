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

#ifndef PATH_SAMPLER__NODE_HPP_
#define PATH_SAMPLER__NODE_HPP_

#include "path_sampler/common_structs.hpp"
#include "path_sampler/parameters.hpp"
#include "path_sampler/type_alias.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sampler_common/transform/spline_transform.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include <sampler_common/structures.hpp>

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace path_sampler
{
class PathSampler : public rclcpp::Node
{
public:
  explicit PathSampler(const rclcpp::NodeOptions & node_options);

protected:  // for the static_centerline_optimizer package
  // argument variables
  vehicle_info_util::VehicleInfo vehicle_info_{};
  mutable DebugData debug_data_{};
  mutable std::shared_ptr<TimeKeeper> time_keeper_ptr_{nullptr};

  // parameters
  TrajectoryParam traj_param_{};
  EgoNearestParam ego_nearest_param_{};
  Parameters params_;
  size_t debug_id_ = 0;

  // variables for subscribers
  Odometry::SharedPtr ego_state_ptr_;
  PredictedObjects::SharedPtr in_objects_ptr_ = std::make_shared<PredictedObjects>();

  // variables for previous information
  std::optional<sampler_common::Path> prev_path_;

  // interface publisher
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr virtual_wall_pub_;

  // interface subscriber
  rclcpp::Subscription<Path>::SharedPtr path_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<PredictedObjects>::SharedPtr objects_sub_;

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
  bool isDataReady(const Path & path, rclcpp::Clock clock) const;
  PlannerData createPlannerData(const Path & path) const;
  std::vector<TrajectoryPoint> generateTrajectory(const PlannerData & planner_data);
  std::vector<TrajectoryPoint> extendTrajectory(
    const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<TrajectoryPoint> & optimized_points) const;
  void resetPreviousData();
  sampler_common::State getPlanningState(
    sampler_common::State & state, const sampler_common::transform::Spline2D & path_spline) const;

  // sub-functions of generateTrajectory
  void copyZ(
    const std::vector<TrajectoryPoint> & from_traj, std::vector<TrajectoryPoint> & to_traj);
  void copyVelocity(
    const std::vector<TrajectoryPoint> & from_traj, std::vector<TrajectoryPoint> & to_traj,
    const geometry_msgs::msg::Pose & ego_pose);
  std::vector<TrajectoryPoint> generatePath(const PlannerData & planner_data);
  void publishVirtualWall(const geometry_msgs::msg::Pose & stop_pose) const;
  void publishDebugMarker(const std::vector<TrajectoryPoint> & traj_points) const;
};
}  // namespace path_sampler

#endif  // PATH_SAMPLER__NODE_HPP_
