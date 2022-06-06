// Copyright 2022 TIER IV, Inc.
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

#ifndef OBSTACLE_CRUISE_PLANNER__PID_BASED_PLANNER__PID_BASED_PLANNER_HPP_
#define OBSTACLE_CRUISE_PLANNER__PID_BASED_PLANNER__PID_BASED_PLANNER_HPP_

#include "obstacle_cruise_planner/pid_based_planner/debug_values.hpp"
#include "obstacle_cruise_planner/pid_based_planner/pid_controller.hpp"
#include "obstacle_cruise_planner/planner_interface.hpp"
#include "tier4_autoware_utils/system/stop_watch.hpp"

#include "tier4_debug_msgs/msg/float32_multi_array_stamped.hpp"
#include "tier4_planning_msgs/msg/stop_reason_array.hpp"
#include "tier4_planning_msgs/msg/stop_speed_exceeded.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <boost/optional.hpp>

#include <memory>
#include <vector>

using tier4_debug_msgs::msg::Float32MultiArrayStamped;
using tier4_planning_msgs::msg::StopSpeedExceeded;

class PIDBasedPlanner : public PlannerInterface
{
public:
  struct CruiseObstacleInfo
  {
    CruiseObstacleInfo(
      const TargetObstacle & obstacle_arg, const double dist_to_cruise_arg,
      const double normalized_dist_to_cruise_arg)
    : obstacle(obstacle_arg),
      dist_to_cruise(dist_to_cruise_arg),
      normalized_dist_to_cruise(normalized_dist_to_cruise_arg)
    {
    }
    TargetObstacle obstacle;
    double dist_to_cruise;
    double normalized_dist_to_cruise;
  };

  struct StopObstacleInfo
  {
    StopObstacleInfo(const TargetObstacle & obstacle_arg, const double dist_to_stop_arg)
    : obstacle(obstacle_arg), dist_to_stop(dist_to_stop_arg)
    {
    }
    TargetObstacle obstacle;
    double dist_to_stop;
  };

  PIDBasedPlanner(
    rclcpp::Node & node, const LongitudinalInfo & longitudinal_info,
    const vehicle_info_util::VehicleInfo & vehicle_info);

  Trajectory generateTrajectory(
    const ObstacleCruisePlannerData & planner_data, boost::optional<VelocityLimit> & vel_limit,
    DebugData & debug_data) override;

  void updateParam(const std::vector<rclcpp::Parameter> & parameters) override;

private:
  void calcObstaclesToCruiseAndStop(
    const ObstacleCruisePlannerData & planner_data,
    boost::optional<StopObstacleInfo> & stop_obstacle_info,
    boost::optional<CruiseObstacleInfo> & cruise_obstacle_info);
  double calcDistanceToObstacle(
    const ObstacleCruisePlannerData & planner_data, const TargetObstacle & obstacle);
  bool isStopRequired(const TargetObstacle & obstacle);

  void planCruise(
    const ObstacleCruisePlannerData & planner_data, boost::optional<VelocityLimit> & vel_limit,
    const boost::optional<CruiseObstacleInfo> & cruise_obstacle_info, DebugData & debug_data);
  VelocityLimit doCruise(
    const ObstacleCruisePlannerData & planner_data, const CruiseObstacleInfo & cruise_obstacle_info,
    std::vector<TargetObstacle> & debug_obstacles_to_cruise,
    visualization_msgs::msg::MarkerArray & debug_walls_marker);

  Trajectory planStop(
    const ObstacleCruisePlannerData & planner_data,
    const boost::optional<StopObstacleInfo> & stop_obstacle_info, DebugData & debug_data);
  boost::optional<size_t> doStop(
    const ObstacleCruisePlannerData & planner_data, const StopObstacleInfo & stop_obstacle_info,
    std::vector<TargetObstacle> & debug_obstacles_to_stop,
    visualization_msgs::msg::MarkerArray & debug_walls_marker) const;

  void publishDebugValues(const ObstacleCruisePlannerData & planner_data) const;

  size_t findExtendedNearestIndex(
    const Trajectory traj, const geometry_msgs::msg::Pose & pose) const
  {
    const auto nearest_idx = tier4_autoware_utils::findNearestIndex(
      traj.points, pose, nearest_dist_deviation_threshold_, nearest_yaw_deviation_threshold_);
    if (nearest_idx) {
      return nearest_idx.get();
    }
    return tier4_autoware_utils::findNearestIndex(traj.points, pose.position);
  }

  // ROS parameters
  double min_accel_during_cruise_;
  double vel_to_acc_weight_;
  double min_cruise_target_vel_;
  double obstacle_velocity_threshold_from_cruise_to_stop_;
  // bool use_predicted_obstacle_pose_;

  // pid controller
  std::unique_ptr<PIDController> pid_controller_;
  double output_ratio_during_accel_;

  // stop watch
  tier4_autoware_utils::StopWatch<
    std::chrono::milliseconds, std::chrono::microseconds, std::chrono::steady_clock>
    stop_watch_;

  // publisher
  rclcpp::Publisher<tier4_planning_msgs::msg::StopReasonArray>::SharedPtr stop_reasons_pub_;
  rclcpp::Publisher<StopSpeedExceeded>::SharedPtr stop_speed_exceeded_pub_;
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr debug_values_pub_;

  boost::optional<double> prev_target_vel_;

  DebugValues debug_values_;
};

#endif  // OBSTACLE_CRUISE_PLANNER__PID_BASED_PLANNER__PID_BASED_PLANNER_HPP_
