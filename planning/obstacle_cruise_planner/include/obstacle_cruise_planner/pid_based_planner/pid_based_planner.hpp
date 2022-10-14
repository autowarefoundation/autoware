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
#include "signal_processing/lowpass_filter_1d.hpp"
#include "tier4_autoware_utils/system/stop_watch.hpp"

#include "tier4_debug_msgs/msg/float32_multi_array_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <boost/optional.hpp>

#include <memory>
#include <vector>

using tier4_debug_msgs::msg::Float32MultiArrayStamped;

class PIDBasedPlanner : public PlannerInterface
{
public:
  struct CruiseObstacleInfo
  {
    CruiseObstacleInfo(
      const TargetObstacle & obstacle_arg, const double dist_to_cruise_arg,
      const double normalized_dist_to_cruise_arg, double dist_to_obstacle_arg)
    : obstacle(obstacle_arg),
      dist_to_cruise(dist_to_cruise_arg),
      normalized_dist_to_cruise(normalized_dist_to_cruise_arg),
      dist_to_obstacle(dist_to_obstacle_arg)
    {
    }
    TargetObstacle obstacle;
    double dist_to_cruise;
    double normalized_dist_to_cruise;
    double dist_to_obstacle;
  };

  PIDBasedPlanner(
    rclcpp::Node & node, const LongitudinalInfo & longitudinal_info,
    const vehicle_info_util::VehicleInfo & vehicle_info, const EgoNearestParam & ego_nearest_param);

  Trajectory generateCruiseTrajectory(
    const ObstacleCruisePlannerData & planner_data, boost::optional<VelocityLimit> & vel_limit,
    DebugData & debug_data) override;

  void updateParam(const std::vector<rclcpp::Parameter> & parameters) override;

private:
  void calcObstaclesToCruise(
    const ObstacleCruisePlannerData & planner_data,
    boost::optional<CruiseObstacleInfo> & cruise_obstacle_info);

  void planCruise(
    const ObstacleCruisePlannerData & planner_data, boost::optional<VelocityLimit> & vel_limit,
    const boost::optional<CruiseObstacleInfo> & cruise_obstacle_info, DebugData & debug_data);
  VelocityLimit doCruise(
    const ObstacleCruisePlannerData & planner_data, const CruiseObstacleInfo & cruise_obstacle_info,
    std::vector<TargetObstacle> & debug_obstacles_to_cruise,
    visualization_msgs::msg::MarkerArray & debug_walls_marker);

  void publishDebugValues(const ObstacleCruisePlannerData & planner_data) const;

  // ROS parameters
  double min_accel_during_cruise_;
  double vel_to_acc_weight_;
  double min_cruise_target_vel_;
  // bool use_predicted_obstacle_pose_;

  // pid controller
  std::unique_ptr<PIDController> pid_controller_;
  double output_ratio_during_accel_;

  // stop watch
  tier4_autoware_utils::StopWatch<
    std::chrono::milliseconds, std::chrono::microseconds, std::chrono::steady_clock>
    stop_watch_;

  // publisher
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr debug_values_pub_;

  boost::optional<double> prev_target_vel_;

  DebugValues debug_values_;

  std::shared_ptr<LowpassFilter1d> lpf_cruise_ptr_;
};

#endif  // OBSTACLE_CRUISE_PLANNER__PID_BASED_PLANNER__PID_BASED_PLANNER_HPP_
