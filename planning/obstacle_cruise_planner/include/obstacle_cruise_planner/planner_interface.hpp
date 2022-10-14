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

#ifndef OBSTACLE_CRUISE_PLANNER__PLANNER_INTERFACE_HPP_
#define OBSTACLE_CRUISE_PLANNER__PLANNER_INTERFACE_HPP_

#include "motion_utils/motion_utils.hpp"
#include "obstacle_cruise_planner/common_structs.hpp"
#include "obstacle_cruise_planner/utils.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "tier4_planning_msgs/msg/stop_reason_array.hpp"
#include "tier4_planning_msgs/msg/stop_speed_exceeded.hpp"
#include "tier4_planning_msgs/msg/velocity_limit.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <boost/optional.hpp>

#include <memory>
#include <vector>

using autoware_auto_perception_msgs::msg::ObjectClassification;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using tier4_planning_msgs::msg::StopSpeedExceeded;
using tier4_planning_msgs::msg::VelocityLimit;

class PlannerInterface
{
public:
  PlannerInterface(
    rclcpp::Node & node, const LongitudinalInfo & longitudinal_info,
    const vehicle_info_util::VehicleInfo & vehicle_info, const EgoNearestParam & ego_nearest_param)
  : longitudinal_info_(longitudinal_info),
    vehicle_info_(vehicle_info),
    ego_nearest_param_(ego_nearest_param)
  {
    stop_reasons_pub_ =
      node.create_publisher<tier4_planning_msgs::msg::StopReasonArray>("~/output/stop_reasons", 1);
    stop_speed_exceeded_pub_ =
      node.create_publisher<StopSpeedExceeded>("~/output/stop_speed_exceeded", 1);
  }

  PlannerInterface() = default;

  void setParams(
    const bool is_showing_debug_info, const double min_behavior_stop_margin,
    const double nearest_dist_deviation_threshold, const double nearest_yaw_deviation_threshold)
  {
    is_showing_debug_info_ = is_showing_debug_info;
    min_behavior_stop_margin_ = min_behavior_stop_margin;
    nearest_dist_deviation_threshold_ = nearest_dist_deviation_threshold;
    nearest_yaw_deviation_threshold_ = nearest_yaw_deviation_threshold;
  }

  Trajectory generateStopTrajectory(
    const ObstacleCruisePlannerData & planner_data, DebugData & debug_data);

  virtual Trajectory generateCruiseTrajectory(
    const ObstacleCruisePlannerData & planner_data, boost::optional<VelocityLimit> & vel_limit,
    DebugData & debug_data) = 0;

  void updateCommonParam(const std::vector<rclcpp::Parameter> & parameters)
  {
    auto & i = longitudinal_info_;

    tier4_autoware_utils::updateParam<double>(parameters, "common.max_accel", i.max_accel);
    tier4_autoware_utils::updateParam<double>(parameters, "common.min_accel", i.min_accel);
    tier4_autoware_utils::updateParam<double>(parameters, "common.max_jerk", i.max_jerk);
    tier4_autoware_utils::updateParam<double>(parameters, "common.min_jerk", i.min_jerk);
    tier4_autoware_utils::updateParam<double>(parameters, "limit.max_accel", i.limit_max_accel);
    tier4_autoware_utils::updateParam<double>(parameters, "limit.min_accel", i.limit_min_accel);
    tier4_autoware_utils::updateParam<double>(parameters, "limit.max_jerk", i.limit_max_jerk);
    tier4_autoware_utils::updateParam<double>(parameters, "limit.min_jerk", i.limit_min_jerk);
    tier4_autoware_utils::updateParam<double>(
      parameters, "common.min_ego_accel_for_rss", i.min_ego_accel_for_rss);
    tier4_autoware_utils::updateParam<double>(
      parameters, "common.min_object_accel_for_rss", i.min_object_accel_for_rss);
    tier4_autoware_utils::updateParam<double>(parameters, "common.idling_time", i.idling_time);
  }

  virtual void updateParam([[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters) {}

  // TODO(shimizu) remove this function
  void setSmoothedTrajectory(const Trajectory::ConstSharedPtr traj)
  {
    smoothed_trajectory_ptr_ = traj;
  }

protected:
  // Parameters
  bool is_showing_debug_info_{false};
  LongitudinalInfo longitudinal_info_;
  double min_behavior_stop_margin_;
  double nearest_dist_deviation_threshold_;
  double nearest_yaw_deviation_threshold_;

  // Publishers
  rclcpp::Publisher<tier4_planning_msgs::msg::StopReasonArray>::SharedPtr stop_reasons_pub_;
  rclcpp::Publisher<StopSpeedExceeded>::SharedPtr stop_speed_exceeded_pub_;

  // Vehicle Parameters
  vehicle_info_util::VehicleInfo vehicle_info_;

  EgoNearestParam ego_nearest_param_;

  // TODO(shimizu) remove these parameters
  Trajectory::ConstSharedPtr smoothed_trajectory_ptr_;

  double calcDistanceToCollisionPoint(
    const ObstacleCruisePlannerData & planner_data,
    const geometry_msgs::msg::Point & collision_point);

  double calcRSSDistance(
    const double ego_vel, const double obj_vel, const double margin = 0.0) const
  {
    const auto & i = longitudinal_info_;
    const double rss_dist_with_margin =
      ego_vel * i.idling_time + std::pow(ego_vel, 2) * 0.5 / std::abs(i.min_ego_accel_for_rss) -
      std::pow(obj_vel, 2) * 0.5 / std::abs(i.min_object_accel_for_rss) + margin;
    return rss_dist_with_margin;
  }

  size_t findEgoIndex(const Trajectory & traj, const geometry_msgs::msg::Pose & ego_pose) const
  {
    const auto traj_points = motion_utils::convertToTrajectoryPointArray(traj);

    const auto & p = ego_nearest_param_;
    return motion_utils::findFirstNearestIndexWithSoftConstraints(
      traj_points, ego_pose, p.dist_threshold, p.yaw_threshold);
  }

  size_t findEgoSegmentIndex(
    const Trajectory & traj, const geometry_msgs::msg::Pose & ego_pose) const
  {
    const auto traj_points = motion_utils::convertToTrajectoryPointArray(traj);

    const auto & p = ego_nearest_param_;
    return motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      traj_points, ego_pose, p.dist_threshold, p.yaw_threshold);
  }
};

#endif  // OBSTACLE_CRUISE_PLANNER__PLANNER_INTERFACE_HPP_
