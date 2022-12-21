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
#include "obstacle_cruise_planner/stop_planning_debug_info.hpp"
#include "obstacle_cruise_planner/type_alias.hpp"
#include "obstacle_cruise_planner/utils.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#include <boost/optional.hpp>

#include <memory>
#include <vector>

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
    stop_reasons_pub_ = node.create_publisher<StopReasonArray>("~/output/stop_reasons", 1);
    velocity_factors_pub_ =
      node.create_publisher<VelocityFactorArray>("/planning/velocity_factors/obstacle_cruise", 1);
    stop_speed_exceeded_pub_ =
      node.create_publisher<StopSpeedExceeded>("~/output/stop_speed_exceeded", 1);
  }

  PlannerInterface() = default;

  void setParam(const bool is_showing_debug_info, const double min_behavior_stop_margin)
  {
    is_showing_debug_info_ = is_showing_debug_info;
    min_behavior_stop_margin_ = min_behavior_stop_margin;
  }

  Trajectory generateStopTrajectory(
    const ObstacleCruisePlannerData & planner_data, DebugData & debug_data);

  virtual Trajectory generateCruiseTrajectory(
    const ObstacleCruisePlannerData & planner_data, boost::optional<VelocityLimit> & vel_limit,
    DebugData & debug_data) = 0;

  void onParam(const std::vector<rclcpp::Parameter> & parameters)
  {
    updateCommonParam(parameters);
    updateParam(parameters);
  }

  Float32MultiArrayStamped getStopPlanningDebugMessage(const rclcpp::Time & current_time) const
  {
    return stop_planning_debug_info_.convertToMessage(current_time);
  }
  virtual Float32MultiArrayStamped getCruisePlanningDebugMessage(
    [[maybe_unused]] const rclcpp::Time & current_time) const
  {
    return Float32MultiArrayStamped{};
  }

protected:
  // Parameters
  bool is_showing_debug_info_{false};
  LongitudinalInfo longitudinal_info_;
  double min_behavior_stop_margin_;

  // Publishers
  rclcpp::Publisher<StopReasonArray>::SharedPtr stop_reasons_pub_;
  rclcpp::Publisher<VelocityFactorArray>::SharedPtr velocity_factors_pub_;
  rclcpp::Publisher<StopSpeedExceeded>::SharedPtr stop_speed_exceeded_pub_;

  // Vehicle Parameters
  vehicle_info_util::VehicleInfo vehicle_info_;

  EgoNearestParam ego_nearest_param_;

  // debug info
  StopPlanningDebugInfo stop_planning_debug_info_;

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

  void updateCommonParam(const std::vector<rclcpp::Parameter> & parameters)
  {
    longitudinal_info_.onParam(parameters);
  }

  virtual void updateParam([[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters) {}

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
