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

#include <memory>
#include <optional>
#include <vector>

class PlannerInterface
{
public:
  PlannerInterface(
    rclcpp::Node & node, const LongitudinalInfo & longitudinal_info,
    const vehicle_info_util::VehicleInfo & vehicle_info, const EgoNearestParam & ego_nearest_param,
    const std::shared_ptr<DebugData> debug_data_ptr)
  : longitudinal_info_(longitudinal_info),
    vehicle_info_(vehicle_info),
    ego_nearest_param_(ego_nearest_param),
    debug_data_ptr_(debug_data_ptr),
    slow_down_param_(SlowDownParam(node))
  {
    stop_reasons_pub_ = node.create_publisher<StopReasonArray>("~/output/stop_reasons", 1);
    velocity_factors_pub_ =
      node.create_publisher<VelocityFactorArray>("/planning/velocity_factors/obstacle_cruise", 1);
    stop_speed_exceeded_pub_ =
      node.create_publisher<StopSpeedExceeded>("~/output/stop_speed_exceeded", 1);
  }

  PlannerInterface() = default;

  void setParam(
    const bool enable_debug_info, const bool enable_calculation_time_info,
    const double min_behavior_stop_margin)
  {
    enable_debug_info_ = enable_debug_info;
    enable_calculation_time_info_ = enable_calculation_time_info;
    min_behavior_stop_margin_ = min_behavior_stop_margin;
  }

  std::vector<TrajectoryPoint> generateStopTrajectory(
    const PlannerData & planner_data, const std::vector<StopObstacle> & stop_obstacles);

  virtual std::vector<TrajectoryPoint> generateCruiseTrajectory(
    const PlannerData & planner_data, const std::vector<TrajectoryPoint> & stop_traj_points,
    const std::vector<CruiseObstacle> & cruise_obstacles,
    std::optional<VelocityLimit> & vel_limit) = 0;

  std::vector<TrajectoryPoint> generateSlowDownTrajectory(
    const PlannerData & planner_data, const std::vector<TrajectoryPoint> & stop_traj_points,
    const std::vector<SlowDownObstacle> & slow_down_obstacles,
    std::optional<VelocityLimit> & vel_limit);

  void onParam(const std::vector<rclcpp::Parameter> & parameters)
  {
    updateCommonParam(parameters);
    updateCruiseParam(parameters);
    slow_down_param_.onParam(parameters);
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
  Float32MultiArrayStamped getSlowDownPlanningDebugMessage(const rclcpp::Time & current_time)
  {
    slow_down_debug_multi_array_.stamp = current_time;
    return slow_down_debug_multi_array_;
  }

protected:
  // Parameters
  bool enable_debug_info_{false};
  bool enable_calculation_time_info_{false};
  LongitudinalInfo longitudinal_info_;
  double min_behavior_stop_margin_;

  // stop watch
  tier4_autoware_utils::StopWatch<
    std::chrono::milliseconds, std::chrono::microseconds, std::chrono::steady_clock>
    stop_watch_;

  // Publishers
  rclcpp::Publisher<StopReasonArray>::SharedPtr stop_reasons_pub_;
  rclcpp::Publisher<VelocityFactorArray>::SharedPtr velocity_factors_pub_;
  rclcpp::Publisher<StopSpeedExceeded>::SharedPtr stop_speed_exceeded_pub_;

  // Vehicle Parameters
  vehicle_info_util::VehicleInfo vehicle_info_;

  EgoNearestParam ego_nearest_param_;

  mutable std::shared_ptr<DebugData> debug_data_ptr_;

  // debug info
  StopPlanningDebugInfo stop_planning_debug_info_;
  Float32MultiArrayStamped slow_down_debug_multi_array_;

  double calcDistanceToCollisionPoint(
    const PlannerData & planner_data, const geometry_msgs::msg::Point & collision_point);

  double calcRSSDistance(
    const double ego_vel, const double obstacle_vel, const double margin = 0.0) const
  {
    const auto & i = longitudinal_info_;
    const double rss_dist_with_margin =
      ego_vel * i.idling_time + std::pow(ego_vel, 2) * 0.5 / std::abs(i.min_ego_accel_for_rss) -
      std::pow(obstacle_vel, 2) * 0.5 / std::abs(i.min_object_accel_for_rss) + margin;
    return rss_dist_with_margin;
  }

  void updateCommonParam(const std::vector<rclcpp::Parameter> & parameters)
  {
    longitudinal_info_.onParam(parameters);
  }

  virtual void updateCruiseParam([[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
  {
  }

  size_t findEgoIndex(
    const std::vector<TrajectoryPoint> & traj_points,
    const geometry_msgs::msg::Pose & ego_pose) const
  {
    const auto & p = ego_nearest_param_;
    return motion_utils::findFirstNearestIndexWithSoftConstraints(
      traj_points, ego_pose, p.dist_threshold, p.yaw_threshold);
  }

  size_t findEgoSegmentIndex(
    const std::vector<TrajectoryPoint> & traj_points,
    const geometry_msgs::msg::Pose & ego_pose) const
  {
    const auto & p = ego_nearest_param_;
    return motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      traj_points, ego_pose, p.dist_threshold, p.yaw_threshold);
  }

private:
  double calculateSlowDownVelocity(const SlowDownObstacle & obstacle) const;
  double calculateDistanceToSlowDownWithAccConstraint(
    const PlannerData & planner_data, const std::vector<TrajectoryPoint> & traj_points,
    const SlowDownObstacle & obstacle, const double dist_to_ego, const double slow_down_vel) const;

  struct SlowDownInfo
  {
    // NOTE: Acceleration limit is applied to lon_dist not to occur sudden brake.
    const double lon_dist;  // from ego pose to slow down point
    const double vel;       // slow down velocity
  };

  struct SlowDownParam
  {
    explicit SlowDownParam(rclcpp::Node & node)
    {
      max_lat_margin = node.declare_parameter<double>("slow_down.max_lat_margin");
      min_lat_margin = node.declare_parameter<double>("slow_down.min_lat_margin");
      max_ego_velocity = node.declare_parameter<double>("slow_down.max_ego_velocity");
      min_ego_velocity = node.declare_parameter<double>("slow_down.min_ego_velocity");
      max_deceleration = node.declare_parameter<double>("slow_down.max_deceleration");
      time_margin_on_target_velocity =
        node.declare_parameter<double>("slow_down.time_margin_on_target_velocity");
    }

    void onParam(const std::vector<rclcpp::Parameter> & parameters)
    {
      tier4_autoware_utils::updateParam<double>(
        parameters, "slow_down.max_lat_margin", max_lat_margin);
      tier4_autoware_utils::updateParam<double>(
        parameters, "slow_down.min_lat_margin", min_lat_margin);
      tier4_autoware_utils::updateParam<double>(
        parameters, "slow_down.max_ego_velocity", max_ego_velocity);
      tier4_autoware_utils::updateParam<double>(
        parameters, "slow_down.min_ego_velocity", min_ego_velocity);
      tier4_autoware_utils::updateParam<double>(
        parameters, "slow_down.max_deceleration", max_deceleration);
      tier4_autoware_utils::updateParam<double>(
        parameters, "slow_down.time_margin_on_target_velocity", time_margin_on_target_velocity);
    }

    double max_lat_margin;
    double min_lat_margin;
    double max_ego_velocity;
    double min_ego_velocity;
    double max_deceleration;
    double time_margin_on_target_velocity;
  };
  SlowDownParam slow_down_param_;
};

#endif  // OBSTACLE_CRUISE_PLANNER__PLANNER_INTERFACE_HPP_
