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

#ifndef OBSTACLE_CRUISE_PLANNER__NODE_HPP_
#define OBSTACLE_CRUISE_PLANNER__NODE_HPP_

#include "obstacle_cruise_planner/common_structs.hpp"
#include "obstacle_cruise_planner/optimization_based_planner/optimization_based_planner.hpp"
#include "obstacle_cruise_planner/pid_based_planner/pid_based_planner.hpp"
#include "obstacle_cruise_planner/type_alias.hpp"
#include "signal_processing/lowpass_filter_1d.hpp"
#include "tier4_autoware_utils/system/stop_watch.hpp"

#include <rclcpp/rclcpp.hpp>

#include <boost/optional.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace motion_planning
{
class ObstacleCruisePlannerNode : public rclcpp::Node
{
public:
  explicit ObstacleCruisePlannerNode(const rclcpp::NodeOptions & node_options);

private:
  // callback functions
  rcl_interfaces::msg::SetParametersResult onParam(
    const std::vector<rclcpp::Parameter> & parameters);
  void onTrajectory(const Trajectory::ConstSharedPtr msg);
  void onSmoothedTrajectory(const Trajectory::ConstSharedPtr msg);

  // member Functions
  ObstacleCruisePlannerData createCruiseData(
    const Trajectory & trajectory, const geometry_msgs::msg::Pose & current_pose,
    const std::vector<TargetObstacle> & obstacles, const bool is_driving_forward);
  ObstacleCruisePlannerData createStopData(
    const Trajectory & trajectory, const geometry_msgs::msg::Pose & current_pose,
    const std::vector<TargetObstacle> & obstacles, const bool is_driving_forward);
  std::vector<TargetObstacle> getTargetObstacles(
    const Trajectory & trajectory, const geometry_msgs::msg::Pose & current_pose,
    const double current_vel, const bool is_driving_forward, DebugData & debug_data);
  std::vector<TargetObstacle> filterObstacles(
    const PredictedObjects & predicted_objects, const Trajectory & traj,
    const geometry_msgs::msg::Pose & current_pose, const double current_vel,
    const bool is_driving_forward, DebugData & debug_data);
  void updateHasStopped(std::vector<TargetObstacle> & target_obstacles);
  void checkConsistency(
    const rclcpp::Time & current_time, const PredictedObjects & predicted_objects,
    const Trajectory & traj, std::vector<TargetObstacle> & target_obstacles);
  double calcCollisionTimeMargin(
    const geometry_msgs::msg::Pose & current_pose, const double current_vel,
    const std::vector<geometry_msgs::msg::PointStamped> & collision_points,
    const PredictedObject & predicted_object, const Trajectory & traj,
    const bool is_driving_forward);
  void publishVelocityLimit(const boost::optional<VelocityLimit> & vel_limit);
  void publishDebugMarker(const DebugData & debug_data) const;
  void publishDebugInfo() const;
  void publishCalculationTime(const double calculation_time) const;

  bool isCruiseObstacle(const uint8_t label);
  bool isStopObstacle(const uint8_t label);
  bool isFrontCollideObstacle(
    const Trajectory & traj, const PredictedObject & object, const size_t first_collision_idx);

  bool is_showing_debug_info_;
  double min_behavior_stop_margin_;
  double obstacle_velocity_threshold_from_cruise_to_stop_;
  double obstacle_velocity_threshold_from_stop_to_cruise_;

  std::vector<int> cruise_obstacle_types_;
  std::vector<int> stop_obstacle_types_;

  // parameter callback result
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  // publisher
  rclcpp::Publisher<Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<VelocityLimit>::SharedPtr vel_limit_pub_;
  rclcpp::Publisher<VelocityLimitClearCommand>::SharedPtr clear_vel_limit_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr debug_marker_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr debug_cruise_wall_marker_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr debug_stop_wall_marker_pub_;
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr debug_stop_planning_info_pub_;
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr debug_cruise_planning_info_pub_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr debug_calculation_time_pub_;

  // subscriber
  rclcpp::Subscription<Trajectory>::SharedPtr traj_sub_;
  rclcpp::Subscription<PredictedObjects>::SharedPtr objects_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<AccelWithCovarianceStamped>::SharedPtr acc_sub_;

  // data for callback functions
  PredictedObjects::ConstSharedPtr in_objects_ptr_{nullptr};
  Odometry::ConstSharedPtr current_odom_ptr_{nullptr};
  AccelWithCovarianceStamped::ConstSharedPtr current_accel_ptr_{nullptr};

  // Vehicle Parameters
  VehicleInfo vehicle_info_;

  // planning algorithm
  enum class PlanningAlgorithm { OPTIMIZATION_BASE, PID_BASE, INVALID };
  PlanningAlgorithm getPlanningAlgorithmType(const std::string & param) const;
  PlanningAlgorithm planning_algorithm_;

  // stop watch
  mutable tier4_autoware_utils::StopWatch<
    std::chrono::milliseconds, std::chrono::microseconds, std::chrono::steady_clock>
    stop_watch_;

  // planner
  std::unique_ptr<PlannerInterface> planner_ptr_{nullptr};

  // previous closest obstacle
  std::shared_ptr<TargetObstacle> prev_closest_obstacle_ptr_{nullptr};

  // obstacle filtering parameter
  struct ObstacleFilteringParam
  {
    double rough_detection_area_expand_width;
    double detection_area_expand_width;
    double decimate_trajectory_step_length;
    // inside
    double crossing_obstacle_velocity_threshold;
    double collision_time_margin;
    // outside
    double outside_rough_detection_area_expand_width;
    double outside_obstacle_min_velocity_threshold;
    double ego_obstacle_overlap_time_threshold;
    double max_prediction_time_for_collision_check;
    double crossing_obstacle_traj_angle_threshold;
    std::vector<int> ignored_outside_obstacle_types;
    // obstacle hold
    double stop_obstacle_hold_time_threshold;
    // prediction resampling
    double prediction_resampling_time_interval;
    double prediction_resampling_time_horizon;
    // goal extension
    double goal_extension_length;
    double goal_extension_interval;
  };
  ObstacleFilteringParam obstacle_filtering_param_;

  bool need_to_clear_vel_limit_{false};
  EgoNearestParam ego_nearest_param_;

  bool is_driving_forward_{true};
  bool disable_stop_planning_{false};

  std::vector<TargetObstacle> prev_target_obstacles_;
};
}  // namespace motion_planning

#endif  // OBSTACLE_CRUISE_PLANNER__NODE_HPP_
