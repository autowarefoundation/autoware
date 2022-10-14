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

#ifndef OBSTACLE_CRUISE_PLANNER__OPTIMIZATION_BASED_PLANNER__OPTIMIZATION_BASED_PLANNER_HPP_
#define OBSTACLE_CRUISE_PLANNER__OPTIMIZATION_BASED_PLANNER__OPTIMIZATION_BASED_PLANNER_HPP_

#include "obstacle_cruise_planner/optimization_based_planner/s_boundary.hpp"
#include "obstacle_cruise_planner/optimization_based_planner/velocity_optimizer.hpp"
#include "obstacle_cruise_planner/planner_interface.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_debug_msgs/msg/float32_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <boost/optional.hpp>

#include <memory>
#include <tuple>
#include <vector>

using autoware_auto_perception_msgs::msg::ObjectClassification;
using autoware_auto_perception_msgs::msg::PredictedPath;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using tier4_debug_msgs::msg::Float32Stamped;

class OptimizationBasedPlanner : public PlannerInterface
{
public:
  OptimizationBasedPlanner(
    rclcpp::Node & node, const LongitudinalInfo & longitudinal_info,
    const vehicle_info_util::VehicleInfo & vehicle_info, const EgoNearestParam & ego_nearest_param);

  Trajectory generateCruiseTrajectory(
    const ObstacleCruisePlannerData & planner_data, boost::optional<VelocityLimit> & vel_limit,
    DebugData & debug_data) override;

private:
  // Member Functions
  std::vector<double> createTimeVector();
  std::tuple<double, double> calcInitialMotion(
    const ObstacleCruisePlannerData & planner_data, const Trajectory & prev_traj);

  bool checkHasReachedGoal(const ObstacleCruisePlannerData & planner_data);

  boost::optional<SBoundaries> getSBoundaries(
    const ObstacleCruisePlannerData & planner_data, const std::vector<double> & time_vec);

  boost::optional<SBoundaries> getSBoundaries(
    const ObstacleCruisePlannerData & planner_data, const TargetObstacle & object,
    const std::vector<double> & time_vec, const double traj_length);

  boost::optional<SBoundaries> getSBoundariesForOnTrajectoryObject(
    const ObstacleCruisePlannerData & planner_data, const std::vector<double> & time_vec,
    const double safety_distance, const TargetObstacle & object, const double traj_length);

  boost::optional<SBoundaries> getSBoundariesForOffTrajectoryObject(
    const ObstacleCruisePlannerData & planner_data, const std::vector<double> & time_vec,
    const double safety_distance, const TargetObstacle & object, const double traj_length);

  bool checkOnTrajectory(
    const ObstacleCruisePlannerData & planner_data, const geometry_msgs::msg::PointStamped & point);

  boost::optional<double> calcTrajectoryLengthFromCurrentPose(
    const autoware_auto_planning_msgs::msg::Trajectory & traj,
    const geometry_msgs::msg::Pose & current_pose);

  geometry_msgs::msg::Pose transformBaseLink2Center(
    const geometry_msgs::msg::Pose & pose_base_link);

  boost::optional<VelocityOptimizer::OptimizationResult> processOptimizedResult(
    const double v0, const VelocityOptimizer::OptimizationResult & opt_result, const double offset);

  void publishDebugTrajectory(
    const ObstacleCruisePlannerData & planner_data, const double offset,
    const std::vector<double> & time_vec, const SBoundaries & s_boundaries,
    const VelocityOptimizer::OptimizationResult & opt_result);

  Trajectory prev_output_;

  // Velocity Optimizer
  std::shared_ptr<VelocityOptimizer> velocity_optimizer_ptr_;

  // Publisher
  rclcpp::Publisher<Trajectory>::SharedPtr boundary_pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr optimized_sv_pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr optimized_st_graph_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_wall_marker_pub_;

  // Resampling Parameter
  double dense_resampling_time_interval_;
  double sparse_resampling_time_interval_;
  double dense_time_horizon_;
  double max_time_horizon_;

  double t_dangerous_;
  double velocity_margin_;

  double replan_vel_deviation_;
  double engage_velocity_;
  double engage_acceleration_;
  double engage_exit_ratio_;
  double stop_dist_to_prohibit_engage_;
};

#endif  // OBSTACLE_CRUISE_PLANNER__OPTIMIZATION_BASED_PLANNER__OPTIMIZATION_BASED_PLANNER_HPP_
