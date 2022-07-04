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

#include "obstacle_cruise_planner/optimization_based_planner/box2d.hpp"
#include "obstacle_cruise_planner/optimization_based_planner/s_boundary.hpp"
#include "obstacle_cruise_planner/optimization_based_planner/st_point.hpp"
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
    const vehicle_info_util::VehicleInfo & vehicle_info);

  Trajectory generateCruiseTrajectory(
    const ObstacleCruisePlannerData & planner_data, boost::optional<VelocityLimit> & vel_limit,
    DebugData & debug_data) override;

private:
  struct TrajectoryData
  {
    TrajectoryData() {}

    Trajectory traj;
    std::vector<double> s;
  };

  struct ObjectData
  {
    geometry_msgs::msg::Pose pose;
    double length;
    double width;
    double time;
  };

  // Member Functions
  std::vector<double> createTimeVector();
  std::tuple<double, double> calcInitialMotion(
    const ObstacleCruisePlannerData & planner_data, const size_t input_closest,
    const Trajectory & prev_traj);

  TrajectoryPoint calcInterpolatedTrajectoryPoint(
    const Trajectory & trajectory, const geometry_msgs::msg::Pose & target_pose);
  bool checkHasReachedGoal(const ObstacleCruisePlannerData & planner_data);
  TrajectoryData getTrajectoryData(
    const Trajectory & traj, const geometry_msgs::msg::Pose & current_pose);

  TrajectoryData resampleTrajectoryData(
    const TrajectoryData & base_traj_data, const double resampling_s_interval,
    const double max_traj_length);
  Trajectory resampleTrajectory(
    const std::vector<double> & base_index, const Trajectory & base_trajectory,
    const std::vector<double> & query_index, const bool use_spline_for_pose = false);

  boost::optional<SBoundaries> getSBoundaries(
    const ObstacleCruisePlannerData & planner_data, const TrajectoryData & ego_traj_data,
    const std::vector<double> & time_vec);

  boost::optional<SBoundaries> getSBoundaries(
    const rclcpp::Time & current_time, const TrajectoryData & ego_traj_data,
    const TargetObstacle & object, const rclcpp::Time & obj_base_time,
    const std::vector<double> & time_vec);

  boost::optional<SBoundaries> getSBoundaries(
    const TrajectoryData & ego_traj_data, const std::vector<double> & time_vec,
    const double safety_distance, const TargetObstacle & object,
    const double dist_to_collision_point);

  boost::optional<SBoundaries> getSBoundaries(
    const rclcpp::Time & current_time, const TrajectoryData & ego_traj_data,
    const std::vector<double> & time_vec, const double safety_distance,
    const TargetObstacle & object, const rclcpp::Time & obj_base_time,
    const PredictedPath & predicted_path);

  boost::optional<PredictedPath> resamplePredictedPath(
    const TargetObstacle & object, const rclcpp::Time & obj_base_time,
    const rclcpp::Time & current_time, const std::vector<double> & resolutions,
    const double horizon);

  boost::optional<double> getDistanceToCollisionPoint(
    const TrajectoryData & ego_traj_data, const ObjectData & obj_data);

  boost::optional<size_t> getCollisionIdx(
    const TrajectoryData & ego_traj, const Box2d & obj_box, const size_t start_idx,
    const size_t end_idx);

  geometry_msgs::msg::Pose transformBaseLink2Center(
    const geometry_msgs::msg::Pose & pose_base_link);

  boost::optional<VelocityOptimizer::OptimizationResult> processOptimizedResult(
    const double v0, const VelocityOptimizer::OptimizationResult & opt_result);

  void publishDebugTrajectory(
    const rclcpp::Time & current_time, const Trajectory & traj, const size_t closest_idx,
    const std::vector<double> & time_vec, const SBoundaries & s_boundaries,
    const VelocityOptimizer::OptimizationResult & opt_result);
  // Calculation time watcher
  tier4_autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch_;

  Trajectory prev_output_;

  // Velocity Optimizer
  std::shared_ptr<VelocityOptimizer> velocity_optimizer_ptr_;

  // Publisher
  rclcpp::Publisher<Trajectory>::SharedPtr boundary_pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr optimized_sv_pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr optimized_st_graph_pub_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr debug_calculation_time_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_wall_marker_pub_;

  // Resampling Parameter
  double resampling_s_interval_;
  double max_trajectory_length_;
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
