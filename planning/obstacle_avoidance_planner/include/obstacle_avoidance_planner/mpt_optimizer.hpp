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

#ifndef OBSTACLE_AVOIDANCE_PLANNER__MPT_OPTIMIZER_HPP_
#define OBSTACLE_AVOIDANCE_PLANNER__MPT_OPTIMIZER_HPP_

#include "gtest/gtest.h"
#include "interpolation/linear_interpolation.hpp"
#include "interpolation/spline_interpolation_points_2d.hpp"
#include "obstacle_avoidance_planner/common_structs.hpp"
#include "obstacle_avoidance_planner/state_equation_generator.hpp"
#include "obstacle_avoidance_planner/type_alias.hpp"
#include "osqp_interface/osqp_interface.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include <Eigen/Core>
#include <Eigen/Sparse>

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace obstacle_avoidance_planner
{
struct Bounds
{
  double lower_bound;
  double upper_bound;

  static Bounds lerp(Bounds prev_bounds, Bounds next_bounds, double ratio)
  {
    const double lower_bound =
      interpolation::lerp(prev_bounds.lower_bound, next_bounds.lower_bound, ratio);
    const double upper_bound =
      interpolation::lerp(prev_bounds.upper_bound, next_bounds.upper_bound, ratio);

    return Bounds{lower_bound, upper_bound};
  }

  void translate(const double offset)
  {
    lower_bound -= offset;
    upper_bound -= offset;
  }
};

struct KinematicState
{
  double lat{0.0};
  double yaw{0.0};

  Eigen::Vector2d toEigenVector() const { return Eigen::Vector2d{lat, yaw}; }
};

struct ReferencePoint
{
  geometry_msgs::msg::Pose pose;
  double longitudinal_velocity_mps;

  // additional information
  double curvature{0.0};
  double delta_arc_length{0.0};
  double alpha{0.0};                          // for minimizing lateral error
  Bounds bounds{};                            // bounds on `pose`
  std::vector<std::optional<double>> beta{};  // for collision-free constraint
  double normalized_avoidance_cost{0.0};

  // bounds and its local pose on each collision-free constraint
  std::vector<Bounds> bounds_on_constraints{};
  std::vector<geometry_msgs::msg::Pose> pose_on_constraints{};

  // optimization result
  std::optional<KinematicState> fixed_kinematic_state{std::nullopt};
  KinematicState optimized_kinematic_state{};
  double optimized_input{};
  std::optional<std::vector<double>> slack_variables{std::nullopt};

  double getYaw() const { return tf2::getYaw(pose.orientation); }

  geometry_msgs::msg::Pose offsetDeviation(const double lat_dev, const double yaw_dev) const
  {
    auto pose_with_deviation = tier4_autoware_utils::calcOffsetPose(pose, 0.0, lat_dev, 0.0);
    pose_with_deviation.orientation =
      tier4_autoware_utils::createQuaternionFromYaw(getYaw() + yaw_dev);
    return pose_with_deviation;
  }
};

class MPTOptimizer
{
public:
  MPTOptimizer(
    rclcpp::Node * node, const bool enable_debug_info, const EgoNearestParam ego_nearest_param,
    const vehicle_info_util::VehicleInfo & vehicle_info, const TrajectoryParam & traj_param,
    const std::shared_ptr<DebugData> debug_data_ptr,
    const std::shared_ptr<TimeKeeper> time_keeper_ptr);

  std::vector<TrajectoryPoint> optimizeTrajectory(
    const PlannerData & planner_data, const std::vector<TrajectoryPoint> & smoothed_points);
  std::optional<std::vector<TrajectoryPoint>> getPrevOptimizedTrajectoryPoints() const;

  void initialize(const bool enable_debug_info, const TrajectoryParam & traj_param);
  void resetPreviousData();
  void onParam(const std::vector<rclcpp::Parameter> & parameters);

  double getTrajectoryLength() const;
  double getDeltaArcLength() const;
  int getNumberOfPoints() const;

private:
  struct ValueMatrix
  {
    Eigen::SparseMatrix<double> Q;
    Eigen::SparseMatrix<double> R;
  };

  struct ObjectiveMatrix
  {
    Eigen::MatrixXd hessian;
    Eigen::VectorXd gradient;
  };

  struct ConstraintMatrix
  {
    Eigen::MatrixXd linear;
    Eigen::VectorXd lower_bound;
    Eigen::VectorXd upper_bound;
  };

  struct MPTParam
  {
    explicit MPTParam(rclcpp::Node * node, const vehicle_info_util::VehicleInfo & vehicle_info);
    MPTParam() = default;
    void onParam(const std::vector<rclcpp::Parameter> & parameters);

    // option
    bool enable_warm_start;
    bool enable_manual_warm_start;
    bool enable_optimization_validation;
    bool steer_limit_constraint;
    int mpt_visualize_sampling_num;  // for debug

    // common
    double delta_arc_length;
    int num_points;

    // kinematics
    double optimization_center_offset;
    double max_steer_rad;

    // clearance
    double hard_clearance_from_road;
    double soft_clearance_from_road;
    double soft_collision_free_weight;

    // weight
    double lat_error_weight;
    double yaw_error_weight;
    double yaw_error_rate_weight;

    double terminal_lat_error_weight;
    double terminal_yaw_error_weight;
    double goal_lat_error_weight;
    double goal_yaw_error_weight;

    double steer_input_weight;
    double steer_rate_weight;

    // avoidance
    double max_bound_fixing_time;
    double max_longitudinal_margin_for_bound_violation;
    double max_avoidance_cost;
    double avoidance_cost_margin;
    double avoidance_cost_band_length;
    double avoidance_cost_decrease_rate;
    double min_drivable_width;
    double avoidance_lat_error_weight;
    double avoidance_yaw_error_weight;
    double avoidance_steer_input_weight;

    // constraint type
    bool soft_constraint;
    bool hard_constraint;
    bool l_inf_norm;

    // vehicle circles
    std::string vehicle_circles_method;
    int vehicle_circles_uniform_circle_num;
    double vehicle_circles_uniform_circle_radius_ratio;
    int vehicle_circles_bicycle_model_num;
    double vehicle_circles_bicycle_model_rear_radius_ratio;
    double vehicle_circles_bicycle_model_front_radius_ratio;
    int vehicle_circles_fitting_uniform_circle_num;

    // validation
    double max_validation_lat_error;
    double max_validation_yaw_error;
  };

  // publisher
  rclcpp::Publisher<Trajectory>::SharedPtr debug_fixed_traj_pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr debug_ref_traj_pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr debug_mpt_traj_pub_;

  // argument
  bool enable_debug_info_;
  EgoNearestParam ego_nearest_param_;
  vehicle_info_util::VehicleInfo vehicle_info_;
  TrajectoryParam traj_param_;
  mutable std::shared_ptr<DebugData> debug_data_ptr_;
  mutable std::shared_ptr<TimeKeeper> time_keeper_ptr_;
  rclcpp::Logger logger_;
  MPTParam mpt_param_;

  StateEquationGenerator state_equation_generator_;
  std::unique_ptr<autoware::common::osqp::OSQPInterface> osqp_solver_ptr_;

  const double osqp_epsilon_ = 1.0e-3;

  // vehicle circles
  std::vector<double> vehicle_circle_longitudinal_offsets_;  // from base_link
  std::vector<double> vehicle_circle_radiuses_;

  // previous data
  int prev_mat_n_ = 0;
  int prev_mat_m_ = 0;
  int prev_solution_status_ = 0;
  std::shared_ptr<std::vector<ReferencePoint>> prev_ref_points_ptr_{nullptr};
  std::shared_ptr<std::vector<TrajectoryPoint>> prev_optimized_traj_points_ptr_{nullptr};

  void updateVehicleCircles();

  std::vector<ReferencePoint> calcReferencePoints(
    const PlannerData & planner_data, const std::vector<TrajectoryPoint> & smoothed_points) const;
  void updateCurvature(
    std::vector<ReferencePoint> & ref_points,
    const SplineInterpolationPoints2d & ref_points_spline) const;
  void updateOrientation(
    std::vector<ReferencePoint> & ref_points,
    const SplineInterpolationPoints2d & ref_points_spline) const;
  void updateBounds(
    std::vector<ReferencePoint> & ref_points,
    const std::vector<geometry_msgs::msg::Point> & left_bound,
    const std::vector<geometry_msgs::msg::Point> & right_bound,
    const geometry_msgs::msg::Pose & ego_pose, const double ego_vel) const;
  void keepMinimumBoundsWidth(std::vector<ReferencePoint> & ref_points) const;
  std::vector<ReferencePoint> extendViolatedBounds(
    const std::vector<ReferencePoint> & ref_points) const;
  void avoidSuddenSteering(
    std::vector<ReferencePoint> & ref_points, const geometry_msgs::msg::Pose & ego_pose,
    const double ego_vel) const;
  void updateVehicleBounds(
    std::vector<ReferencePoint> & ref_points,
    const SplineInterpolationPoints2d & ref_points_spline) const;
  void updateFixedPoint(std::vector<ReferencePoint> & ref_points) const;
  void updateDeltaArcLength(std::vector<ReferencePoint> & ref_points) const;
  void updateExtraPoints(std::vector<ReferencePoint> & ref_points) const;

  ValueMatrix calcValueMatrix(
    const std::vector<ReferencePoint> & reference_points,
    const std::vector<TrajectoryPoint> & traj_points) const;

  ObjectiveMatrix calcObjectiveMatrix(
    const StateEquationGenerator::Matrix & mpt_mat, const ValueMatrix & obj_mat,
    const std::vector<ReferencePoint> & ref_points) const;

  ConstraintMatrix calcConstraintMatrix(
    const StateEquationGenerator::Matrix & mpt_mat,
    const std::vector<ReferencePoint> & ref_points) const;

  std::optional<Eigen::VectorXd> calcOptimizedSteerAngles(
    const std::vector<ReferencePoint> & ref_points, const ObjectiveMatrix & obj_mat,
    const ConstraintMatrix & const_mat);
  Eigen::VectorXd calcInitialSolutionForManualWarmStart(
    const std::vector<ReferencePoint> & ref_points,
    const std::vector<ReferencePoint> & prev_ref_points) const;
  std::pair<ObjectiveMatrix, ConstraintMatrix> updateMatrixForManualWarmStart(
    const ObjectiveMatrix & obj_mat, const ConstraintMatrix & const_mat,
    const std::optional<Eigen::VectorXd> & u0) const;

  void addSteerWeightR(
    std::vector<Eigen::Triplet<double>> & R_triplet_vec,
    const std::vector<ReferencePoint> & ref_points) const;

  std::optional<std::vector<TrajectoryPoint>> calcMPTPoints(
    std::vector<ReferencePoint> & ref_points, const Eigen::VectorXd & optimized_variables,
    const StateEquationGenerator::Matrix & mpt_matrix) const;

  void publishDebugTrajectories(
    const std_msgs::msg::Header & header, const std::vector<ReferencePoint> & ref_points,
    const std::vector<TrajectoryPoint> & mpt_traj_points) const;
  std::vector<TrajectoryPoint> extractFixedPoints(
    const std::vector<ReferencePoint> & ref_points) const;

  size_t getNumberOfSlackVariables() const;
  std::optional<double> calcNormalizedAvoidanceCost(const ReferencePoint & ref_point) const;
};
}  // namespace obstacle_avoidance_planner
#endif  // OBSTACLE_AVOIDANCE_PLANNER__MPT_OPTIMIZER_HPP_
