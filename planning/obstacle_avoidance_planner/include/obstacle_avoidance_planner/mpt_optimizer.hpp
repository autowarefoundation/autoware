// Copyright 2020 Tier IV, Inc.
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
/*
 * This Code is inspired by code from https://github.com/LiJiangnanBit/path_optimizer
 *
 * MIT License
 *
 * Copyright (c) 2020 Li Jiangnan
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef OBSTACLE_AVOIDANCE_PLANNER__MPT_OPTIMIZER_HPP_
#define OBSTACLE_AVOIDANCE_PLANNER__MPT_OPTIMIZER_HPP_

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Sparse"
#include "interpolation/linear_interpolation.hpp"
#include "obstacle_avoidance_planner/common_structs.hpp"
#include "obstacle_avoidance_planner/vehicle_model/vehicle_model_interface.hpp"
#include "osqp_interface/osqp_interface.hpp"
#include "tier4_autoware_utils/system/stop_watch.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#include "autoware_auto_planning_msgs/msg/path_point.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"

#include "boost/optional.hpp"

#include <memory>
#include <vector>

enum class CollisionType { NO_COLLISION = 0, OUT_OF_SIGHT = 1, OUT_OF_ROAD = 2, OBJECT = 3 };

struct Bounds
{
  Bounds() = default;
  Bounds(
    const double lower_bound_, const double upper_bound_, CollisionType lower_collision_type_,
    CollisionType upper_collision_type_)
  : lower_bound(lower_bound_),
    upper_bound(upper_bound_),
    lower_collision_type(lower_collision_type_),
    upper_collision_type(upper_collision_type_)
  {
  }

  double lower_bound;
  double upper_bound;

  CollisionType lower_collision_type;
  CollisionType upper_collision_type;

  bool hasCollisionWithRightObject() const { return lower_collision_type == CollisionType::OBJECT; }

  bool hasCollisionWithLeftObject() const { return upper_collision_type == CollisionType::OBJECT; }

  bool hasCollisionWithObject() const
  {
    return hasCollisionWithRightObject() || hasCollisionWithLeftObject();
  }

  static Bounds lerp(Bounds prev_bounds, Bounds next_bounds, double ratio)
  {
    const double lower_bound =
      interpolation::lerp(prev_bounds.lower_bound, next_bounds.lower_bound, ratio);
    const double upper_bound =
      interpolation::lerp(prev_bounds.upper_bound, next_bounds.upper_bound, ratio);

    if (ratio < 0.5) {
      return Bounds{
        lower_bound, upper_bound, prev_bounds.lower_collision_type,
        prev_bounds.upper_collision_type};
    }

    return Bounds{
      lower_bound, upper_bound, next_bounds.lower_collision_type, next_bounds.upper_collision_type};
  }

  void translate(const double offset)
  {
    lower_bound -= offset;
    upper_bound -= offset;
  }
};

struct ReferencePoint
{
  geometry_msgs::msg::Point p;
  double k = 0;
  double v = 0;
  double yaw = 0;
  double s = 0;
  double alpha = 0.0;
  Bounds bounds;
  bool near_objects;

  // NOTE: fix_kinematic_state is used for two purposes
  //       one is fixing points around ego for stability
  //       second is fixing current ego pose when no velocity for planning from ego pose
  boost::optional<Eigen::Vector2d> fix_kinematic_state = boost::none;
  bool plan_from_ego = true;
  Eigen::Vector2d optimized_kinematic_state;
  double optimized_input;

  //
  std::vector<boost::optional<double>> beta;
  VehicleBounds vehicle_bounds;

  // SequentialBoundsCandidates sequential_bounds_candidates;
  std::vector<geometry_msgs::msg::Pose> vehicle_bounds_poses;  // for debug visualization
};

class MPTOptimizer
{
private:
  struct MPTMatrix
  {
    // Eigen::MatrixXd Aex;
    Eigen::MatrixXd Bex;
    Eigen::VectorXd Wex;
    // Eigen::SparseMatrix<double> Cex;
    // Eigen::SparseMatrix<double> Qex;
    // Eigen::SparseMatrix<double> Rex;
    // Eigen::MatrixXd R1ex;
    // Eigen::MatrixXd R2ex;
    // Eigen::MatrixXd Uref_ex;
  };

  struct ValueMatrix
  {
    Eigen::SparseMatrix<double> Qex;
    Eigen::SparseMatrix<double> Rex;
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

  struct MPTTrajs
  {
    std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> mpt;
    std::vector<ReferencePoint> ref_points;
  };

  const double osqp_epsilon_ = 1.0e-3;

  bool is_showing_debug_info_;
  TrajectoryParam traj_param_;
  VehicleParam vehicle_param_;
  MPTParam mpt_param_;
  std::unique_ptr<VehicleModelInterface> vehicle_model_ptr_;
  std::unique_ptr<autoware::common::osqp::OSQPInterface> osqp_solver_ptr_;

  geometry_msgs::msg::Pose current_ego_pose_;
  double current_ego_vel_;

  int prev_mat_n = 0;
  int prev_mat_m = 0;

  mutable tier4_autoware_utils::StopWatch<
    std::chrono::milliseconds, std::chrono::microseconds, std::chrono::steady_clock>
    stop_watch_;

  std::vector<ReferencePoint> getReferencePoints(
    const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
    const std::unique_ptr<Trajectories> & prev_mpt_points, const bool enable_avoidance,
    const CVMaps & maps, DebugData & debug_data) const;

  void calcPlanningFromEgo(std::vector<ReferencePoint> & ref_points) const;

  /*
  std::vector<ReferencePoint> convertToReferencePoints(
    const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
    const std::unique_ptr<Trajectories> & prev_mpt_points, const bool enable_avoidance,
    const CVMaps & maps, DebugData & debug_data) const;
  */

  std::vector<ReferencePoint> getFixedReferencePoints(
    const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
    const std::unique_ptr<Trajectories> & prev_trajs) const;

  void calcBounds(
    std::vector<ReferencePoint> & ref_points, const bool enable_avoidance, const CVMaps & maps,
    const std::unique_ptr<Trajectories> & prev_trajs, DebugData & debug_data) const;

  void calcVehicleBounds(
    std::vector<ReferencePoint> & ref_points, const CVMaps & maps, DebugData & debug_data,
    const bool enable_avoidance) const;

  // void calcFixState(
  // std::vector<ReferencePoint> & ref_points,
  // const std::unique_ptr<Trajectories> & prev_trajs) const;

  void calcOrientation(std::vector<ReferencePoint> & ref_points) const;

  void calcCurvature(std::vector<ReferencePoint> & ref_points) const;

  void calcArcLength(std::vector<ReferencePoint> & ref_points) const;

  void calcExtraPoints(
    std::vector<ReferencePoint> & ref_points,
    const std::unique_ptr<Trajectories> & prev_trajs) const;

  /*
   * predict equation: Xec = Aex * x0 + Bex * Uex + Wex
   * cost function: J = Xex' * Qex * Xex + (Uex - Uref)' * R1ex * (Uex - Uref_ex) + Uex' * R2ex *
   * Uex Qex = diag([Q,Q,...]), R1ex = diag([R,R,...])
   */
  MPTMatrix generateMPTMatrix(
    const std::vector<ReferencePoint> & reference_points, DebugData & debug_data) const;

  ValueMatrix generateValueMatrix(
    const std::vector<ReferencePoint> & reference_points,
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
    DebugData & debug_data) const;

  void addSteerWeightR(
    std::vector<Eigen::Triplet<double>> & Rex_triplet_vec,
    const std::vector<ReferencePoint> & ref_points) const;

  boost::optional<Eigen::VectorXd> executeOptimization(
    const std::unique_ptr<Trajectories> & prev_trajs, const bool enable_avoidance,
    const MPTMatrix & mpt_mat, const ValueMatrix & obj_mat,
    const std::vector<ReferencePoint> & ref_points, DebugData & debug_data);

  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> getMPTPoints(
    std::vector<ReferencePoint> & fixed_ref_points,
    std::vector<ReferencePoint> & non_fixed_ref_points, const Eigen::VectorXd & Uex,
    const MPTMatrix & mpt_matrix, DebugData & debug_data);

  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> getMPTFixedPoints(
    const std::vector<ReferencePoint> & ref_points) const;

  BoundsCandidates getBoundsCandidates(
    const bool enable_avoidance, const geometry_msgs::msg::Pose & avoiding_point,
    const CVMaps & maps, DebugData & debug_data) const;

  CollisionType getCollisionType(
    const CVMaps & maps, const bool enable_avoidance,
    const geometry_msgs::msg::Pose & avoiding_point, const double traversed_dist,
    const double bound_angle) const;

  boost::optional<double> getClearance(
    const cv::Mat & clearance_map, const geometry_msgs::msg::Point & map_point,
    const nav_msgs::msg::MapMetaData & map_info) const;

  ObjectiveMatrix getObjectiveMatrix(
    const MPTMatrix & mpt_mat, const ValueMatrix & obj_mat,
    [[maybe_unused]] const std::vector<ReferencePoint> & ref_points, DebugData & debug_data) const;

  ConstraintMatrix getConstraintMatrix(
    const bool enable_avoidance, const MPTMatrix & mpt_mat,
    const std::vector<ReferencePoint> & ref_points, DebugData & debug_data) const;

public:
  MPTOptimizer(
    const bool is_showing_debug_info, const TrajectoryParam & traj_param,
    const VehicleParam & vehicle_param, const MPTParam & mpt_param);

  boost::optional<MPTTrajs> getModelPredictiveTrajectory(
    const bool enable_avoidance,
    const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & smoothed_points,
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
    const std::unique_ptr<Trajectories> & prev_trajs, const CVMaps & maps,
    const geometry_msgs::msg::Pose & current_ego_pose, const double current_ego_vel,
    DebugData & debug_data);
};

#endif  // OBSTACLE_AVOIDANCE_PLANNER__MPT_OPTIMIZER_HPP_
