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

#include "obstacle_avoidance_planner/vehicle_model/vehicle_model_interface.hpp"

#include <eigen3/Eigen/Core>

#include <autoware_auto_planning_msgs/msg/path_point.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>

#include <boost/optional.hpp>

#include <memory>
#include <vector>

namespace cv
{
class Mat;
}

namespace autoware::common::osqp
{
class OSQPInterface;
}

struct DebugData;
struct TrajectoryParam;
struct QPParam;
struct ConstrainParam;
struct VehicleParam;
struct Rectangle;
struct CVMaps;
struct Trajectories;

struct ReferencePoint
{
  geometry_msgs::msg::Point p;
  double k = 0;
  double v = 0;
  double yaw = 0;
  geometry_msgs::msg::Quaternion q;
  double s = 0;
  geometry_msgs::msg::Pose top_pose;
  geometry_msgs::msg::Pose mid_pose;
  double delta_yaw_from_p1 = 0;
  double delta_yaw_from_p2 = 0;
  boost::optional<Eigen::VectorXd> fix_state = boost::none;
  Eigen::VectorXd optimized_state;
};

struct Bounds
{
  struct SingleBounds
  {
    SingleBounds & operator=(std::vector<double> & bounds)
    {
      ub = bounds[0];
      lb = bounds[1];
      return *this;
    }
    double ub;  // left
    double lb;  // right
  } c0, c1, c2;
};

struct MPTTrajs
{
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> mpt;
  std::vector<ReferencePoint> ref_points;
};

struct MPTMatrix
{
  Eigen::MatrixXd Aex;
  Eigen::MatrixXd Bex;
  Eigen::MatrixXd Wex;
  Eigen::MatrixXd Cex;
  Eigen::MatrixXd Qex;
  Eigen::MatrixXd R1ex;
  Eigen::MatrixXd R2ex;
  Eigen::MatrixXd Uref_ex;
};

struct ObjectiveMatrix
{
  Eigen::MatrixXd hessian;
  std::vector<double> gradient;
};

struct ConstraintMatrix
{
  Eigen::MatrixXd linear;
  std::vector<double> lower_bound;
  std::vector<double> upper_bound;
};

struct MPTParam
{
  bool is_hard_fixing_terminal_point;
  int num_curvature_sampling_points;
  double base_point_dist_from_base_link;
  double top_point_dist_from_base_link;
  double mid_point_dist_from_base_link;
  double clearance_from_road;
  double clearance_from_object;
  double base_point_weight;
  double top_point_weight;
  double mid_point_weight;
  double lat_error_weight;
  double yaw_error_weight;
  double steer_input_weight;
  double steer_rate_weight;
  double steer_acc_weight;
  double terminal_lat_error_weight;
  double terminal_yaw_error_weight;
  double terminal_path_lat_error_weight;
  double terminal_path_yaw_error_weight;
  double zero_ff_steer_angle;
};

class MPTOptimizer
{
private:
  bool is_showing_debug_info_;

  std::unique_ptr<autoware::common::osqp::OSQPInterface> osqp_solver_ptr_;
  std::unique_ptr<MPTParam> mpt_param_ptr_;
  std::unique_ptr<TrajectoryParam> traj_param_ptr_;
  std::unique_ptr<QPParam> qp_param_ptr_;
  std::unique_ptr<ConstrainParam> constraint_param_ptr_;
  std::unique_ptr<VehicleParam> vehicle_param_ptr_;
  std::unique_ptr<VehicleModelInterface> vehicle_model_ptr_;

  std::vector<ReferencePoint> convertToReferencePoints(
    const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
    const std::unique_ptr<Trajectories> & prev_mpt_points,
    const geometry_msgs::msg::Pose & ego_pose, const CVMaps & maps, DebugData * debug_data) const;

  std::vector<ReferencePoint> getReferencePoints(
    const geometry_msgs::msg::Pose & origin_pose, const geometry_msgs::msg::Pose & ego_pose,
    const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
    const std::unique_ptr<Trajectories> & prev_mpt_points, const CVMaps & maps,
    DebugData * debug_data) const;

  std::vector<ReferencePoint> getBaseReferencePoints(
    const std::vector<geometry_msgs::msg::Point> & interpolated_points,
    const std::unique_ptr<Trajectories> & prev_trajs, DebugData * debug_data) const;

  void calcOrientation(std::vector<ReferencePoint> * ref_points) const;

  void calcVelocity(std::vector<ReferencePoint> * ref_points) const;

  void calcVelocity(
    std::vector<ReferencePoint> * ref_points,
    const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points) const;

  void calcCurvature(std::vector<ReferencePoint> * ref_points) const;

  void calcArcLength(std::vector<ReferencePoint> * ref_points) const;

  void calcExtraPoints(std::vector<ReferencePoint> * ref_points) const;

  void calcFixPoints(
    const std::unique_ptr<Trajectories> & prev_trajs, const geometry_msgs::msg::Pose & ego_pose,
    std::vector<ReferencePoint> * ref_points, DebugData * debug_data) const;

  void calcInitialState(
    std::vector<ReferencePoint> * ref_points, const geometry_msgs::msg::Pose & origin_pose) const;

  void setOptimizedState(ReferencePoint * ref_point, const Eigen::Vector3d & optimized_state) const;

  /*
   * predict equation: Xec = Aex * x0 + Bex * Uex + Wex
   * cost function: J = Xex' * Qex * Xex + (Uex - Uref)' * R1ex * (Uex - Uref_ex) + Uex' * R2ex *
   * Uex Qex = diag([Q,Q,...]), R1ex = diag([R,R,...])
   */
  boost::optional<MPTMatrix> generateMPTMatrix(
    const std::vector<ReferencePoint> & reference_points,
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
    const std::unique_ptr<Trajectories> & prev_trajs) const;

  void addSteerWeightR(Eigen::MatrixXd * R, const std::vector<ReferencePoint> & ref_points) const;

  void addSteerWeightF(Eigen::VectorXd * f) const;

  boost::optional<Eigen::VectorXd> executeOptimization(
    const bool enable_avoidance, const MPTMatrix & m,
    const std::vector<ReferencePoint> & ref_points,
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
    const CVMaps & maps, DebugData * debug_data);

  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> getMPTPoints(
    std::vector<ReferencePoint> & ref_points, const Eigen::VectorXd & Uex,
    const MPTMatrix & mpc_matrix,
    const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & optimized_points) const;

  double calcLateralError(
    const geometry_msgs::msg::Point & target_point, const ReferencePoint & ref_point) const;

  Eigen::VectorXd getState(
    const geometry_msgs::msg::Pose & ego_pose, const ReferencePoint & nearest_ref_point) const;

  std::vector<Bounds> getReferenceBounds(
    const bool enable_avoidance, const std::vector<ReferencePoint> & ref_points,
    const CVMaps & maps, DebugData * debug_data) const;

  boost::optional<std::vector<double>> getBound(
    const bool enable_avoidance, const ReferencePoint & ref_point, const CVMaps & maps) const;

  boost::optional<std::vector<double>> getBoundCandidate(
    const bool enable_avoidance, const ReferencePoint & ref_point, const CVMaps & maps,
    const double min_road_clearance, const double min_obj_clearance, const double rel_initial_lat,
    const std::vector<double> & rough_road_bound) const;

  boost::optional<std::vector<double>> getRoughBound(
    const bool enable_avoidance, const ReferencePoint & ref_point, const CVMaps & maps) const;

  double getTraversedDistance(
    const bool enable_avoidance, const ReferencePoint & ref_point, const double traverse_angle,
    const double initial_value, const CVMaps & maps, const double min_road_clearance,
    const double min_obj_clearance, const bool search_expanding_side = false) const;

  boost::optional<double> getValidLatError(
    const bool enable_avoidance, const ReferencePoint & ref_point, const double initial_value,
    const CVMaps & maps, const double min_road_clearance, const double min_obj_clearance,
    const std::vector<double> & rough_road_bound, const bool search_expanding_side = false) const;

  // TODO(unknown): refactor replace all relevant funcs
  boost::optional<double> getClearance(
    const cv::Mat & clearance_map, const geometry_msgs::msg::Point & map_point,
    const nav_msgs::msg::MapMetaData & map_info) const;

  ObjectiveMatrix getObjectiveMatrix(const Eigen::VectorXd & x0, const MPTMatrix & m) const;

  ConstraintMatrix getConstraintMatrix(
    const bool enable_avoidance, const Eigen::VectorXd & x0, const MPTMatrix & m,
    const CVMaps & maps, const std::vector<ReferencePoint> & ref_points,
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
    DebugData * debug_data) const;

public:
  MPTOptimizer(
    const bool is_showing_debug_info, const QPParam & qp_param, const TrajectoryParam & traj_param,
    const ConstrainParam & constrain_param, const VehicleParam & vehicle_param,
    const MPTParam & mpt_param);

  boost::optional<MPTTrajs> getModelPredictiveTrajectory(
    const bool enable_avoidance,
    const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & smoothed_points,
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
    const std::unique_ptr<Trajectories> & prev_trajs, const CVMaps & maps,
    const geometry_msgs::msg::Pose & ego_pose, DebugData * debug_data);
};

#endif  // OBSTACLE_AVOIDANCE_PLANNER__MPT_OPTIMIZER_HPP_
