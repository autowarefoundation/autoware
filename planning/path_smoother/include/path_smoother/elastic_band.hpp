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

#ifndef PATH_SMOOTHER__ELASTIC_BAND_HPP_
#define PATH_SMOOTHER__ELASTIC_BAND_HPP_

#include "osqp_interface/osqp_interface.hpp"
#include "path_smoother/common_structs.hpp"
#include "path_smoother/type_alias.hpp"

#include <Eigen/Core>

#include <memory>
#include <optional>
#include <tuple>
#include <utility>
#include <vector>

namespace path_smoother
{
class EBPathSmoother
{
public:
  EBPathSmoother(
    rclcpp::Node * node, const bool enable_debug_info, const EgoNearestParam ego_nearest_param,
    const CommonParam & common_param, const std::shared_ptr<TimeKeeper> time_keeper_ptr);

  std::vector<TrajectoryPoint> smoothTrajectory(
    const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & ego_pose);

  void initialize(const bool enable_debug_info, const CommonParam & common_param);
  void resetPreviousData();
  void onParam(const std::vector<rclcpp::Parameter> & parameters);

private:
  struct EBParam
  {
    // qp
    struct QPParam
    {
      int max_iteration;
      double eps_abs;
      double eps_rel;
    };

    EBParam() = default;
    explicit EBParam(rclcpp::Node * node);
    void onParam(const std::vector<rclcpp::Parameter> & parameters);

    // option
    bool enable_warm_start;
    bool enable_optimization_validation;

    // common
    double delta_arc_length;
    int num_points;

    // clearance
    int num_joint_points;
    double clearance_for_fix;
    double clearance_for_joint;
    double clearance_for_smooth;

    // weight
    double smooth_weight;
    double lat_error_weight;

    // qp
    QPParam qp_param;

    // validation
    double max_validation_error;
  };

  struct Constraint2d
  {
    struct Constraint
    {
      Eigen::Vector2d coef;
      double upper_bound;
      double lower_bound;
    };

    Constraint lon;
    Constraint lat;
  };

  // arguments
  bool enable_debug_info_;
  EgoNearestParam ego_nearest_param_;
  CommonParam common_param_;
  EBParam eb_param_;
  mutable std::shared_ptr<TimeKeeper> time_keeper_ptr_;
  rclcpp::Logger logger_;
  rclcpp::Clock clock_;

  // publisher
  rclcpp::Publisher<Trajectory>::SharedPtr debug_eb_traj_pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr debug_eb_fixed_traj_pub_;

  std::unique_ptr<autoware::common::osqp::OSQPInterface> osqp_solver_ptr_;
  std::shared_ptr<std::vector<TrajectoryPoint>> prev_eb_traj_points_ptr_{nullptr};

  std::vector<TrajectoryPoint> insertFixedPoint(
    const std::vector<TrajectoryPoint> & traj_point) const;

  std::tuple<std::vector<TrajectoryPoint>, size_t> getPaddedTrajectoryPoints(
    const std::vector<TrajectoryPoint> & traj_points) const;

  void updateConstraint(
    const std::vector<TrajectoryPoint> & traj_points, const bool is_goal_contained,
    const int pad_start_idx);

  std::optional<std::vector<double>> calcSmoothedTrajectory();

  std::optional<std::vector<TrajectoryPoint>> convertOptimizedPointsToTrajectory(
    const std::vector<double> & optimized_points, const std::vector<TrajectoryPoint> & traj_points,
    const int pad_start_idx) const;
};
}  // namespace path_smoother

#endif  // PATH_SMOOTHER__ELASTIC_BAND_HPP_
