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

#ifndef OBSTACLE_AVOIDANCE_PLANNER__EB_PATH_OPTIMIZER_HPP_
#define OBSTACLE_AVOIDANCE_PLANNER__EB_PATH_OPTIMIZER_HPP_

#include "eigen3/Eigen/Core"
#include "obstacle_avoidance_planner/common_structs.hpp"
#include "osqp_interface/osqp_interface.hpp"
#include "tier4_autoware_utils/system/stop_watch.hpp"

#include "autoware_auto_planning_msgs/msg/path.hpp"
#include "autoware_auto_planning_msgs/msg/path_point.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"

#include "boost/optional.hpp"

#include <memory>
#include <utility>
#include <vector>

class EBPathOptimizer
{
private:
  struct CandidatePoints
  {
    std::vector<geometry_msgs::msg::Pose> fixed_points;
    std::vector<geometry_msgs::msg::Pose> non_fixed_points;
    int begin_path_idx;
    int end_path_idx;
  };

  struct Anchor
  {
    geometry_msgs::msg::Pose pose;
  };

  struct ConstrainRectangles
  {
    ConstrainRectangle object_constrain_rectangle;
    ConstrainRectangle road_constrain_rectangle;
  };

  struct ConstrainLines
  {
    double x_coef;
    double y_coef;
    double lower_bound;
    double upper_bound;
  };

  struct Constrain
  {
    ConstrainLines top_and_bottom;
    ConstrainLines left_and_right;
  };

  struct Rectangle
  {
    geometry_msgs::msg::Point top_left;
    geometry_msgs::msg::Point top_right;
    geometry_msgs::msg::Point bottom_left;
    geometry_msgs::msg::Point bottom_right;
  };

  struct OccupancyMaps
  {
    std::vector<std::vector<int>> object_occupancy_map;
    std::vector<std::vector<int>> road_occupancy_map;
  };

  const bool is_showing_debug_info_;
  const double epsilon_;

  const QPParam qp_param_;
  const TrajectoryParam traj_param_;
  const EBParam eb_param_;
  const VehicleParam vehicle_param_;

  Eigen::MatrixXd default_a_matrix_;
  std::unique_ptr<autoware::common::osqp::OSQPInterface> osqp_solver_ptr_;

  double current_ego_vel_;

  mutable tier4_autoware_utils::StopWatch<
    std::chrono::milliseconds, std::chrono::microseconds, std::chrono::steady_clock>
    stop_watch_;

  Eigen::MatrixXd makePMatrix();

  Eigen::MatrixXd makeAMatrix();

  Anchor getAnchor(
    const std::vector<geometry_msgs::msg::Point> & interpolated_points, const int interpolated_idx,
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points) const;

  Constrain getConstrainFromConstrainRectangle(
    const geometry_msgs::msg::Point & interpolated_point,
    const ConstrainRectangle & constrain_range);

  ConstrainLines getConstrainLines(
    const double dx, const double dy, const geometry_msgs::msg::Point & point,
    const geometry_msgs::msg::Point & opposite_point);

  ConstrainRectangle getConstrainRectangle(const Anchor & anchor, const double clearance) const;

  ConstrainRectangle getConstrainRectangle(
    const Anchor & anchor, const double min_x, const double max_x, const double min_y,
    const double max_y) const;

  int getStraightLineIdx(
    const std::vector<geometry_msgs::msg::Point> & interpolated_points,
    const int farthest_point_idx,
    std::vector<geometry_msgs::msg::Point> & debug_detected_straight_points);

  boost::optional<std::vector<ConstrainRectangle>> getConstrainRectangleVec(
    const autoware_auto_planning_msgs::msg::Path & path,
    const std::vector<geometry_msgs::msg::Point> & interpolated_points, const int num_fixed_points,
    const int farthest_point_idx, const int straight_idx);

  std::vector<geometry_msgs::msg::Point> getPaddedInterpolatedPoints(
    const std::vector<geometry_msgs::msg::Point> & interpolated_points, const int farthest_idx);

  int getNumFixedPoints(
    const std::vector<geometry_msgs::msg::Pose> & fixed_points,
    const std::vector<geometry_msgs::msg::Point> & interpolated_points, const int farthest_idx);

  boost::optional<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>
  getOptimizedTrajectory(
    const autoware_auto_planning_msgs::msg::Path & path, const CandidatePoints & candidate_points,
    DebugData & debug_data);

  void updateConstrain(
    const std::vector<geometry_msgs::msg::Point> & interpolated_points,
    const std::vector<ConstrainRectangle> & rectangle_points);

  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> convertOptimizedPointsToTrajectory(
    const std::vector<double> optimized_points, const std::vector<ConstrainRectangle> & constraint,
    const int farthest_idx);

  std::vector<geometry_msgs::msg::Pose> getFixedPoints(
    const geometry_msgs::msg::Pose & ego_pose,
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
    const std::unique_ptr<Trajectories> & prev_optimized_points);

  CandidatePoints getCandidatePoints(
    const geometry_msgs::msg::Pose & ego_pose,
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
    const std::unique_ptr<Trajectories> & prev_trajs, DebugData & debug_data);

  CandidatePoints getDefaultCandidatePoints(
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points);

  std::pair<std::vector<double>, int64_t> solveQP();

  boost::optional<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>
  calculateTrajectory(
    const std::vector<geometry_msgs::msg::Point> & padded_interpolated_points,
    const std::vector<ConstrainRectangle> & constrain_rectangles, const int farthest_idx,
    DebugData & debug_data);

public:
  EBPathOptimizer(
    const bool is_showing_debug_info, const TrajectoryParam & traj_param, const EBParam & eb_param,
    const VehicleParam & vehicle_param);

  boost::optional<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>> getEBTrajectory(
    const geometry_msgs::msg::Pose & ego_pose, const autoware_auto_planning_msgs::msg::Path & path,
    const std::unique_ptr<Trajectories> & prev_trajs, const double current_ego_vel,
    DebugData & debug_data);
};

#endif  // OBSTACLE_AVOIDANCE_PLANNER__EB_PATH_OPTIMIZER_HPP_
