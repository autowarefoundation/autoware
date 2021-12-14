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

#ifndef OBSTACLE_AVOIDANCE_PLANNER__UTIL_HPP_
#define OBSTACLE_AVOIDANCE_PLANNER__UTIL_HPP_

#include <eigen3/Eigen/Core>
#include <tier4_autoware_utils/trajectory/trajectory.hpp>

#include <autoware_auto_planning_msgs/msg/path_point.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>

#include <boost/optional/optional_fwd.hpp>

#include <memory>
#include <vector>

struct VehicleParam;
struct ReferencePoint;
struct Trajectories;
struct TrajectoryParam;

namespace util
{
template <typename T>
geometry_msgs::msg::Point transformToRelativeCoordinate2D(
  const T & point, const geometry_msgs::msg::Pose & origin);

geometry_msgs::msg::Point transformToAbsoluteCoordinate2D(
  const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Pose & origin);

double calculate2DDistance(
  const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b);

double calculateSquaredDistance(
  const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b);

double getYawFromPoints(
  const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & a_root);

double normalizeRadian(const double angle);

geometry_msgs::msg::Quaternion getQuaternionFromPoints(
  const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & a_root);

geometry_msgs::msg::Quaternion getQuaternionFromYaw(const double yaw);

template <typename T>
geometry_msgs::msg::Point transformMapToImage(
  const T & map_point, const nav_msgs::msg::MapMetaData & occupancy_grid_info);

boost::optional<geometry_msgs::msg::Point> transformMapToOptionalImage(
  const geometry_msgs::msg::Point & map_point,
  const nav_msgs::msg::MapMetaData & occupancy_grid_info);

bool transformMapToImage(
  const geometry_msgs::msg::Point & map_point,
  const nav_msgs::msg::MapMetaData & occupancy_grid_info, geometry_msgs::msg::Point & image_point);

bool interpolate2DPoints(
  const std::vector<double> & x, const std::vector<double> & y, const double resolution,
  std::vector<geometry_msgs::msg::Point> & interpolated_points);

std::vector<geometry_msgs::msg::Point> getInterpolatedPoints(
  const std::vector<geometry_msgs::msg::Pose> & first_points,
  const std::vector<geometry_msgs::msg::Pose> & second_points, const double delta_arc_length);

std::vector<geometry_msgs::msg::Point> getInterpolatedPoints(
  const std::vector<geometry_msgs::msg::Pose> & points, const double delta_arc_length);

std::vector<geometry_msgs::msg::Point> getInterpolatedPoints(
  const std::vector<geometry_msgs::msg::Point> & points, const double delta_arc_length);

std::vector<geometry_msgs::msg::Point> getInterpolatedPoints(
  const std::vector<ReferencePoint> & points, const double delta_arc_length);

template <typename T>
std::vector<geometry_msgs::msg::Point> getInterpolatedPoints(
  const T & points, const double delta_arc_length);

template <typename T>
int getNearestIdx(
  const T & points, const geometry_msgs::msg::Pose & pose, const int default_idx,
  const double delta_yaw_threshold);

int getNearestIdx(
  const std::vector<geometry_msgs::msg::Point> & points, const geometry_msgs::msg::Pose & pose,
  const int default_idx, const double delta_yaw_threshold);

int getNearestIdxOverPoint(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Pose & pose, const int default_idx, const double delta_yaw_threshold);

template <typename T>
int getNearestIdx(const T & points, const geometry_msgs::msg::Point & point, const int default_idx);

int getNearestIdx(
  const std::vector<geometry_msgs::msg::Point> & points, const geometry_msgs::msg::Point & point);

template <typename T>
int getNearestIdx(const T & points, const geometry_msgs::msg::Point & point);

int getNearestIdx(
  const std::vector<ReferencePoint> & points, const double target_s, const int begin_idx);

template <typename T>
int getNearestPointIdx(const T & points, const geometry_msgs::msg::Point & point);

std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> convertPathToTrajectory(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points);

std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>
convertPointsToTrajectoryPointsWithYaw(const std::vector<geometry_msgs::msg::Point> & points);

std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> fillTrajectoryWithVelocity(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & traj_points,
  const double velocity);

template <typename T>
std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> alignVelocityWithPoints(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & traj_points,
  const T & points, const int zero_velocity_traj_idx, const int max_skip_comparison_idx);

struct Rectangle
{
  int min_x_idx = 0;
  int min_y_idx = 0;
  int max_x_idx = 0;
  int max_y_idx = 0;
  int area = 0;
};

std::vector<std::vector<int>> getHistogramTable(const std::vector<std::vector<int>> & input);

Rectangle getLargestRectangleInRow(
  const std::vector<int> & histo, const int current_row, const int row_size);

Rectangle getLargestRectangle(const std::vector<std::vector<int>> & input);

boost::optional<geometry_msgs::msg::Point> getLastExtendedPoint(
  const autoware_auto_planning_msgs::msg::PathPoint & path_point,
  const geometry_msgs::msg::Pose & pose, const double delta_yaw_threshold,
  const double delta_dist_threshold);

boost::optional<autoware_auto_planning_msgs::msg::TrajectoryPoint> getLastExtendedTrajPoint(
  const autoware_auto_planning_msgs::msg::PathPoint & path_point,
  const geometry_msgs::msg::Pose & pose, const double delta_yaw_threshold,
  const double delta_dist_threshold);

struct Footprint
{
  geometry_msgs::msg::Point p;
  geometry_msgs::msg::Point top_left;
  geometry_msgs::msg::Point top_right;
  geometry_msgs::msg::Point bottom_left;
  geometry_msgs::msg::Point bottom_right;
};

std::vector<Footprint> getVehicleFootprints(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & optimized_points,
  const VehicleParam & vehicle_param);

std::vector<double> calcEuclidDist(const std::vector<double> & x, const std::vector<double> & y);

bool hasValidNearestPointFromEgo(
  const geometry_msgs::msg::Pose & ego_pose, const Trajectories & trajs,
  const TrajectoryParam & traj_param);

std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> concatTraj(
  const Trajectories & trajs);

int getZeroVelocityIdx(
  const bool is_showing_debug_info, const std::vector<geometry_msgs::msg::Point> & fine_points,
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  const std::unique_ptr<Trajectories> & opt_trajs, const TrajectoryParam & traj_param);

template <typename T>
int getZeroVelocityIdxFromPoints(
  const T & points, const std::vector<geometry_msgs::msg::Point> & fine_points,
  const int /* default_idx */, const TrajectoryParam & traj_param);

template <typename T>
double getArcLength(const T & points, const int initial_idx = 0);

double getArcLength(
  const std::vector<geometry_msgs::msg::Pose> & points, const int initial_idx = 0);

void logOSQPSolutionStatus(const int solution_status);

}  // namespace util

#endif  // OBSTACLE_AVOIDANCE_PLANNER__UTIL_HPP_
