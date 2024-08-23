// Copyright 2021 Tier IV, Inc. All rights reserved.
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

#ifndef AUTOWARE__FREESPACE_PLANNING_ALGORITHMS__ABSTRACT_ALGORITHM_HPP_
#define AUTOWARE__FREESPACE_PLANNING_ALGORITHMS__ABSTRACT_ALGORITHM_HPP_

#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/math/normalization.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <limits>
#include <vector>

namespace autoware::freespace_planning_algorithms
{
using autoware::universe_utils::normalizeRadian;

geometry_msgs::msg::Pose transformPose(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::TransformStamped & transform);
int discretizeAngle(const double theta, const int theta_size);

struct IndexXYT
{
  int x;
  int y;
  int theta;

  bool operator==(const IndexXYT & index) const
  {
    return (x == index.x && y == index.y && theta == index.theta);
  }
};

struct IndexXY
{
  int x;
  int y;
};

IndexXYT pose2index(
  const nav_msgs::msg::OccupancyGrid & costmap, const geometry_msgs::msg::Pose & pose_local,
  const int theta_size);

geometry_msgs::msg::Pose index2pose(
  const nav_msgs::msg::OccupancyGrid & costmap, const IndexXYT & index, const int theta_size);

geometry_msgs::msg::Pose global2local(
  const nav_msgs::msg::OccupancyGrid & costmap, const geometry_msgs::msg::Pose & pose_global);

geometry_msgs::msg::Pose local2global(
  const nav_msgs::msg::OccupancyGrid & costmap, const geometry_msgs::msg::Pose & pose_local);

struct VehicleShape
{
  double length;  // X [m]
  double width;   // Y [m]
  double base_length;
  double max_steering;
  double base2back;  // base_link to rear [m]
  double min_dimension;
  double max_dimension;

  VehicleShape() = default;

  VehicleShape(
    double length, double width, double base_length, double max_steering, double base2back)
  : length(length),
    width(width),
    base_length(base_length),
    max_steering(max_steering),
    base2back(base2back)
  {
    setMinMaxDimension();
  }

  explicit VehicleShape(
    const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const double margin = 0.0)
  : length(vehicle_info.vehicle_length_m + margin),
    width(vehicle_info.vehicle_width_m + margin),
    base_length(vehicle_info.wheel_base_m),
    max_steering(vehicle_info.max_steer_angle_rad),
    base2back(vehicle_info.rear_overhang_m + margin / 2.0)
  {
    setMinMaxDimension();
  }

  void setMinMaxDimension()
  {
    const auto base2front = length - base2back;
    if (base2back <= base2front) {
      min_dimension = std::min(0.5 * width, base2back);
      max_dimension = std::hypot(base2front, 0.5 * width);
    } else {
      min_dimension = std::min(0.5 * width, base2front);
      max_dimension = std::hypot(base2back, 0.5 * width);
    }
  }
};

struct PlannerCommonParam
{
  // base configs
  double time_limit;  // planning time limit [msec]

  // search configs
  int theta_size;                  // discretized angle table size [-]
  double curve_weight;             // curve moving cost [-]
  double reverse_weight;           // backward moving cost [-]
  double direction_change_weight;  // direction change cost [-]
  double lateral_goal_range;       // reaching threshold, lateral error [m]
  double longitudinal_goal_range;  // reaching threshold, longitudinal error [m]
  double angle_goal_range;         // reaching threshold, angle error [deg]
  double max_turning_ratio;        // factor of max turning range to use [-]
  int turning_steps;               // number of turning steps [-]

  // costmap configs
  int obstacle_threshold;  // obstacle threshold on grid [-]
};

struct PlannerWaypoint
{
  geometry_msgs::msg::PoseStamped pose;
  bool is_back = false;
};

struct PlannerWaypoints
{
  std_msgs::msg::Header header;
  std::vector<PlannerWaypoint> waypoints;

  double compute_length() const;
};

struct EDTData
{
  double distance;
  double angle;
};

class AbstractPlanningAlgorithm
{
public:
  AbstractPlanningAlgorithm(
    const PlannerCommonParam & planner_common_param, const VehicleShape & collision_vehicle_shape)
  : planner_common_param_(planner_common_param), collision_vehicle_shape_(collision_vehicle_shape)
  {
    planner_common_param_.turning_steps = std::max(planner_common_param_.turning_steps, 1);
    collision_vehicle_shape_.max_steering *= planner_common_param_.max_turning_ratio;
    is_collision_table_initialized = false;
  }

  AbstractPlanningAlgorithm(
    const PlannerCommonParam & planner_common_param,
    const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const double margin = 0.0)
  : planner_common_param_(planner_common_param), collision_vehicle_shape_(vehicle_info, margin)
  {
    planner_common_param_.turning_steps = std::max(planner_common_param_.turning_steps, 1);
    collision_vehicle_shape_.max_steering *= planner_common_param_.max_turning_ratio;
  }

  virtual void setMap(const nav_msgs::msg::OccupancyGrid & costmap);
  virtual bool makePlan(
    const geometry_msgs::msg::Pose & start_pose, const geometry_msgs::msg::Pose & goal_pose) = 0;
  virtual bool makePlan(
    const geometry_msgs::msg::Pose & start_pose,
    const std::vector<geometry_msgs::msg::Pose> & goal_candidates) = 0;
  virtual bool hasObstacleOnTrajectory(const geometry_msgs::msg::PoseArray & trajectory) const;
  const PlannerWaypoints & getWaypoints() const { return waypoints_; }
  double getDistanceToObstacle(const geometry_msgs::msg::Pose & pose) const;

  virtual ~AbstractPlanningAlgorithm() {}

protected:
  void computeCollisionIndexes(
    int theta_index, std::vector<IndexXY> & indexes,
    std::vector<IndexXY> & vertex_indexes_2d) const;
  bool detectBoundaryExit(const IndexXYT & base_index) const;
  bool detectCollision(const IndexXYT & base_index) const;
  bool detectCollision(const geometry_msgs::msg::Pose & base_pose) const;

  // cspell: ignore Toriwaki
  /// @brief Computes the euclidean distance to the nearest obstacle for each grid cell.
  /// @cite T., Saito, and J., Toriwaki "New algorithms for euclidean distance transformation of an
  /// n-dimensional digitized picture with applications," Pattern Recognition 27, 1994
  /// https://doi.org/10.1016/0031-3203(94)90133-3
  /// @details first, distance values are computed along each row. Then, the computed values are
  /// used to to compute the minimum distance along each column.
  void computeEDTMap();

  template <typename IndexType>
  inline bool isOutOfRange(const IndexType & index) const
  {
    if (index.x < 0 || static_cast<int>(costmap_.info.width) <= index.x) {
      return true;
    }
    if (index.y < 0 || static_cast<int>(costmap_.info.height) <= index.y) {
      return true;
    }
    return false;
  }

  template <typename IndexType>
  inline bool isWithinMargin(const IndexType & index) const
  {
    if (
      index.x < nb_of_margin_cells_ ||
      static_cast<int>(costmap_.info.width) - index.x < nb_of_margin_cells_) {
      return false;
    }
    if (
      index.y < nb_of_margin_cells_ ||
      static_cast<int>(costmap_.info.height) - index.y < nb_of_margin_cells_) {
      return false;
    }
    return true;
  }

  template <typename IndexType>
  inline bool isObs(const IndexType & index) const
  {
    // NOTE: Accessing by .at() instead makes 1.2 times slower here.
    // Also, boundary check is already done in isOutOfRange before calling this function.
    // So, basically .at() is not necessary.
    return is_obstacle_table_[indexToId(index)];
  }

  template <typename IndexType>
  inline EDTData getObstacleEDT(const IndexType & index) const
  {
    return edt_map_[indexToId(index)];
  }

  // compute single dimensional grid cell index from 2 dimensional index
  template <typename IndexType>
  inline int indexToId(const IndexType & index) const
  {
    return index.y * costmap_.info.width + index.x;
  }

  inline double getVehicleBaseToFrameDistance(const double angle) const
  {
    const double normalized_angle = std::abs(normalizeRadian(angle));
    const double w = 0.5 * collision_vehicle_shape_.width;
    const double l_b = collision_vehicle_shape_.base2back;
    const double l_f = collision_vehicle_shape_.length - l_b;

    if (normalized_angle < atan(w / l_f)) return l_f / cos(normalized_angle);
    if (normalized_angle < M_PI_2) return w / sin(normalized_angle);
    if (normalized_angle < M_PI_2 + atan(l_b / w)) return w / cos(normalized_angle - M_PI_2);
    return l_b / cos(M_PI - normalized_angle);
  }

  PlannerCommonParam planner_common_param_;
  VehicleShape collision_vehicle_shape_;

  // costmap as occupancy grid
  nav_msgs::msg::OccupancyGrid costmap_;

  // collision indexes cache
  std::vector<std::vector<IndexXY>> coll_indexes_table_;

  // vehicle vertex indexes cache
  std::vector<std::vector<IndexXY>> vertex_indexes_table_;

  // is_obstacle's table
  std::vector<bool> is_obstacle_table_;

  // Euclidean distance transform map (distance & angle info to nearest obstacle cell)
  std::vector<EDTData> edt_map_;

  // pose in costmap frame
  geometry_msgs::msg::Pose start_pose_;
  geometry_msgs::msg::Pose goal_pose_;

  // Is collision table initalized
  bool is_collision_table_initialized;

  // result path
  PlannerWaypoints waypoints_;

  int nb_of_margin_cells_;
};

}  // namespace autoware::freespace_planning_algorithms

#endif  // AUTOWARE__FREESPACE_PLANNING_ALGORITHMS__ABSTRACT_ALGORITHM_HPP_
