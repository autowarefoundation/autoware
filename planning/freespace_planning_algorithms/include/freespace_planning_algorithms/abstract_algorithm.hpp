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

#ifndef FREESPACE_PLANNING_ALGORITHMS__ABSTRACT_ALGORITHM_HPP_
#define FREESPACE_PLANNING_ALGORITHMS__ABSTRACT_ALGORITHM_HPP_

#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <tf2/utils.h>

#include <vector>

namespace freespace_planning_algorithms
{
int discretizeAngle(const double theta, const int theta_size);

struct IndexXYT
{
  int x;
  int y;
  int theta;
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
  double length;     // X [m]
  double width;      // Y [m]
  double base2back;  // base_link to rear [m]
};

struct PlannerCommonParam
{
  // base configs
  double time_limit;  // planning time limit [msec]

  // robot configs
  double minimum_turning_radius;  // [m]
  double maximum_turning_radius;  // [m]
  int turning_radius_size;        // discretized turning radius table size [-]

  // search configs
  int theta_size;                  // discretized angle table size [-]
  double curve_weight;             // curve moving cost [-]
  double reverse_weight;           // backward moving cost [-]
  double lateral_goal_range;       // reaching threshold, lateral error [m]
  double longitudinal_goal_range;  // reaching threshold, longitudinal error [m]
  double angle_goal_range;         // reaching threshold, angle error [deg]

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

class AbstractPlanningAlgorithm
{
public:
  AbstractPlanningAlgorithm(
    const PlannerCommonParam & planner_common_param, const VehicleShape & collision_vehicle_shape)
  : planner_common_param_(planner_common_param), collision_vehicle_shape_(collision_vehicle_shape)
  {
  }

  virtual void setMap(const nav_msgs::msg::OccupancyGrid & costmap);
  virtual bool makePlan(
    const geometry_msgs::msg::Pose & start_pose, const geometry_msgs::msg::Pose & goal_pose) = 0;
  virtual bool hasObstacleOnTrajectory(const geometry_msgs::msg::PoseArray & trajectory) const;
  const PlannerWaypoints & getWaypoints() const { return waypoints_; }
  virtual ~AbstractPlanningAlgorithm() {}

protected:
  void computeCollisionIndexes(int theta_index, std::vector<IndexXY> & indexes) const;
  bool detectCollision(const IndexXYT & base_index) const;
  inline bool isOutOfRange(const IndexXYT & index) const
  {
    if (index.x < 0 || static_cast<int>(costmap_.info.width) <= index.x) {
      return true;
    }
    if (index.y < 0 || static_cast<int>(costmap_.info.height) <= index.y) {
      return true;
    }
    return false;
  }
  inline bool isObs(const IndexXYT & index) const
  {
    // NOTE: Accessing by .at() instead makes 1.2 times slower here.
    // Also, boundary check is already done in isOutOfRange before calling this function.
    // So, basically .at() is not necessary.
    return is_obstacle_table_[index.y][index.x];
  }

  PlannerCommonParam planner_common_param_;
  const VehicleShape collision_vehicle_shape_;

  // costmap as occupancy grid
  nav_msgs::msg::OccupancyGrid costmap_;

  // collision indexes cache
  std::vector<std::vector<IndexXY>> coll_indexes_table_;

  // is_obstacle's table
  std::vector<std::vector<bool>> is_obstacle_table_;

  // pose in costmap frame
  geometry_msgs::msg::Pose start_pose_;
  geometry_msgs::msg::Pose goal_pose_;

  // result path
  PlannerWaypoints waypoints_;
};

}  // namespace freespace_planning_algorithms

#endif  // FREESPACE_PLANNING_ALGORITHMS__ABSTRACT_ALGORITHM_HPP_
