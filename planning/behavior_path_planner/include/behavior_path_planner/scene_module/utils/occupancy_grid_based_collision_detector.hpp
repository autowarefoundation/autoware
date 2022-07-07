// Copyright 2022 TIER IV, Inc. All rights reserved.
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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__UTILS__OCCUPANCY_GRID_BASED_COLLISION_DETECTOR_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__UTILS__OCCUPANCY_GRID_BASED_COLLISION_DETECTOR_HPP_

#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <tf2/utils.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <vector>

namespace behavior_path_planner
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

struct OccupancyGridMapParam
{
  // robot configs
  VehicleShape vehicle_shape;

  // costmap configs
  int theta_size;          // discretized angle table size [-]
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
};

class OccupancyGridBasedCollisionDetector
{
public:
  OccupancyGridBasedCollisionDetector() {}
  void setParam(const OccupancyGridMapParam & param) { param_ = param; };
  OccupancyGridMapParam getParam() const { return param_; };
  void setMap(const nav_msgs::msg::OccupancyGrid & costmap);
  nav_msgs::msg::OccupancyGrid getMap() const { return costmap_; };
  void setVehicleShape(const VehicleShape & vehicle_shape) { param_.vehicle_shape = vehicle_shape; }
  bool hasObstacleOnPath(
    const geometry_msgs::msg::PoseArray & path, const bool check_out_of_range) const;
  bool hasObstacleOnPath(
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
    const bool check_out_of_range) const;
  const PlannerWaypoints & getWaypoints() const { return waypoints_; }
  bool detectCollision(const IndexXYT & base_index, const bool check_out_of_range) const;
  virtual ~OccupancyGridBasedCollisionDetector() {}

protected:
  void computeCollisionIndexes(int theta_index, std::vector<IndexXY> & indexes);
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

  OccupancyGridMapParam param_;

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

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__UTILS__OCCUPANCY_GRID_BASED_COLLISION_DETECTOR_HPP_
