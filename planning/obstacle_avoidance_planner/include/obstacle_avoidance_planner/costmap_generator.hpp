// Copyright 2021 Tier IV, Inc.
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

#ifndef OBSTACLE_AVOIDANCE_PLANNER__COSTMAP_GENERATOR_HPP_
#define OBSTACLE_AVOIDANCE_PLANNER__COSTMAP_GENERATOR_HPP_

#include "obstacle_avoidance_planner/common_structs.hpp"
#include "tier4_autoware_utils/system/stop_watch.hpp"

#include "autoware_auto_perception_msgs/msg/predicted_object.hpp"
#include "autoware_auto_planning_msgs/msg/path.hpp"
#include "autoware_auto_planning_msgs/msg/path_point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "boost/optional.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <vector>

class CostmapGenerator
{
public:
  CVMaps getMaps(
    const bool enable_avoidance, const autoware_auto_planning_msgs::msg::Path & path,
    const std::vector<autoware_auto_perception_msgs::msg::PredictedObject> & objects,
    const TrajectoryParam & traj_param, DebugData & debug_data);

private:
  mutable tier4_autoware_utils::StopWatch<
    std::chrono::milliseconds, std::chrono::microseconds, std::chrono::steady_clock>
    stop_watch_;

  cv::Mat getAreaWithObjects(
    const cv::Mat & drivable_area, const cv::Mat & objects_image, DebugData & debug_data) const;

  cv::Mat getClearanceMap(const cv::Mat & drivable_area, DebugData & debug_data) const;

  cv::Mat drawObstaclesOnImage(
    const bool enable_avoidance,
    const std::vector<autoware_auto_perception_msgs::msg::PredictedObject> & objects,
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
    const nav_msgs::msg::MapMetaData & map_info, [[maybe_unused]] const cv::Mat & drivable_area,
    const cv::Mat & clearance_map, const TrajectoryParam & traj_param,
    std::vector<autoware_auto_perception_msgs::msg::PredictedObject> * debug_avoiding_objects,
    DebugData & debug_data) const;

  cv::Mat getDrivableAreaInCV(
    const nav_msgs::msg::OccupancyGrid & occupancy_grid, DebugData & debug_data) const;
};
#endif  // OBSTACLE_AVOIDANCE_PLANNER__COSTMAP_GENERATOR_HPP_
