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
#ifndef OBSTACLE_AVOIDANCE_PLANNER__DEBUG_HPP_
#define OBSTACLE_AVOIDANCE_PLANNER__DEBUG_HPP_

#include <opencv2/core.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <vector>

struct ConstrainRectangle;
struct Bounds;
struct DebugData;
struct VehicleParam;

namespace util
{
struct Footprint;
}

visualization_msgs::msg::MarkerArray getDebugVisualizationMarker(
  const DebugData & debug_data,
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & optimized_points,
  const VehicleParam & vehicle_param);

geometry_msgs::msg::Pose getVirtualWallPose(
  const geometry_msgs::msg::Pose & target_pose, const VehicleParam & vehicle_param);

visualization_msgs::msg::MarkerArray getDebugPointsMarkers(
  const std::vector<geometry_msgs::msg::Point> & interpolated_points,
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & optimized_points,
  const std::vector<geometry_msgs::msg::Point> & straight_points,
  const std::vector<geometry_msgs::msg::Pose> & fixed_points,
  const std::vector<geometry_msgs::msg::Pose> & non_fixed_points);

visualization_msgs::msg::MarkerArray getDebugConstrainMarkers(
  const std::vector<ConstrainRectangle> & constrain_ranges, const std::string & ns);

visualization_msgs::msg::MarkerArray getObjectsMarkerArray(
  const std::vector<autoware_auto_perception_msgs::msg::PredictedObject> & objects,
  const std::string & ns, const double r, const double g, const double b);

visualization_msgs::msg::MarkerArray getRectanglesMarkerArray(
  const std::vector<util::Footprint> & rects, const std::string & ns, const double r,
  const double g, const double b);

visualization_msgs::msg::MarkerArray getRectanglesNumMarkerArray(
  const std::vector<util::Footprint> & rects, const std::string & ns, const double r,
  const double g, const double b);

visualization_msgs::msg::MarkerArray getPointsMarkerArray(
  const std::vector<geometry_msgs::msg::Pose> & points, const std::string & ns, const double r,
  const double g, const double b);

visualization_msgs::msg::MarkerArray getPointsMarkerArray(
  const std::vector<geometry_msgs::msg::Point> & points, const std::string & ns, const double r,
  const double g, const double b);

visualization_msgs::msg::MarkerArray getPointsTextMarkerArray(
  const std::vector<geometry_msgs::msg::Pose> & points, const std::string & ns, const double r,
  const double g, const double b);

visualization_msgs::msg::MarkerArray getPointsTextMarkerArray(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
  const std::string & ns, const double r, const double g, const double b);

visualization_msgs::msg::MarkerArray getBaseBoundsLineMarkerArray(
  const std::vector<Bounds> & bounds, const std::vector<geometry_msgs::msg::Pose> & candidate_p0,
  const std::string & ns, const double r, const double g, const double b);

visualization_msgs::msg::MarkerArray getTopBoundsLineMarkerArray(
  const std::vector<Bounds> & bounds, const std::vector<geometry_msgs::msg::Pose> & candidate_p1,
  const std::string & ns, const double r, const double g, const double b);

visualization_msgs::msg::MarkerArray getMidBoundsLineMarkerArray(
  const std::vector<Bounds> & bounds, const std::vector<geometry_msgs::msg::Pose> & candidate_top,
  const std::string & ns, const double r, const double g, const double b);

visualization_msgs::msg::MarkerArray getVirtualWallMarkerArray(
  const geometry_msgs::msg::Pose & pose, const std::string & ns, const double r, const double g,
  const double b);

visualization_msgs::msg::MarkerArray getVirtualWallTextMarkerArray(
  const geometry_msgs::msg::Pose & pose, const std::string & ns, const double r, const double g,
  const double b);

nav_msgs::msg::OccupancyGrid getDebugCostmap(
  const cv::Mat & clearance_map, const nav_msgs::msg::OccupancyGrid & occupancy_grid);
#endif  // OBSTACLE_AVOIDANCE_PLANNER__DEBUG_HPP_
