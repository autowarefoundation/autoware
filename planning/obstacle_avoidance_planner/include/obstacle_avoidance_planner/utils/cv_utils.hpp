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
#ifndef OBSTACLE_AVOIDANCE_PLANNER__UTILS__CV_UTILS_HPP_
#define OBSTACLE_AVOIDANCE_PLANNER__UTILS__CV_UTILS_HPP_

#include "obstacle_avoidance_planner/common_structs.hpp"
#include "obstacle_avoidance_planner/eb_path_optimizer.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/opencv.hpp"

#include "autoware_auto_perception_msgs/msg/predicted_object.hpp"
#include "autoware_auto_planning_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point32.hpp"

#include "boost/optional/optional_fwd.hpp"

#include <memory>
#include <vector>

namespace util
{
struct Footprint;
}

struct Edges
{
  int front_idx;
  int back_idx;
  geometry_msgs::msg::Point extended_front;
  geometry_msgs::msg::Point extended_back;
  geometry_msgs::msg::Point origin;
};

struct PolygonPoints
{
  std::vector<geometry_msgs::msg::Point> points_in_image;
  std::vector<geometry_msgs::msg::Point> points_in_map;
};

namespace cv_utils
{
void getOccupancyGridValue(
  const nav_msgs::msg::OccupancyGrid & og, const int i, const int j, unsigned char & value);

void putOccupancyGridValue(
  nav_msgs::msg::OccupancyGrid & og, const int i, const int j, const unsigned char value);
}  // namespace cv_utils

namespace cv_polygon_utils
{
PolygonPoints getPolygonPoints(
  const std::vector<geometry_msgs::msg::Point> & points,
  const nav_msgs::msg::MapMetaData & map_info);

PolygonPoints getPolygonPoints(
  const autoware_auto_perception_msgs::msg::PredictedObject & object,
  const nav_msgs::msg::MapMetaData & map_info);

PolygonPoints getPolygonPointsFromBB(
  const autoware_auto_perception_msgs::msg::PredictedObject & object,
  const nav_msgs::msg::MapMetaData & map_info);

PolygonPoints getPolygonPointsFromCircle(
  const autoware_auto_perception_msgs::msg::PredictedObject & object,
  const nav_msgs::msg::MapMetaData & map_info);

PolygonPoints getPolygonPointsFromPolygon(
  const autoware_auto_perception_msgs::msg::PredictedObject & object,
  const nav_msgs::msg::MapMetaData & map_info);

std::vector<cv::Point> getCVPolygon(
  const autoware_auto_perception_msgs::msg::PredictedObject & object,
  const PolygonPoints & polygon_points,
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  const cv::Mat & clearance_map, const nav_msgs::msg::MapMetaData & map_info);

std::vector<cv::Point> getDefaultCVPolygon(
  const std::vector<geometry_msgs::msg::Point> & points_in_image);

std::vector<cv::Point> getExtendedCVPolygon(
  const std::vector<geometry_msgs::msg::Point> & points_in_image,
  const std::vector<geometry_msgs::msg::Point> & points_in_map,
  const geometry_msgs::msg::Pose & nearest_path_point_pose,
  const autoware_auto_perception_msgs::msg::PredictedObject & object, const cv::Mat & clearance_map,
  const nav_msgs::msg::MapMetaData & map_info);

boost::optional<Edges> getEdges(
  const std::vector<geometry_msgs::msg::Point> & points_in_image,
  const std::vector<geometry_msgs::msg::Point> & points_in_map,
  const geometry_msgs::msg::Pose & nearest_path_point_pose,
  const autoware_auto_perception_msgs::msg::PredictedObject & object, const cv::Mat & clearance_map,
  const nav_msgs::msg::MapMetaData & map_info);
}  // namespace cv_polygon_utils

namespace cv_drivable_area_utils
{
bool isOutsideDrivableAreaFromRectangleFootprint(
  const autoware_auto_planning_msgs::msg::TrajectoryPoint & traj_point,
  const cv::Mat & road_clearance_map, const nav_msgs::msg::MapMetaData & map_info,
  const VehicleParam & vehicle_param);

bool isOutsideDrivableAreaFromCirclesFootprint(
  const autoware_auto_planning_msgs::msg::TrajectoryPoint & traj_point,
  const cv::Mat & road_clearance_map, const nav_msgs::msg::MapMetaData & map_info,
  const std::vector<double> vehicle_circle_longitudinal_offsets,
  const double vehicle_circle_radius);
}  // namespace cv_drivable_area_utils

#endif  // OBSTACLE_AVOIDANCE_PLANNER__UTILS__CV_UTILS_HPP_
