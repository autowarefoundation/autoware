// Copyright 2022 TIER IV, Inc.
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

#include "obstacle_velocity_limiter/obstacles.hpp"

#include "obstacle_velocity_limiter/occupancy_grid_utils.hpp"
#include "obstacle_velocity_limiter/pointcloud_utils.hpp"

#include <boost/assign.hpp>
#include <boost/geometry.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <tf2/utils.h>

namespace obstacle_velocity_limiter
{
polygon_t createObjectPolygon(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Vector3 & dimensions,
  const double buffer)
{
  // rename
  const auto x = pose.position.x;
  const auto y = pose.position.y;
  const auto h = dimensions.x + buffer;
  const auto w = dimensions.y + buffer;
  const auto yaw = tf2::getYaw(pose.orientation);

  // create base polygon
  polygon_t obj_poly;
  boost::geometry::exterior_ring(obj_poly) = boost::assign::list_of<point_t>(h / 2.0, w / 2.0)(
    -h / 2.0, w / 2.0)(-h / 2.0, -w / 2.0)(h / 2.0, -w / 2.0)(h / 2.0, w / 2.0);

  // rotate polygon(yaw)
  boost::geometry::strategy::transform::rotate_transformer<boost::geometry::radian, double, 2, 2>
    rotate(-yaw);  // anti-clockwise -> :clockwise rotation
  polygon_t rotate_obj_poly;
  boost::geometry::transform(obj_poly, rotate_obj_poly, rotate);

  // translate polygon(x, y)
  boost::geometry::strategy::transform::translate_transformer<double, 2, 2> translate(x, y);
  polygon_t translate_obj_poly;
  boost::geometry::transform(rotate_obj_poly, translate_obj_poly, translate);
  return translate_obj_poly;
}

multipolygon_t createObjectPolygons(
  const autoware_auto_perception_msgs::msg::PredictedObjects & objects, const double buffer,
  const double min_velocity)
{
  multipolygon_t polygons;
  for (const auto & object : objects.objects) {
    if (
      object.kinematics.initial_twist_with_covariance.twist.linear.x >= min_velocity ||
      object.kinematics.initial_twist_with_covariance.twist.linear.x <= -min_velocity) {
      polygons.push_back(createObjectPolygon(
        object.kinematics.initial_pose_with_covariance.pose, object.shape.dimensions, buffer));
    }
  }
  return polygons;
}

void addSensorObstacles(
  Obstacles & obstacles, const OccupancyGrid & occupancy_grid, const PointCloud & pointcloud,
  const ObstacleMasks & masks, tier4_autoware_utils::TransformListener & transform_listener,
  const std::string & target_frame, const ObstacleParameters & obstacle_params)
{
  if (obstacle_params.dynamic_source == ObstacleParameters::OCCUPANCYGRID) {
    auto grid_map = convertToGridMap(occupancy_grid);
    threshold(grid_map, obstacle_params.occupancy_grid_threshold);
    maskPolygons(grid_map, masks);
    const auto obstacle_lines = extractObstacles(grid_map, occupancy_grid);
    obstacles.lines.insert(obstacles.lines.end(), obstacle_lines.begin(), obstacle_lines.end());
  } else if (obstacle_params.dynamic_source == ObstacleParameters::POINTCLOUD) {
    auto pcd = transformPointCloud(pointcloud, transform_listener, target_frame);
    filterPointCloud(pcd, masks);
    obstacles.points = extractObstacles(pcd);
  }
}
}  // namespace obstacle_velocity_limiter
