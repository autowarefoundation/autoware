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

#ifndef OBSTACLE_VELOCITY_LIMITER__OBSTACLES_HPP_
#define OBSTACLE_VELOCITY_LIMITER__OBSTACLES_HPP_

#include "obstacle_velocity_limiter/parameters.hpp"
#include "obstacle_velocity_limiter/types.hpp"

#include <tier4_autoware_utils/ros/transform_listener.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/overlaps.hpp>
#include <boost/geometry/index/predicates.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace obstacle_velocity_limiter
{

struct Obstacles
{
  multi_linestring_t lines;
  multipoint_t points;
};

namespace bgi = boost::geometry::index;
template <class T>
struct ObstacleTree
{
  explicit ObstacleTree(const T & obstacles);
  [[nodiscard]] std::vector<point_t> intersections(const polygon_t & polygon) const;
};
template <>
struct ObstacleTree<multipoint_t>
{
  bgi::rtree<point_t, bgi::rstar<16>> points_rtree;

  explicit ObstacleTree(const multipoint_t & obstacles) : points_rtree(obstacles) {}

  [[nodiscard]] std::vector<point_t> intersections(const polygon_t & polygon) const
  {
    std::vector<point_t> result;
    // Query the points
    points_rtree.query(bgi::covered_by(polygon), std::back_inserter(result));
    return result;
  }
};

template <>
struct ObstacleTree<multi_linestring_t>
{
  bgi::rtree<segment_t, bgi::rstar<16>> segments_rtree;

  explicit ObstacleTree(const multi_linestring_t & obstacles)
  {
    auto segment_count = 0lu;
    for (const auto & line : obstacles)
      if (!line.empty()) segment_count += line.size() - 1;
    std::vector<segment_t> segments;
    segments.reserve(segment_count);
    for (const auto & line : obstacles)
      for (size_t i = 0; i + 1 < line.size(); ++i) segments.emplace_back(line[i], line[i + 1]);
    segments_rtree = bgi::rtree<segment_t, bgi::rstar<16>>(segments);
  }

  [[nodiscard]] std::vector<point_t> intersections(const polygon_t & polygon) const
  {
    std::vector<point_t> result;
    // Query the segments
    std::vector<segment_t> candidates;
    segments_rtree.query(bgi::intersects(polygon), std::back_inserter(candidates));
    std::vector<point_t> intersection_points;
    for (const auto & candidate : candidates) {
      // need conversion to a linestring to use the 'intersection' and 'within' functions
      const auto ls = linestring_t{candidate.first, candidate.second};
      intersection_points.clear();
      boost::geometry::intersection(ls, polygon, intersection_points);
      if (intersection_points.empty()) {
        // No intersection with the polygon: segment is outside or inside of the polygon
        if (boost::geometry::within(ls, polygon)) {
          result.push_back(candidate.first);
          result.push_back(candidate.second);
        }
      } else {
        result.insert(result.end(), intersection_points.begin(), intersection_points.end());
      }
    }
    return result;
  }
};

struct CollisionChecker
{
  const Obstacles obstacles;
  std::unique_ptr<ObstacleTree<multipoint_t>> point_obstacle_tree_ptr;
  std::unique_ptr<ObstacleTree<multi_linestring_t>> line_obstacle_tree_ptr;

  explicit CollisionChecker(
    Obstacles obs, const size_t rtree_min_points, const size_t rtree_min_segments)
  : obstacles(std::move(obs))
  {
    auto segment_count = 0lu;
    for (const auto & line : obstacles.lines)
      if (!line.empty()) segment_count += line.size() - 1;
    if (segment_count > rtree_min_segments)
      line_obstacle_tree_ptr = std::make_unique<ObstacleTree<multi_linestring_t>>(obstacles.lines);
    if (obstacles.points.size() > rtree_min_points)
      point_obstacle_tree_ptr = std::make_unique<ObstacleTree<multipoint_t>>(obstacles.points);
  }

  [[nodiscard]] std::vector<point_t> intersections(const polygon_t & polygon) const
  {
    std::vector<point_t> result;
    if (line_obstacle_tree_ptr) {
      result = line_obstacle_tree_ptr->intersections(polygon);
    } else {
      boost::geometry::intersection(polygon, obstacles.lines, result);
    }
    if (point_obstacle_tree_ptr) {
      const auto & point_result = point_obstacle_tree_ptr->intersections(polygon);
      result.insert(result.end(), point_result.begin(), point_result.end());
    } else {
      for (const auto & point : obstacles.points)
        if (boost::geometry::intersects(polygon, point)) result.push_back(point);
    }
    return result;
  }
};

/// @brief create a polygon from an object represented by a pose and a size
/// @param [in] pose pose of the object
/// @param [in] dimensions dimensions of the object
/// @param [in] buffer buffer to add to the dimensions of the object
/// @return polygon of the object
polygon_t createObjectPolygon(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Vector3 & dimensions,
  const double buffer);

/// @brief create polygons from a set of predicted object
/// @param [in] objects objects from which to create polygons
/// @param [in] buffer buffer to add to the objects dimensions
/// @param [in] min_velocity objects with velocity lower will be ignored
/// @return polygons of the objects
multi_polygon_t createObjectPolygons(
  const autoware_auto_perception_msgs::msg::PredictedObjects & objects, const double buffer,
  const double min_velocity);

/// @brief add obstacles obtained from sensors to the given Obstacles object
/// @param[out] obstacles Obstacles object in which to add the sensor obstacles
/// @param[in] occupancy_grid occupancy grid
/// @param[in] pointcloud pointcloud
/// @param[in] masks masks used to discard some obstacles
/// @param[in] transform_listener object used to retrieve the latest transform
/// @param[in] target_frame frame of the returned obstacles
/// @param[in] obstacle_params obstacle parameters
void addSensorObstacles(
  Obstacles & obstacles, const OccupancyGrid & occupancy_grid, const PointCloud & pointcloud,
  const ObstacleMasks & masks, tier4_autoware_utils::TransformListener & transform_listener,
  const std::string & target_frame, const ObstacleParameters & obstacle_params);

/// @brief filter obstacles with the given negative and positive masks
/// @param[in] obstacles obstacles to filter
/// @param[in] masks masks used to discard some obstacles
/// @return obstacles that are inside the positive mask and outside of the negative masks
Obstacles filterObstacles(const Obstacles & obstacles, const ObstacleMasks & masks);
}  // namespace obstacle_velocity_limiter
#endif  // OBSTACLE_VELOCITY_LIMITER__OBSTACLES_HPP_
