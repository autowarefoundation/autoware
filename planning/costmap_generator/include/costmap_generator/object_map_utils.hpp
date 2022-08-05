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

/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************
 *
 */

#ifndef COSTMAP_GENERATOR__OBJECT_MAP_UTILS_HPP_
#define COSTMAP_GENERATOR__OBJECT_MAP_UTILS_HPP_

#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <grid_map_msgs/msg/grid_map.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <string>
#include <vector>

namespace object_map
{
/*!
 * Publishes in_gridmap using the specified in_publisher
 * @param[in] in_gridmap GridMap object to publish
 * @param[in] in_publisher Valid Publisher object to use
 */
void PublishGridMap(
  const grid_map::GridMap & in_gridmap,
  const rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr in_publisher);

/*!
 * Convert and publishes a GridMap layer to a standard Ros OccupancyGrid
 * @param[in] in_gridmap GridMap object to extract the layer
 * @param[in] in_publisher ROS Publisher to use to publish the occupancy grid
 * @param[in] in_layer Name of the layer to convert
 * @param[in] in_min_value Minimum value in the layer
 * @param[in] in_max_value Maximum value in the layer
 */

void PublishOccupancyGrid(
  const grid_map::GridMap & in_gridmap,
  const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr in_publisher,
  const std::string & in_layer, double in_min_value, double in_max_value, double in_height);

/*!
 * Projects the in_area_points forming the road, stores the result in out_grid_map.
 * @param[out] out_grid_map GridMap object to add the road grid
 * @param[in] in_points Array of points containing the selected primitives
 * @param[in] in_grid_layer_name Name to assign to the layer
 * @param[in] in_layer_background_value Empty state value
 * @param[in] in_fill_color Value to fill on selected primitives
 * @param[in] in_layer_min_value Minimum value in the layer
 * @param[in] in_layer_max_value Maximum value in the layer
 * @param[in] in_tf_target_frame Target frame to transform the points
 * @param[in] in_tf_source_frame Source frame, where the points are located
 * @param[in] in_tf_listener Valid listener to obtain the transformation
 */
void FillPolygonAreas(
  grid_map::GridMap & out_grid_map,
  const std::vector<std::vector<geometry_msgs::msg::Point>> & in_points,
  const std::string & in_grid_layer_name, const int in_layer_background_value,
  const int in_fill_color, const int in_layer_min_value, const int in_layer_max_value,
  const std::string & in_tf_target_frame, const std::string & in_tf_source_frame,
  const tf2_ros::Buffer & in_tf_buffer);

}  // namespace object_map

#endif  // COSTMAP_GENERATOR__OBJECT_MAP_UTILS_HPP_
