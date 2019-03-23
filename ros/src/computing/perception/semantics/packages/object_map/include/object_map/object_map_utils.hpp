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
#ifndef PROJECT_OBJECT_MAP_UTILS_H
#define PROJECT_OBJECT_MAP_UTILS_H

#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <vector_map/vector_map.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_cv/grid_map_cv.hpp>

namespace object_map
{
  /*!
   * Transforms a point using the given transformation
   * @param[in] in_point Point to transform
   * @param[in] in_tf Transformation to apply
   * @return Transformed point
   */
  geometry_msgs::Point TransformPoint(const geometry_msgs::Point &in_point, const tf::Transform &in_tf);

  /*!
   * Publishes in_gridmap using the specified in_publisher
   * @param[in] in_gridmap GridMap object to publish
   * @param[in] in_publisher Valid Publisher object to use
   */
  void PublishGridMap(const grid_map::GridMap &in_gridmap, const ros::Publisher &in_publisher);

  /*!
   * Convert and publishes a GridMap layer to a standard Ros OccupancyGrid
   * @param[in] in_gridmap GridMap object to extract the layer
   * @param[in] in_publisher ROS Publisher to use to publish the occupancy grid
   * @param[in] in_layer Name of the layer to convert
   * @param[in] in_min_value Minimum value in the layer
   * @param[in] in_max_value Maximum value in the layer
   */
  void PublishOccupancyGrid(const grid_map::GridMap &in_gridmap,
                            const ros::Publisher &in_publisher,
                            const std::string &in_layer,
                            double in_min_value,
                            double in_max_value,
                            double in_height);

  /*!
   * Obtains the registered transform in the tf tree
   * @param[in] in_target_frame Target frame to obtain the transformation
   * @param[in] in_source_frame Source Frame from which is desired to transform
   * @param[in] in_tf_listener Valid Object Listener
   * @return Current Transform from source to target, if available. Otherwise, it returns an identity transform.
   */
  tf::StampedTransform FindTransform(const std::string &in_target_frame,
                                     const std::string &in_source_frame,
                                     const tf::TransformListener &in_tf_listener);

	/*!
		 * Loads regions defined as road inside the vector map, according to the field named "wayarea"
		 */
	void LoadRoadAreasFromVectorMap(ros::NodeHandle& in_private_node_handle,
	                                std::vector<std::vector<geometry_msgs::Point>>& out_area_points);

	/*!
	 * Extracts all the points forming in_area inside in_vectormap
	 * @param[in] in_area Area to extract its points
	 * @param[in] in_vectormap VectorMap object to which in_area belongs
	 * @return Array of points forming in_area
	 */
	std::vector<geometry_msgs::Point>
	SearchAreaPoints(const vector_map::Area &in_area, const vector_map::VectorMap &in_vectormap);

  /*!
   * Projects the in_area_points forming the road, stores the result in out_grid_map.
   * @param[out] out_grid_map GridMap object to add the road grid
   * @param[in] in_area_points Array of points containing the wayareas
   * @param[in] in_grid_layer_name Name to assign to the layer
   * @param[in] in_layer_background_value Empty state value
   * @param[in] in_fill_color Value to fill on wayareas
   * @param[in] in_layer_min_value Minimum value in the layer
   * @param[in] in_layer_max_value Maximum value in the later
   * @param[in] in_tf_target_frame Target frame to transform the wayarea points
   * @param[in] in_tf_source_frame Source frame, where the points are located
   * @param[in] in_tf_listener Valid listener to obtain the transformation
   */
  void FillPolygonAreas(grid_map::GridMap &out_grid_map,
                        const std::vector<std::vector<geometry_msgs::Point>> &in_area_points,
                        const std::string &in_grid_layer_name,
                        const int in_layer_background_value,
                        const int in_fill_color,
                        const int in_layer_min_value,
                        const int in_layer_max_value,
                        const std::string &in_tf_target_frame,
                        const std::string &in_tf_source_frame,
                        const tf::TransformListener &in_tf_listener);



} // namespace object_map

#endif //PROJECT_OBJECT_MAP_UTILS_H
