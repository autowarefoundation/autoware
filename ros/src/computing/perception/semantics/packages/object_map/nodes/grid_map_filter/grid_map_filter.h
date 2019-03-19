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
 ********************/
#ifndef GRID_MAP_FILTER_H
#define GRID_MAP_FILTER_H

#include <iostream>
#include <vector>
#include <string>
#include <chrono>

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_listener.h>

#include <vector_map/vector_map.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_cv/grid_map_cv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include "object_map/object_map_utils.hpp"

namespace object_map
{

	class GridMapFilter
	{
	public:
		GridMapFilter();

		void Run();

	private:
		// handle
		ros::NodeHandle                 nh_;
		ros::NodeHandle                 private_node_handle_;

		ros::Publisher                  grid_map_pub_;
		ros::Subscriber                 occupancy_grid_sub_;

		std::string                     map_frame_;
		std::string                     map_topic_;
		const std::string               grid_road_layer_    = "wayarea";
		double                          dist_transform_distance_;
		bool                            use_dist_transform_;
		bool                            use_wayarea_;
		bool                            use_fill_circle_;
		int                             fill_circle_cost_thresh_;
		double                          circle_radius_;

		int                             OCCUPANCY_ROAD      = 128;
		int                             OCCUPANCY_NO_ROAD   = 255;
		int                             OCCUPANCY_CIRCLE    = 255;
		const int                       grid_min_value_     = 0;
		const int                       grid_max_value_     = 255;
		const int                       costmap_min_        = 0;
		const int                       costmap_max_        = 100;

		tf::TransformListener           tf_listener_;

		std::vector<std::vector<geometry_msgs::Point>> area_points_;

		void OccupancyGridCallback(const nav_msgs::OccupancyGridConstPtr &in_message);

		/*!
		 * Initializes ROS Publisher, Subscribers and sets the configuration parameters
		 */
		void InitializeROSIo();

		/*!
		 * Applies a Distance Transform to the specified layer in in_layer contained in the GridMap
		 * @param[out] out_grid_map GridMap object to add the layer
		 * @param[in] layer Name of the layer to use for the transform
		 */
		void CreateDistanceTransformLayer(grid_map::GridMap &out_grid_map, const std::string &in_layer);

		/*!
		 * Draws a circle in the specified layer in the given GridMap if the cell value is larger than a threshold
		 * @param[out] out_gridmap GridMap object to modify
		 * @param[in] in_layer_name Name of the layer to draw
		 * @param[in] in_draw_threshold threshold to decide if the circle will be drawn
		 * @param[in] in_radius Radius of the circle
		 */
		void DrawCirclesInLayer(grid_map::GridMap &out_gridmap,
		                        const std::string &in_layer_name,
		                        double in_draw_threshold,
		                        double in_radius);
	};

}  // namespace object_map
#endif  // GRID_MAP_FILTER_H
