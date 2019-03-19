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
 */
#ifndef WAYAREA_TO_GRID_H
#define WAYAREA_TO_GRID_H

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

	class WayareaToGrid
	{
	public:
		WayareaToGrid();

		void Run();

	private:
		// handle
		ros::NodeHandle         node_handle_;
		ros::NodeHandle         private_node_handle_;

		ros::Publisher          publisher_grid_map_;
		ros::Publisher          publisher_occupancy_;

		grid_map::GridMap       gridmap_;

		std::string             sensor_frame_;
		std::string             map_frame_;

		const std::string       grid_layer_name_ = "wayarea";

		double                  grid_resolution_;
		double                  grid_length_x_;
		double                  grid_length_y_;
		double                  grid_position_x_;
		double                  grid_position_y_;
		double                  grid_position_z_;

		tf::TransformListener   tf_listener_;

		int                     OCCUPANCY_ROAD      = 128;
		int                     OCCUPANCY_NO_ROAD   = 255;
		const int               grid_min_value_     = 0;
		const int               grid_max_value_     = 255;

		std::vector<std::vector<geometry_msgs::Point>> area_points_;

		/*!
		 * Initializes ROS Publisher, Subscribers and sets the configuration parameters
		 */
		void InitializeROSIo();


	};

}  // namespace object_map

#endif  // WAYAREA_TO_GRID
