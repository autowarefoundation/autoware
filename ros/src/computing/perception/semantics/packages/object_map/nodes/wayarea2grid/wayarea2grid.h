/*
 *  Copyright (c) 2018, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

#include "object_map_utils.hpp"

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

		tf::TransformListener   tf_listener_;

		int                     OCCUPANCY_ROAD      = 128;
		int                     OCCUPANCY_NO_ROAD   = 255;
		const int               grid_min_value_     = 0;
		const int               grid_max_value_     = 255;

		std::vector<std::vector<geometry_msgs::Point>> area_points_;

		/*!
		 * Initializes Ros Publisher, Subscribers and sets the configuration parameters
		 */
		void InitializeRosIo();


	};

}  // namespace object_map

#endif  // WAYAREA_TO_GRID
