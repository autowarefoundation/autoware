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

#include "object_map_utils.hpp"

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
		 * Initializes Ros Publisher, Subscribers and sets the configuration parameters
		 */
		void InitializeRosIo();

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
