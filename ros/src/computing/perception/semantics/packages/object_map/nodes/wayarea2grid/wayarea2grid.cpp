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

#include "wayarea2grid.h"

namespace object_map
{

	WayareaToGrid::WayareaToGrid() :
			private_node_handle_("~")
	{
		InitializeRosIo();
		LoadRoadAreasFromVectorMap(private_node_handle_, area_points_);
	}


	void WayareaToGrid::InitializeRosIo()
	{
		private_node_handle_.param<std::string>("sensor_frame", sensor_frame_, "velodyne");
		private_node_handle_.param<std::string>("map_frame", map_frame_, "map");
		private_node_handle_.param<double>("grid_resolution", grid_resolution_, 0.2);
		private_node_handle_.param<double>("grid_length_x", grid_length_x_, 80);
		private_node_handle_.param<double>("grid_length_y", grid_length_y_, 30);
		private_node_handle_.param<double>("grid_position_x", grid_position_x_, 20);
		private_node_handle_.param<double>("grid_position_x", grid_position_y_, 0);

		publisher_grid_map_ = node_handle_.advertise<grid_map_msgs::GridMap>("grid_map_wayarea", 1, true);
		publisher_occupancy_ = node_handle_.advertise<nav_msgs::OccupancyGrid>("occupancy_wayarea", 1, true);
	}

	void WayareaToGrid::Run()
	{
		bool set_map = false;
		ros::Rate loop_rate(10);

		while (ros::ok())
		{
			if (!set_map)
			{
				gridmap_.add(grid_layer_name_);
				gridmap_.setFrameId(sensor_frame_);
				gridmap_.setGeometry(grid_map::Length(grid_length_x_, grid_length_y_),
				                grid_resolution_,
				                grid_map::Position(grid_position_x_, grid_position_y_));
				set_map = true;
			}

			// timer start
			//auto start = std::chrono::system_clock::now();

			if (!area_points_.empty())
			{
				FillPolygonAreas(gridmap_, area_points_, grid_layer_name_, OCCUPANCY_NO_ROAD, OCCUPANCY_ROAD, grid_min_value_,
				                 grid_max_value_, sensor_frame_, map_frame_,
				                 tf_listener_);
				PublishGridMap(gridmap_, publisher_grid_map_);
				PublishOccupancyGrid(gridmap_, publisher_occupancy_, grid_layer_name_, grid_min_value_, grid_max_value_);
			}

			// timer end
			//auto end = std::chrono::system_clock::now();
			//auto usec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
			//std::cout << "time: " << usec / 1000.0 << " [msec]" << std::endl;

			loop_rate.sleep();
		}
	}

}  // namespace object_map
