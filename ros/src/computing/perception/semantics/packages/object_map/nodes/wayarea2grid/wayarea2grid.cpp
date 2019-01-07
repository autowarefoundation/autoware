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

#include "wayarea2grid.h"

namespace object_map
{

	WayareaToGrid::WayareaToGrid() :
			private_node_handle_("~")
	{
		InitializeROSIo();
		LoadRoadAreasFromVectorMap(private_node_handle_, area_points_);
	}


	void WayareaToGrid::InitializeROSIo()
	{
		private_node_handle_.param<std::string>("sensor_frame", sensor_frame_, "velodyne");
		private_node_handle_.param<std::string>("map_frame", map_frame_, "map");
		private_node_handle_.param<double>("grid_resolution", grid_resolution_, 0.2);
		private_node_handle_.param<double>("grid_length_x", grid_length_x_, 80);
		private_node_handle_.param<double>("grid_length_y", grid_length_y_, 30);
		private_node_handle_.param<double>("grid_position_x", grid_position_x_, 20);
		private_node_handle_.param<double>("grid_position_y", grid_position_y_, 0);
		private_node_handle_.param<double>("grid_position_z", grid_position_z_, -2.f);

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
				PublishOccupancyGrid(gridmap_, publisher_occupancy_, grid_layer_name_, grid_min_value_, grid_max_value_, grid_position_z_);
			}

			// timer end
			//auto end = std::chrono::system_clock::now();
			//auto usec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
			//std::cout << "time: " << usec / 1000.0 << " [msec]" << std::endl;

			loop_rate.sleep();
		}
	}

}  // namespace object_map
