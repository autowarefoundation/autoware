/*
 * Copyright 2016-2019 Autoware Foundation. All rights reserved.
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
 */

/**
 * @brief Way Planner represents the Global planning Module, to generate global plan (reference path) the nodes takes start position, goal position and return sequence of waypoints.
 * @brief Once the node starts (depending on the mapping option) it will load the vector map (road network) and send it to be visualized by RViz. topic name "/vector_map_center_lines_rviz"
 * @param Start_Position in simulation environment like rviz, this node require the user to select start position using "2D Pose Estimate" button and select starting position from the global path, //
 * if localization node is working and ndt_pose or curr_pose messages are published the node will use localization as starting position instead of "2D Pose Estimate"
 * @param Goal_Position destination to generate global plan to. if "replan" option selection used can choose multiple goal positions. goal positions are selected from Rviz using "2D Nav Goal" button.
 * @return global , reference path as list of waypoints. data type is "autoware_msgs::LaneArray" , and the topic name is "lane_waypoints_array"
 */

#include <ros/ros.h>
#include "way_planner_core.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "way_planner");
	WayPlannerNS::way_planner_core global_planner;
	global_planner.PlannerMainLoop();
	return 0;
}
