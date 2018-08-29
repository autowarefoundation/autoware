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
*/

/**
 * @brief global Planner represents the Global planning Module, to generate global plan (reference path) the nodes takes start position, goal position and return sequence of waypoints.
 * @brief Once the node starts (depending on the mapping option) it will load the vector map (road network) and send it to be visualized by RViz. topic name "/vector_map_center_lines_rviz"
 * @param Start_Position in simulation environment like rviz, this node require the user to select start position using "2D Pose Estimate" button and select starting position from the global path, //
 * if localization node is working and ndt_pose or curr_pose messages are published the node will use localization as starting position instead of "2D Pose Estimate"
 * @param Goal_Position destination to generate global plan to. if "replan" option selection used can choose multiple goal positions. goal positions are selected from Rviz using "2D Nav Goal" button.
 * @return global , reference path as list of waypoints. data type is "autoware_msgs::LaneArray" , and the topic name is "lane_waypoints_array"
 */

#include <ros/ros.h>
#include "op_global_planner_core.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "op_global_planner");
	GlobalPlanningNS::GlobalPlanner global_planner;
	global_planner.MainLoop();
	return 0;
}
