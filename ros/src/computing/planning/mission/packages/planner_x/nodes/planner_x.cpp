/*
 *  Copyright (c) 2015, Nagoya University
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
#include "planner_x_core.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "planner_x");
	ros::NodeHandle nh;

	PlannerX *planner_x = new PlannerX(&nh);

  // define subscribers.
	ros::Subscriber sub_current_pose = nh.subscribe("/current_pose", 10, &PlannerX::callbackFromCurrentPose, planner_x);
	ros::Subscriber sub_traffic_light = nh.subscribe("/light_color", 10, &PlannerX::callbackFromLightColor, planner_x);
	ros::Subscriber sub_obj_pose = nh.subscribe("/obj_car/obj_label", 10, &PlannerX::callbackFromObjCar, planner_x);

	ros::Subscriber point_sub = nh.subscribe("/vector_map_info/point_class", 1, &PlannerX::callbackGetVMPoints, planner_x);
	ros::Subscriber lane_sub = nh.subscribe("/vector_map_info/lane", 1, &PlannerX::callbackGetVMLanes, planner_x);
	ros::Subscriber node_sub = nh.subscribe("/vector_map_info/node", 1, &PlannerX::callbackGetVMNodes, planner_x);
	ros::Subscriber stopline_sub = nh.subscribe("/vector_map_info/stop_line", 1, &PlannerX::callbackGetVMStopLines, planner_x);
	ros::Subscriber dtlane_sub = nh.subscribe("/vector_map_info/dtlane", 1, &PlannerX::callbackGetVMCenterLines, planner_x);

	ros::Subscriber initialpose_subscriber = nh.subscribe("initialpose", 10, &PlannerX::callbackSimuInitPose, planner_x);
	ros::Subscriber goalpose_subscriber = nh.subscribe("move_base_simple/goal", 10, &PlannerX::callbackSimuGoalPose, planner_x);



  ros::spin();

  return 0;
}
