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

#include <ros/console.h>

#include "autoware_msgs/ConfigLaneStop.h"
#include "autoware_msgs/traffic_light.h"
#include "autoware_msgs/LaneArray.h"

#include <lane_planner/lane_planner_vmap.hpp>

namespace {

bool config_manual_detection = true;

ros::Publisher traffic_pub;

autoware_msgs::LaneArray current_red_lane;
autoware_msgs::LaneArray current_green_lane;

const autoware_msgs::LaneArray *previous_lane = &current_red_lane;

void select_current_lane(const autoware_msgs::traffic_light& msg)
{
	const autoware_msgs::LaneArray *current;
	switch (msg.traffic_light) {
	case lane_planner::vmap::TRAFFIC_LIGHT_RED:
		current = &current_red_lane;
		break;
	case lane_planner::vmap::TRAFFIC_LIGHT_GREEN:
		current = &current_green_lane;
		break;
	case lane_planner::vmap::TRAFFIC_LIGHT_UNKNOWN:
		current = previous_lane; // if traffic light state is unknown, keep previous state
		break;
	default:
		ROS_ERROR_STREAM("undefined traffic light");
		return;
	}

	if (current->lanes.empty()) {
		ROS_ERROR_STREAM("empty lanes");
		return;
	}

	traffic_pub.publish(*current);

	previous_lane = current;
}

void receive_auto_detection(const autoware_msgs::traffic_light& msg)
{
	if (!config_manual_detection)
		select_current_lane(msg);
}

void receive_manual_detection(const autoware_msgs::traffic_light& msg)
{
	if (config_manual_detection)
		select_current_lane(msg);
}

void cache_red_lane(const autoware_msgs::LaneArray& msg)
{
	current_red_lane = msg;
}

void cache_green_lane(const autoware_msgs::LaneArray& msg)
{
	current_green_lane = msg;
}

void config_parameter(const autoware_msgs::ConfigLaneStop& msg)
{
	config_manual_detection = msg.manual_detection;
}

} // namespace

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lane_stop");

	ros::NodeHandle n;

	int sub_light_queue_size;
	n.param<int>("/lane_stop/sub_light_queue_size", sub_light_queue_size, 1);
	int sub_waypoint_queue_size;
	n.param<int>("/lane_stop/sub_waypoint_queue_size", sub_waypoint_queue_size, 1);
	int sub_config_queue_size;
	n.param<int>("/lane_rule/sub_config_queue_size", sub_config_queue_size, 1);
	int pub_waypoint_queue_size;
	n.param<int>("/lane_stop/pub_waypoint_queue_size", pub_waypoint_queue_size, 1);
	bool pub_waypoint_latch;
	n.param<bool>("/lane_stop/pub_waypoint_latch", pub_waypoint_latch, true);

	traffic_pub = n.advertise<autoware_msgs::LaneArray>("/traffic_waypoints_array", pub_waypoint_queue_size,
								pub_waypoint_latch);

	ros::Subscriber light_sub = n.subscribe("/light_color", sub_light_queue_size, receive_auto_detection);
	ros::Subscriber light_managed_sub = n.subscribe("/light_color_managed", sub_light_queue_size,
							receive_manual_detection);
	ros::Subscriber red_sub = n.subscribe("/red_waypoints_array", sub_waypoint_queue_size, cache_red_lane);
	ros::Subscriber green_sub = n.subscribe("/green_waypoints_array", sub_waypoint_queue_size, cache_green_lane);
	ros::Subscriber config_sub = n.subscribe("/config/lane_stop", sub_config_queue_size, config_parameter);

	ros::spin();

	return EXIT_SUCCESS;
}
