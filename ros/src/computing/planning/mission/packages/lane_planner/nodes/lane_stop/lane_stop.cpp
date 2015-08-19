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

#include <ros/ros.h>
#include <ros/console.h>

#include <waypoint_follower/lane.h>
#include <runtime_manager/traffic_light.h>

static constexpr int TRAFFIC_LIGHT_RED = 0;
static constexpr int TRAFFIC_LIGHT_GREEN = 1;
static constexpr int TRAFFIC_LIGHT_UNKNOWN = 2;

static ros::Publisher pub_traffic;

static int sub_waypoint_queue_size;
static int sub_light_queue_size;
static int pub_waypoint_queue_size;
static bool pub_waypoint_latch;

static waypoint_follower::lane current_red_lane;
static waypoint_follower::lane current_green_lane;

static const waypoint_follower::lane *previous_lane = &current_red_lane;

static void cache_red_lane(const waypoint_follower::lane& msg)
{
	current_red_lane = msg;
}

static void cache_green_lane(const waypoint_follower::lane& msg)
{
	current_green_lane = msg;
}

static void select_current_lane(const runtime_manager::traffic_light& msg)
{
	const waypoint_follower::lane *current;

	switch (msg.traffic_light) {
	case TRAFFIC_LIGHT_RED:
		current = &current_red_lane;
		break;
	case TRAFFIC_LIGHT_GREEN:
		current = &current_green_lane;
		break;
	case TRAFFIC_LIGHT_UNKNOWN:
		current = previous_lane; // if traffic light state is unknown, keep previous state
		break;
	default:
		ROS_ERROR("undefined traffic_light");
		return;
	}

	if (current->waypoints.empty()) {
		ROS_ERROR("empty waypoints");
		return;
	}

	pub_traffic.publish(*current);

	previous_lane = current;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lane_stop");

	ros::NodeHandle n;
	n.param<int>("/lane_stop/sub_waypoint_queue_size", sub_waypoint_queue_size, 1);
	n.param<int>("/lane_stop/sub_light_queue_size", sub_light_queue_size, 1);
	n.param<int>("/lane_stop/pub_waypoint_queue_size", pub_waypoint_queue_size, 1);
	n.param<bool>("/lane_stop/pub_waypoint_latch", pub_waypoint_latch, true);

	ros::Subscriber sub_red = n.subscribe("/red_waypoint", sub_waypoint_queue_size, cache_red_lane);
	ros::Subscriber sub_green = n.subscribe("/green_waypoint", sub_waypoint_queue_size, cache_green_lane);
	ros::Subscriber sub_light = n.subscribe("/traffic_light", sub_light_queue_size, select_current_lane);

	pub_traffic = n.advertise<waypoint_follower::lane>("/traffic_waypoint", pub_waypoint_queue_size,
							   pub_waypoint_latch);

	ros::spin();

	return 0;
}
