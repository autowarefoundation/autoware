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

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <lane_follower/lane.h>
#include <runtime_manager/traffic_light.h>

static constexpr uint32_t SUBSCRIBE_QUEUE_SIZE = 1000;

static constexpr uint32_t ADVERTISE_QUEUE_SIZE = 1000;
static constexpr bool ADVERTISE_LATCH = true;

static constexpr int32_t TRAFFIC_LIGHT_RED     = 0;
static constexpr int32_t TRAFFIC_LIGHT_GREEN   = 1;
static constexpr int32_t TRAFFIC_LIGHT_UNKNOWN = 2;

static ros::Publisher pub_ruled;
static ros::Publisher pub_velocity;

static lane_follower::lane current_red_lane;
static lane_follower::lane current_green_lane;

static void red_waypoint_callback(const lane_follower::lane& msg)
{
	current_red_lane = msg;
}

static void green_waypoint_callback(const lane_follower::lane& msg)
{
	current_green_lane = msg;
}

static void traffic_light_callback(const runtime_manager::traffic_light& msg)
{
	const  lane_follower::lane *current;
	static lane_follower::lane prev_path = current_red_lane;

	switch (msg.traffic_light) {
	case TRAFFIC_LIGHT_RED:
		current = &current_red_lane;
		break;
	case TRAFFIC_LIGHT_GREEN:
		current = &current_green_lane;
		break;
	case TRAFFIC_LIGHT_UNKNOWN:
        current = &prev_path;     // if traffic light state is unknown, keep previous state
		break;
	default:
		ROS_ERROR("unknown traffic_light");
		return;
	}

	if (current->waypoints.empty()) {
		ROS_ERROR("empty waypoints");
		return;
	}

	visualization_msgs::MarkerArray velocities;
	visualization_msgs::Marker velocity;
	velocity.header.stamp = ros::Time::now();
	velocity.header.frame_id = "/map";
	velocity.ns = "waypoint_velocity";
	velocity.action = visualization_msgs::Marker::ADD;
	velocity.lifetime = ros::Duration();
	velocity.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	velocity.scale.z = 0.4;
	velocity.color.r = 1;
	velocity.color.a = 1;

	int i = 0;
	for (const lane_follower::waypoint& waypoint : current->waypoints) {
		velocity.id = i;
		velocity.pose.position = waypoint.pose.pose.position;
		velocity.pose.position.z += 0.2; // more visible

		std::ostringstream ostr;
		ostr << std::fixed << std::setprecision(0)
		     << (waypoint.twist.twist.linear.x * 3.6) << " km/h";
		velocity.text = ostr.str();

		velocities.markers.push_back(velocity);
		++i;
	}

	pub_ruled.publish(*current);
	pub_velocity.publish(velocities);

    prev_path = *current;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lane_stop");

	ros::NodeHandle n;

	ros::Subscriber sub_red = n.subscribe("red_waypoint",
					      SUBSCRIBE_QUEUE_SIZE,
					      red_waypoint_callback);
	ros::Subscriber sub_green = n.subscribe("green_waypoint",
						SUBSCRIBE_QUEUE_SIZE,
						green_waypoint_callback);
	ros::Subscriber sub_light = n.subscribe("traffic_light",
						SUBSCRIBE_QUEUE_SIZE,
						traffic_light_callback);

	pub_ruled = n.advertise<lane_follower::lane>(
		"ruled_waypoint",
		ADVERTISE_QUEUE_SIZE,
		ADVERTISE_LATCH);
	pub_velocity = n.advertise<visualization_msgs::MarkerArray>(
		"waypoint_velocity",
		ADVERTISE_QUEUE_SIZE,
		ADVERTISE_LATCH);

	ros::spin();

	return 0;
}
