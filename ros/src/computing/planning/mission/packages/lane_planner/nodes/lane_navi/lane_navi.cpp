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

#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <tablet_socket/route_cmd.h>

#include <geo_pos_conv.hh>

#include <vmap_utility.hpp>

// #define PUBLISH_TRAJECTORY

static constexpr uint32_t SUBSCRIBE_QUEUE_SIZE = 1000;

static constexpr uint32_t ADVERTISE_QUEUE_SIZE = 1000;
static constexpr bool ADVERTISE_LATCH = true;

static ros::Publisher pub_waypoint;
static ros::Publisher pub_mark;
static ros::Publisher _lane_mark_pub;
#ifdef PUBLISH_TRAJECTORY
static ros::Publisher pub_trajectory;
#endif /* PUBLISH_TRAJECTORY */

static std::vector<map_file::Lane> lanes;
static std::vector<map_file::Node> nodes;
static std::vector<map_file::PointClass> points;

static std::vector<map_file::PointClass> left_lane_points;

static void update_left_lane()
{
	if (lanes.empty() || nodes.empty() || points.empty())
		return;

	for (const map_file::Lane& lane : lanes) {
		if (lane.lno != 1) // leftmost lane
			continue;
		for (const map_file::Node& node : nodes) {
			if (node.nid != lane.bnid &&
			    node.nid != lane.fnid)
				continue;
			for (const map_file::PointClass& point : points) {
				if (point.pid != node.pid)
					continue;
				left_lane_points.push_back(point);
			}
		}
	}
}

static void lane_callback(const map_file::LaneArray& msg)
{
	lanes = msg.lanes;
	update_left_lane();
}

static void node_callback(const map_file::NodeArray& msg)
{
	nodes = msg.nodes;
	update_left_lane();
}

static void point_class_callback(const map_file::PointClassArray& msg)
{
	points = msg.point_classes;
	update_left_lane();
}

static void route_cmd_callback(const tablet_socket::route_cmd& msg)
{
	geo_pos_conv geo;
	geo.set_plane(7);

	geo.llh_to_xyz(msg.point.front().lat, msg.point.front().lon, 0);
	map_file::PointClass start_point =
		search_nearest(left_lane_points, geo.x(), geo.y());

	geo.llh_to_xyz(msg.point.back().lat, msg.point.back().lon, 0);
	map_file::PointClass end_point =
		search_nearest(left_lane_points, geo.x(), geo.y());

	int lane_index = to_lane_index(start_point, nodes, lanes);
	if (lane_index < 0) {
		ROS_ERROR("start lane is not found");
		return;
	}
	map_file::Lane lane = lanes[lane_index];

	int point_index = to_beginning_point_index(lane, nodes, points);
	if (point_index < 0) {
		ROS_ERROR("start beginning point is not found");
		return;
	}
	map_file::PointClass point = points[point_index];

	std_msgs::Header header;
	header.stamp = ros::Time::now();
	header.frame_id = "/map";

	nav_msgs::Path waypoint;
	waypoint.header = header;

	visualization_msgs::Marker mark;
	mark.header = header;
	mark.ns = "waypoint_mark";
	mark.id = 0;
	mark.action = visualization_msgs::Marker::ADD;
	mark.lifetime = ros::Duration();
	mark.type = visualization_msgs::Marker::POINTS;
	mark.scale.x = 0.1;
	mark.scale.y = 0.1;
	mark.color.r = 1;
	mark.color.a = 1;

    static visualization_msgs::Marker lane_waypoint_marker;
    lane_waypoint_marker.header.frame_id = "/map";
    lane_waypoint_marker.header.stamp = ros::Time();
    lane_waypoint_marker.ns = "lane_waypoint_marker";
    lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
    lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
    lane_waypoint_marker.scale.x = 0.2;
    lane_waypoint_marker.pose.orientation.w = 1.0;
    lane_waypoint_marker.color.b = 1.0;
    lane_waypoint_marker.color.g = 0.5;
    lane_waypoint_marker.color.a = 1.0;
    lane_waypoint_marker.frame_locked = true;

	geometry_msgs::PoseStamped posestamped;
	posestamped.header = header;
	posestamped.pose.orientation.w = 1;

	while (1) {
		posestamped.pose.position.x = point.ly;
		posestamped.pose.position.y = point.bx;
		posestamped.pose.position.z = point.h;

		waypoint.poses.push_back(posestamped);
		mark.points.push_back(posestamped.pose.position);
		lane_waypoint_marker.points.push_back(posestamped.pose.position);

		point_index = to_finishing_point_index(lane, nodes, points);
		if (point_index < 0) {
			ROS_ERROR("finishing point is not found");
			return;
		}
		point = points[point_index];

		if (point.bx == end_point.bx &&
		    point.ly == end_point.ly) {
			posestamped.pose.position.x = point.ly;
			posestamped.pose.position.y = point.bx;
			posestamped.pose.position.z = point.h;

			waypoint.poses.push_back(posestamped);
			mark.points.push_back(posestamped.pose.position);
			lane_waypoint_marker.points.push_back(posestamped.pose.position);

			break;
		}

		lane_index = to_next_lane_index(lane, lanes);
		if (lane_index < 0) {
			ROS_ERROR("next lane is not found");
			return;
		}
		lane = lanes[lane_index];

		point_index = to_beginning_point_index(lane, nodes, points);
		if (point_index < 0) {
			ROS_ERROR("beginning point is not found");
			return;
		}
		point = points[point_index];
	}

	pub_waypoint.publish(waypoint);
	pub_mark.publish(mark);
    _lane_mark_pub.publish(lane_waypoint_marker);

#ifdef PUBLISH_TRAJECTORY
	visualization_msgs::Marker trajectory;
	trajectory.header = header;
	trajectory.ns = "_trajectory";
	trajectory.id = 0;
	trajectory.action = visualization_msgs::Marker::ADD;
	trajectory.lifetime = ros::Duration();
	trajectory.type = visualization_msgs::Marker::SPHERE;
	trajectory.pose.orientation.w = 1;
	trajectory.scale.x = 0.2;
	trajectory.scale.y = 0.2;
	trajectory.scale.z = 0.2;
	trajectory.color.r = 1;
	trajectory.color.a = 1;

	ros::Rate rate(1);
	for (const geometry_msgs::Point& position : mark.points) {
		trajectory.pose.position = position;
		pub_trajectory.publish(trajectory);
		rate.sleep();
	}
#endif /* PUBLISH_TRAJECTORY */
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lane_navi");

	ros::NodeHandle n;

	ros::Subscriber sub_lane = n.subscribe("vector_map_info/lane",
					       SUBSCRIBE_QUEUE_SIZE,
					       lane_callback);
	ros::Subscriber sub_node = n.subscribe("vector_map_info/node",
					       SUBSCRIBE_QUEUE_SIZE,
					       node_callback);
	ros::Subscriber sub_point_class = n.subscribe(
		"vector_map_info/point_class",
		SUBSCRIBE_QUEUE_SIZE,
		point_class_callback);

	ros::Rate rate(1);
	while (lanes.empty() || nodes.empty() || points.empty()) {
		ros::spinOnce();
		rate.sleep();
	}

	ros::Subscriber sub = n.subscribe("route_cmd",
					  SUBSCRIBE_QUEUE_SIZE,
					  route_cmd_callback);

	pub_waypoint = n.advertise<nav_msgs::Path>(
		"lane_waypoint",
		ADVERTISE_QUEUE_SIZE,
		ADVERTISE_LATCH);
	pub_mark = n.advertise<visualization_msgs::Marker>(
		"waypoint_mark",
		ADVERTISE_QUEUE_SIZE,
		ADVERTISE_LATCH);

	_lane_mark_pub = n.advertise<visualization_msgs::Marker>(
	     "lane_waypoint_mark",
	     ADVERTISE_QUEUE_SIZE,
	     ADVERTISE_LATCH);

#ifdef PUBLISH_TRAJECTORY
	pub_trajectory = n.advertise<visualization_msgs::Marker>(
		"_trajectory",
		ADVERTISE_QUEUE_SIZE,
		ADVERTISE_LATCH);
#endif /* PUBLISH_TRAJECTORY */

	ros::spin();

	return 0;
}
