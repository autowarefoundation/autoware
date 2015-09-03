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

#include <fstream>

#include <ros/ros.h>
#include <ros/console.h>

#include <waypoint_follower/lane.h>
#include <tablet_socket/route_cmd.h>

#include <geo_pos_conv.hh>
#include <vmap_utility.hpp>

static ros::Publisher pub_waypoint;

static VectorMap vmap_all;
static VectorMap vmap_left_lane;

static int sub_vmap_queue_size;
static int sub_route_queue_size;
static int pub_waypoint_queue_size;
static bool pub_waypoint_latch;

static double velocity;		// km/h
static std::string output_file;

static void cache_left_lane()
{
	if (vmap_all.lanes.empty() || vmap_all.nodes.empty() || vmap_all.points.empty())
		return;

	for (const map_file::Lane& l : vmap_all.lanes) {
		if (l.lno != 1)	// leftmost lane
			continue;
		vmap_left_lane.lanes.push_back(l);
		for (const map_file::Node& n : vmap_all.nodes) {
			if (n.nid != l.bnid && n.nid != l.fnid)
				continue;
			vmap_left_lane.nodes.push_back(n);
			for (const map_file::PointClass& p : vmap_all.points) {
				if (p.pid != n.pid)
					continue;
				vmap_left_lane.points.push_back(p);
			}
		}
	}
}

static bool is_cached_vmap()
{
	return (!vmap_all.lanes.empty() && !vmap_all.nodes.empty() && !vmap_all.points.empty());
}

static void cache_lane(const map_file::LaneArray& msg)
{
	vmap_all.lanes = msg.lanes;
	cache_left_lane();
}

static void cache_node(const map_file::NodeArray& msg)
{
	vmap_all.nodes = msg.nodes;
	cache_left_lane();
}

static void cache_point(const map_file::PointClassArray& msg)
{
	vmap_all.points = msg.point_classes;
	cache_left_lane();
}

static void write_lane_waypoint(const geometry_msgs::Point& point, bool first)
{
	if (first) {
		std::ofstream ofs(output_file.c_str());
		ofs << std::fixed << point.x << ","
		    << std::fixed << point.y << ","
		    << std::fixed << point.z << std::endl;
	} else {
		std::ofstream ofs(output_file.c_str(), std::ios_base::app);
		ofs << std::fixed << point.x << ","
		    << std::fixed << point.y << ","
		    << std::fixed << point.z << ","
		    << std::fixed << velocity << std::endl;
	}
}

static void create_lane_waypoint(const tablet_socket::route_cmd& msg)
{
	if (!is_cached_vmap()) {
		ROS_WARN("not cached vmap");
		return;
	}

	geo_pos_conv geo;
	geo.set_plane(7);
	geo.llh_to_xyz(msg.point.front().lat, msg.point.front().lon, 0);
	map_file::PointClass start_point = vmap_find_nearest_point(vmap_left_lane, geo.x(), geo.y());
	geo.llh_to_xyz(msg.point.back().lat, msg.point.back().lon, 0);
	map_file::PointClass end_point = vmap_find_nearest_point(vmap_left_lane, geo.x(), geo.y());

	map_file::PointClass point = start_point;
	map_file::Lane lane = vmap_find_lane(vmap_all, point);
	if (lane.lnid < 0) {
		ROS_ERROR("no start lane");
		return;
	}

	std_msgs::Header header;
	header.stamp = ros::Time::now();
	header.frame_id = "/map";
	waypoint_follower::lane waypoints;
	waypoints.header = header;
	waypoints.increment = 1;
	waypoint_follower::waypoint waypoint;
	waypoint.pose.header = header;
	waypoint.twist.header = header;
	waypoint.pose.pose.orientation.w = 1;

	bool finish = false;
	for (size_t i = 0; i < std::numeric_limits<std::size_t>::max(); ++i) {
		// msg's X-Y axis is reversed
		waypoint.pose.pose.position.x = point.ly;
		waypoint.pose.pose.position.y = point.bx;
		waypoint.pose.pose.position.z = point.h;
		waypoint.twist.twist.linear.x = velocity / 3.6;	// to m/s
		waypoints.waypoints.push_back(waypoint);
		write_lane_waypoint(waypoint.pose.pose.position, (i == 0));

		if (finish)
			break;

		point = vmap_find_end_point(vmap_all, lane);
		if (point.pid < 0) {
			ROS_ERROR("no end point");
			return;
		}

		if (point.bx == end_point.bx && point.ly == end_point.ly) {
			finish = true;
			continue;
		}

		lane = vmap_find_next_lane(vmap_all, lane);
		if (lane.lnid < 0) {
			ROS_ERROR("no next lane");
			return;
		}

		point = vmap_find_start_point(vmap_all, lane);
		if (point.pid < 0) {
			ROS_ERROR("no start point");
			return;
		}
	}
	if (!finish)
		ROS_ERROR("miss finish");

	pub_waypoint.publish(waypoints);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lane_navi");

	ros::NodeHandle n;
	n.param<int>("/lane_navi/sub_vmap_queue_size", sub_vmap_queue_size, 1);
	n.param<int>("/lane_navi/sub_route_queue_size", sub_route_queue_size, 1);
	n.param<int>("/lane_navi/pub_waypoint_queue_size", pub_waypoint_queue_size, 1);
	n.param<bool>("/lane_navi/pub_waypoint_latch", pub_waypoint_latch, true);
	n.param<double>("/lane_navi/velocity", velocity, 40);
	n.param<std::string>("/lane_navi/output_file", output_file, "/tmp/lane_waypoint.csv");

	ros::Subscriber sub_lane = n.subscribe("/vector_map_info/lane", sub_vmap_queue_size, cache_lane);
	ros::Subscriber sub_node = n.subscribe("/vector_map_info/node", sub_vmap_queue_size, cache_node);
	ros::Subscriber sub_point = n.subscribe("/vector_map_info/point_class", sub_vmap_queue_size, cache_point);
	ros::Subscriber sub_route = n.subscribe("/route_cmd", sub_route_queue_size, create_lane_waypoint);

	pub_waypoint = n.advertise<waypoint_follower::lane>("/lane_waypoint", pub_waypoint_queue_size,
							    pub_waypoint_latch);

	ros::spin();

	return 0;
}
