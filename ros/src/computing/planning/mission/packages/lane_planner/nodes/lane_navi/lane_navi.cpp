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

static void create_lane_waypoint(const tablet_socket::route_cmd& msg);

static bool cached_route = false;

static ros::Publisher pub_waypoint;

static VectorMap vmap_all;
static VectorMap vmap_lane;
static VectorMap vmap_left_lane;

static tablet_socket::route_cmd current_route;

static int sub_vmap_queue_size;
static int sub_route_queue_size;
static int pub_waypoint_queue_size;
static bool pub_waypoint_latch;

static double velocity;		// km/h
static std::string output_file;
static double search_radius;	// m
static int waypoint_max;

static void cache_vmap_lane()
{
	if (vmap_all.lanes.empty() || vmap_all.nodes.empty() || vmap_all.points.empty())
		return;

	VectorMap vmap;
	for (const map_file::Lane& l : vmap_all.lanes) {
		vmap.lanes.push_back(l);
		for (const map_file::Node& n : vmap_all.nodes) {
			if (n.nid != l.bnid && n.nid != l.fnid)
				continue;
			vmap.nodes.push_back(n);
			for (const map_file::PointClass& p : vmap_all.points) {
				if (p.pid != n.pid)
					continue;
				vmap.points.push_back(p);
			}
		}
	}
	vmap_lane = vmap;
}

static void cache_vmap_left_lane()
{
	if (vmap_all.lanes.empty() || vmap_all.nodes.empty() || vmap_all.points.empty())
		return;

	VectorMap vmap;
	for (const map_file::Lane& l : vmap_all.lanes) {
		if (l.lno != 1)	// leftmost lane
			continue;
		vmap.lanes.push_back(l);
		for (const map_file::Node& n : vmap_all.nodes) {
			if (n.nid != l.bnid && n.nid != l.fnid)
				continue;
			vmap.nodes.push_back(n);
			for (const map_file::PointClass& p : vmap_all.points) {
				if (p.pid != n.pid)
					continue;
				vmap.points.push_back(p);
			}
		}
	}
	vmap_left_lane = vmap;
}

static VectorMap cache_vmap_coarse(const tablet_socket::route_cmd& msg)
{
	geo_pos_conv geo;
	geo.set_plane(7);

	VectorMap vmap;
	for (const tablet_socket::Waypoint& w : msg.point) {
		geo.llh_to_xyz(w.lat, w.lon, 0);

		map_file::PointClass p;
		p.bx = geo.x();
		p.ly = geo.y();
		vmap.points.push_back(p);
	}

	return vmap;
}

static bool is_cached_vmap()
{
	return (!vmap_all.lanes.empty() && !vmap_all.nodes.empty() && !vmap_all.points.empty());
}

static bool is_cached_route()
{
	return cached_route;
}

static void cache_route(const tablet_socket::route_cmd& msg)
{
	current_route = msg;
}

static void cache_lane(const map_file::LaneArray& msg)
{
	vmap_all.lanes = msg.lanes;
	cache_vmap_lane();
	cache_vmap_left_lane();
	if (is_cached_route() && is_cached_vmap()) {
		create_lane_waypoint(current_route);
		cached_route = false;
	}
}

static void cache_node(const map_file::NodeArray& msg)
{
	vmap_all.nodes = msg.nodes;
	cache_vmap_lane();
	cache_vmap_left_lane();
	if (is_cached_route() && is_cached_vmap()) {
		create_lane_waypoint(current_route);
		cached_route = false;
	}
}

static void cache_point(const map_file::PointClassArray& msg)
{
	vmap_all.points = msg.point_classes;
	cache_vmap_lane();
	cache_vmap_left_lane();
	if (is_cached_route() && is_cached_vmap()) {
		create_lane_waypoint(current_route);
		cached_route = false;
	}
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
		cache_route(msg);
		cached_route = true;
		return;
	}

	VectorMap vmap_coarse = cache_vmap_coarse(msg);
	if (vmap_coarse.points.size() < 2) {
		ROS_ERROR("lack of waypoint");
		return;
	}

	map_file::PointClass start_point = vmap_find_start_nearest_point(vmap_left_lane,
									 vmap_coarse.points[0],
									 vmap_coarse.points[1],
									 search_radius);
	if (start_point.pid < 0) {
		ROS_ERROR("no start point");
		return;
	}
	map_file::PointClass end_point = vmap_find_end_nearest_point(vmap_left_lane,
								     vmap_coarse.points[msg.point.size() - 1],
								     vmap_coarse.points[msg.point.size() - 2],
								     search_radius);
	if (end_point.pid < 0) {
		ROS_ERROR("no end point");
		return;
	}
	map_file::PointClass point = start_point;
	map_file::Lane lane = vmap_find_lane(vmap_lane, point);
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
	for (int i = 0; i < waypoint_max; ++i) {
		// msg's X-Y axis is reversed
		waypoint.pose.pose.position.x = point.ly;
		waypoint.pose.pose.position.y = point.bx;
		waypoint.pose.pose.position.z = point.h;
		waypoint.twist.twist.linear.x = velocity / 3.6;	// to m/s
		waypoints.waypoints.push_back(waypoint);
		write_lane_waypoint(waypoint.pose.pose.position, (i == 0));

		if (finish)
			break;

		point = vmap_find_end_point(vmap_lane, lane);
		if (point.pid < 0) {
			ROS_ERROR("no end point");
			return;
		}

		if (point.bx == end_point.bx && point.ly == end_point.ly) {
			finish = true;
			continue;
		}

		if (lane.jct >= 1 && lane.jct <= 2) { // bifurcation
			map_file::PointClass p1 = vmap_find_end_point(vmap_lane, lane);
			if (p1.pid < 0) {
				ROS_ERROR("no end point");
				return;
			}

			p1 = vmap_find_nearest_point(vmap_coarse, p1);
			if (p1.pid < 0) {
				ROS_ERROR("no nearest point");
				return;
			}

			map_file::PointClass p2;
			double distance = -1;
			for (const map_file::PointClass& p : vmap_coarse.points) {
				if (distance == -1) {
					if (p.bx == p1.bx && p.ly == p1.ly)
						distance = 0;
					continue;
				}
				p2 = p;
				distance = hypot(p1.bx - p2.bx, p1.ly - p2.ly);
				if (distance > VMAP_JUNCTION_DISTANCE_MAX)
					break;
			}
			if (distance <= 0) {
				ROS_ERROR("no next nearest point");
				return;
			}

			lane = vmap_find_junction_lane(vmap_lane, lane, vmap_compute_direction_angle(p1, p2));
		} else
			lane = vmap_find_next_lane(vmap_lane, lane);
		if (lane.lnid < 0) {
			ROS_ERROR("no next lane");
			return;
		}

		point = vmap_find_start_point(vmap_lane, lane);
		if (point.pid < 0) {
			ROS_ERROR("no start point");
			return;
		}
	}
	if (!finish) {
		ROS_ERROR("miss finish");
		return;
	}

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
	n.param<double>("/lane_navi/search_radius", search_radius, 10);
	n.param<int>("/lane_navi/waypoint_max", waypoint_max, 10000);

	ros::Subscriber sub_lane = n.subscribe("/vector_map_info/lane", sub_vmap_queue_size, cache_lane);
	ros::Subscriber sub_node = n.subscribe("/vector_map_info/node", sub_vmap_queue_size, cache_node);
	ros::Subscriber sub_point = n.subscribe("/vector_map_info/point_class", sub_vmap_queue_size, cache_point);
	ros::Subscriber sub_route = n.subscribe("/route_cmd", sub_route_queue_size, create_lane_waypoint);

	pub_waypoint = n.advertise<waypoint_follower::lane>("/lane_waypoint", pub_waypoint_queue_size,
							    pub_waypoint_latch);

	ros::spin();

	return 0;
}
