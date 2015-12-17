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

#include <sstream>

#include <ros/console.h>
#include <tf/transform_datatypes.h>

#include <map_file/PointClassArray.h>
#include <map_file/LaneArray.h>
#include <map_file/NodeArray.h>
#include <waypoint_follower/LaneArray.h>

#include <lane_planner/vmap.hpp>

namespace {

int waypoint_max;
double search_radius; // meter
double velocity; // km/h
std::string frame_id;
std::string output_file;

ros::Publisher waypoint_pub;

lane_planner::vmap::VectorMap all_vmap;
lane_planner::vmap::VectorMap lane_vmap;
tablet_socket::route_cmd cached_route;

std::vector<std::string> split(const std::string& str, char delim)
{
	std::stringstream ss(str);
	std::string s;
	std::vector<std::string> vec;
	while (std::getline(ss, s, delim))
		vec.push_back(s);

	if (!str.empty() && str.back() == delim)
		vec.push_back(std::string());

	return vec;
}

std::string join(const std::vector<std::string>& vec, char delim)
{
	std::string str;
	for (size_t i = 0; i < vec.size(); ++i) {
		str += vec[i];
		if (i != (vec.size() - 1))
			str += delim;
	}

	return str;
}

int count_lane(const lane_planner::vmap::VectorMap& vmap)
{
	int lcnt = -1;

	for (const map_file::Lane& l : vmap.lanes) {
		if (l.lcnt > lcnt)
			lcnt = l.lcnt;
	}

	return lcnt;
}

void create_waypoint(const tablet_socket::route_cmd& msg)
{
	std_msgs::Header header;
	header.stamp = ros::Time::now();
	header.frame_id = frame_id;

	if (all_vmap.points.empty() || all_vmap.lanes.empty() || all_vmap.nodes.empty()) {
		cached_route.header = header;
		cached_route.point = msg.point;
		return;
	}

	lane_planner::vmap::VectorMap coarse_vmap = lane_planner::vmap::create_coarse_vmap_from_route(msg);
	if (coarse_vmap.points.size() < 2)
		return;

	std::vector<lane_planner::vmap::VectorMap> fine_vmaps;
	lane_planner::vmap::VectorMap fine_mostleft_vmap =
		lane_planner::vmap::create_fine_vmap(lane_vmap, lane_planner::vmap::LNO_MOSTLEFT, coarse_vmap,
						     search_radius, waypoint_max);
	if (fine_mostleft_vmap.points.size() < 2)
		return;
	fine_vmaps.push_back(fine_mostleft_vmap);

	int lcnt = count_lane(fine_mostleft_vmap);
	for (int i = lane_planner::vmap::LNO_MOSTLEFT + 1; i <= lcnt; ++i) {
		lane_planner::vmap::VectorMap v =
			lane_planner::vmap::create_fine_vmap(lane_vmap, i, coarse_vmap, search_radius, waypoint_max);
		if (v.points.size() < 2)
			continue;
		fine_vmaps.push_back(v);
	}

	waypoint_follower::LaneArray lane_waypoint;
	for (const lane_planner::vmap::VectorMap& v : fine_vmaps) {
		waypoint_follower::lane l;
		l.header = header;
		l.increment = 1;

		size_t last_index = v.points.size() - 1;
		for (size_t i = 0; i < v.points.size(); ++i) {
			double yaw;
			if (i == last_index) {
				geometry_msgs::Point p1 =
					lane_planner::vmap::create_geometry_msgs_point(v.points[i]);
				geometry_msgs::Point p2 =
					lane_planner::vmap::create_geometry_msgs_point(v.points[i - 1]);
				yaw = atan2(p2.y - p1.y, p2.x - p1.x);
				yaw -= M_PI;
			} else {
				geometry_msgs::Point p1 =
					lane_planner::vmap::create_geometry_msgs_point(v.points[i]);
				geometry_msgs::Point p2 =
					lane_planner::vmap::create_geometry_msgs_point(v.points[i + 1]);
				yaw = atan2(p2.y - p1.y, p2.x - p1.x);
			}

			waypoint_follower::waypoint w;
			w.pose.header = header;
			w.pose.pose.position = lane_planner::vmap::create_geometry_msgs_point(v.points[i]);
			w.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
			w.twist.header = header;
			w.twist.twist.linear.x = velocity / 3.6; // to m/s
			l.waypoints.push_back(w);
		}
		lane_waypoint.lanes.push_back(l);
	}
	waypoint_pub.publish(lane_waypoint);

	for (size_t i = 0; i < fine_vmaps.size(); ++i) {
		std::stringstream ss;
		ss << "_" << i;

		std::vector<std::string> v1 = split(output_file, '/');
		std::vector<std::string> v2 = split(v1.back(), '.');
		v2[0] = v2.front() + ss.str();
		v1[v1.size() - 1] = join(v2, '.');
		std::string path = join(v1, '/');

		lane_planner::vmap::write_waypoints(fine_vmaps[i].points, velocity, path);
	}
}

void update_values()
{
	if (all_vmap.points.empty() || all_vmap.lanes.empty() || all_vmap.nodes.empty())
		return;

	lane_vmap = lane_planner::vmap::create_lane_vmap(all_vmap, lane_planner::vmap::LNO_ALL);

	if (!cached_route.point.empty()) {
		create_waypoint(cached_route);
		cached_route.point.clear();
		cached_route.point.shrink_to_fit();
	}
}

void cache_point(const map_file::PointClassArray& msg)
{
	all_vmap.points = msg.point_classes;
	update_values();
}

void cache_lane(const map_file::LaneArray& msg)
{
	all_vmap.lanes = msg.lanes;
	update_values();
}

void cache_node(const map_file::NodeArray& msg)
{
	all_vmap.nodes = msg.nodes;
	update_values();
}

} // namespace

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lane_navi");

	ros::NodeHandle n;

	int sub_vmap_queue_size;
	n.param<int>("/lane_navi/sub_vmap_queue_size", sub_vmap_queue_size, 1);
	int sub_route_queue_size;
	n.param<int>("/lane_navi/sub_route_queue_size", sub_route_queue_size, 1);
	int pub_waypoint_queue_size;
	n.param<int>("/lane_navi/pub_waypoint_queue_size", pub_waypoint_queue_size, 1);
	bool pub_waypoint_latch;
	n.param<bool>("/lane_navi/pub_waypoint_latch", pub_waypoint_latch, true);

	n.param<int>("/lane_navi/waypoint_max", waypoint_max, 10000);
	n.param<double>("/lane_navi/search_radius", search_radius, 10);
	n.param<double>("/lane_navi/velocity", velocity, 40);
	n.param<std::string>("/lane_navi/frame_id", frame_id, "map");
	n.param<std::string>("/lane_navi/output_file", output_file, "/tmp/lane_waypoint.csv");

	if (output_file.empty()) {
		ROS_ERROR_STREAM("output filename is empty");
		return EXIT_FAILURE;
	}
	if (output_file.back() == '/') {
		ROS_ERROR_STREAM(output_file << " is directory");
		return EXIT_FAILURE;
	}

	waypoint_pub = n.advertise<waypoint_follower::LaneArray>("/lane_waypoints_array", pub_waypoint_queue_size,
								 pub_waypoint_latch);

	ros::Subscriber route_sub = n.subscribe("/route_cmd", sub_route_queue_size, create_waypoint);
	ros::Subscriber point_sub = n.subscribe("/vector_map_info/point_class", sub_vmap_queue_size, cache_point);
	ros::Subscriber lane_sub = n.subscribe("/vector_map_info/lane", sub_vmap_queue_size, cache_lane);
	ros::Subscriber node_sub = n.subscribe("/vector_map_info/node", sub_vmap_queue_size, cache_node);

	ros::spin();

	return EXIT_SUCCESS;
}
