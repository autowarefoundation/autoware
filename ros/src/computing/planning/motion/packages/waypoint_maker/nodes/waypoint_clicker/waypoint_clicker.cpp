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
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>

#include <vmap_utility.hpp>

static const std::string FRAME_ID = "map";

static ros::Publisher pub_marker;

static VectorMap vmap_all;
static VectorMap vmap_lane;
static VectorMap vmap_coarse;

visualization_msgs::Marker selection;
static tf::StampedTransform transform;
static bool output_first;

static int sub_vmap_queue_size;
static int sub_pose_queue_size;
static int pub_marker_queue_size;

static double velocity;		// km/h
static std::string output_file;
static int waypoint_max;

static bool is_branching(const map_file::Lane& lane)
{
	return (lane.jct >= 1 && lane.jct <= 2);
}

static bool is_merging(const map_file::Lane& lane)
{
	return (lane.jct >= 3 && lane.jct <= 4);
}

static void delete_all_marker()
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = FRAME_ID;
	marker.header.stamp = ros::Time::now();
	marker.id = 0;
	marker.action = visualization_msgs::Marker::DELETE;

	marker.ns = "waypoint";
	pub_marker.publish(marker);
	marker.ns = "branching";
	pub_marker.publish(marker);
	marker.ns = "merging";
	pub_marker.publish(marker);
	marker.ns = "selection";
	pub_marker.publish(marker);
	marker.ns = "route";
	pub_marker.publish(marker);
}

static void publish_waypoint()
{
	visualization_msgs::Marker waypoint;
	waypoint.header.frame_id = FRAME_ID;
	waypoint.header.stamp = ros::Time::now();
	waypoint.ns = "waypoint";
	waypoint.id = 0;
	waypoint.type = visualization_msgs::Marker::SPHERE_LIST;
	waypoint.action = visualization_msgs::Marker::ADD;
	waypoint.scale.x = 0.2;
	waypoint.scale.y = 0.2;
	waypoint.color.r = 1;
	waypoint.color.g = 1;
	waypoint.color.b = 0;
	waypoint.color.a = 1;
	waypoint.frame_locked = true;

	for (const map_file::PointClass& p : vmap_lane.points) {
		// msg's X-Y axis is reversed
		geometry_msgs::Point point;
		point.x = p.ly;
		point.y = p.bx;
		point.z = p.h;
		waypoint.points.push_back(point);
	}

	pub_marker.publish(waypoint);
}

static void publish_branching()
{
	visualization_msgs::Marker branching;
	branching.header.frame_id = FRAME_ID;
	branching.header.stamp = ros::Time::now();
	branching.ns = "branching";
	branching.id = 0;
	branching.type = visualization_msgs::Marker::SPHERE_LIST;
	branching.action = visualization_msgs::Marker::ADD;
	branching.scale.x = 0.2;
	branching.scale.y = 0.2;
	branching.color.r = 0;
	branching.color.g = 1;
	branching.color.b = 0;
	branching.color.a = 1;
	branching.frame_locked = true;

	for (const map_file::Lane& l : vmap_lane.lanes) {
		if (!is_branching(l))
			continue;
		map_file::PointClass p = vmap_find_end_point(vmap_lane, l);
		if (p.pid < 0)
			continue;
		// msg's X-Y axis is reversed
		geometry_msgs::Point point;
		point.x = p.ly;
		point.y = p.bx;
		point.z = p.h;
		branching.points.push_back(point);
	}

	pub_marker.publish(branching);
}

static void publish_merging()
{
	visualization_msgs::Marker merging;
	merging.header.frame_id = FRAME_ID;
	merging.header.stamp = ros::Time::now();
	merging.ns = "merging";
	merging.id = 0;
	merging.type = visualization_msgs::Marker::SPHERE_LIST;
	merging.action = visualization_msgs::Marker::ADD;
	merging.scale.x = 0.2;
	merging.scale.y = 0.2;
	merging.color.r = 1;
	merging.color.g = 0;
	merging.color.b = 0;
	merging.color.a = 1;
	merging.frame_locked = true;

	for (const map_file::Lane& l : vmap_lane.lanes) {
		if (!is_merging(l))
			continue;
		map_file::PointClass p = vmap_find_start_point(vmap_lane, l);
		if (p.pid < 0)
			continue;
		// msg's X-Y axis is reversed
		geometry_msgs::Point point;
		point.x = p.ly;
		point.y = p.bx;
		point.z = p.h;
		merging.points.push_back(point);
	}

	pub_marker.publish(merging);
}

static void publish_selection(const geometry_msgs::PointStamped& pose)
{
	selection.header.stamp = ros::Time::now();

	geometry_msgs::Point point;
	point.x = pose.point.x + transform.getOrigin().x();
	point.y = pose.point.y + transform.getOrigin().y();
	point.z = pose.point.z + transform.getOrigin().z();
	selection.points.push_back(point);

	pub_marker.publish(selection);
}

static void cache_vmap_lane()
{
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

static void cache_lane(const map_file::LaneArray& msg)
{
	vmap_all.lanes = msg.lanes;

	if (vmap_all.lanes.empty() || vmap_all.nodes.empty() || vmap_all.points.empty())
		return;

	cache_vmap_lane();
	delete_all_marker();
	publish_waypoint();
	publish_branching();
	publish_merging();
}

static void cache_node(const map_file::NodeArray& msg)
{
	vmap_all.nodes = msg.nodes;

	if (vmap_all.lanes.empty() || vmap_all.nodes.empty() || vmap_all.points.empty())
		return;

	cache_vmap_lane();
	delete_all_marker();
	publish_waypoint();
	publish_branching();
	publish_merging();
}

static void cache_point(const map_file::PointClassArray& msg)
{
	vmap_all.points = msg.point_classes;

	if (vmap_all.lanes.empty() || vmap_all.nodes.empty() || vmap_all.points.empty())
		return;

	cache_vmap_lane();
	delete_all_marker();
	publish_waypoint();
	publish_branching();
	publish_merging();
}

static void write_route(const geometry_msgs::Point& point, bool first)
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

static void create_route(const geometry_msgs::PointStamped& msg)
{
	if (vmap_all.lanes.empty() || vmap_all.nodes.empty() || vmap_all.points.empty())
		return;

	publish_selection(msg);

	// msg's X-Y axis is reversed
	map_file::PointClass coarse;
	coarse.bx = msg.point.y + transform.getOrigin().y();
	coarse.ly = msg.point.x + transform.getOrigin().x();
	coarse.h = msg.point.z + transform.getOrigin().z();
	vmap_coarse.points.push_back(coarse);

	if (vmap_coarse.points.size() < 2)
		return;

	map_file::PointClass start_point = vmap_find_nearest_point(vmap_lane, vmap_coarse.points.front());
	if (start_point.pid < 0) {
		ROS_ERROR("no start point");
		return;
	}
	map_file::PointClass end_point = vmap_find_nearest_point(vmap_lane, vmap_coarse.points.back());
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

	visualization_msgs::Marker route;
	route.header.frame_id = FRAME_ID;
	route.header.stamp = ros::Time::now();
	route.ns = "route";
	route.id = 0;
	route.type = visualization_msgs::Marker::LINE_STRIP;
	route.action = visualization_msgs::Marker::ADD;
	route.scale.x = 0.2;
	route.scale.y = 0.2;
	route.color.r = 1;
	route.color.g = 1;
	route.color.b = 0;
	route.color.a = 1;
	route.frame_locked = true;

	bool finish = false;
	for (int i = 0; i < waypoint_max; ++i) {
		// msg's X-Y axis is reversed
		geometry_msgs::Point route_point;
		route_point.x = point.ly;
		route_point.y = point.bx;
		route_point.z = point.h;
		write_route(route_point, (i == 0));
		route.points.push_back(route_point);

		if (finish)
			break;

		point = vmap_find_end_point(vmap_lane, lane);
		if (point.pid < 0) {
			ROS_ERROR("no next point");
			return;
		}

		if (point.bx == end_point.bx && point.ly == end_point.ly) {
			finish = true;
			continue;
		}

		if (is_branching(lane)) {
			map_file::PointClass p1 = vmap_find_end_point(vmap_lane, lane);
			if (p1.pid < 0) {
				ROS_ERROR("no branching start point");
				return;
			}

			p1 = vmap_find_nearest_point(vmap_coarse, p1);
			if (p1.pid < 0) {
				ROS_ERROR("no coarse branching start point");
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
				ROS_ERROR("no coarse branching end point");
				return;
			}

			lane = vmap_find_junction_lane(vmap_lane, lane, vmap_compute_direction_angle(p1, p2));
		} else
			lane = vmap_find_next_lane(vmap_lane, lane);
		if (lane.lnid < 0) {
			ROS_ERROR("no next lane");
			return;
		}
	}

	if (!finish) {
		ROS_ERROR("route not found");
		return;
	}

	pub_marker.publish(route);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "waypoint_clicker");

	ros::NodeHandle n;
	n.param<int>("/waypoint_clicker/sub_vmap_queue_size", sub_vmap_queue_size, 1);
	n.param<int>("/waypoint_clicker/sub_pose_queue_size", sub_pose_queue_size, 1);
	n.param<int>("/waypoint_clicker/pub_marker_queue_size", pub_marker_queue_size, 10);
	n.param<double>("/waypoint_clicker/velocity", velocity, 40);
	n.param<std::string>("/waypoint_clicker/output_file", output_file, "/tmp/lane_waypoint.csv");
	n.param<int>("/waypoint_clicker/waypoint_max", waypoint_max, 10000);

	ros::Subscriber sub_lane = n.subscribe("/vector_map_info/lane", sub_vmap_queue_size, cache_lane);
	ros::Subscriber sub_node = n.subscribe("/vector_map_info/node", sub_vmap_queue_size, cache_node);
	ros::Subscriber sub_point = n.subscribe("/vector_map_info/point_class", sub_vmap_queue_size, cache_point);
	ros::Subscriber sub_pose = n.subscribe("/clicked_point", sub_pose_queue_size, create_route);

	pub_marker = n.advertise<visualization_msgs::Marker>("/waypoint_guide", pub_marker_queue_size, true);

	selection.header.frame_id = FRAME_ID;
	selection.ns = "selection";
	selection.id = 0;
	selection.type = visualization_msgs::Marker::SPHERE_LIST;
	selection.action = visualization_msgs::Marker::ADD;
	selection.scale.x = 0.4;
	selection.scale.y = 0.4;
	selection.color.r = 1;
	selection.color.g = 1;
	selection.color.b = 0;
	selection.color.a = 1;
	selection.frame_locked = true;

	tf::TransformListener listener;
	try {
		ros::Time zero = ros::Time(0);
		listener.waitForTransform("map", "world", zero, ros::Duration(10));
		listener.lookupTransform("map", "world", zero, transform);
	} catch (tf::TransformException &ex) {
		ROS_ERROR("%s", ex.what());
	}
	output_first = true;

	ros::spin();

	return 0;
}
