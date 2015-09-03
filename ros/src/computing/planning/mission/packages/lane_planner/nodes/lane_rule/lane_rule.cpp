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
#include <runtime_manager/ConfigLaneRule.h>

#include <vmap_utility.hpp>

// XXX heuristic parameter
static constexpr double CURVE_WEIGHT = 50 * 0.6;
static constexpr double CROSSROAD_WEIGHT = 9.1 * 0.9;
static constexpr double CLOTHOID_WEIGHT = CURVE_WEIGHT;

static constexpr double RADIUS_MAX = 90000000000;

static double config_acceleration = 1; // m/s^2
static int config_number_of_zeros = 1;

static ros::Publisher pub_traffic;
static ros::Publisher pub_red;
static ros::Publisher pub_green;

static VectorMap vmap_all;
static VectorMap vmap_left_lane;

static int sub_vmap_queue_size;
static int sub_waypoint_queue_size;
static int sub_config_queue_size;
static int pub_waypoint_queue_size;
static bool pub_waypoint_latch;

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
	return (!vmap_all.lanes.empty() && !vmap_all.nodes.empty() && !vmap_all.points.empty() &&
		!vmap_all.stoplines.empty() && !vmap_all.dtlanes.empty());
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

static void cache_stopline(const map_file::StopLineArray& msg)
{
	vmap_all.stoplines = msg.stop_lines;
}

static void cache_dtlane(const map_file::DTLaneArray& msg)
{
	vmap_all.dtlanes = msg.dtlanes;
}

static void config_rule(const runtime_manager::ConfigLaneRule& msg)
{
	config_acceleration = msg.acceleration;
	config_number_of_zeros = msg.number_of_zeros;
}

static bool is_straight(const map_file::DTLane& dtlane)
{
	return (dtlane.apara == 0 && dtlane.r == RADIUS_MAX);
}

static bool is_curve(const map_file::DTLane& dtlane)
{
	return (dtlane.apara == 0 && dtlane.r != RADIUS_MAX);
}

static bool is_crossroad(const map_file::DTLane& dtlane)
{
	// XXX take crossroad for 10 radius or less
	return (fabs(dtlane.r) <= 10);
}

static bool is_single_curve(const std::vector<map_file::DTLane>& dtlanes, int index)
{
	int hit = 0, straight = 0;
	int size = dtlanes.size();

	for (int i = index - 1; i >= 0; --i) {
		if (dtlanes[index].r != dtlanes[i].r) {
			++hit;
			if (is_straight(dtlanes[i]))
				++straight;
			break;
		}
	}

	for (int i = index + 1; i < size; ++i) {
		if (dtlanes[index].r != dtlanes[i].r) {
			++hit;
			if (is_straight(dtlanes[i]))
				++straight;
			break;
		}
	}

	if (hit == 0)
		return true;

	if (hit == 1 && straight == 1)
		return true;

	if (straight == 2)
		return true;

	return false;
}

static bool is_clothoid(const map_file::DTLane& dtlane)
{
	return (dtlane.apara != 0);
}

static double compute_reduction(const map_file::DTLane& dtlane, double weight)
{
	return (1 - fabs(1 / dtlane.r) * weight);
}

static double dtlane_to_reduction(const std::vector<map_file::DTLane>& dtlanes, int index)
{
	if (is_straight(dtlanes[index]))
		return 1;

	if (is_curve(dtlanes[index])) {
		if (is_crossroad(dtlanes[index]))
			return compute_reduction(dtlanes[index], CROSSROAD_WEIGHT);

		if (is_single_curve(dtlanes, index))
			return 1;

		return compute_reduction(dtlanes[index], CURVE_WEIGHT);
	}

	if (is_clothoid(dtlanes[index]))
		return compute_reduction(dtlanes[index], CLOTHOID_WEIGHT);

	return 1;
}

static std::vector<map_file::DTLane> search_dtlane(const waypoint_follower::lane& msg)
{
	std::vector<map_file::DTLane> dtlanes;

	// msg's X-Y axis is reversed
	map_file::PointClass start_point = vmap_find_nearest_point(vmap_left_lane,
								   msg.waypoints.front().pose.pose.position.y,
								   msg.waypoints.front().pose.pose.position.x);
	map_file::PointClass end_point = vmap_find_nearest_point(vmap_left_lane,
								 msg.waypoints.back().pose.pose.position.y,
								 msg.waypoints.back().pose.pose.position.x);

	map_file::PointClass point = start_point;
	map_file::Lane lane = vmap_find_lane(vmap_all, point);
	if (lane.lnid < 0) {
		ROS_ERROR("no start lane");
		return dtlanes;
	}

	bool finish = false;
	for (size_t i = 0; i < std::numeric_limits<std::size_t>::max(); ++i) {
		for (const map_file::DTLane& d : vmap_all.dtlanes) {
			if (d.did == lane.did)
				dtlanes.push_back(d);
		}

		if (finish)
			break;

		point = vmap_find_end_point(vmap_all, lane);
		if (point.pid < 0) {
			ROS_ERROR("no end point");
			return dtlanes;
		}

		if (point.bx == end_point.bx && point.ly == end_point.ly) {
			finish = true;
			continue;
		}

		lane = vmap_find_next_lane(vmap_all, lane);
		if (lane.lnid < 0) {
			ROS_ERROR("no next lane");
			return dtlanes;
		}

		point = vmap_find_start_point(vmap_all, lane);
		if (point.pid < 0) {
			ROS_ERROR("no start point");
			return dtlanes;
		}
	}
	if (!finish)
		ROS_ERROR("miss finish");

	return dtlanes;
}

static std::vector<map_file::PointClass> search_stopline_point(const waypoint_follower::lane& msg)
{
	std::vector<map_file::PointClass> stopline_points;

	// msg's X-Y axis is reversed
	map_file::PointClass start_point = vmap_find_nearest_point(vmap_left_lane,
								   msg.waypoints.front().pose.pose.position.y,
								   msg.waypoints.front().pose.pose.position.x);
	map_file::PointClass end_point = vmap_find_nearest_point(vmap_left_lane,
								 msg.waypoints.back().pose.pose.position.y,
								 msg.waypoints.back().pose.pose.position.x);

	map_file::PointClass point = start_point;
	map_file::Lane lane = vmap_find_lane(vmap_all, point);
	if (lane.lnid < 0) {
		ROS_ERROR("no start lane");
		return stopline_points;
	}

	bool finish = false;
	for (size_t i = 0; i < std::numeric_limits<std::size_t>::max(); ++i) {
		for (const map_file::StopLine& s : vmap_all.stoplines) {
			if (s.linkid == lane.lnid)
				stopline_points.push_back(point);
		}

		if (finish)
			break;

		point = vmap_find_end_point(vmap_all, lane);
		if (point.pid < 0) {
			ROS_ERROR("no end point");
			return stopline_points;
		}

		if (point.bx == end_point.bx && point.ly == end_point.ly) {
			finish = true;
			continue;
		}

		lane = vmap_find_next_lane(vmap_all, lane);
		if (lane.lnid < 0) {
			ROS_ERROR("no next lane");
			return stopline_points;
		}

		point = vmap_find_start_point(vmap_all, lane);
		if (point.pid < 0) {
			ROS_ERROR("no start point");
			return stopline_points;
		}
	}
	if (!finish)
		ROS_ERROR("miss finish");

	return stopline_points;
}

static std::vector<size_t> search_stopline_index(const waypoint_follower::lane& msg)
{
	std::vector<size_t> indexes;

	std::vector<map_file::PointClass> stopline_points = search_stopline_point(msg);
	for (const map_file::PointClass& p : stopline_points) {
		size_t i = 0;

		// msg's X-Y axis is reversed
		double min = hypot(p.bx - msg.waypoints[0].pose.pose.position.y,
				   p.ly - msg.waypoints[0].pose.pose.position.x);
		size_t index = i;
		for (const waypoint_follower::waypoint& w : msg.waypoints) {
			// msg's X-Y axis is reversed
			double distance = hypot(p.bx - w.pose.pose.position.y,
						p.ly - w.pose.pose.position.x);
			if (distance < min) {
				min = distance;
				index = i;
			}
			++i;
		}

		indexes.push_back(index);
	}

	return indexes;
}

static waypoint_follower::lane apply_acceleration(const waypoint_follower::lane& msg, double acceleration,
						  size_t start_index, size_t fixed_cnt, double fixed_vel)
{
	waypoint_follower::lane computations = msg;

	if (fixed_cnt == 0)
		return computations;

	for (size_t i = start_index; i < msg.waypoints.size(); ++i) {
		if (i - start_index < fixed_cnt) {
			computations.waypoints[i].twist.twist.linear.x = fixed_vel;
			continue;
		}

		geometry_msgs::Point a = computations.waypoints[i - 1].pose.pose.position;
		geometry_msgs::Point b = computations.waypoints[i].pose.pose.position;
		double distance = hypot(b.x - a.x, b.y - a.y);

		double velocity = computations.waypoints[i - 1].twist.twist.linear.x +
			sqrt(2 * acceleration * distance);
		if (velocity < computations.waypoints[i].twist.twist.linear.x)
			computations.waypoints[i].twist.twist.linear.x = velocity;
		else
			break;
	}

	return computations;
}

static waypoint_follower::lane rule_crossroad(const waypoint_follower::lane& msg, double acceleration)
{
	waypoint_follower::lane computations = msg;

	bool crossroad;
	std::vector<size_t> start_indexes;
	std::vector<size_t> end_indexes;
	for (size_t i = 0; i < msg.waypoints.size(); ++i) {
		map_file::DTLane dtlane;
		dtlane.r = msg.waypoints[i].dtlane.r;
		if (i == 0) {
			crossroad = is_crossroad(dtlane);
			continue;
		}
		if (!crossroad && is_crossroad(dtlane)) {
			start_indexes.push_back(i);
			crossroad = true;
			continue;
		}
		if (crossroad && !is_crossroad(dtlane)) {
			end_indexes.push_back(i - 1);
			crossroad = false;
		}
	}

	for (const size_t& i : end_indexes)
		computations = apply_acceleration(computations, acceleration, i, 1,
						  computations.waypoints[i].twist.twist.linear.x);

	std::reverse(computations.waypoints.begin(), computations.waypoints.end());

	std::vector<size_t> reverse_start_indexes;
	for (const size_t& i : start_indexes)
		reverse_start_indexes.push_back(msg.waypoints.size() - i - 1);
	std::reverse(reverse_start_indexes.begin(), reverse_start_indexes.end());

	for (const size_t& i : reverse_start_indexes)
		computations = apply_acceleration(computations, acceleration, i, 1,
						  computations.waypoints[i].twist.twist.linear.x);

	std::reverse(computations.waypoints.begin(), computations.waypoints.end());

	return computations;
}

static waypoint_follower::lane rule_stopline(const waypoint_follower::lane& msg, double acceleration, size_t fixed_cnt)
{
	waypoint_follower::lane computations = msg;

	std::vector<size_t> indexes = search_stopline_index(msg);
	if (indexes.empty())
		return computations;

	for (const size_t& i : indexes)
		computations = apply_acceleration(computations, acceleration, i, 1, 0);

	std::reverse(computations.waypoints.begin(), computations.waypoints.end());

	std::vector<size_t> reverse_indexes;
	for (const size_t& i : indexes)
		reverse_indexes.push_back(msg.waypoints.size() - i - 1);
	std::reverse(reverse_indexes.begin(), reverse_indexes.end());

	for (const size_t& i : reverse_indexes)
		computations = apply_acceleration(computations, acceleration, i, fixed_cnt, 0);

	std::reverse(computations.waypoints.begin(), computations.waypoints.end());

	return computations;
}

static void publish_waypoint_without_change(const waypoint_follower::lane& msg, const std_msgs::Header& header)
{
	waypoint_follower::lane traffic;
	traffic.header = header;
	traffic.increment = 1;
	waypoint_follower::waypoint waypoint;
	waypoint.pose.header = header;
	waypoint.twist.header = header;

	for (size_t i = 0; i < msg.waypoints.size(); ++i) {
		waypoint.pose.pose = msg.waypoints[i].pose.pose;
		waypoint.twist.twist = msg.waypoints[i].twist.twist;
		traffic.waypoints.push_back(waypoint);
	}

	pub_traffic.publish(traffic);
}

static void create_traffic_waypoint(const waypoint_follower::lane& msg)
{
	std_msgs::Header header;
	header.stamp = ros::Time::now();
	header.frame_id = "/map";

	if (!is_cached_vmap()) {
		ROS_WARN("not cached vmap");
		publish_waypoint_without_change(msg, header);
		return;
	}

	std::vector<map_file::DTLane> dtlanes = search_dtlane(msg);
	if (dtlanes.size() != msg.waypoints.size()) {
		ROS_WARN("not found dtlane");
		publish_waypoint_without_change(msg, header);
		return;
	}

	waypoint_follower::lane green;
	green.header = header;
	green.increment = 1;
	waypoint_follower::lane red;
	red.header = header;
	red.increment = 1;
	waypoint_follower::waypoint waypoint;
	waypoint.pose.header = header;
	waypoint.twist.header = header;

	for (size_t i = 0; i < msg.waypoints.size(); ++i) {
		double reduction = dtlane_to_reduction(dtlanes, i);

		waypoint.pose.pose = msg.waypoints[i].pose.pose;

		waypoint.dtlane.dist = dtlanes[i].dist;
		waypoint.dtlane.dir = dtlanes[i].dir;
		waypoint.dtlane.apara = dtlanes[i].apara;
		waypoint.dtlane.r = dtlanes[i].r;
		waypoint.dtlane.slope = dtlanes[i].slope;
		waypoint.dtlane.cant = dtlanes[i].cant;
		waypoint.dtlane.lw = dtlanes[i].lw;
		waypoint.dtlane.rw = dtlanes[i].rw;

		waypoint.twist.twist = msg.waypoints[i].twist.twist;
		waypoint.twist.twist.linear.x *= reduction;
		green.waypoints.push_back(waypoint);
	}

	waypoint_follower::lane crossroad = rule_crossroad(green, config_acceleration);

	waypoint_follower::lane stopline = rule_stopline(crossroad, config_acceleration, config_number_of_zeros);

	for (size_t i = 0; i < msg.waypoints.size(); ++i) {
		green.waypoints[i].twist.twist = crossroad.waypoints[i].twist.twist;
		waypoint = green.waypoints[i];
		waypoint.twist.twist = stopline.waypoints[i].twist.twist;
		red.waypoints.push_back(waypoint);
	}

	pub_traffic.publish(green);
	pub_green.publish(green);
	pub_red.publish(red);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lane_rule");

	ros::NodeHandle n;
	n.param<int>("/lane_navi/sub_vmap_queue_size", sub_vmap_queue_size, 1);
	n.param<int>("/lane_navi/sub_waypoint_queue_size", sub_waypoint_queue_size, 1);
	n.param<int>("/lane_navi/sub_config_queue_size", sub_config_queue_size, 1);
	n.param<int>("/lane_navi/pub_waypoint_queue_size", pub_waypoint_queue_size, 1);
	n.param<bool>("/lane_navi/pub_waypoint_latch", pub_waypoint_latch, true);

	ros::Subscriber sub_lane = n.subscribe("/vector_map_info/lane", sub_vmap_queue_size, cache_lane);
	ros::Subscriber sub_node = n.subscribe("/vector_map_info/node", sub_vmap_queue_size, cache_node);
	ros::Subscriber sub_point = n.subscribe("/vector_map_info/point_class", sub_vmap_queue_size, cache_point);
	ros::Subscriber sub_stopline = n.subscribe("/vector_map_info/stop_line", sub_vmap_queue_size, cache_stopline);
	ros::Subscriber sub_dtlane = n.subscribe("/vector_map_info/dtlane", sub_vmap_queue_size, cache_dtlane);
	ros::Subscriber sub_waypoint = n.subscribe("/lane_waypoint", sub_waypoint_queue_size, create_traffic_waypoint);
	ros::Subscriber sub_config = n.subscribe("/config/lane_rule", sub_config_queue_size, config_rule);

	pub_traffic = n.advertise<waypoint_follower::lane>("/traffic_waypoint", pub_waypoint_queue_size,
							   pub_waypoint_latch);
	pub_red = n.advertise<waypoint_follower::lane>("/red_waypoint", pub_waypoint_queue_size,
						       pub_waypoint_latch);
	pub_green = n.advertise<waypoint_follower::lane>("/green_waypoint", pub_waypoint_queue_size,
							 pub_waypoint_latch);

	ros::spin();

	return 0;
}
