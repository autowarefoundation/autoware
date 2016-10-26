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

// #define DEBUG

#ifdef DEBUG
#include <sstream>
#endif // DEBUG

#include <ros/console.h>

#include <vector_map/vector_map.h>
#include <runtime_manager/ConfigLaneRule.h>
#include <waypoint_follower/LaneArray.h>

#include <lane_planner/vmap.hpp>

namespace {

double config_acceleration = 1; // m/s^2
int config_number_of_zeros_ahead = 0;
int config_number_of_zeros_behind = 0;

int waypoint_max;
double search_radius; // meter
double curve_weight;
double crossroad_weight;
double clothoid_weight;
std::string frame_id;

ros::Publisher traffic_pub;
ros::Publisher red_pub;
ros::Publisher green_pub;

lane_planner::vmap::VectorMap all_vmap;
lane_planner::vmap::VectorMap lane_vmap;
double curve_radius_min;
double crossroad_radius_min;
double clothoid_radius_min;
waypoint_follower::LaneArray cached_waypoint;

#ifdef DEBUG
visualization_msgs::Marker debug_marker;
ros::Publisher marker_pub;
int marker_cnt;
#endif // DEBUG

waypoint_follower::lane create_new_lane(const waypoint_follower::lane& lane, const std_msgs::Header& header)
{
	waypoint_follower::lane l = lane;
	l.header = header;

	for (waypoint_follower::waypoint& w : l.waypoints) {
		w.pose.header = header;
		w.twist.header = header;
	}

	return l;
}

waypoint_follower::lane apply_acceleration(const waypoint_follower::lane& lane, double acceleration,
					   size_t start_index, size_t fixed_cnt, double fixed_vel)
{
	waypoint_follower::lane l = lane;

	if (fixed_cnt == 0)
		return l;

	double square_vel = fixed_vel * fixed_vel;
	double distance = 0;
	for (size_t i = start_index; i < l.waypoints.size(); ++i) {
		if (i - start_index < fixed_cnt) {
			l.waypoints[i].twist.twist.linear.x = fixed_vel;
			continue;
		}

		geometry_msgs::Point a = l.waypoints[i - 1].pose.pose.position;
		geometry_msgs::Point b = l.waypoints[i].pose.pose.position;
		distance += hypot(b.x - a.x, b.y - a.y);

		double v = sqrt(square_vel + 2 * acceleration * distance);
		if (v < l.waypoints[i].twist.twist.linear.x)
			l.waypoints[i].twist.twist.linear.x = v;
		else
			break;
	}

	return l;
}

waypoint_follower::lane apply_crossroad_acceleration(const waypoint_follower::lane& lane, double acceleration)
{
	waypoint_follower::lane l = lane;

	bool crossroad = false;
	std::vector<size_t> start_indexes;
	std::vector<size_t> end_indexes;
	for (size_t i = 0; i < l.waypoints.size(); ++i) {
		vector_map::DTLane dtlane = lane_planner::vmap::create_vector_map_dtlane(l.waypoints[i].dtlane);
		if (i == 0) {
			crossroad = lane_planner::vmap::is_crossroad_dtlane(dtlane);
			continue;
		}
		if (crossroad) {
			if (!lane_planner::vmap::is_crossroad_dtlane(dtlane)) {
				end_indexes.push_back(i - 1);
				crossroad = false;
			}
		} else {
			if (lane_planner::vmap::is_crossroad_dtlane(dtlane)) {
				start_indexes.push_back(i);
				crossroad = true;
			}
		}
	}
	if (start_indexes.empty() && end_indexes.empty())
		return l;

	for (const size_t i : end_indexes)
		l = apply_acceleration(l, acceleration, i, 1, l.waypoints[i].twist.twist.linear.x);

	std::reverse(l.waypoints.begin(), l.waypoints.end());

	std::vector<size_t> reverse_start_indexes;
	for (const size_t i : start_indexes)
		reverse_start_indexes.push_back(l.waypoints.size() - i - 1);
	std::reverse(reverse_start_indexes.begin(), reverse_start_indexes.end());

	for (const size_t i : reverse_start_indexes)
		l = apply_acceleration(l, acceleration, i, 1, l.waypoints[i].twist.twist.linear.x);

	std::reverse(l.waypoints.begin(), l.waypoints.end());

	return l;
}

waypoint_follower::lane apply_stopline_acceleration(const waypoint_follower::lane& lane, double acceleration,
						    const lane_planner::vmap::VectorMap& fine_vmap, size_t ahead_cnt,
						    size_t behind_cnt)
{
	waypoint_follower::lane l = lane;

	std::vector<size_t> indexes;
	for (size_t i = 0; i < fine_vmap.stoplines.size(); ++i) {
		if (fine_vmap.stoplines[i].id >= 0)
			indexes.push_back(i);
	}
	if (indexes.empty())
		return l;

	for (const size_t i : indexes)
		l = apply_acceleration(l, acceleration, i, behind_cnt + 1, 0);

	std::reverse(l.waypoints.begin(), l.waypoints.end());

	std::vector<size_t> reverse_indexes;
	for (const size_t i : indexes)
		reverse_indexes.push_back(l.waypoints.size() - i - 1);
	std::reverse(reverse_indexes.begin(), reverse_indexes.end());

	for (const size_t i : reverse_indexes)
		l = apply_acceleration(l, acceleration, i, ahead_cnt + 1, 0);

	std::reverse(l.waypoints.begin(), l.waypoints.end());

	return l;
}

double create_reduction(const lane_planner::vmap::VectorMap& fine_vmap, int index)
{
	const vector_map::DTLane& dtlane = fine_vmap.dtlanes[index];

	if (lane_planner::vmap::is_straight_dtlane(dtlane))
		return 1;

	if (lane_planner::vmap::is_curve_dtlane(dtlane)) {
		if (lane_planner::vmap::is_crossroad_dtlane(dtlane))
			return lane_planner::vmap::compute_reduction(dtlane, crossroad_radius_min * crossroad_weight);

		if (lane_planner::vmap::is_connection_dtlane(fine_vmap, index))
			return 1;

		return lane_planner::vmap::compute_reduction(dtlane, curve_radius_min * curve_weight);
	}

	if (lane_planner::vmap::is_clothoid_dtlane(dtlane))
		return lane_planner::vmap::compute_reduction(dtlane, clothoid_radius_min * clothoid_weight);

	return 1;
}

#ifdef DEBUG
std_msgs::ColorRGBA create_color(int index)
{
	std_msgs::ColorRGBA color;
	switch (index) {
	case 0:
		color.r = 0;
		color.g = 0;
		color.b = 0;
		break;
	case 1:
		color.r = 0;
		color.g = 0;
		color.b = 1;
		break;
	case 2:
		color.r = 0;
		color.g = 1;
		color.b = 0;
		break;
	case 3:
		color.r = 0;
		color.g = 1;
		color.b = 1;
		break;
	case 4:
		color.r = 1;
		color.g = 0;
		color.b = 0;
		break;
	case 5:
		color.r = 1;
		color.g = 0;
		color.b = 1;
		break;
	case 6:
		color.r = 1;
		color.g = 1;
		color.b = 0;
		break;
	default:
		color.r = 1;
		color.g = 1;
		color.b = 1;
	}
	color.a = 1;

	return color;
}
#endif // DEBUG

void create_waypoint(const waypoint_follower::LaneArray& msg)
{
	std_msgs::Header header;
	header.stamp = ros::Time::now();
	header.frame_id = frame_id;

	cached_waypoint.lanes.clear();
	cached_waypoint.lanes.shrink_to_fit();
	for (const waypoint_follower::lane& l : msg.lanes)
		cached_waypoint.lanes.push_back(create_new_lane(l, header));
	if (all_vmap.points.empty() || all_vmap.lanes.empty() || all_vmap.nodes.empty() ||
	    all_vmap.stoplines.empty() || all_vmap.dtlanes.empty()) {
		traffic_pub.publish(cached_waypoint);
		return;
	}

#ifdef DEBUG
	marker_cnt = msg.lanes.size();
#endif // DEBUG

	waypoint_follower::LaneArray traffic_waypoint;
	waypoint_follower::LaneArray red_waypoint;
	waypoint_follower::LaneArray green_waypoint;
	for (size_t i = 0; i < msg.lanes.size(); ++i) {
		waypoint_follower::lane lane = create_new_lane(msg.lanes[i], header);

		lane_planner::vmap::VectorMap coarse_vmap =
			lane_planner::vmap::create_coarse_vmap_from_lane(lane);
		if (coarse_vmap.points.size() < 2) {
			traffic_waypoint.lanes.push_back(lane);
			continue;
		}

		lane_planner::vmap::VectorMap fine_vmap =
			lane_planner::vmap::create_fine_vmap(lane_vmap, lane_planner::vmap::LNO_ALL, coarse_vmap,
							     search_radius, waypoint_max);
		if (fine_vmap.points.size() < 2 || fine_vmap.points.size() != lane.waypoints.size()) {
			traffic_waypoint.lanes.push_back(lane);
			continue;
		}

		for (size_t j = 0; j < lane.waypoints.size(); ++j) {
			lane.waypoints[j].twist.twist.linear.x *= create_reduction(fine_vmap, j);
			if (fine_vmap.dtlanes[j].did >= 0) {
				lane.waypoints[j].dtlane =
					lane_planner::vmap::create_waypoint_follower_dtlane(fine_vmap.dtlanes[j]);
			}
		}

		lane = apply_crossroad_acceleration(lane, config_acceleration);

		traffic_waypoint.lanes.push_back(lane);
		green_waypoint.lanes.push_back(lane);

		lane = apply_stopline_acceleration(lane, config_acceleration, fine_vmap, config_number_of_zeros_ahead,
						   config_number_of_zeros_behind);

		red_waypoint.lanes.push_back(lane);

#ifdef DEBUG
		std::stringstream ss;
		ss << "_" << i;

		visualization_msgs::Marker m = debug_marker;
		m.ns = "lane" + ss.str();
		m.color = create_color(i);

		lane_planner::vmap::publish_add_marker(marker_pub, m, fine_vmap.points);
#endif // DEBUG
	}

	traffic_pub.publish(traffic_waypoint);
	red_pub.publish(red_waypoint);
	green_pub.publish(green_waypoint);
}

void update_values()
{
	if (all_vmap.points.empty() || all_vmap.lanes.empty() || all_vmap.nodes.empty() ||
	    all_vmap.stoplines.empty() || all_vmap.dtlanes.empty())
		return;

	lane_vmap = lane_planner::vmap::create_lane_vmap(all_vmap, lane_planner::vmap::LNO_ALL);

	curve_radius_min = lane_planner::vmap::RADIUS_MAX;
	crossroad_radius_min = lane_planner::vmap::RADIUS_MAX;
	clothoid_radius_min = lane_planner::vmap::RADIUS_MAX;
	for (const vector_map::DTLane& d : lane_vmap.dtlanes) {
		double radius_min = fabs(d.r);
		if (lane_planner::vmap::is_curve_dtlane(d)) {
			if (lane_planner::vmap::is_crossroad_dtlane(d)) {
				if (radius_min < crossroad_radius_min)
					crossroad_radius_min = radius_min;
			} else {
				if (radius_min < curve_radius_min)
					curve_radius_min = radius_min;
			}
		} else if (lane_planner::vmap::is_clothoid_dtlane(d)) {
			if (radius_min < clothoid_radius_min)
				clothoid_radius_min = radius_min;
		}
	}

#ifdef DEBUG
	for (int i = 0; i < marker_cnt; ++i) {
		std::stringstream ss;
		ss << "_" << i;

		visualization_msgs::Marker m = debug_marker;
		m.ns = "lane" + ss.str();

		lane_planner::vmap::publish_delete_marker(marker_pub, m);
	}
	marker_cnt = 0;
#endif // DEBUG

	if (!cached_waypoint.lanes.empty()) {
		waypoint_follower::LaneArray update_waypoint = cached_waypoint;
		create_waypoint(update_waypoint);
	}
}

void cache_point(const vector_map::PointArray& msg)
{
	all_vmap.points = msg.data;
	update_values();
}

void cache_lane(const vector_map::LaneArray& msg)
{
	all_vmap.lanes = msg.data;
	update_values();
}

void cache_node(const vector_map::NodeArray& msg)
{
	all_vmap.nodes = msg.data;
	update_values();
}

void cache_stopline(const vector_map::StopLineArray& msg)
{
	all_vmap.stoplines = msg.data;
	update_values();
}

void cache_dtlane(const vector_map::DTLaneArray& msg)
{
	all_vmap.dtlanes = msg.data;
	update_values();
}

void config_parameter(const runtime_manager::ConfigLaneRule& msg)
{
	config_acceleration = msg.acceleration;
	config_number_of_zeros_ahead = msg.number_of_zeros_ahead;
	config_number_of_zeros_behind = msg.number_of_zeros_behind;

	if (!cached_waypoint.lanes.empty()) {
		waypoint_follower::LaneArray update_waypoint = cached_waypoint;
		create_waypoint(update_waypoint);
	}
}

} // namespace

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lane_rule");

	ros::NodeHandle n;

	int sub_vmap_queue_size;
	n.param<int>("/lane_rule/sub_vmap_queue_size", sub_vmap_queue_size, 1);
	int sub_waypoint_queue_size;
	n.param<int>("/lane_rule/sub_waypoint_queue_size", sub_waypoint_queue_size, 1);
	int sub_config_queue_size;
	n.param<int>("/lane_rule/sub_config_queue_size", sub_config_queue_size, 1);
	int pub_waypoint_queue_size;
	n.param<int>("/lane_rule/pub_waypoint_queue_size", pub_waypoint_queue_size, 1);
	bool pub_waypoint_latch;
	n.param<bool>("/lane_rule/pub_waypoint_latch", pub_waypoint_latch, true);
#ifdef DEBUG
	int pub_marker_queue_size;
	n.param<int>("/lane_rule/pub_marker_queue_size", pub_marker_queue_size, 10);
	bool pub_marker_latch;
	n.param<bool>("/lane_rule/pub_marker_latch", pub_marker_latch, true);
#endif // DEBUG

	n.param<int>("/lane_rule/waypoint_max", waypoint_max, 10000);
	n.param<double>("/lane_rule/search_radius", search_radius, 10);
	n.param<double>("/lane_rule/curve_weight", curve_weight, 0.6);
	n.param<double>("/lane_rule/crossroad_weight", crossroad_weight, 0.9);
	n.param<double>("/lane_rule/clothoid_weight", clothoid_weight, 0.215);
	n.param<std::string>("/lane_rule/frame_id", frame_id, "map");

	traffic_pub = n.advertise<waypoint_follower::LaneArray>("/traffic_waypoints_array", pub_waypoint_queue_size,
								pub_waypoint_latch);
	red_pub = n.advertise<waypoint_follower::LaneArray>("/red_waypoints_array", pub_waypoint_queue_size,
							    pub_waypoint_latch);
	green_pub = n.advertise<waypoint_follower::LaneArray>("/green_waypoints_array", pub_waypoint_queue_size,
							      pub_waypoint_latch);

#ifdef DEBUG
	debug_marker.header.frame_id = frame_id;
	debug_marker.id = 0;
	debug_marker.type = visualization_msgs::Marker::LINE_STRIP;
	debug_marker.scale.x = 0.2;
	debug_marker.scale.y = 0.2;
	debug_marker.frame_locked = true;

	marker_pub = n.advertise<visualization_msgs::Marker>("/waypoint_debug", pub_marker_queue_size,
							     pub_marker_latch);
#endif // DEBUG

	ros::Subscriber waypoint_sub = n.subscribe("/lane_waypoints_array", sub_waypoint_queue_size, create_waypoint);
	ros::Subscriber point_sub = n.subscribe("/vector_map_info/point", sub_vmap_queue_size, cache_point);
	ros::Subscriber lane_sub = n.subscribe("/vector_map_info/lane", sub_vmap_queue_size, cache_lane);
	ros::Subscriber node_sub = n.subscribe("/vector_map_info/node", sub_vmap_queue_size, cache_node);
	ros::Subscriber stopline_sub = n.subscribe("/vector_map_info/stop_line", sub_vmap_queue_size, cache_stopline);
	ros::Subscriber dtlane_sub = n.subscribe("/vector_map_info/dtlane", sub_vmap_queue_size, cache_dtlane);
	ros::Subscriber config_sub = n.subscribe("/config/lane_rule", sub_config_queue_size, config_parameter);

	ros::spin();

	return EXIT_SUCCESS;
}
