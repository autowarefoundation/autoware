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

#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <waypoint_follower/lane.h>
#include <runtime_manager/ConfigLaneRule.h>

#include <vmap_utility.hpp>

static constexpr uint32_t SUBSCRIBE_QUEUE_SIZE = 1000;

static constexpr uint32_t ADVERTISE_QUEUE_SIZE = 1000;
static constexpr bool ADVERTISE_LATCH = true;

static constexpr int PRECISION = 6;
static constexpr double ACCIDENT_ERROR = 0.000001;
static constexpr double RADIUS_MAX = 90000000000;
static constexpr double CURVE_WEIGHT = 50 * 0.6; // XXX
static constexpr double CROSSROAD_WEIGHT = 9.1 * 0.9; // XXX
static constexpr double CLOTHOID_WEIGHT = CURVE_WEIGHT;

static const std::string RULED_WAYPOINT_CSV = "/tmp/ruled_waypoint.csv";

static double config_velocity = 40; // Unit: km/h
static double config_difference_around_signal = 2; // Unit: km/h
static int32_t config_number_of_zeros = 1;

static ros::Publisher pub_velocity;
static ros::Publisher pub_ruled;
static ros::Publisher pub_red;
static ros::Publisher pub_green;

static std::vector<map_file::Lane> lanes;
static std::vector<map_file::Node> nodes;
static std::vector<map_file::PointClass> points;
static std::vector<map_file::StopLine> stoplines;
static std::vector<map_file::DTLane> dtlanes;

static std::vector<map_file::PointClass> left_lane_points;

static int waypoint_count;

static std::string ruled_waypoint_csv;

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

static void stop_line_callback(const map_file::StopLineArray& msg)
{
	stoplines = msg.stop_lines;
}

static void dtlane_callback(const map_file::DTLaneArray& msg)
{
	dtlanes = msg.dtlanes;
}

static void config_callback(const runtime_manager::ConfigLaneRule& msg)
{
	config_velocity = msg.velocity;
	config_difference_around_signal = msg.difference_around_signal;
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
	return (dtlane.did >= 5547 && dtlane.did <= 5557); // XXX
}

static bool is_single_curve(const std::vector<map_file::DTLane>& dtls,
			    int index)
{
	int hit = 0, straight = 0;
	int size = dtls.size();

	for (int i = index - 1; i >= 0; --i) {
		if (dtls[index].r != dtls[i].r) {
			++hit;
			if (is_straight(dtls[i]))
				++straight;
			break;
		}
	}

	for (int i = index + 1; i < size; ++i) {
		if (dtls[index].r != dtls[i].r) {
			++hit;
			if (is_straight(dtls[i]))
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

static double dtlane_to_reduction(const std::vector<map_file::DTLane>& dtls,
				  int index)
{
	if (is_straight(dtls[index]))
		return 1;
	if (is_curve(dtls[index])) {
		if (is_crossroad(dtls[index]))
			return compute_reduction(dtls[index],
						 CROSSROAD_WEIGHT);
		if (is_single_curve(dtls, index))
			return 1;
		return compute_reduction(dtls[index], CURVE_WEIGHT);
	}
	if (is_clothoid(dtls[index]))
		return compute_reduction(dtls[index], CLOTHOID_WEIGHT);

	return 1;
}

static std::vector<map_file::DTLane>
search_waypoint_dtlane(const nav_msgs::Path& msg)
{
	std::vector<map_file::DTLane> waypoint_dtlanes;

	// msg's X-Y axis is reversed
	map_file::PointClass start_point = search_nearest(
		left_lane_points,
		msg.poses.front().pose.position.y,
		msg.poses.front().pose.position.x);

	// msg's X-Y axis is reversed
	map_file::PointClass end_point = search_nearest(
		left_lane_points,
		msg.poses.back().pose.position.y,
		msg.poses.back().pose.position.x);

	int lane_index = to_lane_index(start_point, nodes, lanes);
	if (lane_index < 0) {
		ROS_ERROR("start lane is not found");
		return waypoint_dtlanes;
	}
	map_file::Lane lane = lanes[lane_index];

	int point_index = to_beginning_point_index(lane, nodes, points);
	if (point_index < 0) {
		ROS_ERROR("start beginning point is not found");
		return waypoint_dtlanes;
	}
	map_file::PointClass point = points[point_index];

	while (1) {
		for (const map_file::DTLane& dtlane : dtlanes) {
			if (dtlane.did == lane.did)
				waypoint_dtlanes.push_back(dtlane);
		}

		point_index = to_finishing_point_index(lane, nodes, points);
		if (point_index < 0) {
			ROS_ERROR("finishing point is not found");
			return waypoint_dtlanes;
		}
		point = points[point_index];

		lane_index = to_next_lane_index(lane, lanes);
		if (lane_index < 0) {
			ROS_ERROR("next lane is not found");
			return waypoint_dtlanes;
		}
		lane = lanes[lane_index];

		if (point.bx == end_point.bx &&
		    point.ly == end_point.ly) {
			for (const map_file::DTLane& dtlane : dtlanes) {
				if (dtlane.did == lane.did)
					waypoint_dtlanes.push_back(dtlane);
			}

			return waypoint_dtlanes;
		}
	}
}

static std::vector<map_file::PointClass>
search_stopline_point(const nav_msgs::Path& msg)
{
	std::vector<map_file::PointClass> stopline_points;

	// msg's X-Y axis is reversed
	map_file::PointClass start_point = search_nearest(
		left_lane_points,
		msg.poses.front().pose.position.y,
		msg.poses.front().pose.position.x);

	// msg's X-Y axis is reversed
	map_file::PointClass end_point = search_nearest(
		left_lane_points,
		msg.poses.back().pose.position.y,
		msg.poses.back().pose.position.x);

	int lane_index = to_lane_index(start_point, nodes, lanes);
	if (lane_index < 0) {
		ROS_ERROR("start lane is not found");
		return stopline_points;
	}
	map_file::Lane lane = lanes[lane_index];

	int point_index = to_beginning_point_index(lane, nodes, points);
	if (point_index < 0) {
		ROS_ERROR("start beginning point is not found");
		return stopline_points;
	}
	map_file::PointClass point = points[point_index];

	while (1) {
		for (const map_file::StopLine& stopline : stoplines) {
			if (stopline.linkid == lane.lnid)
				stopline_points.push_back(point);
		}

		point_index = to_finishing_point_index(lane, nodes, points);
		if (point_index < 0) {
			ROS_ERROR("finishing point is not found");
			return stopline_points;
		}
		point = points[point_index];

		lane_index = to_next_lane_index(lane, lanes);
		if (lane_index < 0) {
			ROS_ERROR("next lane is not found");
			return stopline_points;
		}
		lane = lanes[lane_index];

		if (point.bx == end_point.bx &&
		    point.ly == end_point.ly) {
			for (const map_file::StopLine& stopline : stoplines) {
				if (stopline.linkid == lane.lnid)
					stopline_points.push_back(point);
			}

			return stopline_points;
		}
	}
}

static std::vector<int> search_stopline_index(const nav_msgs::Path& msg)
{
	std::vector<int> indexes;

	std::vector<map_file::PointClass> stopline_points =
		search_stopline_point(msg);
	for (const map_file::PointClass& point : stopline_points) {
		int i = 0;

		// msg's X-Y axis is reversed
		double min = hypot(point.bx - msg.poses[0].pose.position.y,
				   point.ly - msg.poses[0].pose.position.x);
		int index = i;
		for (const geometry_msgs::PoseStamped& stamped : msg.poses) {
			// msg's X-Y axis is reversed
			double distance = hypot(
				point.bx - stamped.pose.position.y,
				point.ly - stamped.pose.position.x);
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

static std::vector<double> compute_velocity(const nav_msgs::Path& msg,
					    double velocity, double difference,
					    int nzeros)
{
	std::vector<double> computations;
	int loops = msg.poses.size();

	std::vector<int> indexes = search_stopline_index(msg);

	if (indexes.empty() || difference < ACCIDENT_ERROR) {
		ROS_WARN_COND(difference < ACCIDENT_ERROR,
			      "too small difference");
		for (int i = 0; i < loops; ++i)
			computations.push_back(velocity);
		return computations;
	}

	int npaths = static_cast<int>(velocity / difference);

	std::vector<int>::const_iterator iter = indexes.cbegin();
	int start = *iter - npaths - (nzeros - 1);
	int end = *iter + npaths;
	for (int i = 0; i < loops; ++i) {
		double vel;
		if (i <= start)
			vel = velocity;
		else if (i <= (*iter - nzeros))
			vel = velocity - (difference * (i - start));
		else if (i <= *iter)
			vel = 0;
		else if (i <= (end - 1))
			vel = velocity - (difference * (end - i));
		else {
			vel = velocity;
			if ((iter + 1) != indexes.cend()) {
				++iter;
				start = *iter - npaths - (nzeros - 1);
				end = *iter + npaths;
			}
		}

		computations.push_back(vel);
	}

	return computations;
}

static void write_waypoint(const geometry_msgs::Point& point, double velocity,
			   double reduction, const char *filename, bool first)
{
	if (first) {
		std::ofstream ofs(filename);
		ofs << std::fixed << std::setprecision(PRECISION) << point.x
		    << ","
		    << std::fixed << std::setprecision(PRECISION) << point.y
		    << ","
		    << std::fixed << std::setprecision(PRECISION) << point.z
		    << std::endl;
	} else {
		std::ofstream ofs(filename, std::ios_base::app);
		ofs << std::fixed << std::setprecision(PRECISION) << point.x
		    << ","
		    << std::fixed << std::setprecision(PRECISION) << point.y
		    << ","
		    << std::fixed << std::setprecision(PRECISION) << point.z
		    << ","
		    << std::fixed << std::setprecision(PRECISION)
		    << (velocity * reduction)
		    << std::endl;
	}
}

static void lane_waypoint_callback(const nav_msgs::Path& msg)
{
	std_msgs::Header header;
	header.stamp = ros::Time::now();
	header.frame_id = "/map";

	visualization_msgs::MarkerArray velocities;
	visualization_msgs::Marker velocity;
	velocity.header = header;
	velocity.ns = "waypoint_velocity";

	if (waypoint_count > 0) {
		velocity.action = visualization_msgs::Marker::DELETE;
		for (int i = 0; i < waypoint_count; ++i) {
			velocity.id = i;
			velocities.markers.push_back(velocity);
		}
		pub_velocity.publish(velocities);
	}

	velocity.action = visualization_msgs::Marker::ADD;
	velocity.lifetime = ros::Duration();
	velocity.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	velocity.scale.z = 0.4;
	velocity.color.r = 1;
	velocity.color.a = 1;

	waypoint_follower::lane ruled;
	ruled.header = header;
	ruled.increment = 1;

	waypoint_follower::lane red;
	red.header = header;
	red.increment = 1;

	waypoint_follower::waypoint waypoint;
	waypoint.pose.header = header;
	waypoint.twist.header = header;
	waypoint.pose.pose.orientation.w = 1;

	std::vector<map_file::DTLane> waypoint_dtlanes =
		search_waypoint_dtlane(msg);
	if (waypoint_dtlanes.size() != msg.poses.size()) {
		ROS_ERROR("not enough dtlane");
		return;
	}

	std::vector<double> computations = compute_velocity(
		msg,
		config_velocity,
		config_difference_around_signal,
		config_number_of_zeros);

	waypoint_count = msg.poses.size();
	for (int i = 0; i < waypoint_count; ++i) {
		double reduction = dtlane_to_reduction(waypoint_dtlanes, i);

		velocity.id = i;
		velocity.pose.position = msg.poses[i].pose.position;
		velocity.pose.position.z += 0.2; // more visible

		std::ostringstream ostr;
		ostr << std::fixed << std::setprecision(0)
		     << (config_velocity * reduction) << " km/h";
		velocity.text = ostr.str();

		velocities.markers.push_back(velocity);

		waypoint.pose.pose.position = msg.poses[i].pose.position;

		waypoint.twist.twist.linear.x =
			(config_velocity * reduction) / 3.6; // to m/s
		waypoint.dtlane.dist = waypoint_dtlanes[i].dist;
		waypoint.dtlane.dir = waypoint_dtlanes[i].dir;
		waypoint.dtlane.apara = waypoint_dtlanes[i].apara;
		waypoint.dtlane.r = waypoint_dtlanes[i].r;
		waypoint.dtlane.slope = waypoint_dtlanes[i].slope;
		waypoint.dtlane.cant = waypoint_dtlanes[i].cant;
		waypoint.dtlane.lw = waypoint_dtlanes[i].lw;
		waypoint.dtlane.rw = waypoint_dtlanes[i].rw;
		ruled.waypoints.push_back(waypoint);

		write_waypoint(waypoint.pose.pose.position, config_velocity,
			       reduction, ruled_waypoint_csv.c_str(),
			       (i == 0));

		waypoint.twist.twist.linear.x =
			(computations[i] * reduction) / 3.6; // to m/s
		red.waypoints.push_back(waypoint);
	}

	pub_velocity.publish(velocities);
	pub_ruled.publish(ruled);
	pub_red.publish(red);
	pub_green.publish(ruled);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lane_rule");

	ros::NodeHandle n;
	n.param<std::string>("lane_rule/ruled_waypoint_csv",
			     ruled_waypoint_csv, RULED_WAYPOINT_CSV);

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
	ros::Subscriber sub_stop_line = n.subscribe(
		"vector_map_info/stop_line",
		SUBSCRIBE_QUEUE_SIZE,
		stop_line_callback);
	ros::Subscriber sub_dtlane = n.subscribe(
		"vector_map_info/dtlane",
		SUBSCRIBE_QUEUE_SIZE,
		dtlane_callback);

	ros::Rate rate(1);
	while (lanes.empty() || nodes.empty() || points.empty() ||
	       stoplines.empty() || dtlanes.empty()) {
		ros::spinOnce();
		rate.sleep();
	}

	ros::Subscriber sub_config = n.subscribe("config/lane_rule",
						 SUBSCRIBE_QUEUE_SIZE,
						 config_callback);
	ros::Subscriber sub_waypoint = n.subscribe("lane_waypoint",
						   SUBSCRIBE_QUEUE_SIZE,
						   lane_waypoint_callback);

	pub_velocity = n.advertise<visualization_msgs::MarkerArray>(
		"waypoint_velocity",
		ADVERTISE_QUEUE_SIZE,
		ADVERTISE_LATCH);
	pub_ruled = n.advertise<waypoint_follower::lane>(
		"traffic_waypoint",
		ADVERTISE_QUEUE_SIZE,
		ADVERTISE_LATCH);
	pub_red = n.advertise<waypoint_follower::lane>(
		"red_waypoint",
		ADVERTISE_QUEUE_SIZE,
		ADVERTISE_LATCH);
	pub_green = n.advertise<waypoint_follower::lane>(
		"green_waypoint",
		ADVERTISE_QUEUE_SIZE,
		ADVERTISE_LATCH);

	ros::spin();

	return 0;
}
