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
#include <lane_follower/lane.h>
#include <runtime_manager/ConfigLaneRule.h>

#include <vmap_parser.h>

static constexpr uint32_t SUBSCRIBE_QUEUE_SIZE = 1000;

static constexpr uint32_t ADVERTISE_QUEUE_SIZE = 1000;
static constexpr bool ADVERTISE_LATCH = true;

static constexpr int PRECISION = 6;
static constexpr double ACCIDENT_ERROR = 0.000001;

static const std::string VECTOR_MAP_DIRECTORY = "/tmp";
static const std::string RULED_WAYPOINT_CSV = "/tmp/ruled_waypoint.csv";

static double config_velocity = 40; // Unit: km/h
static double config_difference_around_signal = 2; // Unit: km/h

static ros::Publisher pub_velocity;
static ros::Publisher pub_ruled;
static ros::Publisher pub_red;
static ros::Publisher pub_green;

static std::vector<Lane> lanes;
static std::vector<Node> nodes;
static std::vector<Point> points;
static std::vector<StopLine> stoplines;

static std::vector<Point> left_lane_points;

static int waypoint_count;

static std::string ruled_waypoint_csv;

static void config_callback(const runtime_manager::ConfigLaneRule& msg)
{
	config_velocity = msg.velocity;
	config_difference_around_signal = msg.difference_around_signal;
}

static std::vector<Point> search_stopline_point(const nav_msgs::Path& msg)
{
	std::vector<Point> stopline_points;

	// msg's X-Y axis is reversed
	Point start_point = search_nearest(
		left_lane_points,
		msg.poses.front().pose.position.y,
		msg.poses.front().pose.position.x);

	// msg's X-Y axis is reversed
	Point end_point = search_nearest(
		left_lane_points,
		msg.poses.back().pose.position.y,
		msg.poses.back().pose.position.x);

	int lane_index = start_point.to_lane_index(nodes, lanes);
	if (lane_index < 0) {
		ROS_ERROR("start lane is not found");
		return stopline_points;
	}
	Lane lane = lanes[lane_index];

	int point_index = lane.to_beginning_point_index(nodes, points);
	if (point_index < 0) {
		ROS_ERROR("start beginning point is not found");
		return stopline_points;
	}
	Point point = points[point_index];

	while (1) {
		for (const StopLine& stopline : stoplines) {
			if (stopline.linkid() == lane.lnid())
				stopline_points.push_back(point);
		}

		point_index = lane.to_finishing_point_index(nodes, points);
		if (point_index < 0) {
			ROS_ERROR("finishing point is not found");
			return stopline_points;
		}
		point = points[point_index];

		if (point.bx() == end_point.bx() &&
		    point.ly() == end_point.ly()) {
			for (const StopLine& stopline : stoplines) {
				if (stopline.linkid() == lane.lnid())
					stopline_points.push_back(point);
			}

			return stopline_points;
		}

		lane_index = lane.to_next_lane_index(lanes);
		if (lane_index < 0) {
			ROS_ERROR("next lane is not found");
			return stopline_points;
		}
		lane = lanes[lane_index];

		point_index = lane.to_beginning_point_index(nodes, points);
		if (point_index < 0) {
			ROS_ERROR("beginning point is not found");
			return stopline_points;
		}
		point = points[point_index];
	}
}

static std::vector<int> search_stopline_index(const nav_msgs::Path& msg)
{
	std::vector<int> indexes;

	std::vector<Point> stopline_points = search_stopline_point(msg);
	for (const Point& point : stopline_points) {
		int i = 0;

		// msg's X-Y axis is reversed
		double min = hypot(point.bx() - msg.poses[0].pose.position.y,
				   point.ly() - msg.poses[0].pose.position.x);
		int index = i;
		for (const geometry_msgs::PoseStamped& stamped : msg.poses) {
			// msg's X-Y axis is reversed
			double distance = hypot(
				point.bx() - stamped.pose.position.y,
				point.ly() - stamped.pose.position.x);
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
					    double velocity, double difference)
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
	int start = *iter - npaths;
	int end = *iter + npaths;
	for (int i = 0; i < loops; ++i) {
		double vel;
		if (i <= start)
			vel = velocity;
		else if (start < i && i < *iter)
			vel = velocity - (difference * (i - start));
		else if (i == *iter)
			vel = 0;
		else if (*iter < i && i < end)
			vel = velocity - (difference * (end - i));
		else {
			vel = velocity;
			if ((iter + 1) != indexes.cend()) {
				++iter;
				start = *iter - npaths;
				end = *iter + npaths;
			}
		}

		computations.push_back(vel);
	}

	return computations;
}

static void write_waypoint(const geometry_msgs::Point& point, double velocity,
			   const char *filename, bool first)
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
		    << std::fixed << std::setprecision(PRECISION) << velocity
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
	velocity.ns = "velocity";

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

	lane_follower::lane ruled;
	ruled.header = header;
	ruled.increment = 1;

	lane_follower::lane red;
	red.header = header;
	red.increment = 1;

	lane_follower::waypoint waypoint;
	waypoint.pose.header = header;
	waypoint.twist.header = header;
	waypoint.pose.pose.orientation.w = 1;

	std::vector<double> computations = compute_velocity(
		msg,
		config_velocity,
		config_difference_around_signal);

	waypoint_count = msg.poses.size();
	for (int i = 0; i < waypoint_count; ++i) {
		velocity.id = i;
		velocity.pose.position = msg.poses[i].pose.position;
		velocity.pose.position.z += 0.2; // more visible

		std::ostringstream ostr;
		ostr << std::fixed << std::setprecision(0) << config_velocity
		     << " km/h";
		velocity.text = ostr.str();

		velocities.markers.push_back(velocity);

		waypoint.pose.pose.position = msg.poses[i].pose.position;

		waypoint.twist.twist.linear.x =
			config_velocity / 3.6; // to m/s
		ruled.waypoints.push_back(waypoint);

		write_waypoint(waypoint.pose.pose.position, config_velocity,
			       ruled_waypoint_csv.c_str(), (i == 0));

		waypoint.twist.twist.linear.x =
			computations[i] / 3.6; // to m/s
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

	std::string vector_map_directory;

	ros::NodeHandle n;
	n.param<std::string>("lane_rule/vector_map_directory",
			     vector_map_directory, VECTOR_MAP_DIRECTORY);
	n.param<std::string>("lane_rule/ruled_waypoint_csv",
			     ruled_waypoint_csv, RULED_WAYPOINT_CSV);

	lanes = read_lane((vector_map_directory +
			   std::string("/lane.csv")).c_str());
	nodes = read_node((vector_map_directory +
			   std::string("/node.csv")).c_str());
	points = read_point((vector_map_directory +
			     std::string("/point.csv")).c_str());
	stoplines = read_stopline((vector_map_directory +
			       std::string("/stopline.csv")).c_str());

	for (const Lane& lane : lanes) {
		if (lane.lno() != 1) // leftmost lane
			continue;
		for (const Node& node : nodes) {
			if (node.nid() != lane.bnid() &&
			    node.nid() != lane.fnid())
				continue;
			for (const Point& point : points) {
				if (point.pid() != node.pid())
					continue;
				left_lane_points.push_back(point);
			}
		}
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
	pub_ruled = n.advertise<lane_follower::lane>(
		"ruled_waypoint",
		ADVERTISE_QUEUE_SIZE,
		ADVERTISE_LATCH);
	pub_red = n.advertise<lane_follower::lane>(
		"red_waypoint",
		ADVERTISE_QUEUE_SIZE,
		ADVERTISE_LATCH);
	pub_green = n.advertise<lane_follower::lane>(
		"green_waypoint",
		ADVERTISE_QUEUE_SIZE,
		ADVERTISE_LATCH);

	ros::spin();

	return 0;
}
