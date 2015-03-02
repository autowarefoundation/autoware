#include <ros/ros.h>
#include <ros/console.h>

#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <lane_follower/lane.h>

#include <vmap_parser.h>

static constexpr uint32_t SUBSCRIBE_QUEUE_SIZE = 1000;

static constexpr uint32_t ADVERTISE_QUEUE_SIZE = 1000;
static constexpr bool ADVERTISE_LATCH = true;

static double config_velocity = 40; // Unit: km/h
static double config_difference = 2; // Unit: km/h

static ros::Publisher pub_velocity;
static ros::Publisher pub_ruled;
static ros::Publisher pub_stop;

static std::vector<Lane> lanes;
static std::vector<Node> nodes;
static std::vector<Point> points;
static std::vector<Signal> signals;

static std::vector<Point> left_lane_points;

static int waypoint_count;

static std::vector<Point> search_signal_point(const nav_msgs::Path& msg)
{
	std::vector<Point> signal_points;

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
		return signal_points;
	}
	Lane lane = lanes[lane_index];

	int point_index = lane.to_beginning_point_index(nodes, points);
	if (point_index < 0) {
		ROS_ERROR("start beginning point is not found");
		return signal_points;
	}
	Point point = points[point_index];

	while (1) {
		for (const Signal& signal : signals) {
			if (signal.type() == 1 && // red signal
			    signal.linkid() == lane.lnid())
				signal_points.push_back(point);
		}

		point_index = lane.to_finishing_point_index(nodes, points);
		if (point_index < 0) {
			ROS_ERROR("finishing point is not found");
			return signal_points;
		}
		point = points[point_index];

		if (point.bx() == end_point.bx() &&
		    point.ly() == end_point.ly()) {
			for (const Signal& signal : signals) {
				if (signal.type() == 1 && // red signal
				    signal.linkid() == lane.lnid())
					signal_points.push_back(point);
			}

			return signal_points;
		}

		lane_index = lane.to_next_lane_index(lanes);
		if (lane_index < 0) {
			ROS_ERROR("next lane is not found");
			return signal_points;
		}
		lane = lanes[lane_index];

		point_index = lane.to_beginning_point_index(nodes, points);
		if (point_index < 0) {
			ROS_ERROR("beginning point is not found");
			return signal_points;
		}
		point = points[point_index];
	}
}

static std::vector<int> search_signal_index(const nav_msgs::Path& msg)
{
	std::vector<int> indexes;

	std::vector<Point> signal_points = search_signal_point(msg);
	for (const Point& point : signal_points) {
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
					    const double& velocity,
					    const double& difference)
{
	std::vector<double> computations;
	int loops = msg.poses.size();

	std::vector<int> indexes = search_signal_index(msg);
	if (indexes.size() == 0) {
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

	velocity.id = 0;
	velocity.action = visualization_msgs::Marker::ADD;
	velocity.lifetime = ros::Duration();
	velocity.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	velocity.scale.z = 0.4;
	velocity.color.r = 1;
	velocity.color.a = 1;

	lane_follower::lane ruled;
	ruled.header = header;

	lane_follower::lane stop;
	stop.header = header;

	lane_follower::waypoint waypoint;
	waypoint.pose.header = header;
	waypoint.twist.header = header;
	waypoint.pose.pose.orientation.w = 1;

	std::vector<double> computations =
		compute_velocity(msg, config_velocity, config_difference);

	waypoint_count = msg.poses.size();
	for (int i = 0; i < waypoint_count; ++i) {
		velocity.pose.position = msg.poses[i].pose.position;
		velocity.pose.position.z += 0.2; // more visible

		std::ostringstream ostr;
		ostr << std::fixed << std::setprecision(0) << computations[i]
		     << " km/h";
		velocity.text = ostr.str();

		velocities.markers.push_back(velocity);
		++velocity.id;

		waypoint.pose.pose.position = msg.poses[i].pose.position;

		waypoint.twist.twist.linear.x =
			config_velocity / 3.6; // to m/s
		ruled.waypoints.push_back(waypoint);

		waypoint.twist.twist.linear.x =
			computations[i] / 3.6; // to m/s
		stop.waypoints.push_back(waypoint);
	}

	pub_velocity.publish(velocities);
	pub_ruled.publish(ruled);
	pub_stop.publish(stop);
}

int main(int argc, char **argv)
{
	if (argc < 5) {
		ROS_ERROR_STREAM("Usage: " << argv[0]
				 << " lane.csv node.csv point.csv signal.csv");
		std::exit(1);
	}

	const char *lane_csv = static_cast<const char *>(argv[1]);
	const char *node_csv = static_cast<const char *>(argv[2]);
	const char *point_csv = static_cast<const char *>(argv[3]);
	const char *signal_csv = static_cast<const char *>(argv[4]);

	lanes = read_lane(lane_csv);
	nodes = read_node(node_csv);
	points = read_point(point_csv);
	signals = read_signal(signal_csv);

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

	ros::init(argc, argv, "lane_rule");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("lane_waypoint",
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
	pub_stop = n.advertise<lane_follower::lane>(
		"stop_waypoint",
		ADVERTISE_QUEUE_SIZE,
		ADVERTISE_LATCH);

	ros::spin();

	return 0;
}
