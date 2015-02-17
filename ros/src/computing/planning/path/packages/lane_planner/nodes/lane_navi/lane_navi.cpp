#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>

#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

#include <geo_pos_conv.hh>

#include <ui_socket/route_cmd.h>

// #define PUBLISH_TRAJECTORY

static constexpr uint32_t SUBSCRIBE_QUEUE_SIZE = 1000;

static constexpr uint32_t ADVERTISE_QUEUE_SIZE = 10;
static constexpr bool ADVERTISE_LATCH = true;

class Lane {
private:
	int lnid_;
	int did_;
	int blid_;
	int flid_;
	int bnid_;
	int fnid_;
	int jct_;
	int blid2_;
	int blid3_;
	int blid4_;
	int flid2_;
	int flid3_;
	int flid4_;
	int clossid_;
	double span_;
	int lcnt_;
	int lno_;

public:
	explicit Lane(int lnid, int did, int blid, int flid, int bnid,
		      int fnid, int jct, int blid2, int blid3, int blid4,
		      int flid2, int flid3, int flid4, int clossid,
		      double span, int lcnt, int lno);

	int lnid() const;
	int did() const;
	int blid() const;
	int flid() const;
	int bnid() const;
	int fnid() const;
	int jct() const;
	int blid2() const;
	int blid3() const;
	int blid4() const;
	int flid2() const;
	int flid3() const;
	int flid4() const;
	int clossid() const;
	double span() const;
	int lcnt() const;
	int lno() const;
};

int Lane::lnid() const
{
	return lnid_;
}

int Lane::did() const
{
	return did_;
}

int Lane::blid() const
{
	return blid_;
}

int Lane::flid() const
{
	return flid_;
}

int Lane::bnid() const
{
	return bnid_;
}

int Lane::fnid() const
{
	return fnid_;
}

int Lane::jct() const
{
	return jct_;
}

int Lane::blid2() const
{
	return blid2_;
}

int Lane::blid3() const
{
	return blid3_;
}

int Lane::blid4() const
{
	return blid4_;
}

int Lane::flid2() const
{
	return flid2_;
}

int Lane::flid3() const
{
	return flid3_;
}

int Lane::flid4() const
{
	return flid4_;
}

int Lane::clossid() const
{
	return clossid_;
}

double Lane::span() const
{
	return span_;
}

int Lane::lcnt() const
{
	return lcnt_;
}

int Lane::lno() const
{
	return lno_;
}

std::ostream& operator<<(std::ostream& out, const Lane& lane)
{
	out << "Lane(" << "\n"
	    << "\tlnid:    " << lane.lnid() << "\n"
	    << "\tdid:     " << lane.did() << "\n"
	    << "\tblid:    " << lane.blid() << "\n"
	    << "\tflid:    " << lane.flid() << "\n"
	    << "\tbnid:    " << lane.bnid() << "\n"
	    << "\tfnid:    " << lane.fnid() << "\n"
	    << "\tjct:     " << lane.jct() << "\n"
	    << "\tblid2:   " << lane.blid2() << "\n"
	    << "\tblid3:   " << lane.blid3() << "\n"
	    << "\tblid4:   " << lane.blid4() << "\n"
	    << "\tflid2:   " << lane.flid2() << "\n"
	    << "\tflid3:   " << lane.flid3() << "\n"
	    << "\tflid4:   " << lane.flid4() << "\n"
	    << "\tclossid: " << lane.clossid() << "\n"
	    << "\tspan:    " << std::setprecision(9) << lane.span() << "\n"
	    << "\tlcnt:    " << lane.lcnt() << "\n"
	    << "\tlno:     " << lane.lno() << "\n"
	    << ")";
	return out;
}

Lane::Lane(int lnid, int did, int blid, int flid, int bnid,
	   int fnid, int jct, int blid2, int blid3, int blid4,
	   int flid2, int flid3, int flid4, int clossid,
	   double span, int lcnt, int lno)
	: lnid_(lnid), did_(did), blid_(blid), flid_(flid), bnid_(bnid),
	  fnid_(fnid), jct_(jct), blid2_(blid2), blid3_(blid3), blid4_(blid4),
	  flid2_(flid2), flid3_(flid3), flid4_(flid4), clossid_(clossid),
	  span_(span), lcnt_(lcnt), lno_(lno)
{
}

class Node {
private:
	int nid_;
	int pid_;

public:
	explicit Node(int nid, int pid);

	int nid() const;
	int pid() const;
};

int Node::nid() const
{
	return nid_;
}

int Node::pid() const
{
	return pid_;
}

std::ostream& operator<<(std::ostream& out, const Node& node)
{
	out << "Node(" << "\n"
	    << "\tnid: " << node.nid() << "\n"
	    << "\tpid: " << node.pid() << "\n"
	    << ")";
	return out;
}

Node::Node(int nid, int pid)
	: nid_(nid), pid_(pid)
{
}

class Point {
private:
	int pid_;
	double b_;
	double l_;
	double h_;
	double bx_;
	double ly_;
	double ref_;
	int mcode1_;
	int mcode2_;
	int mcode3_;

public:
	explicit Point(int pid, double b, double l, double h, double bx,
		       double ly, double ref, int mcode1, int mcode2,
		       int mcode3);

	int pid() const;
	double b() const;
	double l() const;
	double h() const;
	double bx() const;
	double ly() const;
	double ref() const;
	int mcode1() const;
	int mcode2() const;
	int mcode3() const;
};

int Point::pid() const
{
	return pid_;
}

double Point::b() const
{
	return b_;
}

double Point::l() const
{
	return l_;
}

double Point::h() const
{
	return h_;
}

double Point::bx() const
{
	return bx_;
}

double Point::ly() const
{
	return ly_;
}

double Point::ref() const
{
	return ref_;
}

int Point::mcode1() const
{
	return mcode1_;
}

int Point::mcode2() const
{
	return mcode2_;
}

int Point::mcode3() const
{
	return mcode3_;
}

std::ostream& operator<<(std::ostream& out, const Point& point)
{
	out << "Point(" << "\n"
	    << "\tpid:    " << point.pid() << "\n"
	    << "\tb:      " << std::setprecision(9) << point.b() << "\n"
	    << "\tl:      " << std::setprecision(9) << point.l() << "\n"
	    << "\th:      " << std::setprecision(9) << point.h() << "\n"
	    << "\tbx:     " << std::setprecision(9) << point.bx() << "\n"
	    << "\tly:     " << std::setprecision(9) << point.ly() << "\n"
	    << "\tref:    " << std::setprecision(9) << point.ref() << "\n"
	    << "\tmcode1: " << point.mcode1() << "\n"
	    << "\tmcode2: " << point.mcode2() << "\n"
	    << "\tmcode3: " << point.mcode3() << "\n"
	    << ")";
	return out;
}

Point::Point(int pid, double b, double l, double h, double bx,
	     double ly, double ref, int mcode1, int mcode2,
	     int mcode3)
	: pid_(pid), b_(b), l_(l), h_(h), bx_(bx),
	  ly_(ly), ref_(ref), mcode1_(mcode1), mcode2_(mcode2),
	  mcode3_(mcode3)
{
}

static ros::Publisher pub_waypoint;
static ros::Publisher pub_centerline;
#ifdef PUBLISH_TRAJECTORY
static ros::Publisher pub_trajectory;
#endif /* PUBLISH_TRAJECTORY */

static std::vector<Lane> lanes;
static std::vector<Node> nodes;
static std::vector<Point> points;

static std::vector<Point> left_lane_points;

static Lane parse_lane(const std::string& line)
{
	std::istringstream ss(line);
	std::vector<std::string> columns;

	std::string column;
	while (std::getline(ss, column, ',')) {
		columns.push_back(column);
	}

	return Lane(
		std::stoi(columns[0]),
		std::stoi(columns[1]),
		std::stoi(columns[2]),
		std::stoi(columns[3]),
		std::stoi(columns[4]),
		std::stoi(columns[5]),
		std::stoi(columns[6]),
		std::stoi(columns[7]),
		std::stoi(columns[8]),
		std::stoi(columns[9]),
		std::stoi(columns[10]),
		std::stoi(columns[11]),
		std::stoi(columns[12]),
		std::stoi(columns[13]),
		std::stod(columns[14]),
		std::stoi(columns[15]),
		std::stoi(columns[16])
		);
}

static std::vector<Lane> read_lane(const char *filename)
{
	std::ifstream ifs(filename);
	std::string line;

	std::getline(ifs, line); // Remove first line

	std::vector<Lane> lanes;
	while (std::getline(ifs, line)) {
		lanes.push_back(parse_lane(line));
	}

	return lanes;
}

static Node parse_node(const std::string& line)
{
	std::istringstream ss(line);
	std::vector<std::string> columns;

	std::string column;
	while (std::getline(ss, column, ',')) {
		columns.push_back(column);
	}

	return Node(
		std::stoi(columns[0]),
		std::stoi(columns[1])
		);
}

static std::vector<Node> read_node(const char *filename)
{
	std::ifstream ifs(filename);
	std::string line;

	std::getline(ifs, line); // Remove first line

	std::vector<Node> nodes;
	while (std::getline(ifs, line)) {
		nodes.push_back(parse_node(line));
	}

	return nodes;
}

static Point parse_point(const std::string& line)
{
	std::istringstream ss(line);
	std::vector<std::string> columns;

	std::string column;
	while (std::getline(ss, column, ',')) {
		columns.push_back(column);
	}

	return Point(
		std::stoi(columns[0]),
		std::stod(columns[1]),
		std::stod(columns[2]),
		std::stod(columns[3]),
		std::stod(columns[4]),
		std::stod(columns[5]),
		std::stod(columns[6]),
		std::stoi(columns[7]),
		std::stoi(columns[8]),
		std::stoi(columns[9])
		);
}

static std::vector<Point> read_point(const char *filename)
{
	std::ifstream ifs(filename);
	std::string line;

	std::getline(ifs, line); // Remove first line

	std::vector<Point> points;
	while (std::getline(ifs, line)) {
		points.push_back(parse_point(line));
	}

	return points;
}

static Point search_nearest(const std::vector<Point>& points,
			    double x, double y)
{
	Point nearest_point = points[0];
	double min = hypot(x - points[0].bx(), y - points[0].ly());
	for (const Point& point : points) {
		double distance = hypot(x - point.bx(), y - point.ly());
		if (distance < min) {
			min = distance;
			nearest_point = point;
		}
	}

	return nearest_point;
}

static int point_to_lane_index(const Point& point)
{
	for (const Node& node : nodes) {
		if (node.pid() != point.pid())
			continue;
		for (int i = 0; i < static_cast<int>(lanes.size()); i++) {
			if (lanes[i].bnid() != node.nid())
				continue;
			return i;
		}
	}

	return -1;
}

static int lane_to_next_lane_index(const Lane& lane)
{
	for (int i = 0; i < static_cast<int>(lanes.size()); i++) {
		if (lanes[i].lnid() != lane.flid())
			continue;
		return i;
	}

	return -1;
}

static int lane_to_beginning_point_index(const Lane& lane)
{
	for (const Node& node : nodes) {
		if (node.nid() != lane.bnid())
			continue;
		for (int i = 0; i < static_cast<int>(points.size()); i++) {
			if (points[i].pid() != node.pid())
				continue;
			return i;
		}
	}

	return -1;
}

static int lane_to_finishing_point_index(const Lane& lane)
{
	for (const Node& node : nodes) {
		if (node.nid() != lane.fnid())
			continue;
		for (int i = 0; i < static_cast<int>(points.size()); i++) {
			if (points[i].pid() != node.pid())
				continue;
			return i;
		}
	}

	return -1;
}

static void route_cmd_callback(const ui_socket::route_cmd msg)
{
	geo_pos_conv geo;
	geo.set_plane(7);

	geo.llh_to_xyz(msg.point.front().lat, msg.point.front().lon, 0);
	Point start_point = search_nearest(left_lane_points, geo.x(), geo.y());

	geo.llh_to_xyz(msg.point.back().lat, msg.point.back().lon, 0);
	Point end_point = search_nearest(left_lane_points, geo.x(), geo.y());

	int lane_index = point_to_lane_index(start_point);
	if (lane_index < 0) {
		ROS_ERROR("start lane is not found");
		return;
	}
	Lane lane = lanes[lane_index];

	int point_index = lane_to_beginning_point_index(lane);
	if (point_index < 0) {
		ROS_ERROR("start beginning point is not found");
		return;
	}
	Point point = points[point_index];

	std_msgs::Header header;
	header.stamp = ros::Time::now();
	header.frame_id = "/map";

	nav_msgs::Path waypoint;
	waypoint.header = header;

	visualization_msgs::Marker centerline;
	centerline.header = header;
	centerline.ns = "centerline";
	centerline.id = 0;
	centerline.action = visualization_msgs::Marker::ADD;
	centerline.lifetime = ros::Duration();
	centerline.type = visualization_msgs::Marker::POINTS;
	centerline.scale.x = 0.1;
	centerline.scale.y = 0.1;
	centerline.color.r = 1;
	centerline.color.a = 1;

	geometry_msgs::PoseStamped posestamped;
	posestamped.header = header;
	posestamped.pose.orientation.w = 1;

	while (1) {
		posestamped.pose.position.x = point.ly();
		posestamped.pose.position.y = point.bx();
		posestamped.pose.position.z = point.h();

		waypoint.poses.push_back(posestamped);
		centerline.points.push_back(posestamped.pose.position);

		point_index = lane_to_finishing_point_index(lane);
		if (point_index < 0) {
			ROS_ERROR("finishing point is not found");
			return;
		}
		point = points[point_index];

		if (point.bx() == end_point.bx() &&
		    point.ly() == end_point.ly()) {
			posestamped.pose.position.x = point.ly();
			posestamped.pose.position.y = point.bx();
			posestamped.pose.position.z = point.h();

			waypoint.poses.push_back(posestamped);
			centerline.points.push_back(posestamped.pose.position);

			break;
		}

		lane_index = lane_to_next_lane_index(lane);
		if (lane_index < 0) {
			ROS_ERROR("next lane is not found");
			return;
		}
		lane = lanes[lane_index];

		point_index = lane_to_beginning_point_index(lane);
		if (point_index < 0) {
			ROS_ERROR("beginning point is not found");
			return;
		}
		point = points[point_index];
	}

	pub_waypoint.publish(waypoint);
	pub_centerline.publish(centerline);

#ifdef PUBLISH_TRAJECTORY
	visualization_msgs::Marker trajectory;
	trajectory.header = header;
	trajectory.ns = "trajectory";
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

	for (const geometry_msgs::Point& position : centerline.points) {
		trajectory.pose.position = position;
		pub_trajectory.publish(trajectory);
		sleep(1);
	}
#endif /* PUBLISH_TRAJECTORY */
}

int main(int argc, char **argv)
{
	if (argc < 4) {
		ROS_ERROR_STREAM("Usage: " << argv[0]
				 << " lane.csv node.csv point.csv");
		std::exit(1);
	}

	const char *lane_csv = static_cast<const char *>(argv[1]);
	const char *node_csv = static_cast<const char *>(argv[2]);
	const char *point_csv = static_cast<const char *>(argv[3]);

	lanes = read_lane(lane_csv);
	nodes = read_node(node_csv);
	points = read_point(point_csv);

	for (const Lane& lane : lanes) {
		if (lane.lno() != 1)
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

	ros::init(argc, argv, "lane_navi");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("route_cmd",
					  SUBSCRIBE_QUEUE_SIZE,
					  route_cmd_callback);

	pub_waypoint = n.advertise<nav_msgs::Path>("/lane_waypoint",
						   ADVERTISE_QUEUE_SIZE,
						   ADVERTISE_LATCH);
	pub_centerline = n.advertise<visualization_msgs::Marker>(
		"/lane_centerline",
		ADVERTISE_QUEUE_SIZE,
		ADVERTISE_LATCH);
#ifdef PUBLISH_TRAJECTORY
	pub_trajectory = n.advertise<visualization_msgs::Marker>(
		"/_trajectory",
		ADVERTISE_QUEUE_SIZE,
		ADVERTISE_LATCH);
#endif /* PUBLISH_TRAJECTORY */

	ros::spin();

	return 0;
}
