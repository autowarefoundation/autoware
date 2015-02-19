#include <ros/ros.h>
#include <ros/console.h>

#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <ui_socket/route_cmd.h>

#include <geo_pos_conv.hh>

#include <vmap_parser.h>

// #define PUBLISH_TRAJECTORY

static constexpr uint32_t SUBSCRIBE_QUEUE_SIZE = 1000;

static constexpr uint32_t ADVERTISE_QUEUE_SIZE = 1000;
static constexpr bool ADVERTISE_LATCH = false;

static ros::Publisher pub_waypoint;
static ros::Publisher pub_centerline;
#ifdef PUBLISH_TRAJECTORY
static ros::Publisher pub_trajectory;
#endif /* PUBLISH_TRAJECTORY */

static std::vector<Lane> lanes;
static std::vector<Node> nodes;
static std::vector<Point> points;

static std::vector<Point> left_lane_points;

static void route_cmd_callback(const ui_socket::route_cmd& msg)
{
	geo_pos_conv geo;
	geo.set_plane(7);

	geo.llh_to_xyz(msg.point.front().lat, msg.point.front().lon, 0);
	Point start_point = search_nearest(left_lane_points, geo.x(), geo.y());

	geo.llh_to_xyz(msg.point.back().lat, msg.point.back().lon, 0);
	Point end_point = search_nearest(left_lane_points, geo.x(), geo.y());

	int lane_index = start_point.to_lane_index(nodes, lanes);
	if (lane_index < 0) {
		ROS_ERROR("start lane is not found");
		return;
	}
	Lane lane = lanes[lane_index];

	int point_index = lane.to_beginning_point_index(nodes, points);
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

		point_index = lane.to_finishing_point_index(nodes, points);
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

		lane_index = lane.to_next_lane_index(lanes);
		if (lane_index < 0) {
			ROS_ERROR("next lane is not found");
			return;
		}
		lane = lanes[lane_index];

		point_index = lane.to_beginning_point_index(nodes, points);
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

	ros::Rate rate(1);
	for (const geometry_msgs::Point& position : centerline.points) {
		trajectory.pose.position = position;
		pub_trajectory.publish(trajectory);
		rate.sleep();
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

	pub_waypoint = n.advertise<nav_msgs::Path>(
		"lane_waypoint",
		ADVERTISE_QUEUE_SIZE,
		ADVERTISE_LATCH);
	pub_centerline = n.advertise<visualization_msgs::Marker>(
		"lane_centerline",
		ADVERTISE_QUEUE_SIZE,
		ADVERTISE_LATCH);
#ifdef PUBLISH_TRAJECTORY
	pub_trajectory = n.advertise<visualization_msgs::Marker>(
		"_trajectory",
		ADVERTISE_QUEUE_SIZE,
		ADVERTISE_LATCH);
#endif /* PUBLISH_TRAJECTORY */

	ros::spin();

	return 0;
}
