#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <lane_follower/lane.h>

static constexpr uint32_t SUBSCRIBE_QUEUE_SIZE = 1000;

static constexpr uint32_t ADVERTISE_QUEUE_SIZE = 10;
static constexpr bool ADVERTISE_LATCH = true;

static double config_velocity = 40; // Unit: km/h

static ros::Publisher pub_velocity;
static ros::Publisher pub_ruled;
static ros::Publisher pub_stop;

static uint32_t waypoint_count;

static void lane_waypoint_callback(const nav_msgs::Path& msg)
{
	std_msgs::Header header;
	header.stamp = ros::Time::now();
	header.frame_id = "/map";

	ros::Rate rate(1000);

	visualization_msgs::Marker velocity;
	velocity.header = header;
	velocity.ns = "velocity";

	if (waypoint_count > 0) {
		velocity.action = visualization_msgs::Marker::DELETE;
		for (uint32_t i = 0; i < waypoint_count; ++i) {
			velocity.id = i;
			pub_velocity.publish(velocity);
			rate.sleep();
		}
	}

	velocity.id = 0;
	velocity.action = visualization_msgs::Marker::ADD;
	velocity.lifetime = ros::Duration();
	velocity.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	velocity.scale.z = 0.4;
	velocity.color.r = 1;
	velocity.color.a = 1;

	std::ostringstream ostr;
	ostr << std::fixed << std::setprecision(0) << config_velocity
	     << "km/h";
	velocity.text = ostr.str();

	for (const geometry_msgs::PoseStamped& posestamped : msg.poses) {
		velocity.pose.position = posestamped.pose.position;
		velocity.pose.position.z += 0.2;
		pub_velocity.publish(velocity);
		++velocity.id;
		rate.sleep();
	}

	waypoint_count = velocity.id;

	lane_follower::lane ruled;
	pub_ruled.publish(ruled);

	lane_follower::lane stop;
	pub_stop.publish(stop);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lane_rule");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("lane_waypoint",
					  SUBSCRIBE_QUEUE_SIZE,
					  lane_waypoint_callback);

	pub_velocity = n.advertise<visualization_msgs::Marker>(
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
