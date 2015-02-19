#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <lane_follower/lane.h>

static constexpr uint32_t SUBSCRIBE_QUEUE_SIZE = 1000;

static constexpr uint32_t ADVERTISE_QUEUE_SIZE = 10;
static constexpr bool ADVERTISE_LATCH = true;

static constexpr double DEFAULT_VELOCITY = 40; // Unit: km/h
static constexpr double DEFAULT_VELOCITY_DIFFERENCE = 2; // Unit: km/h

static ros::Publisher pub_velocity;
static ros::Publisher pub_ruled;
static ros::Publisher pub_stop;

static void lane_waypoint_callback(const nav_msgs::Path msg)
{
	// double vel = DEFAULT_VELOCITY;
	// double vel_diff = DEFAULT_VELOCITY_DEIFFERENCE;

	visualization_msgs::Marker velocity;
	pub_velocity.publish(velocity);

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
