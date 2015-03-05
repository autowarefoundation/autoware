#include <ros/ros.h>
#include <ros/console.h>

#include <lane_follower/lane.h>
#include <runtime_manager/traffic_light.h>

static constexpr uint32_t SUBSCRIBE_QUEUE_SIZE = 1000;

static constexpr uint32_t ADVERTISE_QUEUE_SIZE = 1000;
static constexpr bool ADVERTISE_LATCH = true;

static constexpr int32_t TRAFFIC_LIGHT_RED = 0;
static constexpr int32_t TRAFFIC_LIGHT_GREEN = 1;

static ros::Publisher pub_ruled;

static lane_follower::lane current_red_lane;
static lane_follower::lane current_green_lane;

static void red_waypoint_callback(const lane_follower::lane& msg)
{
	current_red_lane = msg;
}

static void green_waypoint_callback(const lane_follower::lane& msg)
{
	current_green_lane = msg;
}

static void traffic_light_callback(const runtime_manager::traffic_light& msg)
{
	const lane_follower::lane *current;

	switch (msg.traffic_light) {
	case TRAFFIC_LIGHT_RED:
		current = &current_red_lane;
		break;
	case TRAFFIC_LIGHT_GREEN:
		current = &current_green_lane;
		break;
	default:
		ROS_ERROR("unknown traffic_light");
		return;
	}

	if (current->waypoints.empty()) {
		ROS_ERROR("empty waypoints");
		return;
	}

	pub_ruled.publish(*current);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lane_stop");

	ros::NodeHandle n;

	ros::Subscriber sub_red = n.subscribe("red_waypoint",
					      SUBSCRIBE_QUEUE_SIZE,
					      red_waypoint_callback);
	ros::Subscriber sub_green = n.subscribe("green_waypoint",
						SUBSCRIBE_QUEUE_SIZE,
						green_waypoint_callback);
	ros::Subscriber sub_light = n.subscribe("traffic_light",
						SUBSCRIBE_QUEUE_SIZE,
						traffic_light_callback);

	pub_ruled = n.advertise<lane_follower::lane>(
		"ruled_waypoint",
		ADVERTISE_QUEUE_SIZE,
		ADVERTISE_LATCH);

	ros::spin();

	return 0;
}
