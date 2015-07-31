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

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <waypoint_follower/lane.h>
#include <runtime_manager/traffic_light.h>

static constexpr uint32_t SUBSCRIBE_QUEUE_SIZE = 1000;

static constexpr uint32_t ADVERTISE_QUEUE_SIZE = 1000;
static constexpr bool ADVERTISE_LATCH = true;

static constexpr int32_t TRAFFIC_LIGHT_RED     = 0;
static constexpr int32_t TRAFFIC_LIGHT_GREEN   = 1;
static constexpr int32_t TRAFFIC_LIGHT_UNKNOWN = 2;

static ros::Publisher pub_ruled;
static ros::Publisher pub_velocity;
static ros::Publisher _lane_mark_pub;

static waypoint_follower::lane current_red_lane;
static waypoint_follower::lane current_green_lane;

std::string PATH_FRAME = "/map";

void createWaypointVelocity(std::vector<waypoint_follower::waypoint> waypoints, visualization_msgs::MarkerArray *marker_array){

  // display by markers the velocity of each waypoint.
    visualization_msgs::Marker velocity;
    velocity.header.frame_id = PATH_FRAME;
    velocity.header.stamp = ros::Time();
    velocity.ns = "waypoint_velocity";
    velocity.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    velocity.action = visualization_msgs::Marker::ADD;
    velocity.scale.z = 0.4;
    velocity.color.a = 1.0;
    velocity.color.r = 1;
    velocity.color.g = 1;
    velocity.color.b = 1;
    velocity.frame_locked = true;

    for (unsigned int i = 0; i < waypoints.size(); i++) {

        //std::cout << _waypoints[i].GetX() << " " << _waypoints[i].GetY() << " " << _waypoints[i].GetZ() << " " << _waypoints[i].GetVelocity_kmh() << std::endl;
        velocity.id = i;
        velocity.pose.position = waypoints[i].pose.pose.position;
        velocity.pose.position.z += 0.2;
        velocity.pose.orientation.w = 1.0;

        // double to string
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(0) << waypoints[i].twist.twist.linear.x * 3.6 << " km/h";
        velocity.text = oss.str();

        //C++11 version
        //std::string velocity = std::to_string(test_pose.velocity_kmh);
        //velocity.erase(velocity.find_first_of(".") + 3);
        //std::string kmh = " km/h";
        //std::string text = velocity + kmh;
        //marker.text = text;

        marker_array->markers.push_back(velocity);
    }
}

void createWaypointMark(std::vector<waypoint_follower::waypoint> waypoints, visualization_msgs::MarkerArray *marker_array)
{
    visualization_msgs::Marker waypoint_mark;
    waypoint_mark.header.frame_id = PATH_FRAME;
    waypoint_mark.header.stamp = ros::Time();
    waypoint_mark.ns = "waypoint_mark";
    waypoint_mark.id = 0;
    waypoint_mark.type = visualization_msgs::Marker::POINTS;
    waypoint_mark.action = visualization_msgs::Marker::ADD;
    waypoint_mark.scale.x = 0.1;
    waypoint_mark.scale.y = 0.1;
    waypoint_mark.color.r = 1.0;
    waypoint_mark.color.g = 1.0;
    waypoint_mark.color.b = 1.0;
    waypoint_mark.color.a = 1.0;
    waypoint_mark.frame_locked = true;

    for (unsigned int i = 0; i < waypoints.size(); i++) {
        geometry_msgs::Point point;
        point = waypoints[i].pose.pose.position;
        waypoint_mark.points.push_back(point);

    }
    marker_array->markers.push_back(waypoint_mark);
}

void createLaneWaypointMarker(int signal, std::vector<waypoint_follower::waypoint> waypoints, visualization_msgs::MarkerArray *marker_array)
{
    visualization_msgs::Marker lane_waypoint_marker;
    lane_waypoint_marker.header.frame_id = PATH_FRAME;
    lane_waypoint_marker.header.stamp = ros::Time();
    lane_waypoint_marker.ns = "lane_waypoint_marker";
    lane_waypoint_marker.id = 0;
    lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
    lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
    lane_waypoint_marker.scale.x = 0.2;
    lane_waypoint_marker.pose.orientation.w = 1.0;

    if (signal == TRAFFIC_LIGHT_GREEN)
            lane_waypoint_marker.color.g = 1.0;
        else if (signal == TRAFFIC_LIGHT_RED)
            lane_waypoint_marker.color.r = 1.0;
        else
            lane_waypoint_marker.color.b = 1.0;
    lane_waypoint_marker.color.a = 1.0;
    lane_waypoint_marker.frame_locked = true;

    for (unsigned int i = 0; i < waypoints.size(); i++) {
        geometry_msgs::Point point;
        point = waypoints[i].pose.pose.position;
        lane_waypoint_marker.points.push_back(point);

    }
    marker_array->markers.push_back(lane_waypoint_marker);
}


static void red_waypoint_callback(const waypoint_follower::lane& msg)
{
	current_red_lane = msg;
}

static void green_waypoint_callback(const waypoint_follower::lane& msg)
{
	current_green_lane = msg;
}

static void traffic_light_callback(const runtime_manager::traffic_light& msg)
{
	const  waypoint_follower::lane *current;
	static waypoint_follower::lane prev_path = current_red_lane;
	static int signal;
	static int prev_signal;
	switch (msg.traffic_light) {
	case TRAFFIC_LIGHT_RED:
		current = &current_red_lane;
		signal = TRAFFIC_LIGHT_RED;
		break;
	case TRAFFIC_LIGHT_GREEN:
		current = &current_green_lane;
		signal = TRAFFIC_LIGHT_GREEN;
		break;
	case TRAFFIC_LIGHT_UNKNOWN:
        current = &prev_path;     // if traffic light state is unknown, keep previous state
        signal = prev_signal;;
		break;
	default:
		ROS_ERROR("unknown traffic_light");
		return;
	}

	if (current->waypoints.empty()) {
		ROS_ERROR("empty waypoints");
		return;
	}


	visualization_msgs::MarkerArray lane_waypoint;
	     createWaypointVelocity(current->waypoints,&lane_waypoint);
	     createWaypointMark(current->waypoints,&lane_waypoint);
	     createLaneWaypointMarker(signal,current->waypoints,&lane_waypoint);
	     _lane_mark_pub.publish(lane_waypoint);
/*
	visualization_msgs::MarkerArray velocities;
	visualization_msgs::Marker velocity;
	velocity.header.stamp = ros::Time::now();
	velocity.header.frame_id = "/map";
	velocity.ns = "waypoint_velocity";
	velocity.action = visualization_msgs::Marker::ADD;
	velocity.lifetime = ros::Duration();
	velocity.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	velocity.scale.z = 0.4;
	velocity.color.r = 1;
	velocity.color.g = 1;
	velocity.color.b = 1;
	velocity.color.a = 1;
	velocity.frame_locked = true;

	visualization_msgs::Marker lane_waypoint_marker;
    lane_waypoint_marker.header.frame_id = "/map";
    lane_waypoint_marker.header.stamp = ros::Time();
    lane_waypoint_marker.ns = "lane_waypoint_marker";
    lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
    lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
    lane_waypoint_marker.scale.x = 0.2;
    lane_waypoint_marker.pose.orientation.w = 1.0;
    lane_waypoint_marker.color.a = 1.0;
    lane_waypoint_marker.frame_locked = true;

    if (msg.traffic_light == TRAFFIC_LIGHT_GREEN)
        lane_waypoint_marker.color.g = 1.0;
    else if (msg.traffic_light == TRAFFIC_LIGHT_RED)
        lane_waypoint_marker.color.r = 1.0;
    else
        lane_waypoint_marker.color.b = 1.0;

	int i = 0;
	for (const waypoint_follower::waypoint& waypoint : current->waypoints) {

	    //for waypoint_velocity
	    velocity.id = i;
		velocity.pose.position = waypoint.pose.pose.position;
		velocity.pose.position.z += 0.2; // more visible

		std::ostringstream ostr;
		ostr << std::fixed << std::setprecision(0)
		     << (waypoint.twist.twist.linear.x * 3.6) << " km/h";
		velocity.text = ostr.str();

		velocities.markers.push_back(velocity);
		++i;

		//for lane_waypoint_marker
		geometry_msgs::Point point;
		point.x = waypoint.pose.pose.position.x;
		point.y = waypoint.pose.pose.position.y;
		point.z = waypoint.pose.pose.position.z;
		lane_waypoint_marker.points.push_back(point);
	}




	pub_velocity.publish(velocities);
	_lane_mark_pub.publish(lane_waypoint_marker);
	*/
	pub_ruled.publish(*current);
    prev_path = *current;
    prev_signal = signal;
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

	pub_ruled = n.advertise<waypoint_follower::lane>(
		"traffic_waypoint",
		ADVERTISE_QUEUE_SIZE,
		ADVERTISE_LATCH);
/*	pub_velocity = n.advertise<visualization_msgs::MarkerArray>(
		"waypoint_velocity",
		ADVERTISE_QUEUE_SIZE,
		ADVERTISE_LATCH);
*/
  _lane_mark_pub = n.advertise<visualization_msgs::MarkerArray>(
      "lane_waypoint_mark",
      ADVERTISE_QUEUE_SIZE,
      ADVERTISE_LATCH);
	/*
	_lane_mark_pub = n.advertise<visualization_msgs::Marker>(
	    "lane_waypoint_mark",
	    ADVERTISE_QUEUE_SIZE,
	    ADVERTISE_LATCH);
*/
	ros::spin();

	return 0;
}
