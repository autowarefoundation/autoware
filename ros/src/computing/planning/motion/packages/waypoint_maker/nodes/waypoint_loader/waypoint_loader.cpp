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
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

#include "runtime_manager/ConfigWaypointLoader.h"
#include "waypoint_follower/lane.h"
#include "waypoint_follower/libwaypoint_follower.h"

#include "vmap_utility.hpp"

struct WP {
    geometry_msgs::Point point;
    double velocity_kmh;
};

std::string PATH_FRAME = "/map";

static constexpr double ACCIDENT_ERROR = 0.000001;

static const std::string RULED_WAYPOINT_CSV = "/tmp/ruled_waypoint.csv";

static double position_offset = 1.8;
static double config_velocity = 40; // Unit: km/h
static double config_difference_around_signal = 2; // Unit: km/h
static int32_t config_number_of_zeros = 1;

static std::vector<map_file::Lane> lanes;
static std::vector<map_file::Node> nodes;
static std::vector<map_file::PointClass> points;
static std::vector<map_file::StopLine> stoplines;
static std::vector<WP> _waypoints;
static std::vector<map_file::PointClass> left_lane_points;

static ros::Publisher red_pub;
static ros::Publisher green_pub;
static ros::Publisher _vel_pub;
static ros::Publisher _mark_pub;
static ros::Publisher _lane_mark_pub;
static ros::Publisher _ruled_pub;
static ros::Publisher _lane_pub;

static waypoint_follower::lane _ruled_waypoint;
static nav_msgs::Path _lane_waypoint;
static bool _vector_map_flag = false;

static WP parseWaypoint(const std::string& line)
{
    std::istringstream ss(line);
    std::vector<std::string> columns;

    std::string column;
    while (std::getline(ss, column, ',')) {
        columns.push_back(column);
    }

    WP waypoint;
    waypoint.point.x = std::stod(columns[0]);
    waypoint.point.y = std::stod(columns[1]);
    waypoint.point.z = std::stod(columns[2]);
    waypoint.velocity_kmh = std::stod(columns[3]);

    return waypoint;

}

static std::vector<WP> readWaypoint(const char *filename)
{
    std::ifstream ifs(filename);
    std::string line;

    std::getline(ifs, line); // Remove first line

    std::vector<WP> waypoints;
    while (std::getline(ifs, line)) {
      waypoints.push_back(parseWaypoint(line));

    }

    return waypoints;
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

        if (point.bx == end_point.bx &&
            point.ly == end_point.ly) {
            for (const map_file::StopLine& stopline : stoplines) {
                if (stopline.linkid == lane.lnid)
                    stopline_points.push_back(point);
            }

            return stopline_points;
        }

        lane_index = to_next_lane_index(lane, lanes);
        if (lane_index < 0) {
            ROS_ERROR("next lane is not found");
            return stopline_points;
        }
        lane = lanes[lane_index];

        point_index = to_beginning_point_index(lane, nodes, points);
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

static void publish_signal_waypoint()
{
    waypoint_follower::lane red_cmd;
    red_cmd.header = _ruled_waypoint.header;
    red_cmd.increment = 1;

    waypoint_follower::lane green_cmd;
    green_cmd.header = _ruled_waypoint.header;
    green_cmd.increment = 1;

    std::vector<double> computations = compute_velocity(
        _lane_waypoint,
        config_velocity,
        config_difference_around_signal,
        config_number_of_zeros);

    for (int i = 0; i < static_cast<int>(_waypoints.size()); i++) {

        // for Red
        waypoint_follower::waypoint waypoint;
        waypoint.pose.header = _ruled_waypoint.header;
        waypoint.twist.header = _ruled_waypoint.header;
        waypoint.pose.pose.position = _waypoints[i].point;
        waypoint.pose.pose.orientation.w = 1.0;
        waypoint.twist.twist.linear.x = computations[i] / 3.6;
        red_cmd.waypoints.push_back(waypoint);

        // for Green
        waypoint.twist.twist.linear.x = config_velocity / 3.6;
        green_cmd.waypoints.push_back(waypoint);
    }
    red_pub.publish(red_cmd);
    green_pub.publish(green_cmd);
}


void createWaypointVelocity(visualization_msgs::MarkerArray *marker_array){

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

    for (unsigned int i = 0; i < _waypoints.size(); i++) {

        //std::cout << _waypoints[i].GetX() << " " << _waypoints[i].GetY() << " " << _waypoints[i].GetZ() << " " << _waypoints[i].GetVelocity_kmh() << std::endl;
        velocity.id = i;
        velocity.pose.position = _waypoints[i].point;
        velocity.pose.position.z += 0.2;
        velocity.pose.orientation.w = 1.0;

        double vel = decelerate(point2vector(_waypoints[i].point),
              point2vector(_waypoints[_waypoints.size() - 1].point),_waypoints[i].velocity_kmh);

        // double to string
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << vel << " km/h";
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

void createWaypointMark(visualization_msgs::MarkerArray *marker_array)
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

    for (unsigned int i = 0; i < _waypoints.size(); i++) {
        geometry_msgs::Point point;
        point = _waypoints[i].point;
        waypoint_mark.points.push_back(point);

    }
    marker_array->markers.push_back(waypoint_mark);
}

void createLaneWaypointMarker(visualization_msgs::MarkerArray *marker_array)
{
    static visualization_msgs::Marker lane_waypoint_marker;
    lane_waypoint_marker.header.frame_id = PATH_FRAME;
    lane_waypoint_marker.header.stamp = ros::Time();
    lane_waypoint_marker.ns = "lane_waypoint_marker";
    lane_waypoint_marker.id = 0;
    lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
    lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
    lane_waypoint_marker.scale.x = 0.2;
    lane_waypoint_marker.pose.orientation.w = 1.0;
    lane_waypoint_marker.color.b = 1.0;
    lane_waypoint_marker.color.g = 0.5;
    lane_waypoint_marker.color.a = 1.0;
    lane_waypoint_marker.frame_locked = true;

    for (unsigned int i = 0; i < _waypoints.size(); i++) {
        geometry_msgs::Point point;
        point = _waypoints[i].point;
        lane_waypoint_marker.points.push_back(point);

    }
    marker_array->markers.push_back(lane_waypoint_marker);
}

void displayLaneWaypoint(){


     visualization_msgs::MarkerArray lane_waypoint;
     createWaypointVelocity(&lane_waypoint);
     createWaypointMark(&lane_waypoint);
     createLaneWaypointMarker(&lane_waypoint);
     _lane_mark_pub.publish(lane_waypoint);

}

void PublishRuledWaypoint()
{

    _ruled_waypoint.header.frame_id = PATH_FRAME;
    _ruled_waypoint.header.stamp = ros::Time(0);
    _ruled_waypoint.increment = 1;

    for (unsigned int i = 0; i < _waypoints.size(); i++) {
        waypoint_follower::waypoint waypoint;
        waypoint.pose.header = _ruled_waypoint.header;
        waypoint.twist.header = _ruled_waypoint.header;
        waypoint.pose.pose.position = _waypoints[i].point;
        waypoint.pose.pose.orientation.w = 1.0;

        double vel = decelerate(point2vector(_waypoints[i].point),
                      point2vector(_waypoints[_waypoints.size() - 1].point),_waypoints[i].velocity_kmh);

        waypoint.twist.twist.linear.x = kmph2mps(vel);
        _ruled_waypoint.waypoints.push_back(waypoint);
    }
    _ruled_pub.publish(_ruled_waypoint);
}

void publishLaneWaypoint()
{

    _lane_waypoint.header.frame_id = PATH_FRAME;
     _lane_waypoint.header.stamp = ros::Time(0);

    for (unsigned int i = 0; i < _waypoints.size(); i++) {
     geometry_msgs::PoseStamped waypoint;
     waypoint.header = _lane_waypoint.header;
     waypoint.pose.position = _waypoints[i].point;
     waypoint.pose.orientation.w = 1.0;
    _lane_waypoint.poses.push_back(waypoint);
     }
     _lane_pub.publish(_lane_waypoint);
}

static void update_left_lane()
{
    if (lanes.empty() || nodes.empty() || points.empty())
        return;

    for (const map_file::Lane& lane : lanes) {
        if (lane.lno != 1) // leftmost lane
            continue;
        for (const map_file::Node& node : nodes) {
            if (node.nid != lane.bnid && node.nid != lane.fnid)
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

static void config_callback(const runtime_manager::ConfigWaypointLoader& msg)
{
    config_velocity = msg.velocity;
    config_difference_around_signal = msg.difference_around_signal;
    config_number_of_zeros = msg.number_of_zeros;


    if (_vector_map_flag)
    publish_signal_waypoint();
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint_loader");
    ros::NodeHandle nh;

    std::string ruled_waypoint_csv;
    nh.param<std::string>("waypoint_loader/ruled_waypoint_csv",
                          ruled_waypoint_csv, RULED_WAYPOINT_CSV);

    ros::Subscriber sub_lane = nh.subscribe("vector_map_info/lane",
                                            10,
                                            lane_callback);
    ros::Subscriber sub_node = nh.subscribe("vector_map_info/node",
                                            10,
                                            node_callback);
    ros::Subscriber sub_point_class = nh.subscribe(
        "vector_map_info/point_class",
        10,
        point_class_callback);
    ros::Subscriber sub_stop_line = nh.subscribe(
        "vector_map_info/stop_line",
        10,
        stop_line_callback);


    //wait vector map for 2 seconds
    ros::Rate rate(1);
    int count = 0;
    while (lanes.empty() || nodes.empty() || points.empty() ||
           stoplines.empty()) {
        ros::spinOnce();
        rate.sleep();
       if(count == 2)
      	break;
     count++;
    }



  if (!lanes.empty() && !nodes.empty() && !points.empty() && !stoplines.empty()) {
    _vector_map_flag = true;
    ROS_INFO("vector map loaded");
  } else {
    _vector_map_flag = false;
    ROS_INFO("vector map not loaded");
  }

    ros::Subscriber config_sub = nh.subscribe("config/waypoint_loader", 10, config_callback);

    _lane_pub = nh.advertise<nav_msgs::Path>("lane_waypoint", 10, true);
    _ruled_pub = nh.advertise<waypoint_follower::lane>("traffic_waypoint", 10, true);
    _lane_mark_pub = nh.advertise<visualization_msgs::MarkerArray>("lane_waypoint_mark", 10, true);

    red_pub = nh.advertise<waypoint_follower::lane>("red_waypoint", 10, true);
    green_pub = nh.advertise<waypoint_follower::lane>("green_waypoint", 10, true);

    _waypoints = readWaypoint(ruled_waypoint_csv.c_str());
    displayLaneWaypoint();
    publishLaneWaypoint();
    PublishRuledWaypoint();

  if (_vector_map_flag)
    publish_signal_waypoint();
    ros::spin();

}
