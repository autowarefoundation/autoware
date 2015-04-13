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
#include <lane_follower/lane.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <runtime_manager/ConfigWaypointLoader.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

#include <vmap_parser.h>

struct pose {
    double x;
    double y;
    double z;
    double velocity_kmh;
};

std::string PATH_FRAME = "/map";

static constexpr double ACCIDENT_ERROR = 0.000001;

static const std::string VECTOR_MAP_DIRECTORY = "/tmp";
static const std::string RULED_WAYPOINT_CSV = "/tmp/ruled_waypoint.csv";

static volatile double config_velocity = 40; // Unit: km/h
static volatile double config_difference_around_signal = 2; // Unit: km/h

static std::vector<Lane> lanes;
static std::vector<Node> nodes;
static std::vector<Point> points;
static std::vector<StopLine> stoplines;

static std::vector<Point> left_lane_points;

std::string vector_map_directory;
std::string ruled_waypoint_csv;

ros::Publisher lane_pub;
ros::Publisher ruled_pub;
ros::Publisher vel_pub;
ros::Publisher mark_pub;
ros::Publisher red_pub;
ros::Publisher green_pub;

lane_follower::lane lane_cmd;
nav_msgs::Path cmd_path;
std::vector<pose> Pose;

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

static void publish_signal_waypoint()
{
    lane_follower::lane red_cmd;
    red_cmd.header = lane_cmd.header;
    red_cmd.increment = 1;

    lane_follower::lane green_cmd;
    green_cmd.header = lane_cmd.header;
    green_cmd.increment = 1;

    std::vector<double> computations =
        compute_velocity(cmd_path, config_velocity, config_difference_around_signal);

    for (int i = 0; i < static_cast<int>(Pose.size()); i++) {

        // for Red
        lane_follower::waypoint waypoint;
        waypoint.pose.header = lane_cmd.header;
        waypoint.twist.header = lane_cmd.header;
        waypoint.pose.pose.position.x = Pose[i].x;
        waypoint.pose.pose.position.y = Pose[i].y;
        waypoint.pose.pose.position.z = Pose[i].z;
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

static void config_callback(const runtime_manager::ConfigWaypointLoader& msg)
{
    config_velocity = msg.velocity;
    config_difference_around_signal = msg.difference_around_signal;

    publish_signal_waypoint();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint_loader");
    ros::NodeHandle nh;

    nh.param<std::string>("waypoint_loader/vector_map_directory",
                          vector_map_directory, VECTOR_MAP_DIRECTORY);

    nh.param<std::string>("waypoint_loader/ruled_waypoint_csv",
                          ruled_waypoint_csv, RULED_WAYPOINT_CSV);

    lanes = read_lane((vector_map_directory + std::string("/lane.csv")).c_str());
    nodes = read_node((vector_map_directory + std::string("/node.csv")).c_str());
    points = read_point((vector_map_directory + std::string("/point.csv")).c_str());
    stoplines = read_stopline((vector_map_directory + std::string("/stopline.csv")).c_str());

    for (const Lane& lane : lanes) {
        if (lane.lno() != 1) // leftmost lane
            continue;
        for (const Node& node : nodes) {
            if (node.nid() != lane.bnid() && node.nid() != lane.fnid())
                continue;
            for (const Point& point : points) {
                if (point.pid() != node.pid())
                    continue;
                left_lane_points.push_back(point);
            }
        }
    }

    ros::Subscriber config_sub = nh.subscribe("config/waypoint_loader", 1000, config_callback);

    lane_pub = nh.advertise<nav_msgs::Path>("lane_waypoint", 1000, true);
    ruled_pub = nh.advertise<lane_follower::lane>("ruled_waypoint", 1000, true);
    vel_pub = nh.advertise<visualization_msgs::MarkerArray>("waypoint_velocity", 1000, true);
    mark_pub = nh.advertise<visualization_msgs::Marker>("waypoint_mark", 1000, true);
    red_pub = nh.advertise<lane_follower::lane>("red_waypoint", 1000, true);
    green_pub = nh.advertise<lane_follower::lane>("green_waypoint", 1000, true);

    // display by markers the velocity of each waypoint.
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    marker.header.frame_id = PATH_FRAME;
    marker.header.stamp = ros::Time();
    marker.ns = "waypoint_velocity";
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.z = 0.4;
    marker.color.a = 1.0;
    marker.color.r = 1.0;


    visualization_msgs::Marker mark;
    mark.header.frame_id = PATH_FRAME;
    mark.header.stamp = ros::Time();
    mark.ns = "waypoint_mark";
    mark.type = visualization_msgs::Marker::POINTS;
    mark.action = visualization_msgs::Marker::ADD;
    mark.scale.x = 0.1;
    mark.scale.y = 0.1;
    mark.color.r = 1.0;
    mark.color.a = 1.0;

    std::ifstream ifs(ruled_waypoint_csv.c_str());
    std::string str;
    int id_count = 0;
    while (getline(ifs, str)) {

      //  std::cout << str << std::endl;
        pose test_pose;
        sscanf(str.c_str(), "%lf,%lf,%lf,%lf", &test_pose.x, &test_pose.y, &test_pose.z, &test_pose.velocity_kmh);
        //std::cout <<test_pose.x << " "<< test_pose.y << " "<<  std::endl;
        marker.id = id_count;
        marker.pose.position.x = test_pose.x;
        marker.pose.position.y = test_pose.y;
        marker.pose.position.z = test_pose.z + 0.2;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // double to string
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(0) << test_pose.velocity_kmh
            << " km/h";
        marker.text = oss.str();

        //C++11 version
        //std::string velocity = std::to_string(test_pose.velocity_kmh);
        //velocity.erase(velocity.find_first_of(".") + 3);
        //std::string kmh = " km/h";
        //std::string text = velocity + kmh;
        //marker.text = text;

        marker_array.markers.push_back(marker);
        Pose.push_back(test_pose);
        id_count++;
    }
    
    ros::Time now = ros::Time::now();
    cmd_path.header.frame_id = PATH_FRAME;
    //cmd_path.header.stamp = now;
    
    lane_cmd.header.frame_id = PATH_FRAME;
    lane_cmd.header.stamp = now;
    lane_cmd.increment = 1;
    
    
    
    for (int i = 0; i < static_cast<int>(Pose.size()); i++) {
      
      // for Path
      geometry_msgs::PoseStamped posestamped;
      posestamped.header = cmd_path.header;
      //      geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(
      //      Pose[i].theta);
      //std::cout << Pose[i].x <<  " " << Pose[i].y << std::endl;
      posestamped.pose.position.x = Pose[i].x;
      posestamped.pose.position.y = Pose[i].y;
      posestamped.pose.position.z = Pose[i].z;
      posestamped.pose.orientation.w = 1.0;
      
      //    std::cout << posestamped.pose.position.x << " " << posestamped.pose.position.y << " " << posestamped.pose.position.z << std::endl;
      cmd_path.poses.push_back(posestamped);
      
      // for Ruled
      lane_follower::waypoint waypoint;
      waypoint.pose.header = lane_cmd.header;
      waypoint.twist.header = lane_cmd.header;
      waypoint.pose.pose.position.x = Pose[i].x;
      waypoint.pose.pose.position.y = Pose[i].y;
      waypoint.pose.pose.position.z = Pose[i].z;
      waypoint.pose.pose.orientation.w = 1.0;
      waypoint.twist.twist.linear.x = Pose[i].velocity_kmh / 3.6;
      
      //   std::cout << waypoint.pose.pose.position.x << " " << waypoint.pose.pose.position.y << " " << waypoint.pose.pose.position.z << " " << waypoint.twist.twist.linear.x << std::endl;
      lane_cmd.waypoints.push_back(waypoint);
      
      // for Mark
      mark.points.push_back(posestamped.pose.position);
    }
    lane_pub.publish(cmd_path);
    ruled_pub.publish(lane_cmd);
    vel_pub.publish(marker_array);
    mark_pub.publish(mark);
    publish_signal_waypoint();
    ros::spin();
    
    
}
