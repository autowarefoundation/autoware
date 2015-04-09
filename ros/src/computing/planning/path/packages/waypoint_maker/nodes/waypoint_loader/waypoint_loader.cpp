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
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <lane_follower/lane.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

struct pose {
    double x;
    double y;
    double z;
    double velocity_kmh;
};

std::string PATH_FRAME = "/map";

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint_loader");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Publisher lane_pub;
    ros::Publisher ruled_pub;
    ros::Publisher vel_pub;
    ros::Publisher mark_pub;

    lane_pub = nh.advertise<nav_msgs::Path>("lane_waypoint", 1000, true);
    ruled_pub = nh.advertise<lane_follower::lane>("ruled_waypoint", 1000, true);
    vel_pub = nh.advertise<visualization_msgs::MarkerArray>("waypoint_velocity", 1000, true);
    mark_pub = nh.advertise<visualization_msgs::Marker>("waypoint_mark", 1000, true);
    std::vector<pose> Pose;

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

    std::string filename = "";
    if (private_nh.getParam("filename", filename) == false) {
        std::cout << "error! usage : rosrun waypoint_maker waypoint_loader _filename:=\"[path file]\"" << std::endl;
        exit(-1);
    }
    std::cout << "filename : " << filename << std::endl;
    std::ifstream ifs(filename.c_str());
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
    nav_msgs::Path cmd_path;
    cmd_path.header.frame_id = PATH_FRAME;
    //cmd_path.header.stamp = now;
    
    lane_follower::lane lane_cmd;
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
    ros::spin();
    
    
}
