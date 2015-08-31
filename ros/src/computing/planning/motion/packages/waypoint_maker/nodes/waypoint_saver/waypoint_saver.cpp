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
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vehicle_socket/CanInfo.h>
#include <fstream>
#include <sstream>
#include <cfloat>

#define NSEC_TO_SEC 0.000000001
#define LOOP_RATE 10

static bool receive_once = false;
static bool _can_info_set = false;
static bool _pose_set = false;
static geometry_msgs::Point _current_pose;
static double _current_velocity;
static int _pose_time_sec;
static int _pose_time_nsec;
static int _can_time_sec;
static int _can_time_nsec;

static ros::Publisher _waypoint_pub;
static ros::Publisher _waypoint_velocity_pub;

static bool IsNearlyZero(geometry_msgs::Point pose)
{

    double abs_x = fabs(pose.x);
    double abs_y = fabs(pose.y);
    double abs_z = fabs(pose.z);

    if (abs_x < DBL_MIN * 100 && abs_y < DBL_MIN * 100 && abs_z < DBL_MIN * 100)
        return true;
    else
        return false;

}

static void CanInfoCallback(const vehicle_socket::CanInfoConstPtr &info)
{
    _current_velocity = info->speed;
    _can_time_sec = info->header.stamp.sec;
    _can_time_nsec = info->header.stamp.nsec;
    _can_info_set = true;
}

static void NDTCallback(const geometry_msgs::PoseStampedConstPtr &pose)
{
    _current_pose = pose->pose.position;
    _pose_time_sec = pose->header.stamp.sec;
    _pose_time_nsec = pose->header.stamp.nsec;
    _pose_set = true;
}

static void DisplaySavedWaypoint(){

    static visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "lane_waypoint";
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.frame_locked = true;

    marker.points.push_back(_current_pose);
   _waypoint_pub.publish(marker);

}

static void DisplayWaypointVelocity(){

    static int id = 0;
    static visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker tmp_marker;
    tmp_marker.header.frame_id = "map";
    tmp_marker.header.stamp = ros::Time();
    tmp_marker.ns = "waypoint_velocity";
    tmp_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    tmp_marker.action = visualization_msgs::Marker::ADD;
    tmp_marker.scale.z = 0.4;
    tmp_marker.color.a = 1.0;
    tmp_marker.color.r = 1.0;

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2) << _current_velocity << " km/h";
    tmp_marker.text = oss.str();

    //C++11 version
    //std::string velocity = std::to_string(test_pose.velocity_kmh);
    //velocity.erase(velocity.find_first_of(".") + 3);
    //std::string kmh = " km/h";
    //std::string text = velocity + kmh;
    //marker.text = text;

    tmp_marker.id = id;
    tmp_marker.pose.position.x = _current_pose.x;
    tmp_marker.pose.position.y = _current_pose.y;
    tmp_marker.pose.position.z = _current_pose.z + 0.2;

    marker_array.markers.push_back(tmp_marker);
    _waypoint_velocity_pub.publish(marker_array);
    id++;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint_saver");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    geometry_msgs::Point prev_pose;
    std::ofstream ofs;
    double interval = 1.0;
    std::string filename = "";

    ros::Subscriber ndt_pose_sub = nh.subscribe("current_pose", 10, NDTCallback);
    ros::Subscriber can_info_sub = nh.subscribe("can_info", 10, CanInfoCallback);

    _waypoint_pub = nh.advertise<visualization_msgs::Marker>("waypoint_mark", 10, true);
    _waypoint_velocity_pub = nh.advertise<visualization_msgs::MarkerArray>("waypoint_velocity", 10, true);



    private_nh.getParam("interval", interval);
    std::cout << "interval = " << interval << std::endl;

    if (private_nh.getParam("save_filename", filename) == false) {
        std::cout << "error! usage : rosrun waypoint_maker waypoint_saver _interval:=[value] _save_filename:=\"[save file]\"" << std::endl;
        exit(-1);
    }

    ofs.open(filename.c_str());
    ros::Rate loop_rate(LOOP_RATE);
    while (ros::ok()) {
        ros::spinOnce();

        if (_pose_set == false) {
            std::cout << "topic waiting..." << std::endl;
            continue;
        }

        if (IsNearlyZero(_current_pose) == true)
            continue;

        if (receive_once != true) {

            ofs << std::fixed << std::setprecision(4) << _current_pose.x << "," << _current_pose.y << "," << _current_pose.z << std::endl;
            receive_once = true;
            DisplaySavedWaypoint();
            prev_pose = _current_pose;

        } else {

            std::cout << "current_pose : " << _current_pose.x << "," << _current_pose.y << "," << _current_pose.z << std::endl;
            std::cout << "prev_pose : " << prev_pose.x << "," << prev_pose.y << "," << prev_pose.z << std::endl;

            double distance = sqrt(pow((_current_pose.x - prev_pose.x), 2) + pow((_current_pose.y - prev_pose.y), 2));
            std::cout << "distance = " << distance << std::endl;

            //if car moves [interval] meter
            if (distance > interval) {

                if (_can_info_set == true) {

                    std::cout << "can_time_sec = " << _can_time_sec << std::endl;
                    std::cout << "pose_time_sec = " << _pose_time_sec << std::endl;
                    std::cout << "can_time_nsec = " << _can_time_nsec << std::endl;
                    std::cout << "pose_time_nsec = " << _pose_time_nsec << std::endl;
                    std::cout << "nsec sub = " << fabs(_can_time_nsec - _pose_time_nsec) * NSEC_TO_SEC << std::endl;

                    //if time lag is less than 1 second
                    if (_can_time_sec == _pose_time_sec && fabs(_can_time_nsec - _pose_time_nsec) * NSEC_TO_SEC < 1) {

                        ofs << std::fixed << std::setprecision(4) << _current_pose.x << "," << _current_pose.y << "," << _current_pose.z << "," << _current_velocity << std::endl;

                        DisplaySavedWaypoint();
                        DisplayWaypointVelocity();
                        prev_pose = _current_pose;

                    }

                } else {

                    ofs << std::fixed << std::setprecision(4) << _current_pose.x << "," << _current_pose.y << "," << _current_pose.z << "," << 0 << std::endl;

                    DisplaySavedWaypoint();
                    DisplayWaypointVelocity();
                    prev_pose = _current_pose;

                }

            }
            loop_rate.sleep();
        }
    }

    return 0;
}
