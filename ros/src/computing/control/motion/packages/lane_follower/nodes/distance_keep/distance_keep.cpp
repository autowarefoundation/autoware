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
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <lane_follower/lane.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <iostream>

#include "lf_func.h"

#define LOOP_RATE 10

static geometry_msgs::TwistStamped _current_twist;
static geometry_msgs::PoseStamped _current_pose; // current pose by the global plane.
static lane_follower::lane _current_path;
//std::vector<geometry_msgs::Pose> _car_pose;
//std::vector<geometry_msgs::Pose> _ped_pose;
static pcl::PointCloud<pcl::PointXYZ> _vscan;

static std::string _current_pose_topic = "ndt";
static const std::string PATH_FRAME = "/map";
static bool _twist_flag = false;
static bool _pose_flag = false;
static bool _path_flag = false;
static bool _vscan_flag = false;

static double _detection_range = 0;
//static int _obstacle_waypoint = -1;
static int _vscan_obstacle_waypoint = -1;
static int _closest_waypoint = -1;
static int _threshold_points = 15;
static double _detection_height_top = 2.0; //actually +2.0m
static double _detection_height_bottom = -2.0;
static double _search_distance = 30;
static int _stop_interval = 5;
static tf::Transform _transform;

//Publisher
static ros::Publisher _twist_pub;
static ros::Publisher _vis_pub;
static ros::Publisher _range_pub;

static void TwistCmdCallback(const geometry_msgs::TwistStampedConstPtr &msg)
{
    _current_twist = *msg;

    if (_twist_flag == false) {
        std::cout << "twist subscribed" << std::endl;
        _twist_flag = true;
    }
}
/*
 static void CarPoseCallback(const geometry_msgs::PoseArrayConstPtr &msg)
 {
 _car_pose = msg->poses;
 }

 static void PedPoseCallback(const geometry_msgs::PoseArrayConstPtr &msg)
 {
 _ped_pose = msg->poses;
 }
 */
static void VscanCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::fromROSMsg(*msg, _vscan);
    if (_vscan_flag == false) {
        std::cout << "vscan subscribed" << std::endl;
        _vscan_flag = true;
    }

}

static void NDTCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    if (_current_pose_topic == "ndt") {
        _current_pose.header = msg->header;
        _current_pose.pose = msg->pose;
        tf::Transform inverse;
        tf::poseMsgToTF(msg->pose, inverse);
        _transform = inverse.inverse();
        if (_pose_flag == false) {
            std::cout << "pose subscribed" << std::endl;
            _pose_flag = true;
        }
    }

}

static void OdometryCallback(const nav_msgs::OdometryConstPtr &msg)
{
    if (_current_pose_topic == "odometry") {
        _current_pose.header = msg->header;
        _current_pose.pose = msg->pose.pose;
        tf::Transform inverse;
        tf::poseMsgToTF(msg->pose.pose, inverse);
        _transform = inverse.inverse();

        if (_pose_flag == false) {
            std::cout << "pose subscribed" << std::endl;
            _pose_flag = true;
        }
    }
}

static void WaypointCallback(const lane_follower::laneConstPtr &msg)
{
    _current_path = *msg;
    if (_path_flag == false) {
        std::cout << "waypoint subscribed" << std::endl;
        _path_flag = true;
    }
}

// display  by markers.
static void DisplayObstacleWaypoint(int i)
{

    visualization_msgs::Marker marker;
    marker.header.frame_id = PATH_FRAME;
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = _current_path.waypoints[i].pose.pose.position;
    marker.pose.orientation = _current_pose.pose.orientation;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = _detection_height_top;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.lifetime = ros::Duration(0.1);
    marker.frame_locked = true;

    _vis_pub.publish(marker);
}

// display  by markers.
static void DisplayDetectionRange(int i)
{

    visualization_msgs::Marker marker;
    marker.header.frame_id = PATH_FRAME;
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = _current_path.waypoints[i].pose.pose.position;
    marker.scale.x = 2 * _detection_range;
    marker.scale.y = 2 * _detection_range;
    marker.scale.z = _detection_height_top;
    marker.color.a = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.frame_locked = true;

    _range_pub.publish(marker);
}

static int GetObstacleWaypointUsingVscan()
{

    if (_vscan.empty() == true)
        return -1;

    for (int i = _closest_waypoint + 1; i < _closest_waypoint + _search_distance; i++) {

        if(i > static_cast<int>(_current_path.waypoints.size()) - 1 )
            return -1;
        DisplayDetectionRange(i);
     //   tf::Vector3 tf_waypoint = TransformWaypoint(i);
        tf::Vector3 tf_waypoint = TransformWaypoint(_transform,_current_path.waypoints[i].pose.pose);
        tf_waypoint.setZ(0);

        //std::cout << "waypoint : "<< tf_waypoint.getX()  << " "<< tf_waypoint.getY() << std::endl;
        int point_count = 0;
        for (pcl::PointCloud<pcl::PointXYZ>::const_iterator item = _vscan.begin(); item != _vscan.end(); item++) {
            if ((item->x == 0 && item->y == 0) || item->z > _detection_height_top || item->z < _detection_height_bottom)
                continue;

            tf::Vector3 point((double) item->x, (double) item->y, 0);
            double dt = tf::tfDistance(point, tf_waypoint);
            if (dt < _detection_range) {
                point_count++;
                //std::cout << "distance :" << dt << std::endl;
                //std::cout << "point : "<< (double) item->x  << " " <<  (double)item->y  <<  " " <<(double) item->z << std::endl;
                //std::cout << "count : "<< point_count << std::endl;
            }

            if (point_count > _threshold_points)
                return i;

        }
    }
    return -1;

    }

static bool ObstacleDetection()
{
    static int false_count = 0;
    static bool prev_detection = false;

    //_closest_waypoint = GetClosestWaypoint();
    _closest_waypoint = GetClosestWaypoint(_transform,_current_path,_closest_waypoint);
    std::cout << "closest_waypoint : " << _closest_waypoint << std::endl;

    _vscan_obstacle_waypoint = GetObstacleWaypointUsingVscan();

    if (prev_detection == false) {
        if (_vscan_obstacle_waypoint != -1) {
            DisplayObstacleWaypoint(_vscan_obstacle_waypoint);
            std::cout << "obstacle waypoint : " << _vscan_obstacle_waypoint << std::endl << std::endl;
            prev_detection = true;
            return true;
        } else {
            prev_detection = false;
            return false;
        }
    } else {
        if (_vscan_obstacle_waypoint != -1) {
            DisplayObstacleWaypoint(_vscan_obstacle_waypoint);
            std::cout << "obstacle waypoint : " << _vscan_obstacle_waypoint << std::endl << std::endl;
            prev_detection = true;
            return true;
        } else {
            false_count++;
        }

        if (false_count == LOOP_RATE * 3) {
            false_count = 0;
            prev_detection = false;
            return false;
        } else {
            DisplayObstacleWaypoint(_vscan_obstacle_waypoint);
            prev_detection = true;
           return true;
        }
    }



}


static double Decelerate()
{
    //calculate distance from my position to waypoint
    tf::Vector3 tf_waypoint = TransformWaypoint(_transform,_current_path.waypoints[_vscan_obstacle_waypoint].pose.pose);
    double distance = tf::tfDistance(_origin_v, tf_waypoint);
    std::cout << "distance " << distance << std::endl;

    //if distance is within stop_interval param, publish 0km/h
    if(distance < _stop_interval){
        return 0;
    }

    double decel_velocity_ms = DecelerateVelocity(distance,_current_twist.twist.linear.x);

    if(decel_velocity_ms < 1.0){
        decel_velocity_ms = 0;
    }

    return decel_velocity_ms;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "distance_keep");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Subscriber twist_sub = nh.subscribe("twist_raw", 1, TwistCmdCallback);
    //ros::Subscriber car_pose_sub = nh.subscribe("car_pose", 1,CarPoseCallback);
    //ros::Subscriber ped_pose_sub = nh.subscribe("pedestrian_pose", 1, PedPoseCallback);
    ros::Subscriber vscan_sub = nh.subscribe("vscan_points", 1, VscanCallback);
    ros::Subscriber ndt_sub = nh.subscribe("control_pose", 1, NDTCallback);
    ros::Subscriber odom_sub = nh.subscribe("odom_pose", 1, OdometryCallback);
    ros::Subscriber waypoint_sub = nh.subscribe("ruled_waypoint", 1, WaypointCallback);

    _twist_pub = nh.advertise<geometry_msgs::TwistStamped>("twist_cmd", 1000);
    _vis_pub = nh.advertise<visualization_msgs::Marker>("obstaclewaypoint_mark", 0);
    _range_pub = nh.advertise<visualization_msgs::Marker>("detection_range", 0);

    private_nh.getParam("detection_range", _detection_range);
    std::cout << "detection_range : " << _detection_range << std::endl;

    private_nh.getParam("threshold_points", _threshold_points);
    std::cout << "threshold_points : " << _threshold_points << std::endl;

    private_nh.getParam("stop_interval", _stop_interval);
    std::cout << "stop_interval : " << _stop_interval << std::endl;

    private_nh.getParam("detection_height_top", _detection_height_top);
    std::cout << "detection_height_top : " << _detection_height_top << std::endl;

    private_nh.getParam("detection_height_bottom", _detection_height_bottom);
    std::cout << "detection_height_bottom : " << _detection_height_bottom << std::endl;

    private_nh.getParam("current_pose_topic", _current_pose_topic);
    std::cout << "current_pose_topic : " << _current_pose_topic << std::endl;

    ros::Rate loop_rate(LOOP_RATE);
    while (ros::ok()) {
        ros::spinOnce();

        if (_pose_flag == false || _path_flag == false) {
            std::cout << "topic waiting..." << std::endl;
            continue;
        }

        bool detection_flag = ObstacleDetection();

        if (_twist_flag == true) {
            geometry_msgs::TwistStamped twist;
            if (detection_flag == true) {
                //decelerate
                std::cout << "twist deceleration..." << std::endl;
                twist.twist.linear.x = Decelerate();
                twist.twist.angular.z = _current_twist.twist.angular.z;
            } else {
                //through
                std::cout << "twist through" << std::endl;
                twist.twist = _current_twist.twist;
            }
            std::cout << "twist.linear.x = " << twist.twist.linear.x << std::endl;
            std::cout << "twist.angular.z = " << twist.twist.angular.z << std::endl;
            std::cout << std::endl;

            twist.header.stamp = _current_twist.header.stamp;
            _twist_pub.publish(twist);
        } else {
            std::cout << "no twist topic" << std::endl;
        }
        loop_rate.sleep();
    }

    return 0;
}
