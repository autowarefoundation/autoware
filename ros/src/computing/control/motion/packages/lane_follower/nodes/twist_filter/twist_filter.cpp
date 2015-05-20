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
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>


#include <iostream>

#define LOOP_RATE 10

class TwistFilter {
private:
    ros::NodeHandle nh;
    ros::Subscriber twist_sub;
    ros::Subscriber car_pose_sub;
    ros::Subscriber ped_pose_sub;
    ros::Subscriber ndt_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber waypoint_sub;

    ros::Publisher twist_pub;
    ros::Publisher vis_pub;

    geometry_msgs::TwistStamped current_twist;
    geometry_msgs::PoseStamped current_pose; // current pose by the global plane.
    lane_follower::lane current_path;
    std::vector<geometry_msgs::Pose> car_pose;
    std::vector<geometry_msgs::Pose> ped_pose;
    std::string current_pose_topic = "odometry";
    const std::string PATH_FRAME = "/map";
    bool twist_flag;
    bool pose_flag;
    bool path_flag;
    bool decelerate_flag;
    double decelerate_ms;
    double velocity_ms;
    double detection_range;
    double stop_distance;

    void TwistCmdCallback(const geometry_msgs::TwistStampedConstPtr &msg);
    void CarPoseCallback(const geometry_msgs::PoseArrayConstPtr &msg);
    void PedPoseCallback(const geometry_msgs::PoseArrayConstPtr &msg);
    void NDTCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void OdometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void WaypointCallback(const lane_follower::laneConstPtr &msg);
    int GetClosestCar();
    int GetClosestPedestrian();
    int GetWaypointObstacleLocate(int car , int ped);


public:
    bool Detection();
    TwistFilter();
    ~TwistFilter();

};

TwistFilter::TwistFilter()
{

    twist_sub = nh.subscribe("twist_cmd", 1, &TwistFilter::TwistCmdCallback, this);
    car_pose_sub = nh.subscribe("car_pose", 1, &TwistFilter::CarPoseCallback, this);
    ped_pose_sub = nh.subscribe("pedestrian_pose", 1, &TwistFilter::PedPoseCallback, this);
    ndt_sub = nh.subscribe("ndt_pose", 1, &TwistFilter::NDTCallback, this);
    odom_sub = nh.subscribe("odom_pose", 1, &TwistFilter::OdometryCallback, this);
    waypoint_sub = nh.subscribe("ruled_waypoint", 1, &TwistFilter::WaypointCallback, this);

    twist_pub = nh.advertise<geometry_msgs::TwistStamped>("twist_filter_cmd", 1000);
    vis_pub = nh.advertise<visualization_msgs::Marker>("stop_waypoint_mark", 0);

    decelerate_flag = false;
    twist_flag = false;
    pose_flag = false;
    path_flag = false;
    decelerate_ms = 0;
    velocity_ms = 0;
    detection_range = 0;
    stop_distance = 0;

    ros::NodeHandle private_nh("~");
    private_nh.getParam("detection_range", detection_range);
    std::cout << "detection_range : " << detection_range << std::endl;

    private_nh.getParam("stop_distance",stop_distance);
    std::cout << "stop_distance : " << stop_distance << std::endl;

}

TwistFilter::~TwistFilter()
{
    std::cout << "END" << std::endl;
}

void TwistFilter::TwistCmdCallback(const geometry_msgs::TwistStampedConstPtr &msg)
{
    current_twist = *msg;
    twist_flag = true;
}

void TwistFilter::CarPoseCallback(const geometry_msgs::PoseArrayConstPtr &msg)
{
    car_pose = msg->poses;
}

void TwistFilter::PedPoseCallback(const geometry_msgs::PoseArrayConstPtr &msg)
{
    ped_pose = msg->poses;
}

void TwistFilter::NDTCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    if(current_pose_topic == "ndt"){
    current_pose.header = msg->header;
    current_pose.pose = msg->pose;
    pose_flag = true;
    }

}

void TwistFilter::OdometryCallback(const nav_msgs::OdometryConstPtr &msg)
{
    if (current_pose_topic == "odometry") {
        current_pose.header = msg->header;
        current_pose.pose = msg->pose.pose;
        pose_flag = true;
    }

}

void TwistFilter::WaypointCallback(const lane_follower::laneConstPtr &msg)
{
    current_path = *msg;
    std::cout << "waypoint subscribed" << std::endl;
    path_flag = true;
}

int TwistFilter::GetClosestCar()
{
   if(car_pose.empty() == true)
        return -1;

    int num = -1;
    tf::Vector3 v1(current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
    double distance = 10000;

    for (unsigned int i = 0; i < car_pose.size(); i++) {
        tf::Vector3 v2(car_pose[i].position.x, car_pose[i].position.y, car_pose[i].position.z);
        double dt = tf::tfDistance(v1, v2);
        if (dt < distance) {
            distance = dt;
            num = i;
        }
    }
    return num;

}

int TwistFilter::GetClosestPedestrian()
{
   if(ped_pose.empty() == true)
        return -1;

    int num = -1;
    tf::Vector3 v1(current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
    double distance = 10000;

    for (unsigned int i = 0; i < ped_pose.size(); i++) {
        tf::Vector3 v2(ped_pose[i].position.x, ped_pose[i].position.y, ped_pose[i].position.z);
        double dt = tf::tfDistance(v1, v2);
        if (dt < distance) {
            distance = dt;
            num = i;
        }
    }
    return num;

}

int TwistFilter::GetWaypointObstacleLocate(int car, int ped)
{
    if ((car == -1 && ped == -1) || current_path.waypoints.empty() == true)
        return -1;

    tf::Vector3 car_v(car_pose[car].position.x, car_pose[car].position.y, car_pose[car].position.z);
    tf::Vector3 ped_v(ped_pose[ped].position.x, ped_pose[ped].position.y, ped_pose[ped].position.z);

    for (unsigned int i = 1; i < current_path.waypoints.size(); i++) {
        tf::Vector3 v2(current_path.waypoints[i].pose.pose.position.x, current_path.waypoints[i].pose.pose.position.y, 0);
        double car_dt = tf::tfDistance(car_v, v2);
        double ped_dt = tf::tfDistance(ped_v, v2);
        double dt = 1000;
        if (car_dt < ped_dt)
            dt = car_dt;
        else
            dt = ped_dt;

        if (dt < detection_range) {
            return i;
        }
    }
    return -1;

}

bool TwistFilter::Detection()
{
    if(twist_flag ==false || pose_flag == false || path_flag == false)
        return false;

    int car_num = GetClosestCar();
    int ped_num = GetClosestPedestrian();
    int waypoint = GetWaypointObstacleLocate(car_num , ped_num);

    std::cout << car_num << " " << ped_num << " " << waypoint << std::endl;

    if (waypoint != -1) {
        tf::Vector3 v1(current_pose.pose.position.x, current_pose.pose.position.y, 0);
        tf::Vector3 v2(current_path.waypoints[waypoint - stop_distance].pose.pose.position.x, current_path.waypoints[waypoint - stop_distance].pose.pose.position.y, 0);

        visualization_msgs::Marker marker;
        marker.header.frame_id = PATH_FRAME;
        marker.header.stamp = ros::Time::now();
        marker.ns = "my_namespace";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = current_path.waypoints[waypoint - stop_distance].pose.pose.position;
        marker.pose.orientation = current_path.waypoints[waypoint - stop_distance].pose.pose.orientation;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        vis_pub.publish(marker);

        double distance = tf::tfDistance(v1, v2);
        std::cout << "distance to obstacle " << distance << std::endl;

        if (decelerate_flag == false) {
            decelerate_ms = pow(current_twist.twist.linear.x, 2) / (2 * distance);
            velocity_ms = current_twist.twist.linear.x;
            decelerate_flag = true;
        }
        std::cout << "decelerate : "<< decelerate_ms << std::endl;

        velocity_ms -= decelerate_ms / LOOP_RATE;
        if (velocity_ms < 0)
            velocity_ms = 0;

        double radius = current_twist.twist.linear.x / current_twist.twist.angular.z;
        current_twist.twist.linear.x = velocity_ms;
        current_twist.twist.angular.z = current_twist.twist.linear.x / radius;

    } else {
        decelerate_ms = 0;
        decelerate_flag = false;
    }
           std::cout << "twist.linear.x = " << current_twist.twist.linear.x << std::endl;
           std::cout << "twist.angular.z = " << current_twist.twist.angular.z << std::endl;
           std::cout << std::endl;

        twist_pub.publish(current_twist);
return true;

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "twist_filter");
    TwistFilter tf;

    ros::Rate loop_rate(LOOP_RATE);
    while (ros::ok()) {
        ros::spinOnce();
        tf.Detection();
        loop_rate.sleep();
    }

    return 0;
}
