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
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <iostream>

#include "geo_pos_conv.hh"

geometry_msgs::Twist _current_velocity;
static bool _use_pose = false;
static bool _init_set = false;
static bool _pose_value_set = false;

double _initial_px = 0.0;
double _initial_py = 0.0;
double _initial_pz = 0.0;
double _initial_ox = 0.0;
double _initial_oy = 0.0;
double _initial_oz = 0.0;
double _initial_ow = 0.0;

std::string _use_topic;

void GNSSCallback(const geometry_msgs::PoseStamped::ConstPtr& input)
{
    if(_use_topic == "gnss"){
    _initial_px = input->pose.position.x;
    _initial_py = input->pose.position.y;
    _initial_pz = input->pose.position.z;
    _initial_ox = input->pose.orientation.x;
    _initial_oy = input->pose.orientation.y;
    _initial_oz = input->pose.orientation.z;
    _initial_ow = input->pose.orientation.w;

    _pose_value_set = true;
    }
}

void NDTCallback(const geometry_msgs::PoseStamped::ConstPtr& input)
{
    if(_use_topic == "ndt"){
    _initial_px = input->pose.position.x;
    _initial_py = input->pose.position.y;
    _initial_pz = input->pose.position.z;
    _initial_ox = input->pose.orientation.x;
    _initial_oy = input->pose.orientation.y;
    _initial_oz = input->pose.orientation.z;
    _initial_ow = input->pose.orientation.w;

    _pose_value_set = true;
    }
}


void CmdCallBack(const geometry_msgs::TwistStampedConstPtr &msg)
{
    _current_velocity = msg->twist;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_pub");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
//publish topic
    ros::Publisher odometry_publisher = nh.advertise<nav_msgs::Odometry>("odom_pose", 1000);

//subscribe topic
    ros::Subscriber cmd_subscriber = nh.subscribe("twist_cmd", 1000, CmdCallBack);
    ros::Subscriber gnss_subscriber = nh.subscribe("gnss_pose", 1000, GNSSCallback);
    ros::Subscriber ndt_subscriber = nh.subscribe("control_pose", 1000, NDTCallback);

//transform
    tf::TransformBroadcaster odom_broadcaster;

    private_nh.getParam("use_pose", _use_pose);
    private_nh.getParam("use_topic", _use_topic);

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    /*
     double x = 0.0;
     double y = 0.0;
     double th = 0.0;
     */
    double x = 0;
    double y = 0;
    double z = 0;
    double ox = 0;
    double oy = 0;
    double oz = 0;
    double ow = 1;
    double th = 0;

    std::cout << "checking use_pose" << std::endl;

    //初期値はroslaunchから
    if (_use_pose == false) {
        std::cout << "use initial pose" << std::endl;
        private_nh.getParam("px", x);
        private_nh.getParam("py", y);
        private_nh.getParam("pz", z);
        private_nh.getParam("ox", ox);
        private_nh.getParam("oy", oy);
        private_nh.getParam("oz", oz);
        private_nh.getParam("ow", ow);
        tf::Quaternion q(ox, oy, oz, ow);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        th = yaw;
        _init_set = true;
    }

    // double vx = 5.0;
    //  double vth = -0.230769;

    //10Hzでループ
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce(); //check subscribe topic

        //  std::cout << "waiting value set..." << std::endl;

        //初期値はGNSSから
        if (_use_pose == true) {
            if (_init_set == false) {
                if (_pose_value_set == true) {
                    x = _initial_px;
                    y = _initial_py;
                    z = _initial_pz;
                    ox = _initial_ox;
                    oy = _initial_oy;
                    oz = _initial_oz;
                    ow = _initial_ow;
                    tf::Quaternion q(ox, oy, oz, ow);
                    tf::Matrix3x3 m(q);
                    double roll, pitch, yaw;
                    m.getRPY(roll, pitch, yaw);
                    th = yaw;

                    _init_set = true;
                } else {
                    //std::cout << "continue" << std::endl;
                    continue;
                }
            }
        }

        double vx = _current_velocity.linear.x;
        double vth = _current_velocity.angular.z;
        current_time = ros::Time::now();

        //compute odometry in a typical way given the velocities of the robot
        double dt = (current_time - last_time).toSec();
        double delta_x = (vx * cos(th)) * dt;
        double delta_y = (vx * sin(th)) * dt;
        double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        std::cout << "delta : (" << delta_x << " " << delta_y << " " << delta_th << ")" << std::endl;
        std::cout << "current_velocity : " << _current_velocity.linear.x << " " << _current_velocity.angular.z << std::endl;
        std::cout << "current_pose : (" << x << " " << y << " " << z << " " << th << ")" << std::endl;
        std::cout << "current_orientation : (" << ox << " " << oy << " " << oz << " " << ow << ")" << std::endl << std::endl;

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = z;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = z;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.angular.z = vth;
        //publish the message
        odometry_publisher.publish(odom);

        last_time = current_time;

        loop_rate.sleep();
    }

}
