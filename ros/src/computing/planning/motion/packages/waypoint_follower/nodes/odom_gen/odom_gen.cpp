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
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <iostream>

static geometry_msgs::Twist _current_velocity;

static double _initial_px = 0.0;
static double _initial_py = 0.0;
static double _initial_pz = 0.0;
static double _initial_ox = 0.0;
static double _initial_oy = 0.0;
static double _initial_oz = 0.0;
static double _initial_ow = 0.0;

static std::string _use_pose;
static bool _initial_set = false;
static bool _pose_set = false;
static void NDTCallback(const geometry_msgs::PoseStamped::ConstPtr& input)
{
    if(_use_pose == "NDT"){
        _initial_px = input->pose.position.x;
        _initial_py = input->pose.position.y;
        _initial_pz = input->pose.position.z;
        _initial_ox = input->pose.orientation.x;
        _initial_oy = input->pose.orientation.y;
        _initial_oz = input->pose.orientation.z;
        _initial_ow = input->pose.orientation.w;

        _initial_set = true;
        _pose_set = false;
    }
}

static void GNSSCallback(const geometry_msgs::PoseStamped::ConstPtr& input)
{
    if(_use_pose == "GNSS"){
        _initial_px = input->pose.position.x;
       _initial_py = input->pose.position.y;
        _initial_pz = input->pose.position.z;
        _initial_ox = input->pose.orientation.x;
        _initial_oy = input->pose.orientation.y;
        _initial_oz = input->pose.orientation.z;
        _initial_ow = input->pose.orientation.w;

        _initial_set = true;
        _pose_set = false;
    }
}

static void CmdCallBack(const geometry_msgs::TwistStampedConstPtr &msg)
{
    _current_velocity = msg->twist;
}

static void initialposeCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &input)
{
  if (_use_pose == "Initial Pos")
  {

    static tf::TransformListener listener;
    tf::StampedTransform transform;
    bool tf_flag = false;
    while (!tf_flag){
      try{
        listener.lookupTransform("map", "world",
                                 ros::Time(0), transform);
        tf_flag = true;
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }

    }

    _initial_px = input->pose.pose.position.x + transform.getOrigin().x();
    _initial_py = input->pose.pose.position.y + transform.getOrigin().y();
    _initial_pz = input->pose.pose.position.z + transform.getOrigin().z();
    _initial_ox = input->pose.pose.orientation.x;
    _initial_oy = input->pose.pose.orientation.y;
    _initial_oz = input->pose.pose.orientation.z;
    _initial_ow = input->pose.pose.orientation.w;

    /*double px = input->pose.pose.position.x;
    double py = input->pose.pose.position.y;
    double pz = input->pose.pose.position.z;
    double ox = input->pose.pose.orientation.x;
    double oy = input->pose.pose.orientation.y;
    double oz = input->pose.pose.orientation.z;
    double ow = input->pose.pose.orientation.w;
*/


    _initial_set = true;
    _pose_set = false;
  }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_gen");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
//publish topic
    ros::Publisher odometry_publisher = nh.advertise<nav_msgs::Odometry>("odom_pose", 10);

//subscribe topic
    ros::Subscriber cmd_subscriber = nh.subscribe("twist_cmd", 10, CmdCallBack);
    ros::Subscriber ndt_subscriber = nh.subscribe("control_pose", 10, NDTCallback);
    ros::Subscriber initialpose_subscriber = nh.subscribe("initialpose",10,initialposeCallback);
    ros::Subscriber gnss_subscriber = nh.subscribe("gnss_pose", 1000, GNSSCallback);

//transform
    tf::TransformBroadcaster odom_broadcaster;

    private_nh.getParam("use_pose", _use_pose);

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();



    std::cout << "checking use_pose" << std::endl;
      /*  private_nh.getParam("initial_pos_x", x);
        private_nh.getParam("initial_pos_y", y);
        private_nh.getParam("initial_pos_z", z);
        double yaw = 0;
        private_nh.getParam("initial_pos_yaw", yaw);
        th = yaw;

        _init_set = true;*/


    double x = 0;
    double y = 0;
    double z = 0;
    double th = 0;

    ros::Rate loop_rate(10); // 10Hz
    while (ros::ok()) {
        ros::spinOnce(); //check subscribe topic
    if (_initial_set)
    {
      if (!_pose_set)
      {
        x = _initial_px;
        y = _initial_py;
        z = _initial_pz;

        tf::Quaternion q(_initial_ox, _initial_oy, _initial_oz, _initial_ow);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        th = yaw;
      }
      _pose_set = true;
    }else{
      continue;
    }

        //  std::cout << "waiting value set..." << std::endl;

        // Initial values are derived from GNSS or NDT
      /*  if (_use_pose != "Initial Pos") {
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
        }*/




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

       // std::cout << "delta (x y th) : (" << delta_x << " " << delta_y << " " << delta_th << ")" << std::endl;
        //std::cout << "current_velocity(linear.x angular.z) : (" << _current_velocity.linear.x << " " << _current_velocity.angular.z << ")"<< std::endl;
       // std::cout << "current_pose : (" << x << " " << y << " " << z << " " << th << ")" << std::endl << std::endl;
        //std::cout << "current_orientation : (" << ox << " " << oy << " " << oz << " " << ow << ")" << std::endl << std::endl;

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "map";
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
        odom.header.frame_id = "map";

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

    return 0;
}
