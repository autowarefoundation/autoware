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

#include <iostream>

#include "runtime_manager/ConfigTwistFilter.h"

//Publisher
static ros::Publisher g_twist_pub;
static double g_lateral_accel_limit = 0.8;

static void configCallback(const runtime_manager::ConfigTwistFilterConstPtr &config)
{
  g_lateral_accel_limit = config->lateral_accel_limit;
  ROS_INFO("g_lateral_accel_limit = %lf",g_lateral_accel_limit);
}

void TwistCmdCallback(const geometry_msgs::TwistStampedConstPtr &msg)
{
  geometry_msgs::TwistStamped twist;
  twist.twist.linear.x = msg->twist.linear.x;

  double accel = msg->twist.linear.x * msg->twist.angular.z;
  ROS_INFO("lateral accel = %lf",accel);



  if(fabs(accel) > g_lateral_accel_limit)
  {
    ROS_INFO("limit over! filtering...");
    if(accel < 0)
      twist.twist.angular.z = (-1) * g_lateral_accel_limit / msg->twist.linear.x;
    else
      twist.twist.angular.z = g_lateral_accel_limit / msg->twist.linear.x;
  }
  else
  {
    //ROS_INFO("limit clear!");
    twist.twist.angular.z = msg->twist.angular.z;
  }

  ROS_INFO("twist.linear.x = %lf, twist.angular.z = %lf",twist.twist.linear.x,twist.twist.angular.z);
  g_twist_pub.publish(twist);

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "twist_filter");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ros::Subscriber twist_sub = nh.subscribe("twist_raw", 1, TwistCmdCallback);
    ros::Subscriber config_sub = nh.subscribe("config/twist_filter", 10, configCallback);
    g_twist_pub = nh.advertise<geometry_msgs::TwistStamped>("twist_cmd", 1000);

    ros::spin();
    return 0;
}
