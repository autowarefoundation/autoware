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

#include "autoware_msgs/ConfigTwistFilter.h"

namespace {

//Publisher
ros::Publisher g_twist_pub;
double g_lateral_accel_limit = 5.0;
double g_omega_limit = 0.1;
double g_lowpass_gain_linear_x = 0.0;
double g_lowpass_gain_angular_z = 0.0;
constexpr double RADIUS_MAX = 9e10;
constexpr double ERROR = 1e-8;

void configCallback(const autoware_msgs::ConfigTwistFilterConstPtr &config)
{
  g_omega_limit = config->omega_limit;
  g_lateral_accel_limit = config->lateral_accel_limit;
  ROS_INFO("g_lateral_accel_limit = %lf",g_lateral_accel_limit);
  g_lowpass_gain_linear_x = config->lowpass_gain_linear_x;
  ROS_INFO("lowpass_gain_linear_x = %lf",g_lowpass_gain_linear_x);
  g_lowpass_gain_angular_z = config->lowpass_gain_angular_z;
  ROS_INFO("lowpass_gain_angular_z = %lf",g_lowpass_gain_angular_z);
}

void TwistCmdCallback(const geometry_msgs::TwistStampedConstPtr &msg)
{
  geometry_msgs::TwistStamped tp(*msg);

  double vel = tp.twist.linear.x;
  const int omega_sgn = (tp.twist.angular.z < 0) ? -1 : 1;
  double omega = tp.twist.angular.z;
  omega = (fabs(omega) < g_omega_limit) ? omega : omega_sgn * g_omega_limit;
  if (fabs(vel) >= ERROR)
  {
    const double kappa = tp.twist.angular.z / vel;
    vel = (fabs(kappa) >= ERROR) ? omega / kappa : vel;
  }
  if (fabs(omega) >= ERROR)
  {

    const int sgn = (vel < 0) ? -1 : 1;
    const double max_v = sgn * fabs(g_lateral_accel_limit / omega);
    const double acc = vel * omega;
    ROS_INFO("lateral accel = %lf", acc);
    if (fabs(acc) > g_lateral_accel_limit)
    {
      vel =  max_v;
    }
  }

  static double lowpass_linear_x = 0;
  static double lowpass_angular_z = 0;
  lowpass_linear_x = g_lowpass_gain_linear_x * lowpass_linear_x + (1 - g_lowpass_gain_linear_x) * vel;
  lowpass_angular_z = g_lowpass_gain_angular_z * lowpass_angular_z + (1 - g_lowpass_gain_angular_z) * omega;

  tp.twist.linear.x = lowpass_linear_x;
  tp.twist.angular.z = lowpass_angular_z;

  ROS_INFO("v: %f -> %f", vel, tp.twist.linear.x);
  g_twist_pub.publish(tp);

}
} // namespace

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
