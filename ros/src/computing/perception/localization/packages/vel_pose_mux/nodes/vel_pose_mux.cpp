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
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "vehicle_socket/CanInfo.h"

namespace
{
ros::Publisher g_vel_publisher;
ros::Publisher g_pose_publisher;

void callbackFromCanInfo(const vehicle_socket::CanInfoConstPtr &msg)
{
  geometry_msgs::Vector3Stamped vel;
  vel.header = msg->header;
  vel.vector.x = (msg->speed * 1000) / (60 * 60);  // km/h -> m/s
  g_vel_publisher.publish(vel);
}

void callbackFromPoseStamped(const geometry_msgs::PoseStampedConstPtr &msg)
{
  g_pose_publisher.publish(*msg);
}

void callbackFromVector3Stamped(const geometry_msgs::Vector3StampedConstPtr &msg)
{
  g_vel_publisher.publish(*msg);
}

}  // namespace

int main(int argc, char **argv)
{
  // set up ros
  ros::init(argc, argv, "vel_pose_mux");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  int32_t pose_mux_select, vel_mux_select;
  bool sim_mode;

  // setting params
  private_nh.param<int32_t>("pose_mux_select", pose_mux_select, int32_t(0));
  //ROS_INFO_STREAM("pose_mux_select : " << pose_mux_select);

  private_nh.param<int32_t>("vel_mux_select", vel_mux_select, int32_t(1));
  //ROS_INFO_STREAM("vel_mux_select : " << vel_mux_select);

  private_nh.param<bool>("sim_mode", sim_mode, false);
  //ROS_INFO_STREAM("sim_mode : " << sim_mode);

  // publish topic
  g_vel_publisher = nh.advertise<geometry_msgs::Vector3Stamped>("current_velocity", 10);
  g_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("current_pose", 10);

  // subscribe topic
  ros::Subscriber pose_subcscriber;
  ros::Subscriber vel_subcscriber;

  if (sim_mode)
  {
    vel_subcscriber = nh.subscribe("sim_velocity", 10, callbackFromVector3Stamped);
    pose_subcscriber = nh.subscribe("sim_pose", 10, callbackFromPoseStamped);
  }
  else
  {
    // pose
    switch (pose_mux_select)
    {
      case 0:  // ndt_localizer
      {
        pose_subcscriber = nh.subscribe("ndt_pose", 10, callbackFromPoseStamped);
        break;
      }
      case 1:  // gnss
      {
        pose_subcscriber = nh.subscribe("gnss_pose", 10, callbackFromPoseStamped);
        break;
      }
      default:
        break;
    }

    // velocity
    switch (vel_mux_select)
    {
      case 0:  // ndt_localizer
      {
        vel_subcscriber = nh.subscribe("estimated_vel", 10, callbackFromVector3Stamped);
        break;
      }
      case 1:  // CAN
      {
        vel_subcscriber = nh.subscribe("can_info", 10, callbackFromCanInfo);
        break;
      }
      default:
        break;
    }
  }

  ros::spin();

  return 0;
}
