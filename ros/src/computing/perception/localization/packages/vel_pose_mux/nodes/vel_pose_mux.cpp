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

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include "vehicle_socket/CanInfo.h"

namespace vel_pose_mux
{

struct VehicleInfo {
  double wheel_base;
  double minimum_turning_radius;
  double maximum_steering_angle;
};

inline double kmph2mps(double velocity_kmph)
{
  return (velocity_kmph * 1000) / (60 * 60);
}

inline double mps2kmph(double velocity_mps)
{
  return (velocity_mps * 60 * 60) / 1000;
}

// convert degree to radian
inline double deg2rad(double deg)
{
  return deg * M_PI / 180;
}

// convert degree to radian
inline double rad2deg(double rad)
{
  return rad * 180 / M_PI;
}

void callbackFromCanInfoAndPublishAsTwistStamped(const vehicle_socket::CanInfoConstPtr &msg,
                                                 ros::Publisher *pub_twist_stamped, ros::Publisher *pub_float,
                                                 const VehicleInfo &vehicle_info)
{
  geometry_msgs::TwistStamped tw;
  tw.header = msg->header;

  // linear velocity
  tw.twist.linear.x = kmph2mps(msg->speed);  // km/h -> m/s

  // angular velocity
  double maximum_tire_angle = rad2deg(asin(vehicle_info.wheel_base / vehicle_info.minimum_turning_radius));  //[degree]
  double current_tire_angle =
      msg->angle * maximum_tire_angle / vehicle_info.maximum_steering_angle;  // steering [degree] -> tire [degree]
  tw.twist.angular.z = sin(deg2rad(current_tire_angle)) * kmph2mps(msg->speed) / vehicle_info.wheel_base;

  pub_twist_stamped->publish(tw);

  std_msgs::Float32 fl;
  fl.data = msg->speed;
  pub_float->publish(fl);
}

void callbackFromCanInfoAndPublishAsTwistStamped(const vehicle_socket::CanInfoConstPtr &msg,
                                                 ros::Publisher *pub_twist_stamped, ros::Publisher *pub_float)
{
  geometry_msgs::TwistStamped tw;
  tw.header = msg->header;

  // linear velocity
  tw.twist.linear.x = kmph2mps(msg->speed);  // km/h -> m/s

  pub_twist_stamped->publish(tw);

  std_msgs::Float32 fl;
  fl.data = msg->speed;
  pub_float->publish(fl);
}

void callbackFromPoseStampedAndPublish(const geometry_msgs::PoseStampedConstPtr &msg, ros::Publisher *pub_message)
{
  pub_message->publish(*msg);
}

void callbackFromTwistStampedAndPublish(const geometry_msgs::TwistStampedConstPtr &msg,
                                        ros::Publisher *pub_twist_stamped, ros::Publisher *pub_float)
{
  pub_twist_stamped->publish(*msg);

  std_msgs::Float32 fl;
  fl.data = mps2kmph(msg->twist.linear.x);
  pub_float->publish(fl);
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
  // ROS_INFO_STREAM("pose_mux_select : " << pose_mux_select);

  private_nh.param<int32_t>("vel_mux_select", vel_mux_select, int32_t(1));
  // ROS_INFO_STREAM("vel_mux_select : " << vel_mux_select);

  private_nh.param<bool>("sim_mode", sim_mode, false);
  // ROS_INFO_STREAM("sim_mode : " << sim_mode);

  bool vehicle_info_flag = false;
  vel_pose_mux::VehicleInfo vehicle_info;
  if (!nh.hasParam("/vehicle_info/wheel_base") || !nh.hasParam("/vehicle_info/minimum_turning_radius") ||
      !nh.hasParam("/vehicle_info/maximum_steering_angle"))
  {
    ROS_INFO("vehicle_info is not set");
  }
  else
  {
    private_nh.getParam("/vehicle_info/wheel_base", vehicle_info.wheel_base);
    // ROS_INFO_STREAM("wheel_base : " << wheel_base);

    private_nh.getParam("/vehicle_info/minimum_turning_radius", vehicle_info.minimum_turning_radius);
    // ROS_INFO_STREAM("minimum_turning_radius : " << minimum_turning_radius);

    private_nh.getParam("/vehicle_info/maximum_steering_angle", vehicle_info.maximum_steering_angle);  //[degree:
    // ROS_INFO_STREAM("maximum_steering_angle : " << maximum_steering_angle);

    vehicle_info_flag = true;
  }

  // publish topic
  ros::Publisher vel_publisher = nh.advertise<geometry_msgs::TwistStamped>("current_velocity", 10);
  ros::Publisher pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("current_pose", 10);
  ros::Publisher linear_viz_publisher = nh.advertise<std_msgs::Float32>("linear_velocity_viz", 10);

  // subscribe topic
  ros::Subscriber pose_subcscriber;
  ros::Subscriber vel_subcscriber;

  if (sim_mode)
  {
    vel_subcscriber = nh.subscribe<geometry_msgs::TwistStamped>(
        "sim_velocity", 10,
        boost::bind(vel_pose_mux::callbackFromTwistStampedAndPublish, _1, &vel_publisher, &linear_viz_publisher));
    pose_subcscriber = nh.subscribe<geometry_msgs::PoseStamped>(
        "sim_pose", 10, boost::bind(vel_pose_mux::callbackFromPoseStampedAndPublish, _1, &pose_publisher));
  }
  else
  {
    // pose
    switch (pose_mux_select)
    {
      case 0:  // ndt_localizer
      {
        pose_subcscriber = nh.subscribe<geometry_msgs::PoseStamped>(
            "ndt_pose", 10, boost::bind(vel_pose_mux::callbackFromPoseStampedAndPublish, _1, &pose_publisher));
        break;
      }
      case 1:  // gnss
      {
        pose_subcscriber = nh.subscribe<geometry_msgs::PoseStamped>(
            "gnss_pose", 10, boost::bind(vel_pose_mux::callbackFromPoseStampedAndPublish, _1, &pose_publisher));
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
        vel_subcscriber = nh.subscribe<geometry_msgs::TwistStamped>(
            "estimate_twist", 10,
            boost::bind(vel_pose_mux::callbackFromTwistStampedAndPublish, _1, &vel_publisher, &linear_viz_publisher));
        break;
      }
      case 1:  // CAN
      {
        if (vehicle_info_flag)  // has vehicle info
        {
          vel_subcscriber = nh.subscribe<vehicle_socket::CanInfo>(
              "can_info", 10, boost::bind(vel_pose_mux::callbackFromCanInfoAndPublishAsTwistStamped, _1, &vel_publisher,
                                          &linear_viz_publisher, vehicle_info));
        }
        else
        {
          vel_subcscriber = nh.subscribe<vehicle_socket::CanInfo>(
              "can_info", 10, boost::bind(vel_pose_mux::callbackFromCanInfoAndPublishAsTwistStamped, _1, &vel_publisher,
                                          &linear_viz_publisher));
        }

        break;
      }
      default:
        break;
    }
  }

  ros::spin();

  return 0;
}
