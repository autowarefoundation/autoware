/*
 *  Copyright (c) 2017, Nagoya University
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
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"geometry
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

#ifndef PACMOD_INTERFACE_H
#define PACMOD_INTERFACE_H

// ROS includes
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <module_comm_msgs/SteerMode.h>
#include <module_comm_msgs/SpeedMode.h>
#include <module_comm_msgs/VelocityAccel.h>
#include <platform_comm_msgs/CurvatureFeedback.h>
#include <platform_comm_msgs/TurnSignalCommand.h>
#include <platform_comm_msgs/GearCommand.h>
#include <platform_comm_msgs/Gear.h>

#include <autoware_msgs/lamp_cmd.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>

namespace pacmod
{
class PacmodInterface
{
public:
  PacmodInterface();
  ~PacmodInterface();

  void run();

private:
  typedef message_filters::sync_policies::ApproximateTime<module_comm_msgs::VelocityAccel, platform_comm_msgs::CurvatureFeedback> CurrentTwistSyncPolicy;

  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // publisher
  ros::Publisher steer_mode_pub_;
  ros::Publisher speed_mode_pub_;
  ros::Publisher turn_signal_pub_;
  ros::Publisher gear_pub_;
  ros::Publisher current_twist_pub_;
  ros::Publisher velocity_overlay_pub_;

  // subscriber
  ros::Subscriber twist_cmd_sub_;
  ros::Subscriber control_mode_sub_;
  ros::Subscriber curvature_cmd_sub_;
  ros::Subscriber lamp_cmd_sub_;
  ros::Subscriber speed_sub_;
  ros::Subscriber lidar_detect_cmd_sub_;

  message_filters::Subscriber<module_comm_msgs::VelocityAccel>* current_velocity_sub_;
  message_filters::Subscriber<platform_comm_msgs::CurvatureFeedback>* current_curvature_sub_;
  message_filters::Synchronizer<CurrentTwistSyncPolicy>* current_twist_sync_;

  // timer
  ros::Timer pacmod_timer_;

  // ros param
  double acceleration_limit_;
  double deceleration_limit_;
  double max_curvature_rate_;
  bool use_curvature_cmd_;
  bool use_timer_publisher_;
  double publish_frequency_;

  // constants
  static constexpr double minimum_linear_x_ = 1e-6;

  // variables
  bool control_mode_ = false;
  double speed_ = 0.0;
  double curvature_ = 0.0;
  std_msgs::Header header_;
  autoware_msgs::lamp_cmd lamp_cmd_;
  uint8_t lidar_detect_cmd_;

  // callbacks
  void callbackFromTwistCmd(const geometry_msgs::TwistStampedConstPtr& msg);
  void callbackFromControlMode(const std_msgs::BoolConstPtr& msg);
  void callbackFromSyncedCurrentTwist(const module_comm_msgs::VelocityAccelConstPtr& msg_velocity, const platform_comm_msgs::CurvatureFeedbackConstPtr& msg_curvature);
  void callbackPacmodTimer(const ros::TimerEvent& event);
  void callbackFromLampCmd(const autoware_msgs::lamp_cmdConstPtr& msg);
  void callbackLidarDetectCmd(const std_msgs::UInt8ConstPtr msg);

  // publisher
  void publishToPacmod();

  // initializer
  void initForROS();
};
}  // pacmod
#endif  // PACMOD_INTERFACE_H
