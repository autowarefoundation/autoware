/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef TWIST_GATE_H
#define TWIST_GATE_H

#include <string>
#include <iostream>
#include <map>
#include <thread>
#include <memory>

#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include "autoware_msgs/ControlCommandStamped.h"
#include "autoware_msgs/RemoteCmd.h"
#include "autoware_msgs/VehicleCmd.h"

#include "tablet_socket_msgs/gear_cmd.h"
#include "tablet_socket_msgs/mode_cmd.h"

//headers in Autowae Health Checker
#include <autoware_health_checker/node_status_publisher.h>

#define CMD_GEAR_D 1
#define CMD_GEAR_R 2
#define CMD_GEAR_B 3
#define CMD_GEAR_N 4
#define CMD_GEAR_P 5

class TwistGate
{
  using remote_msgs_t = autoware_msgs::RemoteCmd;
  using vehicle_cmd_msg_t = autoware_msgs::VehicleCmd;

  friend class TwistGateTestClass;

public:
  TwistGate(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
  ~TwistGate();

private:
  void check_state();
  void watchdog_timer();
  void remote_cmd_callback(const remote_msgs_t::ConstPtr& input_msg);
  void auto_cmd_twist_cmd_callback(const geometry_msgs::TwistStamped::ConstPtr& input_msg);
  void mode_cmd_callback(const tablet_socket_msgs::mode_cmd::ConstPtr& input_msg);
  void gear_cmd_callback(const tablet_socket_msgs::gear_cmd::ConstPtr& input_msg);
  void accel_cmd_callback(const autoware_msgs::AccelCmd::ConstPtr& input_msg);
  void steer_cmd_callback(const autoware_msgs::SteerCmd::ConstPtr& input_msg);
  void brake_cmd_callback(const autoware_msgs::BrakeCmd::ConstPtr& input_msg);
  void lamp_cmd_callback(const autoware_msgs::LampCmd::ConstPtr& input_msg);
  void ctrl_cmd_callback(const autoware_msgs::ControlCommandStamped::ConstPtr& input_msg);
  void state_callback(const std_msgs::StringConstPtr& input_msg);

  bool is_using_decisionmaker();
  void reset_vehicle_cmd_msg();

  // spinOnce for test
  void spinOnce(){ ros::spinOnce(); }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  std::shared_ptr<autoware_health_checker::NodeStatusPublisher> node_status_pub_ptr_;
  ros::Publisher emergency_stop_pub_;
  ros::Publisher control_command_pub_;
  ros::Publisher vehicle_cmd_pub_;
  ros::Publisher state_cmd_pub_;
  ros::Subscriber remote_cmd_sub_;
  std::map<std::string, ros::Subscriber> auto_cmd_sub_stdmap_;

  vehicle_cmd_msg_t twist_gate_msg_;
  std_msgs::Bool emergency_stop_msg_;
  ros::Time remote_cmd_time_;
  ros::Duration timeout_period_;

  std::thread watchdog_timer_thread_;
  bool is_alive;

  enum class CommandMode
  {
    AUTO = 1,
    REMOTE = 2
  } command_mode_,
      previous_command_mode_;
  std_msgs::String command_mode_topic_;

  bool is_state_drive_ = true;
  // still send is true
  bool send_emergency_cmd = false;
};

#endif  // TWIST_GATE_H
