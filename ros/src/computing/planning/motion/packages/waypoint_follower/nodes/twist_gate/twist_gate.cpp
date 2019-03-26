/*
 *  Copyright (c) 2017, Tier IV, Inc.
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

#include "twist_gate.h"

#include <chrono>

TwistGate::TwistGate(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
  : nh_(nh)
  , private_nh_(private_nh)
  , timeout_period_(1.0)
  , command_mode_(CommandMode::AUTO)
  , previous_command_mode_(CommandMode::AUTO)
{
  node_status_pub_ptr_ = std::make_shared<autoware_health_checker::NodeStatusPublisher>(nh_,private_nh_);
  emergency_stop_pub_ = nh_.advertise<std_msgs::Bool>("/emergency_stop", 1, true);
  control_command_pub_ = nh_.advertise<std_msgs::String>("/ctrl_mode", 1);
  vehicle_cmd_pub_ = nh_.advertise<vehicle_cmd_msg_t>("/vehicle_cmd", 1, true);
  state_cmd_pub_ = nh_.advertise<std_msgs::String>("/state_cmd", 1, false);

  remote_cmd_sub_ = nh_.subscribe("/remote_cmd", 1, &TwistGate::remote_cmd_callback, this);

  auto_cmd_sub_stdmap_["twist_cmd"] = nh_.subscribe("/twist_cmd", 1, &TwistGate::auto_cmd_twist_cmd_callback, this);
  auto_cmd_sub_stdmap_["mode_cmd"] = nh_.subscribe("/mode_cmd", 1, &TwistGate::mode_cmd_callback, this);
  auto_cmd_sub_stdmap_["gear_cmd"] = nh_.subscribe("/gear_cmd", 1, &TwistGate::gear_cmd_callback, this);
  auto_cmd_sub_stdmap_["accel_cmd"] = nh_.subscribe("/accel_cmd", 1, &TwistGate::accel_cmd_callback, this);
  auto_cmd_sub_stdmap_["steer_cmd"] = nh_.subscribe("/steer_cmd", 1, &TwistGate::steer_cmd_callback, this);
  auto_cmd_sub_stdmap_["brake_cmd"] = nh_.subscribe("/brake_cmd", 1, &TwistGate::brake_cmd_callback, this);
  auto_cmd_sub_stdmap_["lamp_cmd"] = nh_.subscribe("/lamp_cmd", 1, &TwistGate::lamp_cmd_callback, this);
  auto_cmd_sub_stdmap_["ctrl_cmd"] = nh_.subscribe("/ctrl_cmd", 1, &TwistGate::ctrl_cmd_callback, this);
  auto_cmd_sub_stdmap_["state"] = nh_.subscribe("/decision_maker/state", 1, &TwistGate::state_callback, this);

  twist_gate_msg_.header.seq = 0;
  emergency_stop_msg_.data = false;
  send_emergency_cmd = false;
  node_status_pub_ptr_->ENABLE();

  remote_cmd_time_ = ros::Time::now();
  watchdog_timer_thread_ = std::thread(&TwistGate::watchdog_timer, this);
  is_alive = true;
}

TwistGate::~TwistGate()
{
  is_alive = false;
  watchdog_timer_thread_.join();
}

void TwistGate::reset_vehicle_cmd_msg()
{
  twist_gate_msg_.twist_cmd.twist.linear.x = 0;
  twist_gate_msg_.twist_cmd.twist.angular.z = 0;
  twist_gate_msg_.mode = 0;
  twist_gate_msg_.gear = 0;
  twist_gate_msg_.lamp_cmd.l = 0;
  twist_gate_msg_.lamp_cmd.r = 0;
  twist_gate_msg_.accel_cmd.accel = 0;
  twist_gate_msg_.brake_cmd.brake = 0;
  twist_gate_msg_.steer_cmd.steer = 0;
  twist_gate_msg_.ctrl_cmd.linear_velocity = -1;
  twist_gate_msg_.ctrl_cmd.steering_angle = 0;
}

bool TwistGate::is_using_decisionmaker()
{
  bool using_decision_maker_flag = false;
  std::vector<std::string> node_list;
  ros::master::getNodes(node_list);

  for (const auto& i : node_list)
  {
    if (i == "/decision_maker")
    {
      using_decision_maker_flag = true;
      break;
    }
  }
  return using_decision_maker_flag;
}

void TwistGate::check_state()
{
  if (is_using_decisionmaker() && !is_state_drive_)
  {
    twist_gate_msg_.twist_cmd.twist = geometry_msgs::Twist();
    twist_gate_msg_.ctrl_cmd = autoware_msgs::ControlCommand();
  }
}

void TwistGate::watchdog_timer()
{
  while (is_alive)
  {
    ros::Time now_time = ros::Time::now();
    bool emergency_flag = false;

    // check command mode
    if (previous_command_mode_ != command_mode_)
    {
      if (command_mode_ == CommandMode::AUTO)
      {
        command_mode_topic_.data = "AUTO";
      }
      else if (command_mode_ == CommandMode::REMOTE)
      {
        command_mode_topic_.data = "REMOTE";
      }
      else
      {
        command_mode_topic_.data = "UNDEFINED";
      }

      control_command_pub_.publish(command_mode_topic_);
      previous_command_mode_ = command_mode_;
    }

    // if lost Communication
    if (command_mode_ == CommandMode::REMOTE && now_time - remote_cmd_time_ > timeout_period_)
    {
      emergency_flag = true;
      ROS_WARN("Lost Communication!");
    }

    // if push emergency stop button
    if (emergency_stop_msg_.data == true)
    {
      emergency_flag = true;
      ROS_WARN("Emergency Mode!");
    }

    // Emergency
    if (emergency_flag)
    {
      // Change Auto Mode
      command_mode_ = CommandMode::AUTO;
      if (send_emergency_cmd == false)
      {
        // Change State to Stop
        std_msgs::String state_cmd;
        state_cmd.data = "emergency";
        state_cmd_pub_.publish(state_cmd);
        send_emergency_cmd = true;
      }
      // Set Emergency Stop
      emergency_stop_pub_.publish(emergency_stop_msg_);
      ROS_WARN("Emergency Stop!");
    }
    else {
      send_emergency_cmd = false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void TwistGate::remote_cmd_callback(const remote_msgs_t::ConstPtr& input_msg)
{
  remote_cmd_time_ = ros::Time::now();
  command_mode_ = static_cast<CommandMode>(input_msg->control_mode);
  emergency_stop_msg_.data = static_cast<bool>(input_msg->vehicle_cmd.emergency);

  // Update Emergency Mode
  twist_gate_msg_.emergency = input_msg->vehicle_cmd.emergency;

  if (command_mode_ == CommandMode::REMOTE && emergency_stop_msg_.data == false)
  {
    twist_gate_msg_.header.frame_id = input_msg->vehicle_cmd.header.frame_id;
    twist_gate_msg_.header.stamp = input_msg->vehicle_cmd.header.stamp;
    twist_gate_msg_.header.seq++;
    twist_gate_msg_.twist_cmd.twist = input_msg->vehicle_cmd.twist_cmd.twist;
    twist_gate_msg_.ctrl_cmd = input_msg->vehicle_cmd.ctrl_cmd;
    twist_gate_msg_.accel_cmd = input_msg->vehicle_cmd.accel_cmd;
    twist_gate_msg_.brake_cmd = input_msg->vehicle_cmd.brake_cmd;
    twist_gate_msg_.steer_cmd = input_msg->vehicle_cmd.steer_cmd;
    twist_gate_msg_.gear = input_msg->vehicle_cmd.gear;
    twist_gate_msg_.lamp_cmd = input_msg->vehicle_cmd.lamp_cmd;
    twist_gate_msg_.mode = input_msg->vehicle_cmd.mode;
    vehicle_cmd_pub_.publish(twist_gate_msg_);
  }
}

void TwistGate::auto_cmd_twist_cmd_callback(const geometry_msgs::TwistStamped::ConstPtr& input_msg)
{
  node_status_pub_ptr_->NODE_ACTIVATE();
  node_status_pub_ptr_->CHECK_RATE("/topic/rate/twist_cmd/slow",8,5,1,"topic twist_cmd subscribe rate low.");
  if(command_mode_ == CommandMode::AUTO)
  {
    twist_gate_msg_.header.frame_id = input_msg->header.frame_id;
    twist_gate_msg_.header.stamp = input_msg->header.stamp;
    twist_gate_msg_.header.seq++;
    twist_gate_msg_.twist_cmd.twist = input_msg->twist;

    check_state();
    vehicle_cmd_pub_.publish(twist_gate_msg_);
  }
}

void TwistGate::mode_cmd_callback(const tablet_socket_msgs::mode_cmd::ConstPtr& input_msg)
{
  if (command_mode_ == CommandMode::AUTO)
  {
    // TODO:check this if statement
    if (input_msg->mode == -1 || input_msg->mode == 0)
    {
      reset_vehicle_cmd_msg();
    }
    twist_gate_msg_.header.frame_id = input_msg->header.frame_id;
    twist_gate_msg_.header.stamp = input_msg->header.stamp;
    twist_gate_msg_.header.seq++;
    twist_gate_msg_.mode = input_msg->mode;
    vehicle_cmd_pub_.publish(twist_gate_msg_);
  }
}

void TwistGate::gear_cmd_callback(const tablet_socket_msgs::gear_cmd::ConstPtr& input_msg)
{
  if (command_mode_ == CommandMode::AUTO)
  {
    twist_gate_msg_.gear = input_msg->gear;
    vehicle_cmd_pub_.publish(twist_gate_msg_);
  }
}

void TwistGate::accel_cmd_callback(const autoware_msgs::AccelCmd::ConstPtr& input_msg)
{
  if (command_mode_ == CommandMode::AUTO)
  {
    twist_gate_msg_.header.frame_id = input_msg->header.frame_id;
    twist_gate_msg_.header.stamp = input_msg->header.stamp;
    twist_gate_msg_.header.seq++;
    twist_gate_msg_.accel_cmd.accel = input_msg->accel;
    vehicle_cmd_pub_.publish(twist_gate_msg_);
  }
}

void TwistGate::steer_cmd_callback(const autoware_msgs::SteerCmd::ConstPtr& input_msg)
{
  if (command_mode_ == CommandMode::AUTO)
  {
    twist_gate_msg_.header.frame_id = input_msg->header.frame_id;
    twist_gate_msg_.header.stamp = input_msg->header.stamp;
    twist_gate_msg_.header.seq++;
    twist_gate_msg_.steer_cmd.steer = input_msg->steer;
    vehicle_cmd_pub_.publish(twist_gate_msg_);
  }
}

void TwistGate::brake_cmd_callback(const autoware_msgs::BrakeCmd::ConstPtr& input_msg)
{
  if (command_mode_ == CommandMode::AUTO)
  {
    twist_gate_msg_.header.frame_id = input_msg->header.frame_id;
    twist_gate_msg_.header.stamp = input_msg->header.stamp;
    twist_gate_msg_.header.seq++;
    twist_gate_msg_.brake_cmd.brake = input_msg->brake;
    vehicle_cmd_pub_.publish(twist_gate_msg_);
  }
}

void TwistGate::lamp_cmd_callback(const autoware_msgs::LampCmd::ConstPtr& input_msg)
{
  if (command_mode_ == CommandMode::AUTO)
  {
    twist_gate_msg_.header.frame_id = input_msg->header.frame_id;
    twist_gate_msg_.header.stamp = input_msg->header.stamp;
    twist_gate_msg_.header.seq++;
    twist_gate_msg_.lamp_cmd.l = input_msg->l;
    twist_gate_msg_.lamp_cmd.r = input_msg->r;
    vehicle_cmd_pub_.publish(twist_gate_msg_);
  }
}

void TwistGate::ctrl_cmd_callback(const autoware_msgs::ControlCommandStamped::ConstPtr& input_msg)
{
  if (command_mode_ == CommandMode::AUTO)
  {
    twist_gate_msg_.header.frame_id = input_msg->header.frame_id;
    twist_gate_msg_.header.stamp = input_msg->header.stamp;
    twist_gate_msg_.header.seq++;
    twist_gate_msg_.ctrl_cmd = input_msg->cmd;

    check_state();
    vehicle_cmd_pub_.publish(twist_gate_msg_);
  }
}

void TwistGate::state_callback(const std_msgs::StringConstPtr& input_msg)
{
  if (command_mode_ == CommandMode::AUTO)
  {
    // Set Parking Gear
    if (input_msg->data.find("WaitOrder") != std::string::npos)
    {
      twist_gate_msg_.gear = CMD_GEAR_P;
    }
    // Set Drive Gear
    else
    {
      twist_gate_msg_.gear = CMD_GEAR_D;
    }

    // get drive state
    if (input_msg->data.find("Drive\n") != std::string::npos && input_msg->data.find("VehicleReady\n") != std::string::npos)
    {
      is_state_drive_ = true;
    }
    else
    {
      is_state_drive_ = false;
    }
    vehicle_cmd_pub_.publish(twist_gate_msg_);

    // reset emergency flags
    if (input_msg->data.find("VehicleReady") != std::string::npos)
    {
      emergency_stop_msg_.data = false;
      send_emergency_cmd = false;
    }
  }
}
