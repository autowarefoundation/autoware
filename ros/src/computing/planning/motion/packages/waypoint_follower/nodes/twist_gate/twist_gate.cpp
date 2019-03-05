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

#include <iostream>
#include <thread>
#include <chrono>
#include <map>
#include <memory>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TwistStamped.h>

#include "autoware_msgs/RemoteCmd.h"
#include "autoware_msgs/VehicleCmd.h"
#include "tablet_socket_msgs/mode_cmd.h"
#include "tablet_socket_msgs/gear_cmd.h"
#include "autoware_msgs/AccelCmd.h"
#include "autoware_msgs/BrakeCmd.h"
#include "autoware_msgs/SteerCmd.h"
#include "autoware_msgs/ControlCommandStamped.h"

//headers in Autowae Health Checker
#include <autoware_health_checker/health_checker/health_checker.h>

class TwistGate
{
  using remote_msgs_t = autoware_msgs::RemoteCmd;
  using vehicle_cmd_msg_t = autoware_msgs::VehicleCmd;

  public:
    TwistGate(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    ~TwistGate();
  private:
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

    void reset_vehicle_cmd_msg();

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    std::shared_ptr<autoware_health_checker::HealthChecker> node_status_pub_ptr_;
    ros::Publisher emergency_stop_pub_;
    ros::Publisher control_command_pub_;
    ros::Publisher vehicle_cmd_pub_;
    ros::Publisher state_cmd_pub_;
    ros::Subscriber remote_cmd_sub_;
    std::map<std::string , ros::Subscriber> auto_cmd_sub_stdmap_;

    vehicle_cmd_msg_t twist_gate_msg_;
    std_msgs::Bool emergency_stop_msg_;
    ros::Time remote_cmd_time_;
    ros::Duration timeout_period_;

    std::thread watchdog_timer_thread_;
    enum class CommandMode{AUTO=1, REMOTE=2} command_mode_, previous_command_mode_;
    std_msgs::String command_mode_topic_;

    // still send is true
    bool send_emergency_cmd = false;
};

TwistGate::TwistGate(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh) :
     nh_(nh)
    ,private_nh_(private_nh)
    ,timeout_period_(1.0)
    ,command_mode_(CommandMode::AUTO)
    ,previous_command_mode_(CommandMode::AUTO)
{
  node_status_pub_ptr_ = std::make_shared<autoware_health_checker::HealthChecker>(nh_,private_nh_);
  emergency_stop_pub_ = nh_.advertise<std_msgs::Bool>("/emergency_stop", 1, true);
  control_command_pub_ = nh_.advertise<std_msgs::String>("/ctrl_mode", 1);
  vehicle_cmd_pub_ = nh_.advertise<vehicle_cmd_msg_t>("/vehicle_cmd", 1, true);
  state_cmd_pub_ = nh_.advertise<std_msgs::Int32>("/state_cmd", 1, true);

  remote_cmd_sub_ = nh_.subscribe("/remote_cmd", 1, &TwistGate::remote_cmd_callback, this);

  auto_cmd_sub_stdmap_["twist_cmd"] = nh_.subscribe("/twist_cmd", 1, &TwistGate::auto_cmd_twist_cmd_callback, this);
  auto_cmd_sub_stdmap_["mode_cmd"] = nh_.subscribe("/mode_cmd", 1, &TwistGate::mode_cmd_callback, this);
  auto_cmd_sub_stdmap_["gear_cmd"] = nh_.subscribe("/gear_cmd", 1, &TwistGate::gear_cmd_callback, this);
  auto_cmd_sub_stdmap_["accel_cmd"] = nh_.subscribe("/accel_cmd", 1, &TwistGate::accel_cmd_callback, this);
  auto_cmd_sub_stdmap_["steer_cmd"] = nh_.subscribe("/steer_cmd", 1, &TwistGate::steer_cmd_callback, this);
  auto_cmd_sub_stdmap_["brake_cmd"] = nh_.subscribe("/brake_cmd", 1, &TwistGate::brake_cmd_callback, this);
  auto_cmd_sub_stdmap_["lamp_cmd"] = nh_.subscribe("/lamp_cmd", 1, &TwistGate::lamp_cmd_callback, this);
  auto_cmd_sub_stdmap_["ctrl_cmd"] = nh_.subscribe("/ctrl_cmd", 1, &TwistGate::ctrl_cmd_callback, this);

  twist_gate_msg_.header.seq = 0;
  emergency_stop_msg_.data = false;
  send_emergency_cmd = false;
  node_status_pub_ptr_->ENABLE();

  remote_cmd_time_ = ros::Time::now();
  watchdog_timer_thread_ = std::thread(&TwistGate::watchdog_timer, this);
  watchdog_timer_thread_.detach();
}

TwistGate::~TwistGate()
{
}

void TwistGate::reset_vehicle_cmd_msg()
{
  twist_gate_msg_.twist_cmd.twist.linear.x  = 0;
  twist_gate_msg_.twist_cmd.twist.angular.z = 0;
  twist_gate_msg_.mode                      = 0;
  twist_gate_msg_.gear                      = 0;
  twist_gate_msg_.lamp_cmd.l                = 0;
  twist_gate_msg_.lamp_cmd.r                = 0;
  twist_gate_msg_.accel_cmd.accel           = 0;
  twist_gate_msg_.brake_cmd.brake           = 0;
  twist_gate_msg_.steer_cmd.steer           = 0;
  twist_gate_msg_.ctrl_cmd.linear_velocity  = -1;
  twist_gate_msg_.ctrl_cmd.steering_angle   = 0;
}

void TwistGate::watchdog_timer()
{
  while(1)
  {
    ros::Time now_time = ros::Time::now();
    bool emergency_flag = false;

    // check command mode
    if(previous_command_mode_ != command_mode_) {
      if(command_mode_ == CommandMode::AUTO) {
        command_mode_topic_.data = "AUTO";
      }
      else if(command_mode_ == CommandMode::REMOTE) {
        command_mode_topic_.data = "REMOTE";
      }
      else{
        command_mode_topic_.data = "UNDEFINED";
      }

      control_command_pub_.publish(command_mode_topic_);
      previous_command_mode_ = command_mode_;
    }

    // if lost Communication
    if(command_mode_ == CommandMode::REMOTE && now_time - remote_cmd_time_ >  timeout_period_) {
      emergency_flag = true;
      ROS_WARN("Lost Communication!");
    }

    // if push emergency stop button
    if(emergency_stop_msg_.data == true)
    {
      emergency_flag = true;
      ROS_WARN("Emergency Mode!");
    }

    // Emergency
    if(emergency_flag) {
      // Change Auto Mode
      command_mode_ = CommandMode::AUTO;
      if(send_emergency_cmd == false) {
        // Change State to Stop
        std_msgs::Int32 state_cmd;
        state_cmd.data = 14;
        state_cmd_pub_.publish(state_cmd);
        send_emergency_cmd = true;
      }
      // Set Emergency Stop
      emergency_stop_msg_.data = true;
      emergency_stop_pub_.publish(emergency_stop_msg_);
      ROS_WARN("Emergency Stop!");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void TwistGate::remote_cmd_callback(const remote_msgs_t::ConstPtr& input_msg)
{
  command_mode_ = static_cast<CommandMode>(input_msg->control_mode);
  emergency_stop_msg_.data = static_cast<bool>(input_msg->vehicle_cmd.emergency);
  remote_cmd_time_ = ros::Time::now();

  if(command_mode_ == CommandMode::REMOTE)
  {
    twist_gate_msg_.header.frame_id = input_msg->vehicle_cmd.header.frame_id;
    twist_gate_msg_.header.stamp = input_msg->vehicle_cmd.header.stamp;
    twist_gate_msg_.header.seq++;
    twist_gate_msg_.twist_cmd.twist = input_msg->vehicle_cmd.twist_cmd.twist;
    twist_gate_msg_.ctrl_cmd  = input_msg->vehicle_cmd.ctrl_cmd;
    twist_gate_msg_.accel_cmd = input_msg->vehicle_cmd.accel_cmd;
    twist_gate_msg_.brake_cmd = input_msg->vehicle_cmd.brake_cmd;
    twist_gate_msg_.steer_cmd = input_msg->vehicle_cmd.steer_cmd;
    twist_gate_msg_.gear = input_msg->vehicle_cmd.gear;
    twist_gate_msg_.lamp_cmd = input_msg->vehicle_cmd.lamp_cmd;
    twist_gate_msg_.mode = input_msg->vehicle_cmd.mode;
    twist_gate_msg_.emergency = input_msg->vehicle_cmd.emergency;
    vehicle_cmd_pub_.publish(twist_gate_msg_);
  }
}

void TwistGate::auto_cmd_twist_cmd_callback(const geometry_msgs::TwistStamped::ConstPtr& input_msg)
{
  node_status_pub_ptr_->NODE_ACTIVATE();
  if(command_mode_ == CommandMode::AUTO)
  {
    twist_gate_msg_.header.frame_id = input_msg->header.frame_id;
    twist_gate_msg_.header.stamp = input_msg->header.stamp;
    twist_gate_msg_.header.seq++;
    twist_gate_msg_.twist_cmd.twist = input_msg->twist;
    vehicle_cmd_pub_.publish(twist_gate_msg_);
  }
}

void TwistGate::mode_cmd_callback(const tablet_socket_msgs::mode_cmd::ConstPtr& input_msg)
{
  if(command_mode_ == CommandMode::AUTO)
  {
    //TODO:check this if statement
    if(input_msg->mode == -1 || input_msg->mode == 0){
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
  if(command_mode_ == CommandMode::AUTO)
  {
    twist_gate_msg_.gear = input_msg->gear;
    vehicle_cmd_pub_.publish(twist_gate_msg_);
  }
}

void TwistGate::accel_cmd_callback(const autoware_msgs::AccelCmd::ConstPtr& input_msg)
{
  if(command_mode_ == CommandMode::AUTO)
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
  if(command_mode_ == CommandMode::AUTO)
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
  if(command_mode_ == CommandMode::AUTO)
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
  if(command_mode_ == CommandMode::AUTO)
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
  if(command_mode_ == CommandMode::AUTO)
  {
    twist_gate_msg_.header.frame_id = input_msg->header.frame_id;
    twist_gate_msg_.header.stamp = input_msg->header.stamp;
    twist_gate_msg_.header.seq++;
    twist_gate_msg_.ctrl_cmd = input_msg->cmd;
    vehicle_cmd_pub_.publish(twist_gate_msg_);
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "twist_gate");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  TwistGate twist_gate(nh, private_nh);

  ros::spin();
  return 0;
}
