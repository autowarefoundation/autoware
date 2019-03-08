/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
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

#include <gtest/gtest.h>
#include <ros/ros.h>

#include "twist_gate.h"

class TwistGateTestClass {
public:
  TwistGateTestClass() {
    twist_cmd_publisher =
        nh.advertise<geometry_msgs::TwistStamped>("twist_cmd", 0);
    control_cmd_publisher =
        nh.advertise<autoware_msgs::ControlCommandStamped>("ctrl_cmd", 0);
    decision_maker_state_publisher =
        nh.advertise<std_msgs::String>("decision_maker/state", 0);
    remote_cmd_publisher = nh.advertise<autoware_msgs::RemoteCmd>("remote_cmd", 0);
    mode_cmd_publisher = nh.advertise<tablet_socket_msgs::mode_cmd>("mode_cmd", 0);
    gear_cmd_publisher = nh.advertise<tablet_socket_msgs::gear_cmd>("gear_cmd", 0);
    accel_cmd_publisher = nh.advertise<autoware_msgs::AccelCmd>("accel_cmd", 0);
    steer_cmd_publisher = nh.advertise<autoware_msgs::SteerCmd>("steer_cmd", 0);
    brake_cmd_publisher = nh.advertise<autoware_msgs::BrakeCmd>("brake_cmd", 0);
    lamp_cmd_publisher = nh.advertise<autoware_msgs::LampCmd>("lamp_cmd", 0);
    vehicle_cmd_subscriber = nh.subscribe(
        "/vehicle_cmd", 1, &TwistGateTestClass::vehicleCmdCallback, this);
    state_cmd_subscriber = nh.subscribe(
        "/state_cmd", 1, &TwistGateTestClass::stateCmdCallback, this);
  }

  TwistGate *tg;
  autoware_msgs::VehicleCmd cb_vehicle_cmd;
  std_msgs::String cb_state_cmd;

  ros::NodeHandle nh;
  ros::Publisher twist_cmd_publisher, control_cmd_publisher, remote_cmd_publisher, mode_cmd_publisher, gear_cmd_publisher, accel_cmd_publisher, steer_cmd_publisher, brake_cmd_publisher, lamp_cmd_publisher, decision_maker_state_publisher;
  ros::Subscriber vehicle_cmd_subscriber, state_cmd_subscriber;

  void tgSpinOnce() { tg->spinOnce(); }

  void tgResetVehicleCmdMsg() { tg->reset_vehicle_cmd_msg(); }

  autoware_msgs::VehicleCmd setTgTwistGateMsg(double d_value, int i_value) {
    tg->twist_gate_msg_.twist_cmd.twist.linear.x = d_value;
    tg->twist_gate_msg_.twist_cmd.twist.angular.z = d_value;
    tg->twist_gate_msg_.mode = i_value;
    tg->twist_gate_msg_.gear = i_value;
    tg->twist_gate_msg_.lamp_cmd.l = i_value;
    tg->twist_gate_msg_.lamp_cmd.r = i_value;
    tg->twist_gate_msg_.accel_cmd.accel = i_value;
    tg->twist_gate_msg_.brake_cmd.brake = i_value;
    tg->twist_gate_msg_.steer_cmd.steer = i_value;
    tg->twist_gate_msg_.ctrl_cmd.linear_velocity = i_value;
    tg->twist_gate_msg_.ctrl_cmd.steering_angle = i_value;

    return tg->twist_gate_msg_;
  }

  autoware_msgs::VehicleCmd getTgTwistGateMsg() {return tg->twist_gate_msg_;}

  void publishTwistCmd(double linear_x, double angular_z) {
    geometry_msgs::TwistStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.twist.linear.x = linear_x;
    msg.twist.angular.z = angular_z;

    twist_cmd_publisher.publish(msg);
  }

  void publishControlCmd(double linear_vel, double linear_acc,
                         double steer_angle) {
    autoware_msgs::ControlCommandStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.cmd.linear_velocity = linear_vel;
    msg.cmd.linear_acceleration = linear_acc;
    msg.cmd.steering_angle = steer_angle;

    control_cmd_publisher.publish(msg);
  }

  void publishRemoteCmd(autoware_msgs::RemoteCmd remote_cmd){
    remote_cmd_publisher.publish(remote_cmd);
  }

  void publishModeCmd(int mode){
    tablet_socket_msgs::mode_cmd msg;
    msg.header.stamp = ros::Time::now();
    msg.mode = mode;

    mode_cmd_publisher.publish(msg);
  }

  void publishGearCmd(int gear){
    tablet_socket_msgs::gear_cmd msg;
    msg.header.stamp = ros::Time::now();
    msg.gear = gear;

    gear_cmd_publisher.publish(msg);
  }

  void publishAccelCmd(int accel){
    autoware_msgs::AccelCmd msg;
    msg.header.stamp = ros::Time::now();
    msg.accel = accel;

    accel_cmd_publisher.publish(msg);
  }

  void publishSteerCmd(int steer){
    autoware_msgs::SteerCmd msg;
    msg.header.stamp = ros::Time::now();
    msg.steer = steer;

    steer_cmd_publisher.publish(msg);
  }

  void publishBrakeCmd(int brake){
    autoware_msgs::BrakeCmd msg;
    msg.header.stamp = ros::Time::now();
    msg.brake = brake;

    brake_cmd_publisher.publish(msg);
  }

  void publishLampCmd(int lamp_l, int lamp_r){
    autoware_msgs::LampCmd msg;
    msg.header.stamp = ros::Time::now();
    msg.l = lamp_l;
    msg.r = lamp_r;

    lamp_cmd_publisher.publish(msg);
  }

  void publishDecisionMakerState(std::string states) {
    std_msgs::String msg;
    msg.data = states;

    decision_maker_state_publisher.publish(msg);
  }

  void vehicleCmdCallback(autoware_msgs::VehicleCmd msg) {
    cb_vehicle_cmd = msg;
  }

  void stateCmdCallback(std_msgs::String msg) {
    cb_state_cmd = msg;
  }

  autoware_msgs::VehicleCmd getTwistGateMsg() { return tg->twist_gate_msg_; }

  bool getIsStateDriveFlag() { return tg->is_state_drive_; }
};
