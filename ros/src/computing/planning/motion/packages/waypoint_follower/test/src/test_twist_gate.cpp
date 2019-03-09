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

#include "test_twist_gate.hpp"

class TwistGateTestSuite : public ::testing::Test {
public:
  TwistGateTestSuite() {}
  ~TwistGateTestSuite() {}

  TwistGateTestClass test_obj_;

protected:
  virtual void SetUp() {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    test_obj_.tg = new TwistGate(nh, private_nh);
  };
  virtual void TearDown() { delete test_obj_.tg; };
};

TEST_F(TwistGateTestSuite, resetVehicelCmd) {
  double d_value = 1.5;
  int i_value = 1;

  autoware_msgs::VehicleCmd msg = test_obj_.setTgTwistGateMsg(d_value, i_value);

  ASSERT_EQ(d_value, msg.twist_cmd.twist.linear.x);
  ASSERT_EQ(d_value, msg.twist_cmd.twist.angular.z);
  ASSERT_EQ(i_value, msg.mode);
  ASSERT_EQ(i_value, msg.gear);
  ASSERT_EQ(i_value, msg.lamp_cmd.l);
  ASSERT_EQ(i_value, msg.lamp_cmd.r);
  ASSERT_EQ(i_value, msg.accel_cmd.accel);
  ASSERT_EQ(i_value, msg.brake_cmd.brake);
  ASSERT_EQ(i_value, msg.steer_cmd.steer);
  ASSERT_EQ(i_value, msg.ctrl_cmd.linear_velocity);
  ASSERT_EQ(i_value, msg.ctrl_cmd.steering_angle);

  test_obj_.tgResetVehicleCmdMsg();
  msg = test_obj_.getTgTwistGateMsg();

  ASSERT_EQ(0, msg.twist_cmd.twist.linear.x);
  ASSERT_EQ(0, msg.twist_cmd.twist.angular.z);
  ASSERT_EQ(0, msg.mode);
  ASSERT_EQ(0, msg.gear);
  ASSERT_EQ(0, msg.lamp_cmd.l);
  ASSERT_EQ(0, msg.lamp_cmd.r);
  ASSERT_EQ(0, msg.accel_cmd.accel);
  ASSERT_EQ(0, msg.brake_cmd.brake);
  ASSERT_EQ(0, msg.steer_cmd.steer);
  ASSERT_EQ(-1, msg.ctrl_cmd.linear_velocity);
  ASSERT_EQ(0, msg.ctrl_cmd.steering_angle);
}

TEST_F(TwistGateTestSuite, twistCmdCallback) {
  double linear_x = 5.0;
  double angular_z = 1.5;
  test_obj_.publishTwistCmd(linear_x, angular_z);
  ros::WallDuration(0.1).sleep();
  test_obj_.tgSpinOnce();
  ros::WallDuration(0.1).sleep();
  ros::spinOnce();

  ASSERT_EQ(linear_x, test_obj_.cb_vehicle_cmd.twist_cmd.twist.linear.x);
  ASSERT_EQ(angular_z, test_obj_.cb_vehicle_cmd.twist_cmd.twist.angular.z);
}

TEST_F(TwistGateTestSuite, controlCmdCallback) {
  double linear_vel = 5.0;
  double linear_acc = 1.5;
  double steer_angle = 1.57;
  test_obj_.publishControlCmd(linear_vel, linear_acc, steer_angle);
  ros::WallDuration(0.1).sleep();
  test_obj_.tgSpinOnce();
  ros::WallDuration(0.1).sleep();
  ros::spinOnce();

  ASSERT_EQ(linear_vel, test_obj_.cb_vehicle_cmd.ctrl_cmd.linear_velocity);
  ASSERT_EQ(linear_acc, test_obj_.cb_vehicle_cmd.ctrl_cmd.linear_acceleration);
  ASSERT_EQ(steer_angle, test_obj_.cb_vehicle_cmd.ctrl_cmd.steering_angle);
}

TEST_F(TwistGateTestSuite, remoteCmdCallback) {
  autoware_msgs::RemoteCmd remote_cmd;
  remote_cmd.vehicle_cmd.header.frame_id = "/test_frame";
  remote_cmd.vehicle_cmd.header.stamp = ros::Time::now();
  remote_cmd.vehicle_cmd.twist_cmd.twist.linear.x = 5.0;
  remote_cmd.vehicle_cmd.twist_cmd.twist.angular.z = 0.785;
  remote_cmd.vehicle_cmd.ctrl_cmd.linear_velocity = 4.0;
  remote_cmd.vehicle_cmd.ctrl_cmd.linear_acceleration = 3.0;
  remote_cmd.vehicle_cmd.ctrl_cmd.steering_angle = 0.393;
  remote_cmd.vehicle_cmd.accel_cmd.accel = 10;
  remote_cmd.vehicle_cmd.brake_cmd.brake = 20;
  remote_cmd.vehicle_cmd.steer_cmd.steer = 30;
  remote_cmd.vehicle_cmd.gear = CMD_GEAR_P;
  remote_cmd.vehicle_cmd.lamp_cmd.l = 1;
  remote_cmd.vehicle_cmd.lamp_cmd.r = 1;
  remote_cmd.vehicle_cmd.mode = 6;
  remote_cmd.vehicle_cmd.emergency = 0;
  remote_cmd.control_mode = 2;

  test_obj_.publishRemoteCmd(remote_cmd);
  ros::WallDuration(0.1).sleep();
  test_obj_.tgSpinOnce();
  ros::WallDuration(0.1).sleep();
  ros::spinOnce();

  ASSERT_EQ(remote_cmd.vehicle_cmd.header.frame_id
      , test_obj_.cb_vehicle_cmd.header.frame_id);
  ASSERT_EQ(remote_cmd.vehicle_cmd.header.stamp
      , test_obj_.cb_vehicle_cmd.header.stamp);
  ASSERT_EQ(remote_cmd.vehicle_cmd.twist_cmd.twist.linear.x
      , test_obj_.cb_vehicle_cmd.twist_cmd.twist.linear.x);
  ASSERT_EQ(remote_cmd.vehicle_cmd.twist_cmd.twist.angular.z
      , test_obj_.cb_vehicle_cmd.twist_cmd.twist.angular.z);
  ASSERT_EQ(remote_cmd.vehicle_cmd.ctrl_cmd.linear_velocity
      , test_obj_.cb_vehicle_cmd.ctrl_cmd.linear_velocity);
  ASSERT_EQ(remote_cmd.vehicle_cmd.ctrl_cmd.linear_acceleration
      , test_obj_.cb_vehicle_cmd.ctrl_cmd.linear_acceleration);
  ASSERT_EQ(remote_cmd.vehicle_cmd.ctrl_cmd.steering_angle
      , test_obj_.cb_vehicle_cmd.ctrl_cmd.steering_angle);
  ASSERT_EQ(remote_cmd.vehicle_cmd.accel_cmd.accel
      , test_obj_.cb_vehicle_cmd.accel_cmd.accel);
  ASSERT_EQ(remote_cmd.vehicle_cmd.brake_cmd.brake
      , test_obj_.cb_vehicle_cmd.brake_cmd.brake);
  ASSERT_EQ(remote_cmd.vehicle_cmd.steer_cmd.steer
      , test_obj_.cb_vehicle_cmd.steer_cmd.steer);
  ASSERT_EQ(remote_cmd.vehicle_cmd.gear
      , test_obj_.cb_vehicle_cmd.gear);
  ASSERT_EQ(remote_cmd.vehicle_cmd.lamp_cmd.l
      , test_obj_.cb_vehicle_cmd.lamp_cmd.l);
  ASSERT_EQ(remote_cmd.vehicle_cmd.lamp_cmd.r
      , test_obj_.cb_vehicle_cmd.lamp_cmd.r);
  ASSERT_EQ(remote_cmd.vehicle_cmd.mode
      , test_obj_.cb_vehicle_cmd.mode);
}


TEST_F(TwistGateTestSuite, remoteCmdEmergency) {
  autoware_msgs::RemoteCmd remote_cmd;
  remote_cmd.vehicle_cmd.emergency = 1;

  test_obj_.publishRemoteCmd(remote_cmd);
  ros::WallDuration(0.1).sleep();
  test_obj_.tgSpinOnce();
  ros::WallDuration(0.1).sleep();
  ros::spinOnce();

  ASSERT_EQ("emergency", test_obj_.cb_state_cmd.data);
}

TEST_F(TwistGateTestSuite, modeCmdCallback) {
  int mode = 8;

  test_obj_.publishModeCmd(mode);
  ros::WallDuration(0.1).sleep();
  test_obj_.tgSpinOnce();
  ros::WallDuration(0.1).sleep();
  ros::spinOnce();

  ASSERT_EQ(mode, test_obj_.cb_vehicle_cmd.mode);
}

TEST_F(TwistGateTestSuite, gearCmdCallback) {
  int gear = CMD_GEAR_D;

  test_obj_.publishGearCmd(gear);
  ros::WallDuration(0.1).sleep();
  test_obj_.tgSpinOnce();
  ros::WallDuration(0.1).sleep();
  ros::spinOnce();

  ASSERT_EQ(gear, test_obj_.cb_vehicle_cmd.gear);
}

TEST_F(TwistGateTestSuite, accelCmdCallback) {
  int accel = 100;

  test_obj_.publishAccelCmd(accel);
  ros::WallDuration(0.1).sleep();
  test_obj_.tgSpinOnce();
  ros::WallDuration(0.1).sleep();
  ros::spinOnce();

  ASSERT_EQ(accel, test_obj_.cb_vehicle_cmd.accel_cmd.accel);
}

TEST_F(TwistGateTestSuite, steerCmdCallback) {
  int steer = 100;

  test_obj_.publishSteerCmd(steer);
  ros::WallDuration(0.1).sleep();
  test_obj_.tgSpinOnce();
  ros::WallDuration(0.1).sleep();
  ros::spinOnce();

  ASSERT_EQ(steer, test_obj_.cb_vehicle_cmd.steer_cmd.steer);
}

TEST_F(TwistGateTestSuite, brakeCmdCallback) {
  int brake = 100;

  test_obj_.publishBrakeCmd(brake);
  ros::WallDuration(0.1).sleep();
  test_obj_.tgSpinOnce();
  ros::WallDuration(0.1).sleep();
  ros::spinOnce();

  ASSERT_EQ(brake, test_obj_.cb_vehicle_cmd.brake_cmd.brake);
}

TEST_F(TwistGateTestSuite, lampCmdCallback) {
  int lamp_l = 1;
  int lamp_r = 1;

  test_obj_.publishLampCmd(lamp_l, lamp_r);
  ros::WallDuration(0.1).sleep();
  test_obj_.tgSpinOnce();
  ros::WallDuration(0.1).sleep();
  ros::spinOnce();

  ASSERT_EQ(lamp_l, test_obj_.cb_vehicle_cmd.lamp_cmd.l);
  ASSERT_EQ(lamp_r, test_obj_.cb_vehicle_cmd.lamp_cmd.r);
}

TEST_F(TwistGateTestSuite, stateCallback) {
  test_obj_.publishDecisionMakerState("VehicleReady\nWaitOrder\nWaitEngage\n");
  test_obj_.tgSpinOnce();
  ros::WallDuration(0.1).sleep();
  ros::spinOnce();

  autoware_msgs::VehicleCmd tg_msg = test_obj_.getTwistGateMsg();
  ASSERT_EQ(CMD_GEAR_P, tg_msg.gear);
  ASSERT_EQ(false, test_obj_.getIsStateDriveFlag());

  test_obj_.publishDecisionMakerState("VehicleReady\nDriving\nDrive\n");
  test_obj_.tgSpinOnce();
  ros::WallDuration(0.1).sleep();
  ros::spinOnce();

  tg_msg = test_obj_.getTwistGateMsg();
  ASSERT_EQ(CMD_GEAR_D, tg_msg.gear);
  ASSERT_EQ(true, test_obj_.getIsStateDriveFlag());
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "TwistGateTestNode");
  return RUN_ALL_TESTS();
}
