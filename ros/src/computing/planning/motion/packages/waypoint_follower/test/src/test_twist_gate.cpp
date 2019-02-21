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

#include <ros/ros.h>
#include <gtest/gtest.h>

#include "test_twist_gate.hpp"

class TwistGateTestSuite : public ::testing::Test
{
public:
  TwistGateTestSuite(){}
  ~TwistGateTestSuite(){}

  TwistGateTestClass test_obj_;

protected:
  virtual void SetUp()
  {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    test_obj_.tg = new TwistGate(nh, private_nh);
  };
  virtual void TearDown()
  {
    delete test_obj_.tg;
  };
};

TEST_F(TwistGateTestSuite, twistCmdCallback)
{
  double linear_x = 5.0;
  double angular_z = 1.5;
  test_obj_.publishTwistCmd(linear_x, angular_z);
  ros::spinOnce();
  ros::WallDuration(0.1).sleep();
  test_obj_.tgSpinOnce();
  ros::WallDuration(0.1).sleep();
  ros::spinOnce();

  ASSERT_EQ(linear_x, test_obj_.cb_vehicle_cmd.twist_cmd.twist.linear.x);
  ASSERT_EQ(angular_z, test_obj_.cb_vehicle_cmd.twist_cmd.twist.angular.z);
}

TEST_F(TwistGateTestSuite, controlCmdCallback)
{
  double linear_vel = 5.0;
  double linear_acc = 1.5;
  double steer_angle = 1.57;
  test_obj_.publishControlCmd(linear_vel, linear_acc, steer_angle);
  ros::spinOnce();
  ros::WallDuration(0.1).sleep();
  test_obj_.tgSpinOnce();
  ros::WallDuration(0.1).sleep();
  ros::spinOnce();

  ASSERT_EQ(linear_vel, test_obj_.cb_vehicle_cmd.ctrl_cmd.linear_velocity);
  ASSERT_EQ(linear_acc, test_obj_.cb_vehicle_cmd.ctrl_cmd.linear_acceleration);
  ASSERT_EQ(steer_angle, test_obj_.cb_vehicle_cmd.ctrl_cmd.steering_angle);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "TwistGateTestNode");
  return RUN_ALL_TESTS();
}
