/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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

#include "test_lane_select.hpp"

namespace lane_planner {

class LaneSelectTestSuite : public ::testing::Test {
public:
  LaneSelectTestSuite() {}
  ~LaneSelectTestSuite() {}

  LaneSelectTestClass test_obj_;

protected:
  virtual void SetUp() { test_obj_.lsn = new LaneSelectNode(); };
  virtual void TearDown() { delete test_obj_.lsn; };
};

TEST_F(LaneSelectTestSuite, publishVehicleLocation) {
  ASSERT_EQ(test_obj_.vehicle_location_sub.getNumPublishers(), 1U)
      << "No publisher exist!";

  test_obj_.publishTrafficWaypointsArray();
  test_obj_.publishCurrentPose(-0.5, 0.0, 0.0);
  test_obj_.publishCurrentVelocity(0);
  ros::spinOnce();
  ros::WallDuration(0.1).sleep();
  test_obj_.lsnSpinOnce();
  ros::WallDuration(0.1).sleep();
  ros::spinOnce();

  ASSERT_EQ(0, test_obj_.cb_vehicle_location.waypoint_index)
      << "Waypoint index does not match."
      << "It should be -1";
  ASSERT_EQ(test_obj_.lane_array_id_,
            test_obj_.cb_vehicle_location.lane_array_id)
      << "LaneArray id does not match."
      << "It should be " << test_obj_.lane_array_id_;

  test_obj_.publishCurrentPose(0.5, 0.0, 0.0);
  test_obj_.publishCurrentVelocity(0);
  ros::spinOnce();
  ros::WallDuration(0.1).sleep();
  test_obj_.lsnSpinOnce();
  ros::WallDuration(0.1).sleep();
  ros::spinOnce();

  ASSERT_EQ(1, test_obj_.cb_vehicle_location.waypoint_index)
      << "Waypoint index does not match."
      << "It should be 1";
}

} // namespace lane_planner

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "LaneSelectTestNode");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
