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

#include "amathutils_lib/amathutils.hpp"
#include "decision_maker_node.hpp"

#include "test_class.hpp"

namespace decision_maker {

class TestSuite : public ::testing::Test {
public:
  TestSuite() {}
  ~TestSuite() {}

  TestClass test_obj_;

protected:
  virtual void SetUp() {
    int argc;
    char **argv;
    test_obj_.dmn = new DecisionMakerNode(argc, argv);
  };
  virtual void TearDown() { delete test_obj_.dmn; };
};

TEST_F(TestSuite, getSteeringStateFromWaypoint) {
	test_obj_.createFinalWaypoints();
  ASSERT_EQ(test_obj_.getSteeringStateFromWaypoint(),
            autoware_msgs::WaypointState::STR_STRAIGHT)
      << "Could not get the expected state"
      << "It should be " << autoware_msgs::WaypointState::STR_STRAIGHT;

  test_obj_.setSteeringState(20, autoware_msgs::WaypointState::STR_LEFT);
  ASSERT_EQ(test_obj_.getSteeringStateFromWaypoint(),
            autoware_msgs::WaypointState::STR_LEFT)
      << "Could not get the expected state"
      << "It should be " << autoware_msgs::WaypointState::STR_LEFT;

  test_obj_.setSteeringState(20, autoware_msgs::WaypointState::STR_RIGHT);
  ASSERT_EQ(test_obj_.getSteeringStateFromWaypoint(),
            autoware_msgs::WaypointState::STR_RIGHT)
      << "Could not get the expected state"
      << "It should be " << autoware_msgs::WaypointState::STR_RIGHT;
}

TEST_F(TestSuite, getEventStateFromWaypoint) {
	test_obj_.setCurrentPose(0, 0, 0);

	test_obj_.createFinalWaypoints();
  ASSERT_EQ(test_obj_.getEventStateFromWaypoint(),
            autoware_msgs::WaypointState::TYPE_EVENT_NULL)
      << "Could not get the expected state"
      << "It should be " << autoware_msgs::WaypointState::TYPE_EVENT_NULL;

  test_obj_.setEventState(20, autoware_msgs::WaypointState::TYPE_EVENT_GOAL);
  ASSERT_EQ(test_obj_.getEventStateFromWaypoint(),
            autoware_msgs::WaypointState::TYPE_EVENT_GOAL)
      << "Could not get the expected state"
      << "It should be " << autoware_msgs::WaypointState::TYPE_EVENT_GOAL;

  test_obj_.setEventState(20, autoware_msgs::WaypointState::TYPE_EVENT_MIDDLE_GOAL);
  ASSERT_EQ(test_obj_.getEventStateFromWaypoint(),
            autoware_msgs::WaypointState::TYPE_EVENT_MIDDLE_GOAL)
      << "Could not get the expected state"
      << "It should be "
      << autoware_msgs::WaypointState::TYPE_EVENT_MIDDLE_GOAL;

  test_obj_.setEventState(20, autoware_msgs::WaypointState::TYPE_EVENT_POSITION_STOP);
  ASSERT_EQ(test_obj_.getEventStateFromWaypoint(),
            autoware_msgs::WaypointState::TYPE_EVENT_POSITION_STOP)
      << "Could not get the expected state"
      << "It should be "
      << autoware_msgs::WaypointState::TYPE_EVENT_POSITION_STOP;

  test_obj_.setEventState(20, autoware_msgs::WaypointState::TYPE_EVENT_BUS_STOP);
  ASSERT_EQ(test_obj_.getEventStateFromWaypoint(),
            autoware_msgs::WaypointState::TYPE_EVENT_BUS_STOP)
      << "Could not get the expected state"
      << "It should be " << autoware_msgs::WaypointState::TYPE_EVENT_BUS_STOP;

  test_obj_.setEventState(20, autoware_msgs::WaypointState::TYPE_EVENT_PARKING);
  ASSERT_EQ(test_obj_.getEventStateFromWaypoint(),
            autoware_msgs::WaypointState::TYPE_EVENT_PARKING)
      << "Could not get the expected state"
      << "It should be " << autoware_msgs::WaypointState::TYPE_EVENT_PARKING;
}

TEST_F(TestSuite, getStopSignStateFromWaypoint) {
	test_obj_.setCurrentPose(0, 0, 0);
	test_obj_.setCurrentVelocity(10.0);

	test_obj_.createFinalWaypoints();
  std::pair<uint8_t, int> ret1 = test_obj_.getStopSignStateFromWaypoint();
  ASSERT_EQ(ret1.first, autoware_msgs::WaypointState::NULLSTATE)
      << "Could not get the expected state"
      << "It should be " << autoware_msgs::WaypointState::NULLSTATE;
  ASSERT_EQ(ret1.second, -1) << "Could not get the expected state"
                             << "It should be " << -1;

  test_obj_.setStopState(20, autoware_msgs::WaypointState::TYPE_STOPLINE);
  std::pair<uint8_t, int> ret2 = test_obj_.getStopSignStateFromWaypoint();
  ASSERT_EQ(ret2.first, autoware_msgs::WaypointState::TYPE_STOPLINE)
      << "Could not get the expected state"
      << "It should be " << autoware_msgs::WaypointState::TYPE_STOPLINE;
  ASSERT_EQ(ret2.second, 20) << "Could not get the expected state"
                             << "It should be " << 20;

  test_obj_.setStopState(20, autoware_msgs::WaypointState::TYPE_STOP);
  std::pair<uint8_t, int> ret3 = test_obj_.getStopSignStateFromWaypoint();
  ASSERT_EQ(ret3.first, autoware_msgs::WaypointState::TYPE_STOP)
      << "Could not get the expected state"
      << "It should be " << autoware_msgs::WaypointState::TYPE_STOP;
  ASSERT_EQ(ret3.second, 20) << "Could not get the expected state"
                             << "It should be " << 20;
}

} // namespace decision_maker
