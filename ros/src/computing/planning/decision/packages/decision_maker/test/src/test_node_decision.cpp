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

TEST_F(TestSuite, isLocalizationConvergence) {
  geometry_msgs::Point pointA, pointB;
  pointA.x = 10.0;
  pointA.y = 10.0;
  pointA.z = 0.0;
  pointB.x = 10.0;
  pointB.y = 12.0;
  pointB.z = 0.0;

  for (int i=0; i<10; i++)
  {
    ASSERT_FALSE(test_obj_.isLocalizationConvergence(pointA)) << "It should be false";
  }
  ASSERT_TRUE(test_obj_.isLocalizationConvergence(pointA)) << "It should be true";

  for (int i=0; i<5; i++)
  {
    ASSERT_TRUE(test_obj_.isLocalizationConvergence(pointA)) << "It should be true";
    ASSERT_TRUE(test_obj_.isLocalizationConvergence(pointB)) << "It should be true";
  }
  ASSERT_TRUE(test_obj_.isLocalizationConvergence(pointA)) << "It should be true";

  pointB.y = 12.1;
  ASSERT_FALSE(test_obj_.isLocalizationConvergence(pointB)) << "It should be true";
}

TEST_F(TestSuite, isArrivedGoal) {
	test_obj_.createFinalWaypoints();

	test_obj_.setCurrentPose(100, 0, 0);
	test_obj_.setCurrentVelocity(0.0);
  ASSERT_TRUE(test_obj_.isArrivedGoal()) << "Current pose is outside the target range."
                               << "It should be true";

  test_obj_.setCurrentPose(100, 0, 0);
  test_obj_.setCurrentVelocity(3.0);
  ASSERT_FALSE(test_obj_.isArrivedGoal()) << "Current pose is inside the target range."
                                << "It should be false";

  test_obj_.setCurrentPose(90, 0, 0);
  test_obj_.setCurrentVelocity(0.0);
  ASSERT_FALSE(test_obj_.isArrivedGoal()) << "Current pose is inside the target range."
                                << "It should be false";

  test_obj_.setCurrentPose(90, 0, 0);
  test_obj_.setCurrentVelocity(3.0);
  ASSERT_FALSE(test_obj_.isArrivedGoal()) << "Current pose is inside the target range."
                                << "It should be false";
}

} // namespace decision_maker
