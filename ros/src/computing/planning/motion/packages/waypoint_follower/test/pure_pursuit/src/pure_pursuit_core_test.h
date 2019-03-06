#ifndef __PURE_PURSUIT_NODE_TEST_H__
#define __PURE_PURSUIT_NODE_TEST_H__
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
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <pure_pursuit/pure_pursuit_core.h>

class PurePursuitNodeTest : public ::testing::Test
{
public:
  PurePursuitNode node_;
};

TEST(callbackFromWaypoints, inputPositivePath)
{
  PurePursuitNodeTest test;
  autoware_msgs:Lane original_lane;
  original_lane.waypoints.resize(3, autoware_msgs::Waypoints());
  for (int i = 0; i < 3; i++)
  {
    original_lane.waypoints[i].pose.pose.position.x = i;
  }
  const autoware_msgs::LaneConstPtr lp(original_lane);
  test.node_.callbackFromWayPoints(lp);
  ASSERT_EQ(test.node_.direction_, 1) <<
    "direction is not matching to positive lane."
}

TEST(callbackFromWaypoints, inputNegativePath)
{
  PurePursuitNodeTest test;
  autoware_msgs:Lane original_lane;
  original_lane.waypoints.resize(3, autoware_msgs::Waypoints());
  for (int i = 0; i < 3; i++)
  {
    original_lane.waypoints[i].pose.pose.position.x = -i;
  }
  const autoware_msgs::LaneConstPtr lp(original_lane);
  test.node_.callbackFromWayPoints(lp);
  ASSERT_EQ(test.node_.direction_, -1) <<
    "direction is not matching to negative lane."
}
// If original lane is empty, new lane is also empty.
TEST(connectVirtualLastWaypoints, inputEmptyLane)
{
  PurePursuitNodeTest test;
  autoware_msgs:Lane original_lane, new_lane;
  test.node_.connectVirtualLastWaypoints(&new_lane, 1);
  ASSERT_EQ(original_lane, new_lane) <<
    "Input empty lane, and output is not empty";
}

// If the original lane exceeds 2 points,
// the additional part will be updated at
// the interval of the first 2 points.
TEST(connectVirtualLastWaypoints, inputNormalLane)
{
  PurePursuitNodeTest test;
  autoware_msgs:Lane original_lane;
  original_lane.waypoints.resize(2, autoware_msgs::Waypoints());
  for (int i = 0; i < 2; i++)
  {
    original_lane.waypoints[i].pose.pose.position.x = i;
  }
  autoware_msgs:Lane new_lane(original_lane);
  test.node_.connectVirtualLastWaypoints(&new_lane, 1);

  ASSERT_LT(original_lane.waypoints.size(), new_lane.waypoints.size()) <<
    "Fail to expand waypoints";
}
#endif
