/*
 * Copyright 2018 Autoware Foundation. All rights reserved.
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

#include "astar_search/astar_search.h"

#include "test_class.h"

TEST_F(TestSuite, checkPoseAndPoint_ToIndex)
{
  //Check poseToIndex

  // start_pose is located at the center of the grid
  test_obj_.poseToIndex(start_pose_, &index_x, &index_y, &index_theta);
  ASSERT_EQ(index_x, test_obj_.costmap_.info.width/2) << "start_pose.index_x should be " << test_obj_.costmap_.info.width/2;
  ASSERT_EQ(index_y, test_obj_.costmap_.info.height/2) << "start_pose.index_y should be " << test_obj_.costmap_.info.height/2;
  ASSERT_EQ(index_theta, 0) << "start_pose.index_theta should be " << 0;

  // goal_pose is 10 cells to the right from start_pose
  test_obj_.poseToIndex(goal_pose_, &index_x, &index_y, &index_theta);
  ASSERT_EQ(index_x, test_obj_.costmap_.info.width/2+10) << "goal_pose.index_x should be " << test_obj_.costmap_.info.width/2+10;
  ASSERT_EQ(index_y, test_obj_.costmap_.info.height/2) << "goal_pose.index_y should be " << test_obj_.costmap_.info.height/2;
  ASSERT_EQ(index_theta, 0) << "goal_pose.index_theta should be " << 0;

  //Check pointToIndex
  geometry_msgs::Point test_point;

  test_point.x = start_pose_.position.x;
  test_point.y = start_pose_.position.y;
  test_point.z = start_pose_.position.z;

  test_obj_.pointToIndex(test_point, &index_x, &index_y);
  ASSERT_EQ(index_x, test_obj_.costmap_.info.width/2) << "start_point.index_x should be " << test_obj_.costmap_.info.width/2;
  ASSERT_EQ(index_y, test_obj_.costmap_.info.height/2) << "start_point.index_y should be " << test_obj_.costmap_.info.height/2;

  test_point.x = goal_pose_.position.x;
  test_point.y = goal_pose_.position.y;
  test_point.z = goal_pose_.position.z;

  test_obj_.pointToIndex(test_point, &index_x, &index_y);
  ASSERT_EQ(index_x, test_obj_.costmap_.info.width/2+10) << "goal_pose.index_x should be " << test_obj_.costmap_.info.width/2+10;
  ASSERT_EQ(index_y, test_obj_.costmap_.info.height/2) << "goal_pose.index_y should be " << test_obj_.costmap_.info.height/2;
}

TEST_F(TestSuite, checkIsOutOfRange)
{

  // Check that the indexes are out and within range
  index_x = test_obj_.costmap_.info.width/2;
  index_y = test_obj_.costmap_.info.height/2;
  ASSERT_FALSE(test_obj_.isOutOfRange(index_x, index_y)) << "[index_x,index_y] : [" << index_x << "," << index_y << "] Should be outOfRange";

  // Check that the indexes are out and within range
  index_x = test_obj_.costmap_.info.width + 1;
  index_y = test_obj_.costmap_.info.height + 1;
  ASSERT_TRUE(test_obj_.isOutOfRange(index_x, index_y)) << "[index_x,index_y] : [" << index_x << "," << index_y << "] Should be outOfRange";

  index_x = -1;
  index_y = -1;
  ASSERT_TRUE(test_obj_.isOutOfRange(index_x, index_y)) << "[index_x,index_y] : [" << index_x << "," << index_y << "] Should be outOfRange";
}

TEST_F(TestSuite, checkSetNodes)
{

  // start_pose_ within range
  ASSERT_TRUE(test_obj_.setStartNode(start_pose_)) << "Should be true";
  // start_pose_ out of range
  start_pose_.position.x = (test_obj_.costmap_.info.width + 10)*test_obj_.costmap_.info.resolution;
  ASSERT_FALSE(test_obj_.setStartNode(start_pose_)) << "Should be false";

  // goal_pose_ out of range
  ASSERT_FALSE(test_obj_.setGoalNode(start_pose_)) << "Should be false";
  // goal_pose_ within range
  ASSERT_TRUE(test_obj_.setGoalNode(goal_pose_)) << "Should be true";

  // Check isGoal
  ASSERT_TRUE(test_obj_.isGoal(goal_pose_.position.x - 0.1, goal_pose_.position.y, 0)) << "Goal should be goal";
  ASSERT_FALSE(test_obj_.isGoal(goal_pose_.position.x - 1, goal_pose_.position.y, 0)) << "Should NOT be goal";

}

TEST_F(TestSuite, checkIsObs)
{

  // Initialise costmap
  test_obj_.astar_search_obj.initialize(test_obj_.costmap_);

  ASSERT_TRUE(test_obj_.isObs(test_obj_.obstacle_indexes_[0].first, test_obj_.obstacle_indexes_[0].second)) << "[index_x,index_y] : [" << test_obj_.obstacle_indexes_[0].first << "," << test_obj_.obstacle_indexes_[0].second<< "] Should be obstacle";
  ASSERT_FALSE(test_obj_.isObs(1,1)) << "[index_x,index_y] : [" << 1 << "," << 1 << "] Should NOT be obstacle";

}

TEST_F(TestSuite, checkDetectCollision)
{

  // Node at start_pose_
  index_x = test_obj_.costmap_.info.width/2;
  index_y = test_obj_.costmap_.info.height/2;
  index_theta = 0;
  SimpleNode start_sn(index_x, index_y, index_theta, 0, 0);
  ASSERT_FALSE(test_obj_.detectCollision(start_sn)) << "start_pose_ node should not collide";

  // Node just before obstacle
  index_x = test_obj_.obstacle_indexes_[0].first - 1;
  index_y = test_obj_.obstacle_indexes_[0].second;
  index_theta = 0;
  SimpleNode obs_sn(index_x, index_y, index_theta, 0, 0);
  ASSERT_TRUE(test_obj_.detectCollision(obs_sn)) << "node before obstacle should collide";

}

TEST_F(TestSuite, checkSetPath)
{

  test_obj_.astar_search_obj.initialize(test_obj_.costmap_);
  test_obj_.setStartNode(start_pose_);
  test_obj_.setGoalNode(goal_pose_);

  ASSERT_TRUE(test_obj_.search());
  ASSERT_FALSE(test_obj_.astar_search_obj.getPath().poses.empty()) << "path should not be empty";

  test_obj_.astar_search_obj.reset();
  ASSERT_TRUE(test_obj_.astar_search_obj.getPath().poses.empty()) << "path should be empty after reset";

  // Block goal with obstacle
  int cnt = 0;
  for (int row = 0; row < test_obj_.costmap_.info.height; ++row)
  {
    for (int col = 0; col < test_obj_.costmap_.info.width; ++col)
    {
      for (std::vector<std::pair<int, int>>::iterator it = test_obj_.obstacle_indexes_.begin(); it != test_obj_.obstacle_indexes_.end(); it++)
      {
        if (col == it->first && row == it->second)
        {
          test_obj_.costmap_.data.at(cnt) = 100;
        }
      }
      cnt++;
    }
  }

  test_obj_.astar_search_obj.initialize(test_obj_.costmap_);
  test_obj_.setStartNode(start_pose_);
  test_obj_.setGoalNode(goal_pose_);
  ASSERT_FALSE(test_obj_.search());
  ASSERT_TRUE(test_obj_.astar_search_obj.getPath().poses.empty()) << "path should be empty after reset";
}

TEST_F(TestSuite, checkMakePlan)
{
  test_obj_.astar_search_obj.initialize(test_obj_.costmap_);
  ASSERT_TRUE(test_obj_.astar_search_obj.makePlan(start_pose_, goal_pose_)) << "makePlan should return True";
}
