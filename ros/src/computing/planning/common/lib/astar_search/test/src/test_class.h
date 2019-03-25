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

class TestClass{
public:
  TestClass();

  AstarSearch astar_search_obj;

  void createStateUpdateTable();
  bool search();
  void poseToIndex(const geometry_msgs::Pose& pose, int* index_x, int* index_y, int* index_theta);
  void pointToIndex(const geometry_msgs::Point& point, int* index_x, int* index_y);
  bool isOutOfRange(int index_x, int index_y);
  void setPath(const SimpleNode& goal);
  bool setStartNode(const geometry_msgs::Pose& start_pose);
  bool setGoalNode(const geometry_msgs::Pose& goal_pose);
  bool isGoal(double x, double y, double theta);
  bool isObs(int index_x, int index_y);
  bool detectCollision(const SimpleNode& sn);
  bool calcWaveFrontHeuristic(const SimpleNode& sn);
  bool detectCollisionWaveFront(const WaveFrontNode& sn);

  nav_msgs::OccupancyGrid costmap_;

  std::vector<std::pair<int, int>> obstacle_indexes_;

};

class TestSuite: public ::testing::Test {
public:
  TestSuite(){}
  ~TestSuite(){}

  TestClass test_obj_;

  geometry_msgs::Pose start_pose_;
  geometry_msgs::Pose goal_pose_;
  int index_x, index_y, index_theta;

  virtual void SetUp()
  {
    test_obj_.astar_search_obj.initialize(test_obj_.costmap_);

    start_pose_.position.x = 0;
    start_pose_.position.y = 0;
    start_pose_.position.z = 0;
    start_pose_.orientation.x = 0;
    start_pose_.orientation.y = 0;
    start_pose_.orientation.z = 0;
    start_pose_.orientation.w = 1;

    goal_pose_.position.x = test_obj_.costmap_.info.resolution * 10;
    goal_pose_.position.y = 0;
    goal_pose_.position.z = 0;
    goal_pose_.orientation.x = 0;
    goal_pose_.orientation.y = 0;
    goal_pose_.orientation.z = 0;
    goal_pose_.orientation.w = 1;

    index_x = 0;
    index_y = 0;
    index_theta = 0;
  }
};
