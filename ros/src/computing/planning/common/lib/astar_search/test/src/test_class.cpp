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

#include "test_class.h"

TestClass::TestClass()
{

  obstacle_indexes_.push_back(std::make_pair(0,0));

  obstacle_indexes_.push_back(std::make_pair(24,0));
  obstacle_indexes_.push_back(std::make_pair(24,1));
  obstacle_indexes_.push_back(std::make_pair(24,2));
  obstacle_indexes_.push_back(std::make_pair(24,3));
  obstacle_indexes_.push_back(std::make_pair(24,4));
  obstacle_indexes_.push_back(std::make_pair(24,5));
  obstacle_indexes_.push_back(std::make_pair(24,6));
  obstacle_indexes_.push_back(std::make_pair(24,7));
  obstacle_indexes_.push_back(std::make_pair(24,8));
  obstacle_indexes_.push_back(std::make_pair(24,9));
  obstacle_indexes_.push_back(std::make_pair(24,10));

  costmap_.header.seq = 1;
  costmap_.header.stamp = ros::Time::now();
  costmap_.header.frame_id = "world";

  costmap_.info.map_load_time = costmap_.header.stamp;
  costmap_.info.resolution = 1;
  costmap_.info.width = 41;
  costmap_.info.height = 11;

  // Costmap origin set to be the centre of the grid (as in points2costmap.cpp)
  costmap_.info.origin.position.x = -costmap_.info.resolution*costmap_.info.width/2;
  costmap_.info.origin.position.y = -costmap_.info.resolution*costmap_.info.height/2;
  costmap_.info.origin.position.z = 0;
  costmap_.info.origin.orientation.x = 0;
  costmap_.info.origin.orientation.y = 0;
  costmap_.info.origin.orientation.z = 0;
  costmap_.info.origin.orientation.w = 1;

  for (int idx = 0; idx < costmap_.info.width*costmap_.info.height; ++idx)
  {
    costmap_.data.push_back(0);
  }

  int cnt = 0;
  for (int row = 0; row < costmap_.info.height; ++row)
  {
    for (int col = 0; col < costmap_.info.width; ++col)
    {
      for (std::vector<std::pair<int, int>>::iterator it = obstacle_indexes_.begin(); it != obstacle_indexes_.begin()+1; it++)
      {
        if (col == it->first && row == it->second)
        {
          costmap_.data.at(cnt) = 100;
        }
      }
      cnt++;
    }
  }

  ros::NodeHandle nh("~");
  nh.setParam("lateral_goal_range", 1.5);
}

void TestClass::createStateUpdateTable()
{
  astar_search_obj.createStateUpdateTable();
}
bool TestClass::search()
{
  return astar_search_obj.search();
}
void TestClass::poseToIndex(const geometry_msgs::Pose& pose, int* index_x, int* index_y, int* index_theta)
{
  astar_search_obj.poseToIndex(pose, index_x, index_y, index_theta);
}
void TestClass::pointToIndex(const geometry_msgs::Point& point, int* index_x, int* index_y)
{
  astar_search_obj.pointToIndex(point, index_x, index_y);
}
bool TestClass::isOutOfRange(int index_x, int index_y)
{
  return astar_search_obj.isOutOfRange(index_x, index_y);
}
void TestClass::setPath(const SimpleNode& goal)
{
  astar_search_obj.setPath(goal);
}
bool TestClass::setStartNode(const geometry_msgs::Pose& start_pose)
{
  return astar_search_obj.setStartNode(start_pose);
}
bool TestClass::setGoalNode(const geometry_msgs::Pose& goal_pose)
{
  return astar_search_obj.setGoalNode(goal_pose);
}
bool TestClass::isGoal(double x, double y, double theta)
{
  return astar_search_obj.isGoal(x, y, theta);
}
bool TestClass::isObs(int index_x, int index_y)
{
  return astar_search_obj.isObs(index_x, index_y);
}
bool TestClass::detectCollision(const SimpleNode& sn)
{
  return astar_search_obj.detectCollision(sn);
}
bool TestClass::calcWaveFrontHeuristic(const SimpleNode& sn)
{
  return astar_search_obj.calcWaveFrontHeuristic(sn);
}
bool TestClass::detectCollisionWaveFront(const WaveFrontNode& sn)
{
  return astar_search_obj.detectCollisionWaveFront(sn);
}
