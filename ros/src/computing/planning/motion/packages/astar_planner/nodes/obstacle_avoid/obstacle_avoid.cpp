/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "astar_search.h"
#include "search_info_ros.h"

#include <autoware_msgs/LaneArray.h>

namespace
{
autoware_msgs::lane createPublishWaypoints(const autoware_msgs::lane& ref_lane, int closest_waypoint,
                                                    int size)
{
  autoware_msgs::lane follow_lane;

  follow_lane.header = ref_lane.header;
  follow_lane.increment = ref_lane.increment;

  // Push "size" waypoints from closest
  for (int i = 0; i < size; i++)
  {
    if (closest_waypoint + i >= static_cast<int>(ref_lane.waypoints.size()))
      break;

    follow_lane.waypoints.push_back(ref_lane.waypoints[closest_waypoint + i]);
  }

  return follow_lane;
}

void createAvoidWaypoints(const nav_msgs::Path& astar_path, const astar_planner::SearchInfo& search_info, int size,
                          autoware_msgs::lane* avoid_lane, int* end_of_avoid_index)
{
  int closest_waypoint_index = search_info.getClosestWaypointIndex();

  avoid_lane->waypoints.clear();

  // Get global lane
  const autoware_msgs::lane& current_lane = search_info.getCurrentWaypoints();
  avoid_lane->header = current_lane.header;
  avoid_lane->increment = current_lane.increment;

  // Set waypoints from closest to beginning of avoiding
  for (int i = closest_waypoint_index; i < search_info.getStartWaypointIndex(); i++)
  {
    avoid_lane->waypoints.push_back(current_lane.waypoints.at(i));
  }

  double avoid_velocity = avoid_lane->waypoints.back().twist.twist.linear.x;

  if (avoid_velocity > search_info.getAvoidVelocityLimitMPS())
    avoid_velocity = search_info.getAvoidVelocityLimitMPS();

  // Set waypoints for avoiding
  for (const auto& pose : astar_path.poses)
  {
    autoware_msgs::waypoint wp;
    wp.pose = pose;
    wp.twist.twist.linear.x = avoid_velocity;

    avoid_lane->waypoints.push_back(wp);
  }

  // To know here is the end of avoiding
  *end_of_avoid_index = avoid_lane->waypoints.size();

  // Set waypoints from the end of avoiding
  for (int i = search_info.getGoalWaypointIndex() + 1; i < search_info.getGoalWaypointIndex() + size; i++)
  {
    if (i >= static_cast<int>(current_lane.waypoints.size()))
      break;

    avoid_lane->waypoints.push_back(current_lane.waypoints.at(i));
  }
}

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle_avoid");
  ros::NodeHandle n;

  astar_planner::AstarSearch astar;
  astar_planner::SearchInfo search_info;

  // ROS subscribers
  ros::Subscriber map_sub = n.subscribe("grid_map_visualization/distance_transform", 1,
                                        &astar_planner::SearchInfo::mapCallback, &search_info);
  ros::Subscriber start_sub =
      n.subscribe("current_pose", 1, &astar_planner::SearchInfo::currentPoseCallback, &search_info);
  ros::Subscriber waypoints_sub =
      n.subscribe("base_waypoints", 1, &astar_planner::SearchInfo::waypointsCallback, &search_info);
  ros::Subscriber obstacle_waypoint_sub =
      n.subscribe("obstacle_waypoint", 1, &astar_planner::SearchInfo::obstacleWaypointCallback, &search_info);
  ros::Subscriber closest_waypoint_sub =
      n.subscribe("closest_waypoint", 1, &astar_planner::SearchInfo::closestWaypointCallback, &search_info);
  // TODO: optional
  // ros::Subscriber goal_sub = n.subscribe("/move_base_simple/goal", 1, &astar_planner::SearchInfo::goalCallback,
  // &search_info);
  ros::Subscriber current_velocity_sub =
      n.subscribe("current_velocity", 1, &astar_planner::SearchInfo::currentVelocityCallback, &search_info);
  ros::Subscriber state_sub = n.subscribe("state", 1, &astar_planner::SearchInfo::stateCallback, &search_info);

  // ROS publishers
  ros::Publisher path_pub = n.advertise<nav_msgs::Path>("astar_path", 1, true);
  ros::Publisher waypoints_pub = n.advertise<autoware_msgs::lane>("safety_waypoints", 1, true);

  ros::Rate loop_rate(10);

  // variables for avoidance
  autoware_msgs::lane avoid_lane;
  int end_of_avoid_index = -1;
  bool avoidance = false;
  while (ros::ok())
  {
    ros::spinOnce();

    int closest_waypoint;

    // We switch 2 waypoints, original path and avoiding path
    if (avoidance)
      closest_waypoint = getClosestWaypoint(avoid_lane, search_info.getCurrentPose().pose);
    else
      closest_waypoint = search_info.getClosestWaypointIndex();

    // there are no waypoints we can follow
    if (closest_waypoint < 0 || !search_info.getPathSet())
    {
      loop_rate.sleep();
      continue;
    }

    // Follow the original waypoints
    if (!avoidance)
    {
      autoware_msgs::lane publish_lane;
      publish_lane = createPublishWaypoints(search_info.getSubscribedWaypoints(), closest_waypoint, 100);
      waypoints_pub.publish(publish_lane);
    }
    // Follow the avoiding waypoints
    else
    {
      // create waypoints from closest on avoid_lane
      autoware_msgs::lane publish_lane;
      publish_lane = createPublishWaypoints(avoid_lane, closest_waypoint, 100);
      waypoints_pub.publish(publish_lane);

      // End of avoidance
      if (closest_waypoint > end_of_avoid_index)
      {
        avoidance = false;

        // Return to the original waypoints
        search_info.setCurrentWaypoints(search_info.getSubscribedWaypoints());

        loop_rate.sleep();
        continue;
      }
    }

    // Initialize vector for A* search, this runs only once
    if (search_info.getMapSet() && !astar.getNodeInitialized())
      astar.initializeNode(search_info.getMap());

    // Waiting for the call for avoidance ...
    if (!search_info.getMapSet() || !search_info.getStartSet() || !search_info.getGoalSet())
    {
      search_info.reset();
      loop_rate.sleep();
      continue;
    }

    // Run astar search
    ros::WallTime timer_begin = ros::WallTime::now();

    bool result = astar.makePlan(search_info.getStartPose().pose, search_info.getGoalPose().pose, search_info.getMap(),
                                 search_info.getUpperBoundDistance());

    ros::WallTime timer_end = ros::WallTime::now();
    double time_ms = (timer_end - timer_begin).toSec() * 1000;
    ROS_INFO("planning time: %lf [ms]", time_ms);

    // debug mode
    if (!search_info.getChangePath())
    {
      static double msec_sum = 0;
      static int plan_count = 0;
      plan_count++;
      msec_sum += time_ms;
      std::cout << "average time so far: " << msec_sum / plan_count << std::endl;
    }

    if (result)
    {
      std::cout << "Found goal!" << std::endl;
      path_pub.publish(astar.getPath());

      createAvoidWaypoints(astar.getPath(), search_info, 100, &avoid_lane, &end_of_avoid_index);

      if (search_info.getChangePath())
        avoidance = true;
    }
    else
    {
      std::cout << "can't find goal..." << std::endl;
    }

    // Reset flags
    search_info.reset();
    astar.reset();

    loop_rate.sleep();
  }

  return 0;
}
