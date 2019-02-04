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
#include <autoware_msgs/LaneArray.h>
#include "waypoint_replanner.h"

namespace waypoint_maker
{
class WaypointReplannerNode
{
public:
  WaypointReplannerNode();
  ~WaypointReplannerNode();
private:
  ros::NodeHandle nh_;
  std::map<std::string, ros::Publisher> lane_pub_;
  ros::Subscriber lane_sub_, config_sub_;
  bool replanning_mode_, realtime_tuning_mode_;
  WaypointReplanner replanner_;
  autoware_msgs::LaneArray lane_array_;
  bool withoutDecisionMaker();
  void replan(autoware_msgs::LaneArray &lane_array);
  void publishLaneArray();
  void laneCallback(const autoware_msgs::LaneArray::ConstPtr& lane_array);
  void configCallback(const autoware_config_msgs::ConfigWaypointReplanner::ConstPtr& conf);
};

WaypointReplannerNode::WaypointReplannerNode() : replanning_mode_(false)
{
  lane_pub_["with_decision"] = nh_.advertise<autoware_msgs::LaneArray>("/based/lane_waypoints_array", 10, true);
  lane_pub_["without_decision"] = nh_.advertise<autoware_msgs::LaneArray>("/lane_waypoints_array", 10, true);
  lane_sub_ = nh_.subscribe("/based/lane_waypoints_raw", 1, &WaypointReplannerNode::laneCallback, this);
  config_sub_ = nh_.subscribe("/config/waypoint_replanner", 1, &WaypointReplannerNode::configCallback, this);
}

WaypointReplannerNode::~WaypointReplannerNode()
{
}

void WaypointReplannerNode::replan(autoware_msgs::LaneArray& lane_array)
{
  for (auto &el : lane_array.lanes)
  {
    replanner_.replanLaneWaypointVel(el);
  }
}

bool WaypointReplannerNode::withoutDecisionMaker()
{
  std::vector<std::string> node_list;
  ros::master::getNodes(node_list);
  return (std::find(node_list.begin(), node_list.end(), "/decision_maker") == node_list.end());
}

void WaypointReplannerNode::publishLaneArray()
{
  autoware_msgs::LaneArray array(lane_array_);
  if (replanning_mode_)
  {
    replan(array);
  }
  lane_pub_["with_decision"].publish(array);
  if (withoutDecisionMaker())
  {
    lane_pub_["without_decision"].publish(array);
  }
}

void WaypointReplannerNode::laneCallback(const autoware_msgs::LaneArray::ConstPtr& lane_array)
{
  lane_array_ = *lane_array;
  publishLaneArray();
}

void WaypointReplannerNode::configCallback(const autoware_config_msgs::ConfigWaypointReplanner::ConstPtr& conf)
{
  replanning_mode_ = conf->replanning_mode;
  realtime_tuning_mode_ = conf->realtime_tuning_mode;
  replanner_.initParameter(conf);
  if (lane_array_.lanes.empty() || !realtime_tuning_mode_)
  {
    return;
  }
  publishLaneArray();
}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_replanner");
  waypoint_maker::WaypointReplannerNode wr;
  ros::spin();

  return 0;
}
