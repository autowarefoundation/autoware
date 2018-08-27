/*
 *  Copyright (c) 2018, TierIV, Inc.

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
#include "waypoint_replanner.h"

namespace waypoint_maker
{

WaypointReplanner::WaypointReplanner() : replanning_mode_(false)
{
  lane_pub_ = nh_.advertise<autoware_msgs::LaneArray>("/based/lane_waypoints_array", 10, true);
  lane_sub_ = nh_.subscribe("/based/lane_waypoints_raw", 1, &WaypointReplanner::laneCallback, this);
  config_sub_ = nh_.subscribe("/config/waypoint_replanner", 1, &WaypointReplanner::configCallback, this);
}

WaypointReplanner::~WaypointReplanner()
{
}

void WaypointReplanner::replan(autoware_msgs::LaneArray* lane_array)
{
  if (!lane_array)
  {
    return;
  }
  for (auto &el : lane_array->lanes)
  {
    replanner_.replanLaneWaypointVel(&el);
  }
}

void WaypointReplanner::publishLaneArray()
{
  autoware_msgs::LaneArray array(lane_array_);
  if (replanning_mode_)
  {
    replan(&array);
  }
  lane_pub_.publish(array);
}

void WaypointReplanner::laneCallback(const autoware_msgs::LaneArray::ConstPtr& lane_array)
{
  lane_array_ = *lane_array;
  publishLaneArray();
}

void WaypointReplanner::configCallback(const autoware_msgs::ConfigWaypointReplanner::ConstPtr& conf)
{
  replanning_mode_ = conf->replanning_mode;
  replanner_.initParameter(conf);
  if (lane_array_.lanes.empty())
  {
    return;
  }
  publishLaneArray();
}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_replanner");
  waypoint_maker::WaypointReplanner wr;
  ros::spin();

  return 0;
}
