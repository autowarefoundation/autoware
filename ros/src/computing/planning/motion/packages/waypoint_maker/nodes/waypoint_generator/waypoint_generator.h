/*
 *  Copyright (c) 2018, TierIV,Inc.
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
#ifndef __WAYPOINT_GENERATOR_H__
#define __WAYPOINT_GENERATOR_H__

#define VMap lane_planner::vmap

#include <geometry_msgs/PointStamped.h>
#include <ros/console.h>
#include <tf/transform_listener.h>

#include <vector_map/vector_map.h>
#include <autoware_msgs/LaneArray.h>

#include <lane_planner/lane_planner_vmap.hpp>

namespace waypoint_maker
{

class WaypointGenerator
{
public:
  WaypointGenerator();
  ~WaypointGenerator();

private:
  ros::NodeHandle nh_, private_nh_;
  ros::Publisher larray_pub_;
  ros::Subscriber vmap_point_sub_, vmap_lane_sub_, vmap_node_sub_;
  VMap::VectorMap all_vmap_, lane_vmap_;
  int waypoint_max_;

  bool checkEmpty(const VMap::VectorMap& vmap);
  VMap::VectorMap createVMapWithLane(const VMap::VectorMap& lane_vmap, int waypoint_max) const;
  void initLaneArray(autoware_msgs::LaneArray *larray, unsigned int size);
  void convertVMapToLaneArray(const VMap::VectorMap& vmap, autoware_msgs::LaneArray *larray);
  void createRoute();
  void updateValues();
  void cachePoint(const vector_map::PointArray& point);
  void cacheLane(const vector_map::LaneArray& lane);
  void cacheNode(const vector_map::NodeArray& node);
};

};
#endif
