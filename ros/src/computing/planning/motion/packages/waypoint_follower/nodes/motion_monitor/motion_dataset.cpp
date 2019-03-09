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

#include "motion_dataset.h"

MotionDataset::MotionDataset() : is_sync_(false){}

void MotionDataset::reset()
{
  lane_.clear();
  pose_.clear();
  location_.clear();
}

void MotionDataset::sync()
{
  for (auto lane_itr = lane_.rbegin(); lane_itr != lane_.rend(); ++lane_itr)
  {
    auto location_itr =
      std::find_if(location_.rbegin(), location_.rend(),
      [&](const autoware_msgs::VehicleLocation& loc){return lane_itr->header.seq == loc.header.seq;});
    if (location_itr == location_.rend())
    {
      continue;
    }
    double min_time = DBL_MAX;
    auto sync_pose = pose_.rend();
    for (auto pose_itr = pose_.rbegin(); pose_itr != pose_.rend(); ++pose_itr)
    {
      const double duration = fabs((location_itr->header.stamp - pose_itr->header.stamp).toSec());
      if (min_time <= duration)
      {
        continue;
      }
      min_time = duration;
      sync_pose = pose_itr;
    }
    const autoware_msgs::Lane lane(*lane_itr);
    const geometry_msgs::PoseStamped pose(*sync_pose);
    const autoware_msgs::VehicleLocation location(*location_itr);
    reset();
    lane_.emplace_back(lane);
    pose_.emplace_back(pose);
    location_.emplace_back(location);
    return;
  }
  reset();
}

const bool MotionDataset::check() const
{
  if (lane_.empty() || pose_.empty() || location_.empty())
  {
    return false;
  }
  const int& idx = location_.front().waypoint_index;
  return (idx >= 0 && idx < lane_.front().waypoints.size());
}

const int MotionDataset::getLaneSize() const
{
  return !lane_.empty() ? lane_.front().waypoints.size() : 0;
}

const std::pair<int, int> MotionDataset::calcNearestPair(unsigned int search_width) const
{
  if (!check() || getLaneSize() < 2 || location_.front().waypoint_index == -1)
  {
    return std::pair<int, int>();
  }
  const geometry_msgs::Point& pos = pose_.front().pose.position;
  const int end_idx = std::max(location_.front().waypoint_index - (int)search_width, 0);
  std::map<double, int> distance;
  for (int i = location_.front().waypoint_index; i >= end_idx; --i)
  {
    const geometry_msgs::Point& wp_pos = lane_.front().waypoints.at(i).pose.pose.position;
    const double dist = getPlaneDistance(wp_pos, pos);
    distance[dist] = i;
  }
  const int nearest = distance.begin()->second;
  const int other =
    (nearest == location_.front().waypoint_index) ? location_.front().waypoint_index - 1 :
    (nearest == end_idx) ? end_idx + 1 :
    std::find_if(distance.begin(), distance.end(), [&](std::pair<double, int> dataset)
      { return (dataset.second == nearest + 1) || (dataset.second == nearest - 1); })->second;
  return std::make_pair(nearest, other);
}
