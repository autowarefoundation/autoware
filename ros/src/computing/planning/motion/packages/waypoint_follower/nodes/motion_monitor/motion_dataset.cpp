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

MotionDataset::MotionDataset() : init_({false})
{}

void MotionDataset::reset()
{
  lane_ = autoware_msgs::Lane();
  pose_ = geometry_msgs::PoseStamped();
  closest_idx_ = std_msgs::Int32();
  init_.fill(false);
}

const bool MotionDataset::check() const
{
  if (std::find(init_.begin(), init_.end(), false) != init_.end())
  {
    return false;
  }
  if (lane_.waypoints.empty())
  {
    return false;
  }
  const int& idx = closest_idx_.data;
  return (idx >= 0 && idx < lane_.waypoints.size());
}

const int MotionDataset::getLaneSize() const
{
  return init_.at(DataType::TYPE_LANE) ? lane_.waypoints.size() : 0;
}

const std::pair<int, int> MotionDataset::calcNearestPair(unsigned int search_width) const
{
  if (!check() || getLaneSize() < 2 || closest_idx_.data == -1)
  {
    return std::pair<int, int>();
  }
  const geometry_msgs::Point& pos = pose_.pose.position;
  const int end_idx = std::max(closest_idx_.data - (int)search_width, 0);
  std::map<double, int> distance;
  for (int i = closest_idx_.data; i >= end_idx; --i)
  {
    const geometry_msgs::Point& wp_pos = lane_.waypoints.at(i).pose.pose.position;
    const double dist = getPlaneDistance(wp_pos, pos);
    distance[dist] = i;
  }
  const int nearest = distance.begin()->second;
  const int other =
    (nearest == closest_idx_.data) ? closest_idx_.data - 1 :
    (nearest == end_idx) ? end_idx + 1 :
    std::find_if(distance.begin(), distance.end(), [&](std::pair<double, int> dataset)
      { return (dataset.second == nearest + 1) || (dataset.second == nearest - 1); })->second;
  return std::make_pair(nearest, other);
}
