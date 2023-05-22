// Copyright 2021 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <autoware_auto_planning_msgs/msg/path_point_with_lane_id.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/utils.h>

namespace test
{

inline autoware_auto_planning_msgs::msg::PathWithLaneId generatePath(
  double x0, double y0, double x, double y, int nb_points)
{
  autoware_auto_planning_msgs::msg::PathWithLaneId path{};
  double x_step = (x - x0) / (nb_points - 1);
  double y_step = (y - y0) / (nb_points - 1);
  for (int i = 0; i < nb_points; ++i) {
    autoware_auto_planning_msgs::msg::PathPointWithLaneId point{};
    point.point.pose.position.x = x0 + x_step * i;
    point.point.pose.position.y = y0 + y_step * i;
    point.point.pose.position.z = 0.0;
    path.points.push_back(point);
  }
  return path;
}

inline geometry_msgs::msg::Pose generatePose(double x)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = 0.0;
  p.position.z = 1.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  p.orientation = tf2::toMsg(q);
  return p;
}

}  // namespace test

#endif  // UTILS_HPP_
