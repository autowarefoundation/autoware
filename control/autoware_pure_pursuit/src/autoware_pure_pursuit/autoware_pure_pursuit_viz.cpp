// Copyright 2020 Tier IV, Inc.
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

#include "autoware/pure_pursuit/autoware_pure_pursuit_viz.hpp"

#include "autoware/pure_pursuit/util/marker_helper.hpp"
#include "autoware/pure_pursuit/util/planning_utils.hpp"

#include <vector>

namespace
{
std::vector<geometry_msgs::msg::Point> generateTrajectoryCircle(
  const geometry_msgs::msg::Point & target, const geometry_msgs::msg::Pose & current_pose)
{
  constexpr double theta_range = M_PI / 10;
  constexpr double step_rad = 0.005;

  const double radius = autoware::pure_pursuit::planning_utils::calcRadius(target, current_pose);

  std::vector<geometry_msgs::msg::Point> trajectory_circle;
  for (double theta = -theta_range; theta <= theta_range; theta += step_rad) {
    geometry_msgs::msg::Point p;
    p.x = radius * sin(theta);
    p.y = radius * (1 - cos(theta));
    p.z = target.z;

    trajectory_circle.push_back(
      autoware::pure_pursuit::planning_utils::transformToAbsoluteCoordinate2D(p, current_pose));
  }

  return trajectory_circle;
}

}  // namespace

namespace autoware::pure_pursuit
{
visualization_msgs::msg::Marker createNextTargetMarker(
  const geometry_msgs::msg::Point & next_target)
{
  auto marker = createDefaultMarker(
    "map", "next_target", 0, visualization_msgs::msg::Marker::SPHERE,
    createMarkerScale(0.3, 0.3, 0.3), createMarkerColor(0.0, 1.0, 0.0, 1.0));

  marker.pose.position = next_target;

  return marker;
}

visualization_msgs::msg::Marker createTrajectoryCircleMarker(
  const geometry_msgs::msg::Point & target, const geometry_msgs::msg::Pose & current_pose)
{
  auto marker = createDefaultMarker(
    "map", "trajectory_circle", 0, visualization_msgs::msg::Marker::LINE_STRIP,
    createMarkerScale(0.05, 0, 0), createMarkerColor(1.0, 1.0, 1.0, 1.0));

  const auto trajectory_circle = generateTrajectoryCircle(target, current_pose);
  for (auto p : trajectory_circle) {
    marker.points.push_back(p);
    marker.colors.push_back(marker.color);
  }

  return marker;
}
}  // namespace autoware::pure_pursuit
