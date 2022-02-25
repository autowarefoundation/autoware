// Copyright 2015-2019 Autoware Foundation
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

#ifndef NDT_SCAN_MATCHER__PARTICLE_HPP_
#define NDT_SCAN_MATCHER__PARTICLE_HPP_

#include <geometry_msgs/msg/pose.hpp>

struct Particle
{
  Particle(
    const geometry_msgs::msg::Pose & a_initial_pose, const geometry_msgs::msg::Pose & a_result_pose,
    const double a_score, const int a_iteration)
  : initial_pose(a_initial_pose), result_pose(a_result_pose), score(a_score), iteration(a_iteration)
  {
  }
  geometry_msgs::msg::Pose initial_pose;
  geometry_msgs::msg::Pose result_pose;
  double score;
  int iteration;
};

#endif  // NDT_SCAN_MATCHER__PARTICLE_HPP_
