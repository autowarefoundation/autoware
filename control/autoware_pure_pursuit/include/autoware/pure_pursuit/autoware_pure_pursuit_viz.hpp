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

#ifndef AUTOWARE__PURE_PURSUIT__AUTOWARE_PURE_PURSUIT_VIZ_HPP_
#define AUTOWARE__PURE_PURSUIT__AUTOWARE_PURE_PURSUIT_VIZ_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <vector>
namespace autoware::pure_pursuit
{
visualization_msgs::msg::Marker createNextTargetMarker(
  const geometry_msgs::msg::Point & next_target);

visualization_msgs::msg::Marker createTrajectoryCircleMarker(
  const geometry_msgs::msg::Point & target, const geometry_msgs::msg::Pose & current_pose);
}  // namespace autoware::pure_pursuit

#endif  // AUTOWARE__PURE_PURSUIT__AUTOWARE_PURE_PURSUIT_VIZ_HPP_
