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

#ifndef NDT_SCAN_MATCHER__DEBUG_HPP_
#define NDT_SCAN_MATCHER__DEBUG_HPP_

#include "ndt_scan_matcher/particle.hpp"

#include <visualization_msgs/msg/marker_array.hpp>

#include <string>

visualization_msgs::msg::MarkerArray make_debug_markers(
  const builtin_interfaces::msg::Time & stamp, const std::string & map_frame_,
  const geometry_msgs::msg::Vector3 & scale, const Particle & particle, const size_t i);

#endif  // NDT_SCAN_MATCHER__DEBUG_HPP_
