// Copyright 2023 TIER IV, Inc.
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
#ifndef AUTOWARE__PATH_OPTIMIZER__DEBUG_MARKER_HPP_
#define AUTOWARE__PATH_OPTIMIZER__DEBUG_MARKER_HPP_

#include "autoware/path_optimizer/common_structs.hpp"
#include "autoware/path_optimizer/type_alias.hpp"
#include "autoware/universe_utils/geometry/geometry.hpp"
#include "autoware/universe_utils/ros/marker_helper.hpp"
#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"
#include "rclcpp/clock.hpp"

#include <memory>
#include <string>
#include <vector>

namespace autoware::path_optimizer
{
MarkerArray getDebugMarker(
  const DebugData & debug_data,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & optimized_points,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const bool publish_extra_marker);
}  // namespace autoware::path_optimizer
#endif  // AUTOWARE__PATH_OPTIMIZER__DEBUG_MARKER_HPP_
