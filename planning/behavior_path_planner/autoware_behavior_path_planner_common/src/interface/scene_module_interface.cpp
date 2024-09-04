// Copyright 2024 TIER IV, Inc.
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

#include "autoware/behavior_path_planner_common/interface/scene_module_interface.hpp"

namespace autoware::behavior_path_planner
{
void SceneModuleInterface::setDrivableLanes(const std::vector<DrivableLanes> & drivable_lanes)
{
  drivable_lanes_marker_ =
    marker_utils::createDrivableLanesMarkerArray(drivable_lanes, "drivable_lanes");
}

void SceneModuleInterface::onEntry()
{
  RCLCPP_DEBUG(getLogger(), "%s %s", name_.c_str(), __func__);

  stop_reason_ = StopReason();

  processOnEntry();
}
}  // namespace autoware::behavior_path_planner
