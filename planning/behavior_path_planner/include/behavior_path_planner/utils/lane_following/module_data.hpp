// Copyright 2021-2023 TIER IV, Inc.
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

#ifndef BEHAVIOR_PATH_PLANNER__UTILS__LANE_FOLLOWING__MODULE_DATA_HPP_
#define BEHAVIOR_PATH_PLANNER__UTILS__LANE_FOLLOWING__MODULE_DATA_HPP_

namespace behavior_path_planner
{

struct LaneFollowingParameters
{
  double lane_change_prepare_duration{0.0};

  // finding closest lanelet
  double distance_threshold{0.0};
  double yaw_threshold{0.0};
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__UTILS__LANE_FOLLOWING__MODULE_DATA_HPP_
