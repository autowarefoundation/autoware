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

#ifndef BEHAVIOR_PATH_PLANNER__MODULE_STATUS_HPP_
#define BEHAVIOR_PATH_PLANNER__MODULE_STATUS_HPP_

#ifdef USE_OLD_ARCHITECTURE
#include <behaviortree_cpp_v3/basic_types.h>
#endif

namespace behavior_path_planner
{

#ifdef USE_OLD_ARCHITECTURE
using ModuleStatus = BT::NodeStatus;
#else
enum class ModuleStatus {
  IDLE = 0,
  RUNNING = 1,
  SUCCESS = 2,
  FAILURE = 3,
  // SKIPPED = 4,
};
#endif

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__MODULE_STATUS_HPP_
