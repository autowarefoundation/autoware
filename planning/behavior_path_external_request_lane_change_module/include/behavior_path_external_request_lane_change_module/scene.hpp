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

#ifndef BEHAVIOR_PATH_EXTERNAL_REQUEST_LANE_CHANGE_MODULE__SCENE_HPP_
#define BEHAVIOR_PATH_EXTERNAL_REQUEST_LANE_CHANGE_MODULE__SCENE_HPP_

#include "behavior_path_lane_change_module/scene.hpp"

#include <memory>

namespace behavior_path_planner
{
class ExternalRequestLaneChange : public NormalLaneChange
{
public:
  ExternalRequestLaneChange(
    const std::shared_ptr<LaneChangeParameters> & parameters, Direction direction);

  ExternalRequestLaneChange(const ExternalRequestLaneChange &) = delete;
  ExternalRequestLaneChange(ExternalRequestLaneChange &&) = delete;
  ExternalRequestLaneChange & operator=(const ExternalRequestLaneChange &) = delete;
  ExternalRequestLaneChange & operator=(ExternalRequestLaneChange &&) = delete;
  ~ExternalRequestLaneChange() override = default;
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_EXTERNAL_REQUEST_LANE_CHANGE_MODULE__SCENE_HPP_
