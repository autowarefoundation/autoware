// Copyright 2020 Tier IV, Inc. All rights reserved.
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

#include "goal_distance_calculator/goal_distance_calculator.hpp"

namespace goal_distance_calculator
{
Output GoalDistanceCalculator::update(const Input & input)
{
  Output output{};

  output.goal_deviation =
    tier4_autoware_utils::calcPoseDeviation(input.route->goal_point.pose, input.current_pose->pose);

  return output;
}

}  // namespace goal_distance_calculator
