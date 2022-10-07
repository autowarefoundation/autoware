// Copyright 2022 TIER IV, Inc.
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

#ifndef SCENE_MODULE__RUN_OUT__STATE_MACHINE_HPP_
#define SCENE_MODULE__RUN_OUT__STATE_MACHINE_HPP_

#include "scene_module/run_out/utils.hpp"

#include <memory>
#include <string>

namespace behavior_velocity_planner
{
namespace run_out_utils
{

class StateMachine
{
public:
  enum class State {
    GO = 0,
    STOP,
    APPROACH,
    UNKNOWN,
  };

  struct StateInput
  {
    double current_velocity;
    double dist_to_collision;
    boost::optional<DynamicObstacle> current_obstacle;
  };

  explicit StateMachine(const StateParam & state_param) { state_param_ = state_param; }
  State getCurrentState() const { return state_; }
  boost::optional<DynamicObstacle> getTargetObstacle() const { return target_obstacle_; }
  std::string toString(const State & state) const;
  void updateState(const StateInput & state_input, rclcpp::Clock & clock);

private:
  StateParam state_param_;
  State state_{State::GO};
  rclcpp::Time stop_time_;
  rclcpp::Time prev_approach_time_;
  boost::optional<DynamicObstacle> prev_obstacle_{};
  boost::optional<DynamicObstacle> target_obstacle_{};
};
}  // namespace run_out_utils
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__RUN_OUT__STATE_MACHINE_HPP_
