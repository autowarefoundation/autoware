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

#include "scene_module/run_out/state_machine.hpp"

namespace behavior_velocity_planner
{
namespace run_out_utils
{
using State = StateMachine::State;

std::string StateMachine::toString(const State & state) const
{
  switch (state) {
    case State::GO:
      return "GO";

    case State::STOP:
      return "STOP";

    case State::APPROACH:
      return "APPROACH";

    default:
      return "UNKNOWN";
  }
}

void StateMachine::updateState(const StateInput & state_input, rclcpp::Clock & clock)
{
  // no obstacles
  if (!state_input.current_obstacle) {
    state_ = State::GO;
    return;
  }

  switch (state_) {
    case State::GO: {
      // if current velocity is less than the threshold, transit to STOP state
      if (state_input.current_velocity < state_param_.stop_thresh) {
        stop_time_ = clock.now();
        state_ = State::STOP;
        return;
      }

      // continue GO state
      return;
    }

    case State::STOP: {
      // if current velocity is larger than the threshold, transit to STOP state
      if (state_input.current_velocity > state_param_.stop_thresh) {
        state_ = State::GO;
        return;
      }

      // if STOP state continues for a certain time, transit to APPROACH state
      const auto elapsed_time = (clock.now() - stop_time_).seconds();
      if (elapsed_time > state_param_.stop_time_thresh) {
        state_ = State::APPROACH;
        return;
      }

      // continue STOP state
      return;
    }

    case State::APPROACH: {
      // if the obstacle is far enough from ego, transit to GO state
      const bool enough_dist_from_obstacle =
        state_input.dist_to_collision > state_param_.disable_approach_dist;
      if (enough_dist_from_obstacle) {
        state_ = State::GO;
        return;
      }

      // continue APPROACH state
      return;
    }

    default: {
      state_ = State::UNKNOWN;
      return;
    }
  }
}

}  // namespace run_out_utils
}  // namespace behavior_velocity_planner
