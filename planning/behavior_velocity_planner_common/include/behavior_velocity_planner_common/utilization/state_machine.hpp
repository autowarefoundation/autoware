// Copyright 2021 Tier IV, Inc.
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

#ifndef BEHAVIOR_VELOCITY_PLANNER_COMMON__UTILIZATION__STATE_MACHINE_HPP_
#define BEHAVIOR_VELOCITY_PLANNER_COMMON__UTILIZATION__STATE_MACHINE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <utility>

namespace behavior_velocity_planner
{
/**
 * @brief Manage stop-go states with safety margin time.
 */
class StateMachine
{
public:
  enum State {
    STOP = 0,
    GO,
  };
  static std::string toString(const State & state)
  {
    if (state == State::STOP) {
      return "STOP";
    } else if (state == State::GO) {
      return "GO";
    } else {
      return "";
    }
  }
  StateMachine()
  {
    state_ = State::GO;
    margin_time_ = 0.0;
    duration_ = 0.0;
  }
  void setStateWithMarginTime(State state, rclcpp::Logger logger, rclcpp::Clock & clock)
  {
    /* same state request */
    if (state_ == state) {
      start_time_ = nullptr;  // reset timer
      return;
    }

    /* GO -> STOP */
    if (state == State::STOP) {
      state_ = State::STOP;
      start_time_ = nullptr;  // reset timer
      return;
    }

    /* STOP -> GO */
    if (state == State::GO) {
      if (start_time_ == nullptr) {
        start_time_ = std::make_shared<rclcpp::Time>(clock.now());
      } else {
        duration_ = (clock.now() - *start_time_).seconds();
        if (duration_ > margin_time_) {
          state_ = State::GO;
          start_time_ = nullptr;  // reset timer
        }
      }
      return;
    }
    RCLCPP_ERROR(logger, "Unsuitable state. ignore request.");
  }

  void setMarginTime(const double t) { margin_time_ = t; }
  void setState(State state) { state_ = state; }
  State getState() const { return state_; }
  double getDuration() const { return duration_; }

private:
  State state_;                               //! current state
  double margin_time_;                        //! margin time when transit to Go from Stop
  double duration_;                           //! duration time when transit to Go from Stop
  std::shared_ptr<rclcpp::Time> start_time_;  //! first time received GO when STOP state
};

}  // namespace behavior_velocity_planner
#endif  // BEHAVIOR_VELOCITY_PLANNER_COMMON__UTILIZATION__STATE_MACHINE_HPP_
