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

#include "utils.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/state_machine.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <iostream>
#include <limits>

using StateMachine = autoware::behavior_velocity_planner::StateMachine;
using State = autoware::behavior_velocity_planner::StateMachine::State;

int enumToInt(State s)
{
  return static_cast<int>(s);
}

TEST(state_machine, on_initialized)
{
  StateMachine state_machine = StateMachine();
  EXPECT_EQ(enumToInt(State::GO), enumToInt(state_machine.getState()));
}

TEST(state_machine, set_state_stop)
{
  StateMachine state_machine = StateMachine();
  state_machine.setState(State::STOP);
  EXPECT_EQ(enumToInt(State::STOP), enumToInt(state_machine.getState()));
}

TEST(state_machine, set_state_stop_with_margin_time)
{
  StateMachine state_machine = StateMachine();
  const double margin_time = 1.0;
  state_machine.setMarginTime(margin_time);
  rclcpp::Clock current_time = rclcpp::Clock(RCL_ROS_TIME);
  rclcpp::Logger logger = rclcpp::get_logger("test_set_state_with_margin_time");
  // DO NOT SET GO until margin time past
  EXPECT_EQ(enumToInt(state_machine.getState()), enumToInt(State::GO));
  state_machine.setStateWithMarginTime(State::STOP, logger, current_time);
  // set STOP immediately when stop is set
  EXPECT_EQ(enumToInt(state_machine.getState()), enumToInt(State::STOP));
}

TEST(state_machine, set_state_go_with_margin_time)
{
  StateMachine state_machine = StateMachine();
  const double margin_time = 0.2;
  state_machine.setMarginTime(margin_time);
  rclcpp::Logger logger = rclcpp::get_logger("test_set_state_with_margin_time");
  state_machine.setState(State::STOP);
  size_t loop_counter = 0;
  // loop until state change from STOP -> GO
  while (state_machine.getState() == State::STOP) {
    EXPECT_EQ(enumToInt(state_machine.getState()), enumToInt(State::STOP));
    rclcpp::Clock current_time = rclcpp::Clock(RCL_ROS_TIME);
    if (state_machine.getDuration() > margin_time) {
      std::cerr << "stop duration is larger than margin time" << std::endl;
    }
    EXPECT_FALSE(state_machine.getDuration() > margin_time);
    state_machine.setStateWithMarginTime(State::GO, logger, current_time);
    loop_counter++;
  }
  // time past STOP -> GO
  if (loop_counter > 2) {
    EXPECT_TRUE(state_machine.getDuration() > margin_time);
    EXPECT_EQ(enumToInt(state_machine.getState()), enumToInt(State::GO));
  } else {
    std::cerr << "[Warning] computational resource is not enough" << std::endl;
  }
}
