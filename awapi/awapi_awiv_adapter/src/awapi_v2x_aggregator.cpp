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

#include "awapi_awiv_adapter/awapi_v2x_aggregator.hpp"

#include <memory>
#include <string>

namespace autoware_api
{
namespace
{
std::string createKey(const Command & command) { return command.type + "-" + command.id; }

std::string createKey(const State & state) { return state.type + "-" + state.id; }
}  // namespace

AutowareIvV2XAggregator::AutowareIvV2XAggregator(rclcpp::Node & node)
: logger_(node.get_logger().get_child("awapi_awiv_v2x_aggregator")), clock_(node.get_clock())
{
}

CommandArray::ConstSharedPtr AutowareIvV2XAggregator::updateV2XCommand(
  const CommandArray::ConstSharedPtr & msg)
{
  // Update data
  for (const auto & command : msg->commands) {
    const auto key = createKey(command);
    command_map_[key] = command;
  }

  // Pick valid data
  auto output = std::make_shared<CommandArray>();
  output->stamp = clock_->now();
  for (const auto & [key, command] : command_map_) {
    // Calculate time diff
    const auto delay = (clock_->now() - command.stamp).seconds();

    // Ignore future data considering clock's error
    if (delay < -max_clock_error_sec_) {
      RCLCPP_DEBUG(
        logger_, "future command: delay=%f, max_clock_error=%f", delay, max_clock_error_sec_);
      continue;
    }

    // Ignore old data
    if (delay > max_delay_sec_) {
      RCLCPP_DEBUG(logger_, "old command: delay=%f, max_delay_sec=%f", delay, max_delay_sec_);
      continue;
    }

    output->commands.push_back(command);
  }

  return output;
}

StateArray::ConstSharedPtr AutowareIvV2XAggregator::updateV2XState(
  const StateArray::ConstSharedPtr & msg)
{
  // Update data
  for (const auto & state : msg->states) {
    const auto key = createKey(state);
    state_map_[key] = state;
  }

  // Pick valid data
  auto output = std::make_shared<StateArray>();
  output->stamp = clock_->now();
  for (const auto & [key, state] : state_map_) {
    // Calculate time diff
    const auto delay = (clock_->now() - state.stamp).seconds();

    // Ignore future data considering clock's error
    if (delay < -max_clock_error_sec_) {
      RCLCPP_DEBUG(
        logger_, "future state: delay=%f, max_clock_error=%f", delay, max_clock_error_sec_);
      continue;
    }

    // Ignore old data
    if (delay > max_delay_sec_) {
      RCLCPP_DEBUG(logger_, "old state: delay=%f, max_delay_sec=%f", delay, max_delay_sec_);
      continue;
    }

    output->states.push_back(state);
  }

  return output;
}

}  // namespace autoware_api
