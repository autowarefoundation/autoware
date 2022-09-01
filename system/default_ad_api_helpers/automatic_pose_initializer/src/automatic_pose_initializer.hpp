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

#ifndef AUTOMATIC_POSE_INITIALIZER_HPP_
#define AUTOMATIC_POSE_INITIALIZER_HPP_

#include <autoware_ad_api_specs/localization.hpp>
#include <component_interface_utils/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>

namespace automatic_pose_initializer
{

class AutomaticPoseInitializer : public rclcpp::Node
{
public:
  AutomaticPoseInitializer();

private:
  void on_timer();
  using Initialize = autoware_ad_api::localization::Initialize;
  using State = autoware_ad_api::localization::InitializationState;
  rclcpp::CallbackGroup::SharedPtr group_cli_;
  rclcpp::TimerBase::SharedPtr timer_;
  component_interface_utils::Client<Initialize>::SharedPtr cli_initialize_;
  component_interface_utils::Subscription<State>::SharedPtr sub_state_;
  State::Message state_;
};

}  // namespace automatic_pose_initializer

#endif  // AUTOMATIC_POSE_INITIALIZER_HPP_
