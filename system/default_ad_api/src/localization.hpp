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

#ifndef LOCALIZATION_HPP_
#define LOCALIZATION_HPP_

#include <autoware_ad_api_specs/localization.hpp>
#include <component_interface_specs/localization.hpp>
#include <rclcpp/rclcpp.hpp>

// This file should be included after messages.
#include "utils/types.hpp"

namespace default_ad_api
{

class LocalizationNode : public rclcpp::Node
{
public:
  explicit LocalizationNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::CallbackGroup::SharedPtr group_cli_;
  Srv<autoware_ad_api::localization::Initialize> srv_initialize_;
  Pub<autoware_ad_api::localization::InitializationState> pub_state_;
  Cli<localization_interface::Initialize> cli_initialize_;
  Sub<localization_interface::InitializationState> sub_state_;
};

}  // namespace default_ad_api

#endif  // LOCALIZATION_HPP_
