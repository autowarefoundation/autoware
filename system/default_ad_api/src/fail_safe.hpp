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

#ifndef FAIL_SAFE_HPP_
#define FAIL_SAFE_HPP_

#include <autoware_ad_api_specs/fail_safe.hpp>
#include <component_interface_specs/system.hpp>
#include <component_interface_utils/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>

// This file should be included after messages.
#include "utils/types.hpp"

namespace default_ad_api
{

class FailSafeNode : public rclcpp::Node
{
public:
  explicit FailSafeNode(const rclcpp::NodeOptions & options);

private:
  using MrmState = autoware_ad_api::fail_safe::MrmState::Message;
  Pub<autoware_ad_api::fail_safe::MrmState> pub_mrm_state_;
  Sub<system_interface::MrmState> sub_mrm_state_;
  MrmState prev_state_;
  void on_state(const MrmState::ConstSharedPtr msg);
};

}  // namespace default_ad_api

#endif  // FAIL_SAFE_HPP_
