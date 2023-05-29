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

#ifndef AUTOWARE_AD_API_SPECS__LOCALIZATION_HPP_
#define AUTOWARE_AD_API_SPECS__LOCALIZATION_HPP_

#include <rclcpp/qos.hpp>

#include <autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>
#include <autoware_adapi_v1_msgs/srv/initialize_localization.hpp>

namespace autoware_ad_api::localization
{

struct Initialize
{
  using Service = autoware_adapi_v1_msgs::srv::InitializeLocalization;
  static constexpr char name[] = "/api/localization/initialize";
};

struct InitializationState
{
  using Message = autoware_adapi_v1_msgs::msg::LocalizationInitializationState;
  static constexpr char name[] = "/api/localization/initialization_state";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
};

}  // namespace autoware_ad_api::localization

#endif  // AUTOWARE_AD_API_SPECS__LOCALIZATION_HPP_
