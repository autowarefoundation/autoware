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

#ifndef AUTOWARE_AD_API_SPECS__PERCEPTION_HPP_
#define AUTOWARE_AD_API_SPECS__PERCEPTION_HPP_

#include <rclcpp/qos.hpp>

#include <autoware_adapi_v1_msgs/msg/dynamic_object_array.hpp>

namespace autoware_ad_api::perception
{

struct DynamicObjectArray
{
  using Message = autoware_adapi_v1_msgs::msg::DynamicObjectArray;
  static constexpr char name[] = "/api/perception/objects";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};

}  // namespace autoware_ad_api::perception

#endif  // AUTOWARE_AD_API_SPECS__PERCEPTION_HPP_
