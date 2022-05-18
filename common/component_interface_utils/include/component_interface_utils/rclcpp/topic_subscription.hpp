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

#ifndef COMPONENT_INTERFACE_UTILS__RCLCPP__TOPIC_SUBSCRIPTION_HPP_
#define COMPONENT_INTERFACE_UTILS__RCLCPP__TOPIC_SUBSCRIPTION_HPP_

#include <rclcpp/subscription.hpp>

namespace component_interface_utils
{

/// The wrapper class of rclcpp::Subscription. This is for future use and no functionality now.
template <class SpecT>
class Subscription
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Subscription)
  using SpecType = SpecT;
  using WrapType = rclcpp::Subscription<typename SpecT::Message>;

  /// Constructor.
  explicit Subscription(typename WrapType::SharedPtr subscription)
  {
    subscription_ = subscription;  // to keep the reference count
  }

private:
  RCLCPP_DISABLE_COPY(Subscription)
  typename WrapType::SharedPtr subscription_;
};

}  // namespace component_interface_utils

#endif  // COMPONENT_INTERFACE_UTILS__RCLCPP__TOPIC_SUBSCRIPTION_HPP_
