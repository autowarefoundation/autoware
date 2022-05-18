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

#ifndef COMPONENT_INTERFACE_UTILS__RCLCPP__TOPIC_PUBLISHER_HPP_
#define COMPONENT_INTERFACE_UTILS__RCLCPP__TOPIC_PUBLISHER_HPP_

#include <rclcpp/publisher.hpp>

namespace component_interface_utils
{

/// The wrapper class of rclcpp::Publisher. This is for future use and no functionality now.
template <class SpecT>
class Publisher
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Publisher)
  using SpecType = SpecT;
  using WrapType = rclcpp::Publisher<typename SpecT::Message>;

  /// Constructor.
  explicit Publisher(typename WrapType::SharedPtr publisher)
  {
    publisher_ = publisher;  // to keep the reference count
  }

  /// Publish a message.
  void publish(const typename SpecT::Message & msg) { publisher_->publish(msg); }

private:
  RCLCPP_DISABLE_COPY(Publisher)
  typename WrapType::SharedPtr publisher_;
};

}  // namespace component_interface_utils

#endif  // COMPONENT_INTERFACE_UTILS__RCLCPP__TOPIC_PUBLISHER_HPP_
