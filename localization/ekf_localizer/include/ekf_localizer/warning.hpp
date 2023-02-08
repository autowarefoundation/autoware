// Copyright 2022 Autoware Foundation
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

#ifndef EKF_LOCALIZER__WARNING_HPP_
#define EKF_LOCALIZER__WARNING_HPP_

#include <rclcpp/rclcpp.hpp>

#include <string>

class Warning
{
public:
  explicit Warning(rclcpp::Node * node) : node_(node) {}

  void warn(const std::string & message) const
  {
    RCLCPP_WARN(node_->get_logger(), "%s", message.c_str());
  }

  void warnThrottle(const std::string & message, const int duration_milliseconds) const
  {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *(node_->get_clock()),
      std::chrono::milliseconds(duration_milliseconds).count(), "%s", message.c_str());
  }

private:
  rclcpp::Node * node_;
};

#endif  // EKF_LOCALIZER__WARNING_HPP_
