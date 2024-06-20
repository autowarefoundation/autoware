// Copyright 2020 Tier IV, Inc.
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

#ifndef AUTOWARE__UNIVERSE_UTILS__ROS__PROCESSING_TIME_PUBLISHER_HPP_
#define AUTOWARE__UNIVERSE_UTILS__ROS__PROCESSING_TIME_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <map>
#include <sstream>
#include <string>

namespace autoware::universe_utils
{
class ProcessingTimePublisher
{
public:
  explicit ProcessingTimePublisher(
    rclcpp::Node * node, const std::string & name = "~/debug/processing_time_ms",
    const rclcpp::QoS & qos = rclcpp::QoS(1))
  {
    pub_processing_time_ =
      node->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(name, qos);
  }

  void publish(const std::map<std::string, double> & processing_time_map)
  {
    diagnostic_msgs::msg::DiagnosticStatus status;

    for (const auto & m : processing_time_map) {
      diagnostic_msgs::msg::KeyValue key_value;
      key_value.key = m.first;
      key_value.value = to_string_with_precision(m.second, 3);
      status.values.push_back(key_value);
    }

    pub_processing_time_->publish(status);
  }

private:
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr pub_processing_time_;

  template <class T>
  std::string to_string_with_precision(const T & value, const int precision)
  {
    std::ostringstream oss;
    oss.precision(precision);
    oss << std::fixed << value;
    return oss.str();
  }
};
}  // namespace autoware::universe_utils

#endif  // AUTOWARE__UNIVERSE_UTILS__ROS__PROCESSING_TIME_PUBLISHER_HPP_
