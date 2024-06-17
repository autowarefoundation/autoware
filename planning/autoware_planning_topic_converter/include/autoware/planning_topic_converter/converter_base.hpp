// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTOWARE__PLANNING_TOPIC_CONVERTER__CONVERTER_BASE_HPP_
#define AUTOWARE__PLANNING_TOPIC_CONVERTER__CONVERTER_BASE_HPP_

#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <string>

namespace autoware::planning_topic_converter
{

template <typename InputType, typename OutputType>
class ConverterBase : public rclcpp::Node
{
public:
  ConverterBase(const std::string & node_name, const rclcpp::NodeOptions & options)
  : rclcpp::Node(node_name, options)
  {
    const auto input_topic = this->declare_parameter<std::string>("input_topic");
    const auto output_topic = this->declare_parameter<std::string>("output_topic");

    pub_ = this->create_publisher<OutputType>(output_topic, 1);
    sub_ = this->create_subscription<InputType>(
      input_topic, 1, std::bind(&ConverterBase::process, this, std::placeholders::_1));
  }

protected:
  virtual void process(const typename InputType::ConstSharedPtr msg) = 0;
  typename rclcpp::Publisher<OutputType>::SharedPtr pub_;
  typename rclcpp::Subscription<InputType>::SharedPtr sub_;

private:
};

}  // namespace autoware::planning_topic_converter

#endif  // AUTOWARE__PLANNING_TOPIC_CONVERTER__CONVERTER_BASE_HPP_
