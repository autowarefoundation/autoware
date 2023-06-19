// Copyright 2023 The Autoware Foundation
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
#ifndef ADAPTER_BASE_HPP_
#define ADAPTER_BASE_HPP_

#include "adapter_base_interface.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace autoware_auto_msgs_adapter
{

template <typename SourceT, typename TargetT>
class AdapterBase : public AdapterBaseInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AdapterBase<SourceT, TargetT>)

  AdapterBase(
    rclcpp::Node & node, const std::string & topic_name_source,
    const std::string & topic_name_target, const rclcpp::QoS & qos = rclcpp::QoS{1})
  {
    pub_target_ = node.create_publisher<TargetT>(topic_name_target, qos);
    sub_source_ = node.create_subscription<SourceT>(
      topic_name_source, qos, std::bind(&AdapterBase::callback, this, std::placeholders::_1));
  }

protected:
  virtual TargetT convert(const SourceT & msg_source) = 0;

private:
  typename rclcpp::Publisher<TargetT>::SharedPtr pub_target_;
  typename rclcpp::Subscription<SourceT>::SharedPtr sub_source_;

  void callback(const typename SourceT::SharedPtr msg_source)
  {
    pub_target_->publish(convert(*msg_source));
  }
};

}  // namespace autoware_auto_msgs_adapter

#endif  // ADAPTER_BASE_HPP_
