// Copyright 2024 The Autoware Contributors
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

#include "diagnostic_graph_utils/subscription.hpp"

namespace diagnostic_graph_utils
{

DiagGraphSubscription::DiagGraphSubscription()
{
  graph_ = std::make_shared<DiagGraph>();
}

DiagGraphSubscription::DiagGraphSubscription(rclcpp::Node & node, size_t depth)
{
  graph_ = std::make_shared<DiagGraph>();
  subscribe(node, depth);
}

void DiagGraphSubscription::subscribe(rclcpp::Node & node, size_t depth)
{
  const auto qos_struct = rclcpp::QoS(1).transient_local();
  const auto qos_status = rclcpp::QoS(depth);

  sub_struct_ = node.create_subscription<DiagGraphStruct>(
    "/diagnostics_graph/struct", qos_struct,
    std::bind(&DiagGraphSubscription::on_struct, this, std::placeholders::_1));
  sub_status_ = node.create_subscription<DiagGraphStatus>(
    "/diagnostics_graph/status", qos_status,
    std::bind(&DiagGraphSubscription::on_status, this, std::placeholders::_1));
}

void DiagGraphSubscription::register_create_callback(const CallbackType & callback)
{
  create_callback_ = callback;
}

void DiagGraphSubscription::register_update_callback(const CallbackType & callback)
{
  update_callback_ = callback;
}

void DiagGraphSubscription::on_struct(const DiagGraphStruct & msg)
{
  graph_->create(msg);
  if (create_callback_) create_callback_(graph_);
}

void DiagGraphSubscription::on_status(const DiagGraphStatus & msg)
{
  if (graph_->update(msg)) {
    if (update_callback_) update_callback_(graph_);
  }
}

}  // namespace diagnostic_graph_utils
