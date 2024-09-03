// Copyright 2022 The Autoware Contributors
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

#include "stop_check_module.hpp"

namespace autoware::pose_initializer
{
StopCheckModule::StopCheckModule(rclcpp::Node * node, double buffer_duration)
: VehicleStopCheckerBase(node, buffer_duration)
{
  sub_twist_ = node->create_subscription<TwistWithCovarianceStamped>(
    "stop_check_twist", 1, std::bind(&StopCheckModule::on_twist, this, std::placeholders::_1));
}

void StopCheckModule::on_twist(TwistWithCovarianceStamped::ConstSharedPtr msg)
{
  TwistStamped twist;
  twist.header = msg->header;
  twist.twist = msg->twist.twist;
  addTwist(twist);
}
}  // namespace autoware::pose_initializer
