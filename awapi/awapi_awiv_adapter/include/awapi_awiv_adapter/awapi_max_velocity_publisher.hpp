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

#ifndef AWAPI_AWIV_ADAPTER__AWAPI_MAX_VELOCITY_PUBLISHER_HPP_
#define AWAPI_AWIV_ADAPTER__AWAPI_MAX_VELOCITY_PUBLISHER_HPP_

#include "awapi_awiv_adapter/awapi_autoware_util.hpp"

#include <rclcpp/rclcpp.hpp>

#include <tier4_planning_msgs/msg/velocity_limit.hpp>

namespace autoware_api
{
class AutowareIvMaxVelocityPublisher
{
public:
  AutowareIvMaxVelocityPublisher(rclcpp::Node & node, const double default_max_velocity);
  void statePublisher(const AutowareInfo & aw_info);

private:
  // publisher
  rclcpp::Publisher<tier4_planning_msgs::msg::VelocityLimit>::SharedPtr pub_state_;

  bool calcMaxVelocity(
    const tier4_api_msgs::msg::VelocityLimit::ConstSharedPtr & max_velocity_ptr,
    const tier4_api_msgs::msg::StopCommand::ConstSharedPtr & temporary_stop_ptr,
    float * max_velocity);

  double default_max_velocity_;
};

}  // namespace autoware_api

#endif  // AWAPI_AWIV_ADAPTER__AWAPI_MAX_VELOCITY_PUBLISHER_HPP_
