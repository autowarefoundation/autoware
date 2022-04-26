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

#ifndef DEFAULT_AD_API__NODES__INTERFACE_VERSION_HPP_
#define DEFAULT_AD_API__NODES__INTERFACE_VERSION_HPP_

#include "default_ad_api/specs/interface/version.hpp"

#include <component_interface_utils/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>

namespace default_ad_api
{

class InterfaceVersionNode : public rclcpp::Node
{
public:
  explicit InterfaceVersionNode(const rclcpp::NodeOptions & options);

private:
  using InterfaceVersion = autoware_ad_api_msgs::srv::InterfaceVersion;

  component_interface_utils::Service<ad_api::interface::version::T>::SharedPtr srv_;
  void onInterfaceVersion(
    const InterfaceVersion::Request::SharedPtr request,
    const InterfaceVersion::Response::SharedPtr response);
};

}  // namespace default_ad_api

#endif  // DEFAULT_AD_API__NODES__INTERFACE_VERSION_HPP_
