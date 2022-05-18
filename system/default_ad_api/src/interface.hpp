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

#ifndef INTERFACE_HPP_
#define INTERFACE_HPP_

#include "default_ad_api/specs/interface/version.hpp"
#include "utils/types.hpp"

#include <component_interface_utils/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>

namespace default_ad_api
{

class InterfaceNode : public rclcpp::Node
{
public:
  explicit InterfaceNode(const rclcpp::NodeOptions & options);

private:
  Service<ad_api::interface::version::T>::SharedPtr srv_;
};

}  // namespace default_ad_api

#endif  // INTERFACE_HPP_
