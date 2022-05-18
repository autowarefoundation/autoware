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

#ifndef COMPONENT_INTERFACE_UTILS__SPECS_HPP_
#define COMPONENT_INTERFACE_UTILS__SPECS_HPP_

#include <rclcpp/qos.hpp>

namespace component_interface_utils
{

template <class SpecT>
rclcpp::QoS get_qos()
{
  rclcpp::QoS qos(SpecT::depth);
  qos.reliability(SpecT::reliability);
  qos.durability(SpecT::durability);
  return qos;
}

}  // namespace component_interface_utils

#endif  // COMPONENT_INTERFACE_UTILS__SPECS_HPP_
