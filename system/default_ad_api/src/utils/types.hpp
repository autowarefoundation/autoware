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

#ifndef UTILS__TYPES_HPP_
#define UTILS__TYPES_HPP_

#include <component_interface_utils/rclcpp.hpp>

#define MESSAGE_ARG(Type) const Type::ConstSharedPtr message
#define SERVICE_ARG(Type) \
  const Type::Request::SharedPtr request, const Type::Response::SharedPtr response
#define SERVICE_ARG_NO_REQ(Type) \
  const Type::Request::SharedPtr, const Type::Response::SharedPtr response

namespace default_ad_api
{

template <class T>
using Publisher = component_interface_utils::Publisher<T>;
template <class T>
using Subscription = component_interface_utils::Subscription<T>;
template <class T>
using Client = component_interface_utils::Client<T>;
template <class T>
using Service = component_interface_utils::Service<T>;

}  // namespace default_ad_api

#endif  // UTILS__TYPES_HPP_
