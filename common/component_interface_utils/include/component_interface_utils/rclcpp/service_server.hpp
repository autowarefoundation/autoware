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

#ifndef COMPONENT_INTERFACE_UTILS__RCLCPP__SERVICE_SERVER_HPP_
#define COMPONENT_INTERFACE_UTILS__RCLCPP__SERVICE_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>

namespace component_interface_utils
{

/// The wrapper class of rclcpp::Service for logging.
template <class SpecT>
class Service
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Service)

  template <class NodeT>
  using CallbackType = void (NodeT::*)(
    typename SpecT::Service::Request::SharedPtr, typename SpecT::Service::Response::SharedPtr);

  /// Constructor.
  explicit Service(typename rclcpp::Service<typename SpecT::Service>::SharedPtr service)
  {
    service_ = service;  // to keep the reference count
  }

  /// Create a service callback with logging added.
  template <class CallbackT>
  static auto wrap(CallbackT && callback, const rclcpp::Logger & logger)
  {
    auto wrapped = [logger, callback](
                     typename SpecT::Service::Request::SharedPtr request,
                     typename SpecT::Service::Response::SharedPtr response) {
#ifdef ROS_DISTRO_GALACTIC
      using rosidl_generator_traits::to_yaml;
#endif
      RCLCPP_INFO_STREAM(logger, "service call: " << SpecT::name << "\n" << to_yaml(*request));
      callback(request, response);
      RCLCPP_INFO_STREAM(logger, "service exit: " << SpecT::name << "\n" << to_yaml(*response));
    };
    return wrapped;
  }

private:
  RCLCPP_DISABLE_COPY(Service)
  typename rclcpp::Service<typename SpecT::Service>::SharedPtr service_;
};

}  // namespace component_interface_utils

#endif  // COMPONENT_INTERFACE_UTILS__RCLCPP__SERVICE_SERVER_HPP_
