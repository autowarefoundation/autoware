// Copyright 2021 Tier IV, Inc.
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

#ifndef TIER4_API_UTILS__RCLCPP__SERVICE_HPP_
#define TIER4_API_UTILS__RCLCPP__SERVICE_HPP_

#include "rclcpp/service.hpp"

namespace tier4_api_utils
{
template <typename ServiceT>
class Service
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Service)

  explicit Service(typename rclcpp::Service<ServiceT>::SharedPtr service) : service_(service) {}

  template <typename CallbackT>
  static auto wrap(CallbackT && callback, const rclcpp::Logger & logger)
  {
    auto wrapped_callback = [logger, callback](
                              typename ServiceT::Request::SharedPtr request,
                              typename ServiceT::Response::SharedPtr response) {
      RCLCPP_INFO(logger, "service request");
      callback(request, response);
      RCLCPP_INFO(logger, "service response");
    };
    return wrapped_callback;
  }

private:
  RCLCPP_DISABLE_COPY(Service)

  typename rclcpp::Service<ServiceT>::SharedPtr service_;
};

}  // namespace tier4_api_utils

#endif  // TIER4_API_UTILS__RCLCPP__SERVICE_HPP_
