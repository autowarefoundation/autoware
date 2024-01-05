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

#ifndef TIER4_API_UTILS__RCLCPP__CLIENT_HPP_
#define TIER4_API_UTILS__RCLCPP__CLIENT_HPP_

#include "rclcpp/client.hpp"
#include "tier4_api_utils/types/response.hpp"

#include <chrono>
#include <utility>

namespace tier4_api_utils
{
template <typename ServiceT>
class Client
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Client)

  using ResponseStatus = tier4_external_api_msgs::msg::ResponseStatus;
  using AutowareServiceResult = std::pair<ResponseStatus, typename ServiceT::Response::SharedPtr>;

  Client(typename rclcpp::Client<ServiceT>::SharedPtr client, const rclcpp::Logger & logger)
  : client_(client), logger_(logger)
  {
  }

  AutowareServiceResult call(
    const typename ServiceT::Request::SharedPtr & request,
    const std::chrono::nanoseconds & timeout = std::chrono::seconds(2))
  {
    RCLCPP_DEBUG(logger_, "client request");

    if (!client_->service_is_ready()) {
      RCLCPP_DEBUG(logger_, "client available");
      return {response_error("Internal service is not available."), nullptr};
    }

    auto future = client_->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) {
      RCLCPP_DEBUG(logger_, "client timeout");
      return {response_error("Internal service has timed out."), nullptr};
    }

    RCLCPP_DEBUG(logger_, "client response");
    return {response_success(), future.get()};
  }

private:
  RCLCPP_DISABLE_COPY(Client)

  typename rclcpp::Client<ServiceT>::SharedPtr client_;
  rclcpp::Logger logger_;
};

}  // namespace tier4_api_utils

#endif  // TIER4_API_UTILS__RCLCPP__CLIENT_HPP_
