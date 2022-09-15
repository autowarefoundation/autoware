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

#include "ndt_module.hpp"

#include <component_interface_specs/localization.hpp>
#include <component_interface_utils/rclcpp/exceptions.hpp>

#include <memory>

using ServiceException = component_interface_utils::ServiceException;
using Initialize = localization_interface::Initialize;
using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

NdtModule::NdtModule(rclcpp::Node * node) : logger_(node->get_logger())
{
  cli_align_ = node->create_client<RequestPoseAlignment>("ndt_align");
}

PoseWithCovarianceStamped NdtModule::align_pose(const PoseWithCovarianceStamped & pose)
{
  const auto req = std::make_shared<RequestPoseAlignment::Request>();
  req->pose_with_covariance = pose;

  if (!cli_align_->service_is_ready()) {
    throw component_interface_utils::ServiceUnready("NDT align server is not ready.");
  }

  RCLCPP_INFO(logger_, "Call NDT align server.");
  const auto res = cli_align_->async_send_request(req).get();
  if (!res->success) {
    RCLCPP_INFO(logger_, "NDT align server failed.");
    throw ServiceException(
      Initialize::Service::Response::ERROR_ESTIMATION, "NDT align server failed.");
  }
  RCLCPP_INFO(logger_, "NDT align server succeeded.");

  // Overwrite the covariance.
  return res->pose_with_covariance;
}
