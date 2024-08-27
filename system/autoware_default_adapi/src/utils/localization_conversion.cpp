// Copyright 2024 TIER IV, Inc.
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

#include "localization_conversion.hpp"

#include <memory>

namespace autoware::default_adapi::localization_conversion
{

InternalInitializeRequest convert_request(const ExternalInitializeRequest & external)
{
  auto internal = std::make_shared<InternalInitializeRequest::element_type>();
  internal->pose_with_covariance = external->pose;
  internal->method = tier4_localization_msgs::srv::InitializeLocalization::Request::AUTO;
  return internal;
}

ExternalResponse convert_response(const InternalResponse & internal)
{
  // TODO(Takagi, Isamu): check error code correspondence
  ExternalResponse external;
  external.success = internal.success;
  external.code = internal.code;
  external.message = internal.message;
  return external;
}

}  // namespace autoware::default_adapi::localization_conversion
