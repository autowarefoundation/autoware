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

#ifndef COMPONENT_INTERFACE_UTILS__RCLCPP__EXCEPTIONS_HPP_
#define COMPONENT_INTERFACE_UTILS__RCLCPP__EXCEPTIONS_HPP_

#include <autoware_adapi_v1_msgs/msg/response_status.hpp>

#include <exception>
#include <string>

namespace component_interface_utils
{

class ServiceException : public std::exception
{
public:
  using ResponseStatus = autoware_adapi_v1_msgs::msg::ResponseStatus;
  using ResponseStatusCode = ResponseStatus::_code_type;

  ServiceException(ResponseStatusCode code, const std::string & message, bool success = false)
  {
    success_ = success;
    code_ = code;
    message_ = message;
  }

  template <class T>
  void set(T & status) const
  {
    status.success = success_;
    status.code = code_;
    status.message = message_;
  }

  ResponseStatus status() const
  {
    ResponseStatus status;
    status.success = success_;
    status.code = code_;
    status.message = message_;
    return status;
  }

private:
  bool success_;
  ResponseStatusCode code_;
  std::string message_;
};

class ServiceUnready : public ServiceException
{
public:
  explicit ServiceUnready(const std::string & message)
  : ServiceException(ResponseStatus::SERVICE_UNREADY, message, false)
  {
  }
};

class ServiceTimeout : public ServiceException
{
public:
  explicit ServiceTimeout(const std::string & message)
  : ServiceException(ResponseStatus::SERVICE_TIMEOUT, message, false)
  {
  }
};

class TransformError : public ServiceException
{
public:
  explicit TransformError(const std::string & message)
  : ServiceException(ResponseStatus::TRANSFORM_ERROR, message, false)
  {
  }
};

class ParameterError : public ServiceException
{
public:
  explicit ParameterError(const std::string & message)
  : ServiceException(ResponseStatus::PARAMETER_ERROR, message, false)
  {
  }
};

class NoEffectWarning : public ServiceException
{
public:
  explicit NoEffectWarning(const std::string & message)
  : ServiceException(ResponseStatus::NO_EFFECT, message, true)
  {
  }
};

}  // namespace component_interface_utils

#endif  // COMPONENT_INTERFACE_UTILS__RCLCPP__EXCEPTIONS_HPP_
