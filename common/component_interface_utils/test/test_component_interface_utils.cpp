// Copyright 2023 The Autoware Contributors
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

#include "component_interface_utils/rclcpp/exceptions.hpp"
#include "component_interface_utils/specs.hpp"
#include "component_interface_utils/status.hpp"
#include "gtest/gtest.h"

TEST(interface, utils)
{
  {
    using component_interface_utils::ServiceException;
    using ResponseStatus = autoware_adapi_v1_msgs::msg::ResponseStatus;
    using ResponseStatusCode = ResponseStatus::_code_type;

    ResponseStatusCode code = 10;
    const std::string message = "test_exception";
    ServiceException service(code, message);
    ResponseStatus code_back;
    code_back = service.status();
    EXPECT_EQ(code_back.code, code);
    EXPECT_EQ(code_back.message, message);
  }

  {
    using component_interface_utils::ServiceException;
    using ResponseStatus = autoware_adapi_v1_msgs::msg::ResponseStatus;
    using ResponseStatusCode = ResponseStatus::_code_type;

    ResponseStatusCode code = 10;
    const std::string message = "test_exception";
    ServiceException service(code, message);
    ResponseStatus code_set;
    service.set(code_set);
    EXPECT_EQ(code_set.code, code);
    EXPECT_EQ(code_set.message, message);
  }

  {
    using component_interface_utils::ServiceException;
    using ResponseStatus = autoware_adapi_v1_msgs::msg::ResponseStatus;
    using ResponseStatusCode = ResponseStatus::_code_type;
    using component_interface_utils::status::copy;

    class status_test
    {
    public:
      status_test(ResponseStatusCode code, const std::string & message, bool success = false)
      {
        status.code = code;
        status.message = message;
        status.success = success;
      }
      ResponseStatus status;
    };

    const status_test status_in(10, "test_exception", true);
    auto status_copy = std::make_shared<status_test>(100, "test_exception_copy", false);
    copy(&status_in, status_copy);

    EXPECT_EQ(status_in.status.code, status_copy->status.code);
    EXPECT_EQ(status_in.status.message, status_copy->status.message);
    EXPECT_EQ(status_in.status.success, status_copy->status.success);
  }
}
