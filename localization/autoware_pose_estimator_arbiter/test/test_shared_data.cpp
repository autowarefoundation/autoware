// Copyright 2023 Autoware Foundation
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

#include "shared_data.hpp"

#include <gtest/gtest.h>

TEST(SharedData, callback_invoked_correctly)
{
  autoware::pose_estimator_arbiter::CallbackInvokingVariable<int> variable;

  // register callback
  bool callback_invoked = false;
  int processed_value = 0;
  variable.register_callback([&](const int & value) {
    callback_invoked = true;
    processed_value = 2 * value;
  });

  EXPECT_FALSE(variable.has_value());
  EXPECT_FALSE(callback_invoked);
  EXPECT_EQ(processed_value, 0);

  // set value and invoke callback
  const int expected_value = 10;
  variable.set_and_invoke(expected_value);

  EXPECT_TRUE(variable.has_value());
  EXPECT_TRUE(callback_invoked);
  EXPECT_TRUE(variable() == expected_value);
  EXPECT_TRUE(processed_value == 2 * expected_value);
}

TEST(SharedData, multiple_callback_invoked_correctly)
{
  autoware::pose_estimator_arbiter::CallbackInvokingVariable<int> variable;

  // register callback
  int callback_invoked_num = 0;
  variable.register_callback([&](const int &) { callback_invoked_num++; });
  variable.register_callback([&](const int &) { callback_invoked_num++; });
  variable.register_callback([&](const int &) { callback_invoked_num++; });

  // set value and invoke callback
  variable.set_and_invoke(10);

  EXPECT_EQ(callback_invoked_num, 3);
}
