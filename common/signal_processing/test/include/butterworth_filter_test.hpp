// Copyright 2022 Tier IV, Inc.
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

#ifndef BUTTERWORTH_FILTER_TEST_HPP_
#define BUTTERWORTH_FILTER_TEST_HPP_

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "signal_processing/butterworth.hpp"

class ButterWorthTestFixture : public ::testing::Test
{
protected:
  ButterWorthTestFixture() = default;

  ~ButterWorthTestFixture() override = default;
};

#endif  // BUTTERWORTH_FILTER_TEST_HPP_
