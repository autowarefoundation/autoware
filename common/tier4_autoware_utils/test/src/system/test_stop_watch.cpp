// Copyright 2020 Tier IV, Inc.
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

#include "tier4_autoware_utils/system/stop_watch.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

TEST(system, StopWatch_sec)
{
  using tier4_autoware_utils::StopWatch;

  StopWatch<std::chrono::seconds> stop_watch;

  stop_watch.tic("total");
  std::this_thread::sleep_for(std::chrono::seconds(1));

  stop_watch.tic();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  const auto t1 = stop_watch.toc();

  std::this_thread::sleep_for(std::chrono::seconds(1));
  const auto t2 = stop_watch.toc(true);

  std::this_thread::sleep_for(std::chrono::seconds(1));
  const auto t3 = stop_watch.toc();

  const auto t4 = stop_watch.toc("total");

  constexpr double error_sec = 0.01;
  EXPECT_NEAR(t1, 1.0, error_sec);
  EXPECT_NEAR(t2, 2.0, error_sec);
  EXPECT_NEAR(t3, 1.0, error_sec);
  EXPECT_NEAR(t4, 4.0, error_sec);
  ASSERT_ANY_THROW(stop_watch.toc("invalid_key"));
}

TEST(system, StopWatch_msec)
{
  using tier4_autoware_utils::StopWatch;

  StopWatch<std::chrono::milliseconds> stop_watch;

  stop_watch.tic();
  std::this_thread::sleep_for(std::chrono::seconds(1));

  constexpr double error_msec = 10.0;
  EXPECT_NEAR(stop_watch.toc(), 1000.0, error_msec);
}
