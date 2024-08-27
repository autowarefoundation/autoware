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
#include "autoware/universe_utils/system/time_keeper.hpp"

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <sstream>
#include <thread>

class TimeKeeperTest : public ::testing::Test
{
protected:
  std::ostringstream oss;
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<autoware::universe_utils::ProcessingTimeDetail>::SharedPtr publisher;
  std::unique_ptr<autoware::universe_utils::TimeKeeper> time_keeper;

  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>("test_node");
    publisher = node->create_publisher<autoware::universe_utils::ProcessingTimeDetail>(
      "~/debug/processing_time_tree", 1);
    time_keeper = std::make_unique<autoware::universe_utils::TimeKeeper>(&oss, publisher);
  }
};

TEST_F(TimeKeeperTest, BasicFunctionality)
{
  using autoware::universe_utils::ScopedTimeTrack;

  {
    ScopedTimeTrack st{"main_func", *time_keeper};

    {  // funcA
      ScopedTimeTrack st{"funcA", *time_keeper};
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    {  // funcB
      ScopedTimeTrack st{"funcB", *time_keeper};
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      {  // funcC
        ScopedTimeTrack st{"funcC", *time_keeper};
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
  }

  // Check if the output contains all function names
  std::string output = oss.str();
  EXPECT_TRUE(output.find("main_func") != std::string::npos);
  EXPECT_TRUE(output.find("funcA") != std::string::npos);
  EXPECT_TRUE(output.find("funcB") != std::string::npos);
  EXPECT_TRUE(output.find("funcC") != std::string::npos);
}

TEST_F(TimeKeeperTest, MultiThreadWarning)
{
  testing::internal::CaptureStderr();

  std::thread t([this]() {
    time_keeper->start_track("ThreadFunction");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    time_keeper->end_track("ThreadFunction");
  });

  time_keeper->start_track("MainFunction");
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  time_keeper->end_track("MainFunction");

  t.join();

  std::string err = testing::internal::GetCapturedStderr();
  EXPECT_TRUE(
    err.find("TimeKeeper::start_track() is called from a different thread. Ignoring the call.") !=
    std::string::npos);
}
