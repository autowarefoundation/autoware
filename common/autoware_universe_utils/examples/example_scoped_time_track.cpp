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

#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("scoped_time_track_example");

  auto time_keeper = std::make_shared<autoware::universe_utils::TimeKeeper>();

  time_keeper->add_reporter(&std::cout);

  auto publisher =
    node->create_publisher<autoware::universe_utils::ProcessingTimeDetail>("processing_time", 10);

  time_keeper->add_reporter(publisher);

  auto publisher_str = node->create_publisher<std_msgs::msg::String>("processing_time_str", 10);

  time_keeper->add_reporter(publisher_str);

  auto funcA = [&time_keeper]() {
    autoware::universe_utils::ScopedTimeTrack st("funcA", *time_keeper);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  };

  auto funcB = [&time_keeper, &funcA]() {
    autoware::universe_utils::ScopedTimeTrack st("funcB", *time_keeper);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    funcA();
  };

  auto funcC = [&time_keeper, &funcB]() {
    autoware::universe_utils::ScopedTimeTrack st("funcC", *time_keeper);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    funcB();
  };

  funcC();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
