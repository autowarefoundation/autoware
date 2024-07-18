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

#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

class ExampleNode : public rclcpp::Node
{
public:
  ExampleNode() : Node("time_keeper_example")
  {
    publisher_ =
      create_publisher<autoware::universe_utils::ProcessingTimeDetail>("processing_time", 1);

    time_keeper_ = std::make_shared<autoware::universe_utils::TimeKeeper>(publisher_, &std::cerr);
    // You can also add a reporter later by add_reporter.
    // time_keeper_->add_reporter(publisher_);
    // time_keeper_->add_reporter(&std::cerr);

    timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&ExampleNode::func_a, this));
  }

private:
  std::shared_ptr<autoware::universe_utils::TimeKeeper> time_keeper_;
  rclcpp::Publisher<autoware::universe_utils::ProcessingTimeDetail>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_str_;
  rclcpp::TimerBase::SharedPtr timer_;

  void func_a()
  {
    // Start constructing ProcessingTimeTree (because func_a is the root function)
    autoware::universe_utils::ScopedTimeTrack st("func_a", *time_keeper_);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    time_keeper_->comment("This is a comment for func_a");
    func_b();
    // End constructing ProcessingTimeTree. After this, the tree will be reported (publishing
    // message and outputting to std::cerr)
  }

  void func_b()
  {
    autoware::universe_utils::ScopedTimeTrack st("func_b", *time_keeper_);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    time_keeper_->comment("This is a comment for func_b");
    func_c();
  }

  void func_c()
  {
    autoware::universe_utils::ScopedTimeTrack st("func_c", *time_keeper_);
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
    time_keeper_->comment("This is a comment for func_c");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExampleNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
