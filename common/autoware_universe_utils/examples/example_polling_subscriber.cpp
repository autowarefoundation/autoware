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

#include "autoware/universe_utils/ros/polling_subscriber.hpp"

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

// PublisherNode class definition
class PublisherNode : public rclcpp::Node
{
public:
  PublisherNode() : Node("publisher_node")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&PublisherNode::publish_message, this));
  }

private:
  void publish_message()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello from publisher node";
    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// SubscriberNode class definition
class SubscriberNode : public rclcpp::Node
{
public:
  SubscriberNode() : Node("subscriber_node")
  {
    subscription_ = autoware::universe_utils::InterProcessPollingSubscriber<
      std_msgs::msg::String>::create_subscription(this, "topic");

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&SubscriberNode::timer_callback, this));
  }

private:
  void timer_callback() const
  {
    auto msg = subscription_->takeData();
    if (msg) {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
  }

  autoware::universe_utils::InterProcessPollingSubscriber<std_msgs::msg::String>::SharedPtr
    subscription_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto publisher_node = std::make_shared<PublisherNode>();
  auto subscriber_node = std::make_shared<SubscriberNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(publisher_node);
  executor.add_node(subscriber_node);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
