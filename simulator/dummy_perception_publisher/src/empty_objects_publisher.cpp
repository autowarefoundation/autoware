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

#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>

#include <memory>
#include <utility>

class EmptyObjectsPublisher : public rclcpp::Node
{
public:
  EmptyObjectsPublisher() : Node("empty_objects_publisher")
  {
    empty_objects_pub_ = this->create_publisher<autoware_perception_msgs::msg::PredictedObjects>(
      "~/output/objects", 1);

    using std::chrono_literals::operator""ms;
    timer_ = rclcpp::create_timer(
      this, get_clock(), 100ms, std::bind(&EmptyObjectsPublisher::timerCallback, this));
  }

private:
  rclcpp::Publisher<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr empty_objects_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void timerCallback()
  {
    autoware_perception_msgs::msg::PredictedObjects empty_objects;
    empty_objects.header.frame_id = "map";
    empty_objects.header.stamp = this->now();
    empty_objects_pub_->publish(empty_objects);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EmptyObjectsPublisher>());
  rclcpp::shutdown();
  return 0;
}
