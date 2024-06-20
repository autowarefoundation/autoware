// Copyright 2024 The Autoware Contributors
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

#ifndef AUTOWARE__UNIVERSE_UTILS__ROS__PUBLISHED_TIME_PUBLISHER_HPP_
#define AUTOWARE__UNIVERSE_UTILS__ROS__PUBLISHED_TIME_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_msgs/msg/published_time.hpp>
#include <std_msgs/msg/header.hpp>

#include <cstring>
#include <functional>
#include <map>
#include <string>
#include <utility>

namespace autoware::universe_utils
{

class PublishedTimePublisher
{
public:
  explicit PublishedTimePublisher(
    rclcpp::Node * node, std::string publisher_topic_suffix = "/debug/published_time",
    const rclcpp::QoS & qos = rclcpp::QoS(1))
  : node_(node), publisher_topic_suffix_(std::move(publisher_topic_suffix)), qos_(qos)
  {
  }

  void publish_if_subscribed(
    const rclcpp::PublisherBase::ConstSharedPtr & publisher, const rclcpp::Time & stamp)
  {
    const auto & gid_key = publisher->get_gid();

    // if the publisher is not in the map, create a new publisher for published time
    ensure_publisher_exists(gid_key, publisher->get_topic_name());

    const auto & pub_published_time = publishers_[gid_key];

    // Check if there are any subscribers, otherwise don't do anything
    if (pub_published_time->get_subscription_count() > 0) {
      PublishedTime published_time;

      published_time.header.stamp = stamp;
      published_time.published_stamp = rclcpp::Clock().now();

      pub_published_time->publish(published_time);
    }
  }

  void publish_if_subscribed(
    const rclcpp::PublisherBase::ConstSharedPtr & publisher, const std_msgs::msg::Header & header)
  {
    const auto & gid_key = publisher->get_gid();

    // if the publisher is not in the map, create a new publisher for published time
    ensure_publisher_exists(gid_key, publisher->get_topic_name());

    const auto & pub_published_time = publishers_[gid_key];

    // Check if there are any subscribers, otherwise don't do anything
    if (pub_published_time->get_subscription_count() > 0) {
      PublishedTime published_time;

      published_time.header = header;
      published_time.published_stamp = rclcpp::Clock().now();

      pub_published_time->publish(published_time);
    }
  }

private:
  rclcpp::Node * node_;
  std::string publisher_topic_suffix_;
  rclcpp::QoS qos_;

  using PublishedTime = autoware_internal_msgs::msg::PublishedTime;

  // Custom comparison struct for rmw_gid_t
  struct GidCompare
  {
    bool operator()(const rmw_gid_t & lhs, const rmw_gid_t & rhs) const
    {
      return std::memcmp(lhs.data, rhs.data, RMW_GID_STORAGE_SIZE) < 0;
    }
  };

  // ensure that the publisher exists in publisher_ map, if not, create a new one
  void ensure_publisher_exists(const rmw_gid_t & gid_key, const std::string & topic_name)
  {
    if (publishers_.find(gid_key) == publishers_.end()) {
      publishers_[gid_key] =
        node_->create_publisher<PublishedTime>(topic_name + publisher_topic_suffix_, qos_);
    }
  }

  // store them for each different publisher of the node
  std::map<rmw_gid_t, rclcpp::Publisher<PublishedTime>::SharedPtr, GidCompare> publishers_;
};
}  // namespace autoware::universe_utils

#endif  // AUTOWARE__UNIVERSE_UTILS__ROS__PUBLISHED_TIME_PUBLISHER_HPP_
