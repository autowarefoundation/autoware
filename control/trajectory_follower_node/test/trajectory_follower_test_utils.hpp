// Copyright 2021 The Autoware Foundation
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

#ifndef TRAJECTORY_FOLLOWER_TEST_UTILS_HPP_
#define TRAJECTORY_FOLLOWER_TEST_UTILS_HPP_

#include "fake_test_node/fake_test_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

#include "geometry_msgs/msg/transform_stamped.hpp"

#include <memory>
#include <string>

namespace test_utils
{
using FakeNodeFixture = autoware::tools::testing::FakeTestNode;

inline void waitForMessage(
  const std::shared_ptr<rclcpp::Node> & node, FakeNodeFixture * fixture, const bool & received_flag,
  const std::chrono::duration<int> max_wait_time = std::chrono::seconds{10LL},
  const bool fail_on_timeout = true)
{
  const auto dt{std::chrono::milliseconds{100LL}};
  auto time_passed{std::chrono::milliseconds{0LL}};
  while (!received_flag) {
    rclcpp::spin_some(node);
    rclcpp::spin_some(fixture->get_fake_node());
    std::this_thread::sleep_for(dt);
    time_passed += dt;
    if (time_passed > max_wait_time) {
      if (fail_on_timeout) {
        throw std::runtime_error(std::string("Did not receive a message soon enough"));
      } else {
        break;
      }
    }
  }
}

inline geometry_msgs::msg::TransformStamped getDummyTransform()
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.transform.translation.x = 0.0;
  transform_stamped.transform.translation.y = 0.0;
  transform_stamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  transform_stamped.transform.rotation.x = q.x();
  transform_stamped.transform.rotation.y = q.y();
  transform_stamped.transform.rotation.z = q.z();
  transform_stamped.transform.rotation.w = q.w();
  transform_stamped.header.frame_id = "map";
  transform_stamped.child_frame_id = "base_link";
  return transform_stamped;
}

// TODO(Horibe): modify the controller nodes so that they does not publish topics when data is not
// ready. then, remove this function.
template <typename T>
inline void spinWhile(T & node)
{
  for (size_t i = 0; i < 10; i++) {
    rclcpp::spin_some(node);
    const auto dt{std::chrono::milliseconds{100LL}};
    std::this_thread::sleep_for(dt);
  }
}
}  // namespace test_utils

#endif  // TRAJECTORY_FOLLOWER_TEST_UTILS_HPP_
