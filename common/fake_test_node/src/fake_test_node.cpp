// Copyright 2021 Apex.AI, Inc.
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

/// \copyright Copyright 2021 Apex.AI, Inc.
/// All rights reserved.

#include <fake_test_node/fake_test_node.hpp>

#include <memory>
#include <string>

namespace
{
constexpr auto kSpinThread = false;
constexpr auto kArgc = 0;

std::string sanitize_test_name(const std::string & name)
{
  auto sanitize_test_name = name;
  std::replace(sanitize_test_name.begin(), sanitize_test_name.end(), '/', '_');
  return sanitize_test_name;
}

}  // namespace

namespace autoware
{
namespace tools
{
namespace testing
{

void detail::FakeNodeCore::set_up(const std::string & test_name)
{
  ASSERT_FALSE(rclcpp::ok());
  rclcpp::init(kArgc, nullptr);
  ASSERT_TRUE(rclcpp::ok());
  m_fake_node = std::make_shared<rclcpp::Node>("FakeNodeForTest_" + sanitize_test_name(test_name));
  m_tf_listener =
    std::make_shared<tf2_ros::TransformListener>(m_tf_buffer, m_fake_node, kSpinThread);
}

void detail::FakeNodeCore::tear_down()
{
  (void)rclcpp::shutdown();
}

std::string detail::get_test_name(const ::testing::TestInfo * info)
{
  if (!info) {
    throw std::runtime_error("No test info available.");
  }
  return std::string{info->test_case_name()} + "_" + info->name();
}

void FakeTestNode::SetUp()
{
  set_up(detail::get_test_name(::testing::UnitTest::GetInstance()->current_test_info()));
}

void FakeTestNode::TearDown()
{
  tear_down();
}

}  // namespace testing
}  // namespace tools
}  // namespace autoware
