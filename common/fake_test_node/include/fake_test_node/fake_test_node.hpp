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

#ifndef FAKE_TEST_NODE__FAKE_TEST_NODE_HPP_
#define FAKE_TEST_NODE__FAKE_TEST_NODE_HPP_

#include <fake_test_node/visibility_control.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <type_traits>

namespace autoware
{
namespace tools
{
namespace testing
{

///
/// @brief      Forward definition of the parametrized fake node to be used with `TEST_P`.
///
/// @tparam     T     Type of the parameter as forwarded to testing::TestWithParam<T>.
///
template <typename T>
class FAKE_TEST_NODE_PUBLIC FakeTestNodeParametrized;

///
/// @brief      A forward definition of the non-parametrized fake node to be used with `TEST_F`.
///
class FAKE_TEST_NODE_PUBLIC FakeTestNode;

namespace detail
{

///
/// @brief      This class defines a test fake node parametrized with the parameter type.
///
/// @details    The user must typedef this node to a new name and use that new name in their
///             `TEST_P` (with parameter type provided) or `TEST_F` (with no parameter type
///             provided) entries. It takes care of the boilerplate creation of a node and whenever
///             a publisher or a subscription are created it checks that they have a matching
///             subscription or publisher respectively on the same topic.
///
/// @note       This class should not be used by the user, but FakeTestNode and
///             FakeTestNodeParametrized should be used instead.
///
class FAKE_TEST_NODE_PUBLIC FakeNodeCore
{
public:
  ///
  /// @brief      Initialize rclcpp, tf and setup the fake node.
  ///
  /// @param[in]  test_name  The name of the currently run test, usually received from the fixture.
  ///
  void set_up(const std::string & test_name);

  ///
  /// @brief      Shuts down rclcpp.
  ///
  void tear_down();

  ///
  /// @brief      Create a publisher with a custom topic and message type.
  ///
  /// @details    This does not do much more than create the relevant publisher and check that there
  ///             is a subscription with the relevant topic being requested.
  ///
  /// @param[in]  topic         The topic
  /// @param[in]  timeout       The timeout for matching to a subscription
  /// @param[in]  qos           The QoS profile for the publisher handler
  ///
  /// @tparam     MsgT          Type of messages to publish.
  ///
  /// @throws     runtime_error If no matching subscriber for `topic` found within `timeout`
  ///
  /// @return     A publisher pointer;
  ///
  template <typename MsgT>
  typename rclcpp::Publisher<MsgT>::SharedPtr create_publisher(
    const std::string & topic,
    const std::chrono::milliseconds & timeout = std::chrono::seconds{10LL},
    const rclcpp::QoS & qos = rclcpp::QoS(rclcpp::KeepLast(10)))
  {
    // Set the QoS profile history depth
    typename rclcpp::Publisher<MsgT>::SharedPtr publisher =
      m_fake_node->create_publisher<MsgT>(topic, qos);

    std::chrono::milliseconds spent_time{0LL};
    std::chrono::milliseconds dt{100LL};
    while (m_fake_node->count_subscribers(topic) < 1) {
      spent_time += dt;
      if (spent_time > timeout) {
        throw std::runtime_error(
          std::string("No matching subscriber to the mock topic '") + topic + "' that we publish");
      }
      std::this_thread::sleep_for(dt);
    }
    return publisher;
  }

  ///
  /// @brief      Creates a subscription to a certain message type for a given node.
  ///
  /// @param[in]  topic            The topic to subscribe to
  /// @param[in]  publishing_node  The node that publishes the data that this subscription
  ///                              subscribes to. This function will check that the newly created
  ///                              subscription matches a publisher in this node.
  /// @param[in]  callback         The callback to be called by the subscription
  /// @param[in]  timeout          The timeout for matching to a publisher
  /// @param[in]  qos              The QoS profile for the subscription handler
  ///
  /// @tparam     MsgT             Message type to which this must subscribe
  /// @tparam     NodeT            The type of the node under test
  ///
  /// @throws     runtime_error If no matching publisher for `topic` is found within `timeout`
  ///
  /// @return     Returns a subscription pointer.
  ///
  template <typename MsgT, typename NodeT>
  typename rclcpp::Subscription<MsgT>::SharedPtr create_subscription(
    const std::string & topic, const NodeT & publishing_node,
    std::function<void(const typename MsgT::SharedPtr msg)> callback,
    const std::chrono::milliseconds & timeout = std::chrono::seconds{10LL},
    const rclcpp::QoS & qos = rclcpp::QoS(rclcpp::KeepLast(10)))
  {
    auto subscription = m_fake_node->create_subscription<MsgT>(topic, qos, callback);
    std::chrono::milliseconds spent_time{0LL};
    std::chrono::milliseconds dt{100LL};
    while (publishing_node.count_publishers(topic) < 1) {
      spent_time += dt;
      if (spent_time > timeout) {
        throw std::runtime_error(
          std::string("The node under test '") + publishing_node.get_name() +
          "' is not publishing the topic '" + topic + "' that we listen to.");
      }
      std::this_thread::sleep_for(dt);
    }
    return subscription;
  }

  ///
  /// @brief      Gets the mock node.
  ///
  /// @return     The mock node.
  ///
  inline rclcpp::Node::SharedPtr get_fake_node() { return m_fake_node; }

  ///
  /// @brief      Gets the tf buffer.
  ///
  /// @return     The tf buffer.
  ///
  inline tf2::BufferCore & get_tf_buffer() { return m_tf_buffer; }

private:
  std::shared_ptr<rclcpp::Node> m_fake_node{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
  tf2::BufferCore m_tf_buffer;
};

///
/// @brief      A utility function that gets the full test name from the current fixture instance.
///
/// @param[in]  info  The current test information
///
/// @return     The full test name.
///
FAKE_TEST_NODE_PUBLIC std::string get_test_name(const ::testing::TestInfo * info);

}  // namespace detail

///
/// @brief      This class describes a fake test node that inherits from the parametrized GTest
///             fixture.
///
/// @tparam     T     Type of parameter to be used in the fixture.
///
template <typename T>
class FAKE_TEST_NODE_PUBLIC FakeTestNodeParametrized : public detail::FakeNodeCore,
                                                       public ::testing::TestWithParam<T>
{
  using FakeNodeCore::FakeNodeCore;

public:
  ///
  /// @brief      Override the setup function of the fixture.
  ///
  void SetUp() override
  {
    set_up(detail::get_test_name(::testing::UnitTest::GetInstance()->current_test_info()));
  }

  ///
  /// @brief      Override the tear down function of the fixture.
  ///
  void TearDown() override { tear_down(); }
};

///
/// @brief      This class describes a fake test node that inherits from the parametrized GTest
///             fixture.
///
class FAKE_TEST_NODE_PUBLIC FakeTestNode : public detail::FakeNodeCore, public ::testing::Test
{
  using FakeNodeCore::FakeNodeCore;

public:
  ///
  /// @brief      Override the setup function of the fixture.
  ///
  void SetUp() override;

  ///
  /// @brief      Override the tear down function of the fixture.
  ///
  void TearDown() override;
};

}  // namespace testing
}  // namespace tools
}  // namespace autoware

#endif  // FAKE_TEST_NODE__FAKE_TEST_NODE_HPP_
