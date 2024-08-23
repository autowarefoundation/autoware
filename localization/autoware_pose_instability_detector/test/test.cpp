// Copyright 2023- Autoware Foundation
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

#include "autoware/pose_instability_detector/pose_instability_detector.hpp"
#include "test_message_helper_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>
#include <rcl_yaml_param_parser/parser.h>

#include <memory>
#include <string>
#include <vector>

class TestPoseInstabilityDetector : public ::testing::Test
{
  using TwistWithCovarianceStamped = geometry_msgs::msg::TwistWithCovarianceStamped;
  using Odometry = nav_msgs::msg::Odometry;
  using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
  using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;

protected:
  void SetUp() override
  {
    const std::string yaml_path =
      ament_index_cpp::get_package_share_directory("autoware_pose_instability_detector") +
      "/config/pose_instability_detector.param.yaml";

    rcl_params_t * params_st = rcl_yaml_node_struct_init(rcl_get_default_allocator());
    if (!rcl_parse_yaml_file(yaml_path.c_str(), params_st)) {
      std::cerr << "Failed to parse yaml file : " << yaml_path << std::endl;
      std::exit(1);
    }

    const rclcpp::ParameterMap param_map = rclcpp::parameter_map_from(params_st, "");
    rclcpp::NodeOptions node_options;
    for (const auto & param_pair : param_map) {
      for (const auto & param : param_pair.second) {
        node_options.parameter_overrides().push_back(param);
      }
    }

    subject_ =
      std::make_shared<autoware::pose_instability_detector::PoseInstabilityDetector>(node_options);
    executor_.add_node(subject_);

    helper_ = std::make_shared<TestMessageHelperNode>();
    executor_.add_node(helper_);

    rcl_yaml_node_struct_fini(params_st);
  }

  void TearDown() override
  {
    executor_.remove_node(subject_);
    executor_.remove_node(helper_);
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<autoware::pose_instability_detector::PoseInstabilityDetector> subject_;
  std::shared_ptr<TestMessageHelperNode> helper_;
};

TEST_F(TestPoseInstabilityDetector, output_ok_when_twist_matches_odometry)  // NOLINT
{
  // send the first odometry message (start x = 10)
  builtin_interfaces::msg::Time timestamp{};
  timestamp.sec = 0;
  timestamp.nanosec = 0;
  helper_->send_odometry_message(timestamp, 10.0, 0.0, 0.0);

  // send the twist message1 (move 1m in x direction)
  timestamp.sec = 0;
  timestamp.nanosec = 5e8;
  helper_->send_twist_message(timestamp, 2.0, 0.0, 0.0);

  // process the above message (by timer_callback)
  helper_->received_diagnostic_array_flag = false;
  while (!helper_->received_diagnostic_array_flag) {
    executor_.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // send the twist message2 (move 1m in x direction)
  timestamp.sec = 1;
  timestamp.nanosec = 0;
  helper_->send_twist_message(timestamp, 2.0, 0.0, 0.0);

  // send the second odometry message (finish x = 12)
  timestamp.sec = 2;
  timestamp.nanosec = 0;
  helper_->send_odometry_message(timestamp, 14.0, 0.0, 0.0);

  executor_.spin_some();

  // process the above messages (by timer_callback)
  helper_->received_diagnostic_array_flag = false;
  while (!helper_->received_diagnostic_array_flag) {
    executor_.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // check result
  const diagnostic_msgs::msg::DiagnosticStatus & diagnostic_status =
    helper_->received_diagnostic_array.status[0];
  EXPECT_TRUE(diagnostic_status.level == diagnostic_msgs::msg::DiagnosticStatus::OK);
}

TEST_F(TestPoseInstabilityDetector, output_warn_when_twist_is_too_small)  // NOLINT
{
  // send the first odometry message (start x = 10)
  builtin_interfaces::msg::Time timestamp{};
  timestamp.sec = 0;
  timestamp.nanosec = 0;
  helper_->send_odometry_message(timestamp, 10.0, 0.0, 0.0);

  // send the twist message1 (move 0.1m in x direction)
  timestamp.sec = 0;
  timestamp.nanosec = 5e8;
  helper_->send_twist_message(timestamp, 0.2, 0.0, 0.0);

  // process the above message (by timer_callback)
  helper_->received_diagnostic_array_flag = false;
  while (!helper_->received_diagnostic_array_flag) {
    executor_.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // send the twist message2 (move 0.1m in x direction)
  timestamp.sec = 1;
  timestamp.nanosec = 0;
  helper_->send_twist_message(timestamp, 0.2, 0.0, 0.0);

  // send the second odometry message (finish x = 12)
  timestamp.sec = 2;
  timestamp.nanosec = 0;
  helper_->send_odometry_message(timestamp, 14.0, 0.0, 0.0);

  executor_.spin_some();

  // process the above messages (by timer_callback)
  helper_->received_diagnostic_array_flag = false;
  while (!helper_->received_diagnostic_array_flag) {
    executor_.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // check result
  const diagnostic_msgs::msg::DiagnosticStatus & diagnostic_status =
    helper_->received_diagnostic_array.status[0];
  EXPECT_TRUE(diagnostic_status.level == diagnostic_msgs::msg::DiagnosticStatus::WARN);
}

TEST_F(TestPoseInstabilityDetector, does_not_crash_even_if_abnormal_odometry_data_comes)  // NOLINT
{
  // [Condition] There is no twist_msg between the two target odometry_msgs.
  // Normally this doesn't seem to happen.
  // As far as I can think, this happens when the odometry msg stops (so the next timer callback
  // will refer to the same odometry msg, and the timestamp difference will be calculated as 0)
  // This test case shows that an error occurs when two odometry msgs come in close succession and
  // there is no other odometry msg.
  // Referring again, this doesn't normally seem to happen in usual operation.

  builtin_interfaces::msg::Time timestamp{};

  // send the twist message1
  timestamp.sec = 0;
  timestamp.nanosec = 0;
  helper_->send_twist_message(timestamp, 0.2, 0.0, 0.0);

  // send the first odometry message after the first twist message
  timestamp.sec = 0;
  timestamp.nanosec = 5e8 + 1;
  helper_->send_odometry_message(timestamp, 10.0, 0.0, 0.0);

  // process the above message (by timer_callback)
  helper_->received_diagnostic_array_flag = false;
  while (!helper_->received_diagnostic_array_flag) {
    executor_.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // send the second odometry message before the second twist message
  timestamp.sec = 0;
  timestamp.nanosec = 5e8 + 1e7;
  helper_->send_odometry_message(timestamp, 12.0, 0.0, 0.0);

  // send the twist message2
  timestamp.sec = 1;
  timestamp.nanosec = 0;
  helper_->send_twist_message(timestamp, 0.2, 0.0, 0.0);

  // process the above messages (by timer_callback)
  helper_->received_diagnostic_array_flag = false;
  while (!helper_->received_diagnostic_array_flag) {
    executor_.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // provoke timer callback again
  helper_->received_diagnostic_array_flag = false;
  while (!helper_->received_diagnostic_array_flag) {
    executor_.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // This test is OK if pose_instability_detector does not crash. The diagnostics status is not
  // checked.
  SUCCEED();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
