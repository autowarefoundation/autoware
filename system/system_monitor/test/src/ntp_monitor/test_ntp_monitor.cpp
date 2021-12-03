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

#include "system_monitor/ntp_monitor/ntp_monitor.hpp"

#include <rclcpp/rclcpp.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/process.hpp>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include <memory>
#include <string>

namespace fs = boost::filesystem;
using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

char ** argv_;

class TestNTPMonitor : public NTPMonitor
{
  friend class NTPMonitorTestSuite;

public:
  TestNTPMonitor(const std::string & node_name, const rclcpp::NodeOptions & options)
  : NTPMonitor(node_name, options)
  {
  }

  void diagCallback(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr diag_msg)
  {
    array_ = *diag_msg;
  }

  void changeOffsetWarn(float offset_warn) { offset_warn_ = offset_warn; }
  void changeOffsetError(float offset_error) { offset_error_ = offset_error; }

  void setNtpdateExists(bool ntpdate_exists) { ntpdate_exists_ = ntpdate_exists; }

  void update() { updater_.force_update(); }

  const std::string removePrefix(const std::string & name)
  {
    return boost::algorithm::erase_all_copy(name, prefix_);
  }

  bool findDiagStatus(const std::string & name, DiagStatus & status)  // NOLINT
  {
    for (int i = 0; i < array_.status.size(); ++i) {
      if (removePrefix(array_.status[i].name) == name) {
        status = array_.status[i];
        return true;
      }
    }
    return false;
  }

private:
  diagnostic_msgs::msg::DiagnosticArray array_;
  const std::string prefix_ = std::string(this->get_name()) + ": ";
};

class NTPMonitorTestSuite : public ::testing::Test
{
public:
  NTPMonitorTestSuite()
  {
    // Get directory of executable
    const fs::path exe_path(argv_[0]);
    exe_dir_ = exe_path.parent_path().generic_string();
    // Get dummy executable path
    ntpdate_ = exe_dir_ + "/ntpdate";
  }

protected:
  std::unique_ptr<TestNTPMonitor> monitor_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_;
  std::string exe_dir_;
  std::string ntpdate_;

  void SetUp()
  {
    using std::placeholders::_1;
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions node_options;
    monitor_ = std::make_unique<TestNTPMonitor>("test_ntp_monitor", node_options);
    sub_ = monitor_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 1000, std::bind(&TestNTPMonitor::diagCallback, monitor_.get(), _1));

    // Remove dummy executable if exists
    if (fs::exists(ntpdate_)) {
      fs::remove(ntpdate_);
    }
  }

  void TearDown()
  {
    // Remove dummy executable if exists
    if (fs::exists(ntpdate_)) {
      fs::remove(ntpdate_);
    }
    rclcpp::shutdown();
  }

  bool findValue(const DiagStatus status, const std::string & key, std::string & value)  // NOLINT
  {
    for (auto itr = status.values.begin(); itr != status.values.end(); ++itr) {
      if (itr->key == key) {
        value = itr->value;
        return true;
      }
    }
    return false;
  }

  void modifyPath()
  {
    // Modify PATH temporarily
    auto env = boost::this_process::environment();
    std::string new_path = env["PATH"].to_string();
    new_path.insert(0, fmt::format("{}:", exe_dir_));
    env["PATH"] = new_path;
  }
};

TEST_F(NTPMonitorTestSuite, offsetWarnTest)
{
  // Verify normal behavior
  {
    // Publish topic
    monitor_->update();

    // Give time to publish
    rclcpp::WallRate(2).sleep();
    rclcpp::spin_some(monitor_->get_node_base_interface());

    // Verify
    DiagStatus status;
    std::string value;
    ASSERT_TRUE(monitor_->findDiagStatus("NTP Offset", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }

  // Verify warning
  {
    // Change warning level
    monitor_->changeOffsetWarn(0.0);

    // Publish topic
    monitor_->update();

    // Give time to publish
    rclcpp::WallRate(2).sleep();
    rclcpp::spin_some(monitor_->get_node_base_interface());

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("NTP Offset", status));
    ASSERT_EQ(status.level, DiagStatus::WARN);
  }

  // Verify normal behavior
  {
    // Change back to normal
    monitor_->changeOffsetWarn(0.05);

    // Publish topic
    monitor_->update();

    // Give time to publish
    rclcpp::WallRate(2).sleep();
    rclcpp::spin_some(monitor_->get_node_base_interface());

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("NTP Offset", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }
}

TEST_F(NTPMonitorTestSuite, offsetErrorTest)
{
  // Verify normal behavior
  {
    // Publish topic
    monitor_->update();

    // Give time to publish
    rclcpp::WallRate(2).sleep();
    rclcpp::spin_some(monitor_->get_node_base_interface());

    // Verify
    DiagStatus status;
    std::string value;
    ASSERT_TRUE(monitor_->findDiagStatus("NTP Offset", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }

  // Verify warning
  {
    // Change warning level
    monitor_->changeOffsetError(0.0);

    // Publish topic
    monitor_->update();

    // Give time to publish
    rclcpp::WallRate(2).sleep();
    rclcpp::spin_some(monitor_->get_node_base_interface());

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("NTP Offset", status));
    ASSERT_EQ(status.level, DiagStatus::ERROR);
  }

  // Verify normal behavior
  {
    // Change back to normal
    monitor_->changeOffsetError(5.0);

    // Publish topic
    monitor_->update();

    // Give time to publish
    rclcpp::WallRate(2).sleep();
    rclcpp::spin_some(monitor_->get_node_base_interface());

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("NTP Offset", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }
}

TEST_F(NTPMonitorTestSuite, offsetNtpdateNotFoundTest)
{
  // Set flag false
  monitor_->setNtpdateExists(false);

  // Publish topic
  monitor_->update();

  // Give time to publish
  rclcpp::WallRate(2).sleep();
  rclcpp::spin_some(monitor_->get_node_base_interface());

  // Verify
  DiagStatus status;
  std::string value;
  ASSERT_TRUE(monitor_->findDiagStatus("NTP Offset", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "ntpdate error");
  ASSERT_TRUE(findValue(status, "ntpdate", value));
  ASSERT_STREQ(
    value.c_str(),
    "Command 'ntpdate' not found, but can be installed with: sudo apt install ntpdate");
}

TEST_F(NTPMonitorTestSuite, offsetNtpdateErrorTest)
{
  // Symlink ntpdate1 to ntpdate
  fs::create_symlink(exe_dir_ + "/ntpdate1", ntpdate_);

  // Modify PATH temporarily
  modifyPath();

  // Publish topic
  monitor_->update();

  // Give time to publish
  rclcpp::WallRate(2).sleep();
  rclcpp::spin_some(monitor_->get_node_base_interface());

  // Verify
  DiagStatus status;
  std::string value;
  ASSERT_TRUE(monitor_->findDiagStatus("NTP Offset", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "ntpdate error");
}

int main(int argc, char ** argv)
{
  argv_ = argv;
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
