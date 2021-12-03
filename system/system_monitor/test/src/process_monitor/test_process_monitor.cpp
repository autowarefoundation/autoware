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

#include "system_monitor/process_monitor/process_monitor.hpp"

#include <rclcpp/rclcpp.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/process.hpp>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include <memory>
#include <string>

namespace bp = boost::process;
namespace fs = boost::filesystem;
using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

char ** argv_;

class TestProcessMonitor : public ProcessMonitor
{
  friend class ProcessMonitorTestSuite;

public:
  TestProcessMonitor(const std::string & node_name, const rclcpp::NodeOptions & options)
  : ProcessMonitor(node_name, options)
  {
  }

  void diagCallback(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr diag_msg)
  {
    array_ = *diag_msg;
  }

  int getNumOfProcs() const { return num_of_procs_; }

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

class ProcessMonitorTestSuite : public ::testing::Test
{
public:
  ProcessMonitorTestSuite()
  {
    // Get directory of executable
    const fs::path exe_path(argv_[0]);
    exe_dir_ = exe_path.parent_path().generic_string();
    // Get dummy executable path
    top_ = exe_dir_ + "/top";
    echo_ = exe_dir_ + "/echo";
    sed_ = exe_dir_ + "/sed";
    sort_ = exe_dir_ + "/sort";
  }

protected:
  std::unique_ptr<TestProcessMonitor> monitor_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_;
  std::string exe_dir_;
  std::string top_;
  std::string echo_;
  std::string sed_;
  std::string sort_;

  void SetUp()
  {
    using std::placeholders::_1;
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions node_options;
    monitor_ = std::make_unique<TestProcessMonitor>("test_process_monitor", node_options);
    sub_ = monitor_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 1000, std::bind(&TestProcessMonitor::diagCallback, monitor_.get(), _1));

    // Remove dummy executable if exists
    if (fs::exists(top_)) {
      fs::remove(top_);
    }
    if (fs::exists(echo_)) {
      fs::remove(echo_);
    }
    if (fs::exists(sed_)) {
      fs::remove(sed_);
    }
    if (fs::exists(sort_)) {
      fs::remove(sort_);
    }
  }

  void TearDown()
  {
    // Remove dummy executable if exists
    if (fs::exists(top_)) {
      fs::remove(top_);
    }
    if (fs::exists(echo_)) {
      fs::remove(echo_);
    }
    if (fs::exists(sed_)) {
      fs::remove(sed_);
    }
    if (fs::exists(sort_)) {
      fs::remove(sort_);
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

TEST_F(ProcessMonitorTestSuite, tasksSummaryTest)
{
  // Publish topic
  monitor_->update();

  // Give time to publish
  rclcpp::WallRate(2).sleep();
  rclcpp::spin_some(monitor_->get_node_base_interface());

  // Verify
  DiagStatus status;
  ASSERT_TRUE(monitor_->findDiagStatus("Tasks Summary", status));
  ASSERT_EQ(status.level, DiagStatus::OK);
}

TEST_F(ProcessMonitorTestSuite, highLoadProcTest)
{
  // Publish topic
  monitor_->update();

  // Give time to publish
  rclcpp::WallRate(2).sleep();
  rclcpp::spin_some(monitor_->get_node_base_interface());

  // Verify
  DiagStatus status;
  std::string value;

  for (int i = 0; i < monitor_->getNumOfProcs(); ++i) {
    ASSERT_TRUE(monitor_->findDiagStatus(fmt::format("High-load Proc[{}]", i), status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }
}

TEST_F(ProcessMonitorTestSuite, highMemProcTest)
{
  // Publish topic
  monitor_->update();

  // Give time to publish
  rclcpp::WallRate(2).sleep();
  rclcpp::spin_some(monitor_->get_node_base_interface());

  // Verify
  DiagStatus status;
  std::string value;

  for (int i = 0; i < monitor_->getNumOfProcs(); ++i) {
    ASSERT_TRUE(monitor_->findDiagStatus(fmt::format("High-mem Proc[{}]", i), status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }
}

TEST_F(ProcessMonitorTestSuite, topErrorTest)
{
  // Symlink top1 to top
  fs::create_symlink(exe_dir_ + "/top1", top_);

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

  ASSERT_TRUE(monitor_->findDiagStatus("Tasks Summary", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "top error");
  ASSERT_TRUE(findValue(status, "top", value));
  ASSERT_STREQ(value.c_str(), "");

  for (int i = 0; i < monitor_->getNumOfProcs(); ++i) {
    ASSERT_TRUE(monitor_->findDiagStatus(fmt::format("High-load Proc[{}]", i), status));
    ASSERT_EQ(status.level, DiagStatus::ERROR);
    ASSERT_STREQ(status.message.c_str(), "top error");

    ASSERT_TRUE(monitor_->findDiagStatus(fmt::format("High-mem Proc[{}]", i), status));
    ASSERT_EQ(status.level, DiagStatus::ERROR);
    ASSERT_STREQ(status.message.c_str(), "top error");
  }
}

TEST_F(ProcessMonitorTestSuite, matchingPatternNotFoundTest)
{
  // Symlink top2 to top
  fs::create_symlink(exe_dir_ + "/top2", top_);

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

  ASSERT_TRUE(monitor_->findDiagStatus("Tasks Summary", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "matching pattern not found");
}

TEST_F(ProcessMonitorTestSuite, invalidFormatTest)
{
  // Symlink top3 to top
  fs::create_symlink(exe_dir_ + "/top3", top_);

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

  ASSERT_TRUE(monitor_->findDiagStatus("Tasks Summary", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "invalid format");
}

TEST_F(ProcessMonitorTestSuite, echoErrorTest)
{
  // Symlink sed1 to sed
  fs::create_symlink(exe_dir_ + "/echo1", echo_);

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

  ASSERT_TRUE(monitor_->findDiagStatus("Tasks Summary", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "echo error");
}

TEST_F(ProcessMonitorTestSuite, sedErrorTest)
{
  // Symlink sed1 to sed
  fs::create_symlink(exe_dir_ + "/sed1", sed_);

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

  ASSERT_TRUE(monitor_->findDiagStatus("Tasks Summary", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "sed error");
}

TEST_F(ProcessMonitorTestSuite, sortErrorTest)
{
  // Symlink sort1 to sort
  fs::create_symlink(exe_dir_ + "/sort1", sort_);

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

  for (int i = 0; i < monitor_->getNumOfProcs(); ++i) {
    ASSERT_TRUE(monitor_->findDiagStatus(fmt::format("High-mem Proc[{}]", i), status));
    ASSERT_EQ(status.level, DiagStatus::ERROR);
    ASSERT_STREQ(status.message.c_str(), "sort error");
  }
}

int main(int argc, char ** argv)
{
  argv_ = argv;
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
