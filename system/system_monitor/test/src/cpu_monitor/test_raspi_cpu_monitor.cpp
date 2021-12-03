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

#include "system_monitor/cpu_monitor/raspi_cpu_monitor.hpp"

#include <rclcpp/rclcpp.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/process.hpp>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include <fstream>
#include <memory>
#include <string>
#include <vector>

static constexpr const char * TEST_FILE = "test";

namespace fs = boost::filesystem;
using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

char ** argv_;

class TestCPUMonitor : public CPUMonitor
{
  friend class CPUMonitorTestSuite;

public:
  TestCPUMonitor(const std::string & node_name, const rclcpp::NodeOptions & options)
  : CPUMonitor(node_name, options)
  {
  }

  void diagCallback(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr diag_msg)
  {
    array_ = *diag_msg;
  }

  void addTempName(const std::string & path) { temps_.emplace_back(path, path); }
  void clearTempNames() { temps_.clear(); }

  void addFreqName(int index, const std::string & path) { freqs_.emplace_back(index, path); }
  void clearFreqNames() { freqs_.clear(); }

  void setMpstatExists(bool mpstat_exists) { mpstat_exists_ = mpstat_exists; }

  void changeUsageWarn(float usage_warn) { usage_warn_ = usage_warn; }
  void changeUsageError(float usage_error) { usage_error_ = usage_error; }

  void changeLoad1Warn(float load1_warn) { load1_warn_ = load1_warn; }
  void changeLoad5Warn(float load5_warn) { load5_warn_ = load5_warn; }

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

class CPUMonitorTestSuite : public ::testing::Test
{
public:
  CPUMonitorTestSuite()
  {
    // Get directory of executable
    const fs::path exe_path(argv_[0]);
    exe_dir_ = exe_path.parent_path().generic_string();
    // Get dummy executable path
    mpstat_ = exe_dir_ + "/mpstat";
  }

protected:
  std::unique_ptr<TestCPUMonitor> monitor_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_;
  std::string exe_dir_;
  std::string mpstat_;

  void SetUp()
  {
    using std::placeholders::_1;
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions node_options;
    monitor_ = std::make_unique<TestCPUMonitor>("test_cpu_monitor", node_options);
    sub_ = monitor_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 1000, std::bind(&TestCPUMonitor::diagCallback, monitor_.get(), _1));
    monitor_->getTempNames();
    monitor_->getFreqNames();

    // Remove test file if exists
    if (fs::exists(TEST_FILE)) {
      fs::remove(TEST_FILE);
    }
    // Remove dummy executable if exists
    if (fs::exists(mpstat_)) {
      fs::remove(mpstat_);
    }
  }

  void TearDown()
  {
    // Remove test file if exists
    if (fs::exists(TEST_FILE)) {
      fs::remove(TEST_FILE);
    }
    // Remove dummy executable if exists
    if (fs::exists(mpstat_)) {
      fs::remove(mpstat_);
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

TEST_F(CPUMonitorTestSuite, tempWarnTest)
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
    ASSERT_TRUE(monitor_->findDiagStatus("CPU Temperature", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }

  // Add test file to list
  monitor_->addTempName(TEST_FILE);

  // Verify warning
  {
    // Write warning level
    std::ofstream ofs(TEST_FILE);
    ofs << 90000 << std::endl;

    // Publish topic
    monitor_->update();

    // Give time to publish
    rclcpp::WallRate(2).sleep();
    rclcpp::spin_some(monitor_->get_node_base_interface());

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("CPU Temperature", status));
    ASSERT_EQ(status.level, DiagStatus::WARN);
  }

  // Verify normal behavior
  {
    // Write normal level
    std::ofstream ofs(TEST_FILE);
    ofs << 89900 << std::endl;

    // Publish topic
    monitor_->update();

    // Give time to publish
    rclcpp::WallRate(2).sleep();
    rclcpp::spin_some(monitor_->get_node_base_interface());

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("CPU Temperature", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }
}

TEST_F(CPUMonitorTestSuite, tempErrorTest)
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
    ASSERT_TRUE(monitor_->findDiagStatus("CPU Temperature", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }

  // Add test file to list
  monitor_->addTempName(TEST_FILE);

  // Verify error
  {
    // Write error level
    std::ofstream ofs(TEST_FILE);
    ofs << 95000 << std::endl;

    // Publish topic
    monitor_->update();

    // Give time to publish
    rclcpp::WallRate(2).sleep();
    rclcpp::spin_some(monitor_->get_node_base_interface());

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("CPU Temperature", status));
    ASSERT_EQ(status.level, DiagStatus::ERROR);
  }

  // Verify normal behavior
  {
    // Write normal level
    std::ofstream ofs(TEST_FILE);
    ofs << 89900 << std::endl;

    // Publish topic
    monitor_->update();

    // Give time to publish
    rclcpp::WallRate(2).sleep();
    rclcpp::spin_some(monitor_->get_node_base_interface());

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("CPU Temperature", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }
}

TEST_F(CPUMonitorTestSuite, tempTemperatureFilesNotFoundTest)
{
  // Clear list
  monitor_->clearTempNames();

  // Publish topic
  monitor_->update();

  // Give time to publish
  rclcpp::WallRate(2).sleep();
  rclcpp::spin_some(monitor_->get_node_base_interface());

  // Verify
  DiagStatus status;
  ASSERT_TRUE(monitor_->findDiagStatus("CPU Temperature", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "temperature files not found");
}

TEST_F(CPUMonitorTestSuite, tempFileOpenErrorTest)
{
  // Add test file to list
  monitor_->addTempName(TEST_FILE);

  // Publish topic
  monitor_->update();

  // Give time to publish
  rclcpp::WallRate(2).sleep();
  rclcpp::spin_some(monitor_->get_node_base_interface());

  // Verify
  DiagStatus status;
  std::string value;
  ASSERT_TRUE(monitor_->findDiagStatus("CPU Temperature", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "file open error");
  ASSERT_TRUE(findValue(status, "file open error", value));
  ASSERT_STREQ(value.c_str(), TEST_FILE);
}

TEST_F(CPUMonitorTestSuite, usageWarnTest)
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
    ASSERT_TRUE(monitor_->findDiagStatus("CPU Usage", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }

  // Verify warning
  {
    // Change warning level
    monitor_->changeUsageWarn(0.0);

    // Publish topic
    monitor_->update();

    // Give time to publish
    rclcpp::WallRate(2).sleep();
    rclcpp::spin_some(monitor_->get_node_base_interface());

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("CPU Usage", status));
    ASSERT_EQ(status.level, DiagStatus::WARN);
  }

  // Verify normal behavior
  {
    // Change back to normal
    monitor_->changeUsageWarn(0.90);

    // Publish topic
    monitor_->update();

    // Give time to publish
    rclcpp::WallRate(2).sleep();
    rclcpp::spin_some(monitor_->get_node_base_interface());

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("CPU Usage", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }
}

TEST_F(CPUMonitorTestSuite, usageErrorTest)
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
    ASSERT_TRUE(monitor_->findDiagStatus("CPU Usage", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }

  // Verify warning
  {
    // Change warning level
    monitor_->changeUsageError(0.0);

    // Publish topic
    monitor_->update();

    // Give time to publish
    rclcpp::WallRate(2).sleep();
    rclcpp::spin_some(monitor_->get_node_base_interface());

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("CPU Usage", status));
    ASSERT_EQ(status.level, DiagStatus::ERROR);
  }

  // Verify normal behavior
  {
    // Change back to normal
    monitor_->changeUsageError(1.00);

    // Publish topic
    monitor_->update();

    // Give time to publish
    rclcpp::WallRate(2).sleep();
    rclcpp::spin_some(monitor_->get_node_base_interface());

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("CPU Usage", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }
}

TEST_F(CPUMonitorTestSuite, usageMpstatNotFoundTest)
{
  // Set flag false
  monitor_->setMpstatExists(false);

  // Publish topic
  monitor_->update();

  // Give time to publish
  rclcpp::WallRate(2).sleep();
  rclcpp::spin_some(monitor_->get_node_base_interface());

  // Verify
  DiagStatus status;
  std::string value;
  ASSERT_TRUE(monitor_->findDiagStatus("CPU Usage", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "mpstat error");
  ASSERT_TRUE(findValue(status, "mpstat", value));
  ASSERT_STREQ(
    value.c_str(),
    "Command 'mpstat' not found, but can be installed with: sudo apt install sysstat");
}

TEST_F(CPUMonitorTestSuite, load1WarnTest)
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
    ASSERT_TRUE(monitor_->findDiagStatus("CPU Load Average", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }

  // Verify warning
  {
    // Change warning level
    monitor_->changeLoad1Warn(0.0);

    // Publish topic
    monitor_->update();

    // Give time to publish
    rclcpp::WallRate(2).sleep();
    rclcpp::spin_some(monitor_->get_node_base_interface());

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("CPU Load Average", status));
    ASSERT_EQ(status.level, DiagStatus::WARN);
  }

  // Verify normal behavior
  {
    // Change back to normal
    monitor_->changeLoad1Warn(0.90);

    // Publish topic
    monitor_->update();

    // Give time to publish
    rclcpp::WallRate(2).sleep();
    rclcpp::spin_some(monitor_->get_node_base_interface());

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("CPU Load Average", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }
}

TEST_F(CPUMonitorTestSuite, load5WarnTest)
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
    ASSERT_TRUE(monitor_->findDiagStatus("CPU Load Average", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }

  // Verify warning
  {
    // Change warning level
    monitor_->changeLoad5Warn(0.0);

    // Publish topic
    monitor_->update();

    // Give time to publish
    rclcpp::WallRate(2).sleep();
    rclcpp::spin_some(monitor_->get_node_base_interface());

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("CPU Load Average", status));
    ASSERT_EQ(status.level, DiagStatus::WARN);
  }

  // Verify normal behavior
  {
    // Change back to normal
    monitor_->changeLoad5Warn(0.80);

    // Publish topic
    monitor_->update();

    // Give time to publish
    rclcpp::WallRate(2).sleep();
    rclcpp::spin_some(monitor_->get_node_base_interface());

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("CPU Load Average", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }
}

TEST_F(CPUMonitorTestSuite, DISABLED_throttlingTest)
{
  // Publish topic
  monitor_->update();

  // Give time to publish
  rclcpp::WallRate(2).sleep();
  rclcpp::spin_some(monitor_->get_node_base_interface());

  // Verify
  DiagStatus status;
  ASSERT_TRUE(monitor_->findDiagStatus("CPU Thermal Throttling", status));
  ASSERT_EQ(status.level, DiagStatus::OK);
}

TEST_F(CPUMonitorTestSuite, freqTest)
{
  // Publish topic
  monitor_->update();

  // Give time to publish
  rclcpp::WallRate(2).sleep();
  rclcpp::spin_some(monitor_->get_node_base_interface());

  // Verify
  DiagStatus status;
  ASSERT_TRUE(monitor_->findDiagStatus("CPU Frequency", status));
  ASSERT_EQ(status.level, DiagStatus::OK);
}

TEST_F(CPUMonitorTestSuite, freqFrequencyFilesNotFoundTest)
{
  // Clear list
  monitor_->clearFreqNames();

  // Publish topic
  monitor_->update();

  // Give time to publish
  rclcpp::WallRate(2).sleep();
  rclcpp::spin_some(monitor_->get_node_base_interface());

  // Verify
  DiagStatus status;
  ASSERT_TRUE(monitor_->findDiagStatus("CPU Frequency", status));

  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "frequency files not found");
}

TEST_F(CPUMonitorTestSuite, usageMpstatErrorTest)
{
  // Symlink mpstat1 to mpstat
  fs::create_symlink(exe_dir_ + "/mpstat1", mpstat_);

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

  ASSERT_TRUE(monitor_->findDiagStatus("CPU Usage", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "mpstat error");
  ASSERT_TRUE(findValue(status, "mpstat", value));
}

TEST_F(CPUMonitorTestSuite, usageMpstatExceptionTest)
{
  // Symlink mpstat2 to mpstat
  fs::create_symlink(exe_dir_ + "/mpstat2", mpstat_);

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

  ASSERT_TRUE(monitor_->findDiagStatus("CPU Usage", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "mpstat exception");
}

// for coverage
class DummyCPUMonitor : public CPUMonitorBase
{
  friend class CPUMonitorTestSuite;

public:
  DummyCPUMonitor(const std::string & node_name, const rclcpp::NodeOptions & options)
  : CPUMonitorBase(node_name, options)
  {
  }
  void update() { updater_.force_update(); }
};

TEST_F(CPUMonitorTestSuite, dummyCPUMonitorTest)
{
  rclcpp::NodeOptions options;
  std::unique_ptr<DummyCPUMonitor> monitor =
    std::make_unique<DummyCPUMonitor>("dummy_cpu_monitor", options);
  monitor->getTempNames();
  monitor->getFreqNames();
  // Publish topic
  monitor->update();
}

int main(int argc, char ** argv)
{
  argv_ = argv;
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
