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

#include "system_monitor/hdd_monitor/hdd_monitor.hpp"
#include "system_monitor/hdd_reader/hdd_reader.hpp"

#include <rclcpp/rclcpp.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/filesystem.hpp>
#include <boost/process.hpp>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include <memory>
#include <string>

namespace fs = boost::filesystem;
using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

char ** argv_;

class TestHddMonitor : public HddMonitor
{
  friend class HddMonitorTestSuite;

public:
  TestHddMonitor(const std::string & node_name, const rclcpp::NodeOptions & options)
  : HddMonitor(node_name, options)
  {
  }

  void diagCallback(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr diag_msg)
  {
    array_ = *diag_msg;
  }

  void addHddParams(
    const std::string & name, float temp_warn, float temp_error, float usage_warn,
    float usage_error)
  {
    HddParam param;
    param.temp_warn_ = temp_warn;
    param.temp_error_ = temp_error;
    param.usage_warn_ = usage_warn;
    param.usage_error_ = usage_error;
    hdd_params_[name] = param;
  }

  void changeHddParams(float temp_warn, float temp_error, float usage_warn, float usage_error)
  {
    for (auto itr = hdd_params_.begin(); itr != hdd_params_.end(); ++itr) {
      itr->second.temp_warn_ = temp_warn;
      itr->second.temp_error_ = temp_error;
      itr->second.usage_warn_ = usage_warn;
      itr->second.usage_error_ = usage_error;
    }
  }

  void removeHddParams(const std::string & name)
  {
    for (auto itr = hdd_params_.begin(); itr != hdd_params_.end(); ++itr) {
      if (itr->first == name) {
        hdd_params_.erase(itr);
        break;
      }
    }
  }

  void clearHddParams() { hdd_params_.clear(); }

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

class HddMonitorTestSuite : public ::testing::Test
{
public:
  HddMonitorTestSuite()
  {
    // Get directory of executable
    const fs::path exe_path(argv_[0]);
    exe_dir_ = exe_path.parent_path().generic_string();
    // Get dummy executable path
    df_ = exe_dir_ + "/df";
  }

protected:
  std::unique_ptr<TestHddMonitor> monitor_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_;
  std::string exe_dir_;
  std::string df_;

  void SetUp()
  {
    using std::placeholders::_1;
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions node_options;
    monitor_ = std::make_unique<TestHddMonitor>("test_hdd_monitor", node_options);
    sub_ = monitor_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 1000, std::bind(&TestHddMonitor::diagCallback, monitor_.get(), _1));
    // Remove dummy executable if exists
    if (fs::exists(df_)) {
      fs::remove(df_);
    }
  }

  void TearDown()
  {
    // Remove dummy executable if exists
    if (fs::exists(df_)) {
      fs::remove(df_);
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

enum ThreadTestMode {
  Normal = 0,
  Hot,
  CriticalHot,
  ReturnsError,
  RecvTimeout,
  RecvNoData,
  FormatError,
};

bool stop_thread;
pthread_mutex_t mutex;

void * hdd_reader(void * args)
{
  ThreadTestMode * mode = reinterpret_cast<ThreadTestMode *>(args);

  // Create a new socket
  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    return nullptr;
  }

  // Allow address reuse
  int ret = 0;
  int opt = 1;
  ret = setsockopt(
    sock, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<char *>(&opt), (socklen_t)sizeof(opt));
  if (ret < 0) {
    close(sock);
    return nullptr;
  }

  // Give the socket FD the local address ADDR
  sockaddr_in addr;
  memset(&addr, 0, sizeof(sockaddr_in));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(7635);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  // cppcheck-suppress cstyleCast
  ret = bind(sock, (struct sockaddr *)&addr, sizeof(addr));
  if (ret < 0) {
    close(sock);
    return nullptr;
  }

  // Prepare to accept connections on socket FD
  ret = listen(sock, 5);
  if (ret < 0) {
    close(sock);
    return nullptr;
  }

  sockaddr_in client;
  socklen_t len = sizeof(client);

  // Await a connection on socket FD
  int new_sock = accept(sock, reinterpret_cast<sockaddr *>(&client), &len);
  if (new_sock < 0) {
    close(sock);
    return nullptr;
  }

  // Receive list of device from a socket
  char buf[1024] = "";
  ret = recv(new_sock, buf, sizeof(buf) - 1, 0);
  if (ret < 0) {
    close(sock);
    return nullptr;
  }
  // No data received
  if (ret == 0) {
    close(sock);
    return nullptr;
  }

  ret = 0;
  std::ostringstream oss;
  boost::archive::text_oarchive oa(oss);
  HddInfoList list;
  HddInfo info = {0};

  switch (*mode) {
    case Normal:
      info.error_code_ = 0;
      info.temp_ = 40;
      list["/dev/sda"] = info;
      oa << list;
      ret = write(new_sock, oss.str().c_str(), oss.str().length());
      break;

    case Hot:
      info.error_code_ = 0;
      info.temp_ = 55;
      list["/dev/sda"] = info;
      oa << list;
      ret = write(new_sock, oss.str().c_str(), oss.str().length());
      break;

    case CriticalHot:
      info.error_code_ = 0;
      info.temp_ = 70;
      list["/dev/sda"] = info;
      oa << list;
      ret = write(new_sock, oss.str().c_str(), oss.str().length());
      break;

    case ReturnsError:
      info.error_code_ = EACCES;
      list["/dev/sda"] = info;
      oa << list;
      ret = write(new_sock, oss.str().c_str(), oss.str().length());
      break;

    case RecvTimeout:
      // Wait for recv timeout
      while (true) {
        pthread_mutex_lock(&mutex);
        if (stop_thread) {
          break;
        }
        pthread_mutex_unlock(&mutex);
        sleep(1);
      }
      break;

    case RecvNoData:
      // Send nothing, close socket immediately
      break;

    case FormatError:
      // Send wrong data
      oa << "test";
      ret = write(new_sock, oss.str().c_str(), oss.str().length());
      break;

    default:
      break;
  }

  // Close the file descriptor FD
  close(new_sock);
  close(sock);

  return nullptr;
}

TEST_F(HddMonitorTestSuite, tempNormalTest)
{
  pthread_t th;
  ThreadTestMode mode = Normal;
  pthread_create(&th, nullptr, hdd_reader, &mode);
  // Wait for thread started
  rclcpp::WallRate(10).sleep();

  // Publish topic
  monitor_->update();

  pthread_join(th, NULL);

  // Give time to publish
  rclcpp::WallRate(2).sleep();
  rclcpp::spin_some(monitor_->get_node_base_interface());

  // Verify
  DiagStatus status;
  std::string value;
  ASSERT_TRUE(monitor_->findDiagStatus("HDD Temperature", status));
  ASSERT_EQ(status.level, DiagStatus::OK);
}

TEST_F(HddMonitorTestSuite, tempWarnTest)
{
  pthread_t th;
  ThreadTestMode mode = Hot;
  pthread_create(&th, nullptr, hdd_reader, &mode);
  // Wait for thread started
  rclcpp::WallRate(10).sleep();

  // Publish topic
  monitor_->update();

  pthread_join(th, NULL);

  // Give time to publish
  rclcpp::WallRate(2).sleep();
  rclcpp::spin_some(monitor_->get_node_base_interface());

  // Verify
  DiagStatus status;
  std::string value;
  ASSERT_TRUE(monitor_->findDiagStatus("HDD Temperature", status));
  ASSERT_EQ(status.level, DiagStatus::WARN);
}

TEST_F(HddMonitorTestSuite, tempErrorTest)
{
  pthread_t th;
  ThreadTestMode mode = CriticalHot;
  pthread_create(&th, nullptr, hdd_reader, &mode);
  // Wait for thread started
  rclcpp::WallRate(10).sleep();

  // Publish topic
  monitor_->update();

  pthread_join(th, NULL);

  // Give time to publish
  rclcpp::WallRate(2).sleep();
  rclcpp::spin_some(monitor_->get_node_base_interface());

  // Verify
  DiagStatus status;
  std::string value;
  ASSERT_TRUE(monitor_->findDiagStatus("HDD Temperature", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
}

TEST_F(HddMonitorTestSuite, tempReturnsErrorTest)
{
  pthread_t th;
  ThreadTestMode mode = ReturnsError;
  pthread_create(&th, nullptr, hdd_reader, &mode);
  // Wait for thread started
  rclcpp::WallRate(10).sleep();

  // Publish topic
  monitor_->update();

  pthread_join(th, NULL);

  // Give time to publish
  rclcpp::WallRate(2).sleep();
  rclcpp::spin_some(monitor_->get_node_base_interface());

  // Verify
  DiagStatus status;
  std::string value;
  ASSERT_TRUE(monitor_->findDiagStatus("HDD Temperature", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "hdd_reader error");
  ASSERT_TRUE(findValue(status, "HDD 0: hdd_reader", value));
  ASSERT_STREQ(value.c_str(), strerror(EACCES));
}

TEST_F(HddMonitorTestSuite, tempRecvTimeoutTest)
{
  pthread_t th;
  ThreadTestMode mode = RecvTimeout;
  pthread_create(&th, nullptr, hdd_reader, &mode);
  // Wait for thread started
  rclcpp::WallRate(10).sleep();

  // Publish topic
  monitor_->update();

  // Recv timeout occurs, thread is no longer needed
  pthread_mutex_lock(&mutex);
  stop_thread = true;
  pthread_mutex_unlock(&mutex);
  pthread_join(th, NULL);

  // Give time to publish
  rclcpp::WallRate(2).sleep();
  rclcpp::spin_some(monitor_->get_node_base_interface());

  // Verify
  DiagStatus status;
  std::string value;
  ASSERT_TRUE(monitor_->findDiagStatus("HDD Temperature", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "recv error");
  ASSERT_TRUE(findValue(status, "recv", value));
  ASSERT_STREQ(value.c_str(), strerror(EWOULDBLOCK));
}

TEST_F(HddMonitorTestSuite, tempRecvNoDataTest)
{
  pthread_t th;
  ThreadTestMode mode = RecvNoData;
  pthread_create(&th, nullptr, hdd_reader, &mode);
  // Wait for thread started
  rclcpp::WallRate(10).sleep();

  // Publish topic
  monitor_->update();

  pthread_join(th, NULL);

  // Give time to publish
  rclcpp::WallRate(2).sleep();
  rclcpp::spin_some(monitor_->get_node_base_interface());

  // Verify
  DiagStatus status;
  std::string value;
  ASSERT_TRUE(monitor_->findDiagStatus("HDD Temperature", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "recv error");
  ASSERT_TRUE(findValue(status, "recv", value));
  ASSERT_STREQ(value.c_str(), "No data received");
}

TEST_F(HddMonitorTestSuite, tempFormatErrorTest)
{
  pthread_t th;
  ThreadTestMode mode = FormatError;
  pthread_create(&th, nullptr, hdd_reader, &mode);
  // Wait for thread started
  rclcpp::WallRate(10).sleep();

  // Publish topic
  monitor_->update();

  pthread_join(th, NULL);

  // Give time to publish
  rclcpp::WallRate(2).sleep();
  rclcpp::spin_some(monitor_->get_node_base_interface());

  // Verify
  DiagStatus status;
  std::string value;
  ASSERT_TRUE(monitor_->findDiagStatus("HDD Temperature", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "recv error");
  ASSERT_TRUE(findValue(status, "recv", value));
  ASSERT_STREQ(value.c_str(), "input stream error");
}

TEST_F(HddMonitorTestSuite, tempConnectErrorTest)
{
  // Publish topic
  monitor_->update();

  // Give time to publish
  rclcpp::WallRate(2).sleep();
  rclcpp::spin_some(monitor_->get_node_base_interface());

  // Verify
  DiagStatus status;
  std::string value;
  ASSERT_TRUE(monitor_->findDiagStatus("HDD Temperature", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "connect error");
  ASSERT_TRUE(findValue(status, "connect", value));
  ASSERT_STREQ(value.c_str(), strerror(ECONNREFUSED));
}

TEST_F(HddMonitorTestSuite, tempInvalidDiskParameterTest)
{
  // Clear list
  monitor_->clearHddParams();

  // Publish topic
  monitor_->update();

  // Give time to publish
  rclcpp::WallRate(2).sleep();
  rclcpp::spin_some(monitor_->get_node_base_interface());

  // Verify
  DiagStatus status;
  ASSERT_TRUE(monitor_->findDiagStatus("HDD Temperature", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "invalid disk parameter");
}

TEST_F(HddMonitorTestSuite, tempNoSuchDeviceTest)
{
  // Add test file to list
  monitor_->addHddParams("/dev/sdx", 55.0, 77.0, 0.95, 0.99);

  pthread_t th;
  ThreadTestMode mode = Normal;
  pthread_create(&th, nullptr, hdd_reader, &mode);
  // Wait for thread started
  rclcpp::WallRate(10).sleep();

  // Publish topic
  monitor_->update();

  pthread_join(th, NULL);

  // Give time to publish
  rclcpp::WallRate(2).sleep();
  rclcpp::spin_some(monitor_->get_node_base_interface());

  // Verify
  DiagStatus status;
  std::string value;
  ASSERT_TRUE(monitor_->findDiagStatus("HDD Temperature", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "hdd_reader error");
  ASSERT_TRUE(findValue(status, "HDD 1: hdd_reader", value));
  ASSERT_STREQ(value.c_str(), strerror(ENOENT));

  // Remove test fie from list
  monitor_->removeHddParams("/dev/sdx");
}

TEST_F(HddMonitorTestSuite, usageWarnTest)
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
    ASSERT_TRUE(monitor_->findDiagStatus("HDD Usage", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }

  // Verify warning
  {
    // Change warning level
    monitor_->changeHddParams(55.0, 77.0, 0.00, 0.99);

    // Publish topic
    monitor_->update();

    // Give time to publish
    rclcpp::WallRate(2).sleep();
    rclcpp::spin_some(monitor_->get_node_base_interface());

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("HDD Usage", status));
    ASSERT_EQ(status.level, DiagStatus::WARN);
  }

  // Verify normal behavior
  {
    // Change back to normal
    monitor_->changeHddParams(55.0, 77.0, 0.95, 0.99);

    // Publish topic
    monitor_->update();

    // Give time to publish
    rclcpp::WallRate(2).sleep();
    rclcpp::spin_some(monitor_->get_node_base_interface());

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("HDD Usage", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }
}

TEST_F(HddMonitorTestSuite, usageErrorTest)
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
    ASSERT_TRUE(monitor_->findDiagStatus("HDD Usage", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }

  // Verify warning
  {
    // Change error level
    monitor_->changeHddParams(55.0, 77.0, 0.95, 0.00);

    // Publish topic
    monitor_->update();

    // Give time to publish
    rclcpp::WallRate(2).sleep();
    rclcpp::spin_some(monitor_->get_node_base_interface());

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("HDD Usage", status));
    ASSERT_EQ(status.level, DiagStatus::ERROR);
  }

  // Verify normal behavior
  {
    // Change back to normal
    monitor_->changeHddParams(55.0, 77.0, 0.95, 0.99);

    // Publish topic
    monitor_->update();

    // Give time to publish
    rclcpp::WallRate(2).sleep();
    rclcpp::spin_some(monitor_->get_node_base_interface());

    // Verify
    DiagStatus status;
    ASSERT_TRUE(monitor_->findDiagStatus("HDD Usage", status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }
}

TEST_F(HddMonitorTestSuite, usageDfErrorTest)
{
  // Symlink df1 to df
  fs::create_symlink(exe_dir_ + "/df1", df_);

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

  ASSERT_TRUE(monitor_->findDiagStatus("HDD Usage", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "df error");
  ASSERT_TRUE(findValue(status, "HDD 0: df", value));
}

int main(int argc, char ** argv)
{
  argv_ = argv;
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
