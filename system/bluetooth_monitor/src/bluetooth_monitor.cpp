// Copyright 2022 The Autoware Contributors
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

#include "bluetooth_monitor/bluetooth_monitor.hpp"

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <algorithm>
#include <string>
#include <vector>

#define FMT_HEADER_ONLY
#include <fmt/format.h>

BluetoothMonitor::BluetoothMonitor(const rclcpp::NodeOptions & options)
: Node("bluetooth_monitor", options),
  updater_(this),
  socket_(-1),
  port_(declare_parameter<int64_t>("port"))
{
  // Get host name
  char host_name[HOST_NAME_MAX + 1];
  gethostname(host_name, sizeof(host_name));

  // Build L2ping configuration
  config_.l2ping.timeout = declare_parameter<int64_t>("timeout");
  config_.l2ping.rtt_warn = declare_parameter<double>("rtt_warn");
  config_.addresses = declare_parameter<std::vector<std::string>>("addresses");

  updater_.add("bluetooth_connection", this, &BluetoothMonitor::checkConnection);
  updater_.setHardwareID(host_name);
}

bool BluetoothMonitor::connectService(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Create a new socket
  socket_ = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_ < 0) {
    stat.summary(DiagStatus::ERROR, FUNCTION_ERROR_STR);
    stat.add("socket", strerror(errno));
    return false;
  }

  // Specify the receiving timeouts until reporting an error
  timeval tv{10, 0};
  int ret = setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  if (ret < 0) {
    stat.summary(DiagStatus::ERROR, FUNCTION_ERROR_STR);
    stat.add("setsockopt", strerror(errno));
    return false;
  }

  // Connect the socket referred to by the file descriptor
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(port_);
  address.sin_addr.s_addr = htonl(INADDR_ANY);

  ret = connect(socket_, reinterpret_cast<sockaddr *>(&address), sizeof(address));
  if (ret < 0) {
    stat.summary(DiagStatus::ERROR, FUNCTION_ERROR_STR);
    stat.add("connect", strerror(errno));
    return false;
  }

  return true;
}

bool BluetoothMonitor::sendConfig(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::ostringstream out_stream;
  boost::archive::text_oarchive archive(out_stream);
  archive & config_;

  // Write list of devices to FD
  int ret = write(socket_, out_stream.str().c_str(), out_stream.str().length());
  if (ret < 0) {
    stat.summary(DiagStatus::ERROR, FUNCTION_ERROR_STR);
    stat.add("write", strerror(errno));
    return false;
  }

  return true;
}

bool BluetoothMonitor::receiveData(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  char buffer[1024]{};

  int ret = recv(socket_, buffer, sizeof(buffer) - 1, 0);
  if (ret < 0) {
    stat.summary(DiagStatus::ERROR, FUNCTION_ERROR_STR);
    stat.add("recv", strerror(errno));
    return false;
  }
  // No data received
  if (ret == 0) {
    stat.summary(DiagStatus::ERROR, FUNCTION_ERROR_STR);
    stat.add("recv", "No data received");
    return false;
  }

  // Restore device status list
  try {
    std::istringstream in_stream(buffer);
    boost::archive::text_iarchive archive(in_stream);
    archive >> status_list_;
  } catch (const std::exception & e) {
    stat.summary(DiagStatus::ERROR, "Exception occurred");
    stat.add("Exception", e.what());
    return false;
  }

  return true;
}

void BluetoothMonitor::closeConnection()
{
  close(socket_);
}

void BluetoothMonitor::setErrorLevel(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // No device
  if (status_list_.size() == 0) {
    stat.summary(DiagStatus::OK, "No device connected");
    return;
  }

  int whole_level = DiagStatus::OK;
  StatusCode whole_status_code = StatusCode::OK;
  int index = 0;
  for (const auto & status : status_list_) {
    stat.add(fmt::format("Device {}: Status", index), status_string_list_.at(status.status_code));
    stat.add(fmt::format("Device {}: Name", index), status.name);
    stat.add(fmt::format("Device {}: Manufacturer", index), status.manufacturer);
    stat.add(fmt::format("Device {}: Address", index), status.address);
    stat.addf(fmt::format("Device {}: RTT", index), "%.2f ms", status.time_difference);

    if (status.status_code == StatusCode::FUNCTION_ERROR) {
      stat.add(fmt::format("Device {}: {}", index, status.function_name), status.error_message);
    }

    int level = status_error_list_.at(status.status_code);
    whole_level = std::max(whole_level, level);
    whole_status_code = std::max(whole_status_code, status.status_code);
    ++index;
  }

  stat.summary(whole_level, status_string_list_.at(whole_status_code));
}

void BluetoothMonitor::checkConnection(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (config_.addresses.empty()) {
    RCLCPP_ERROR(get_logger(), "Invalid device parameter.");
    stat.summary(DiagStatus::ERROR, "Invalid device parameter");
    return;
  }

  // Connect to L2ping service
  if (!connectService(stat)) {
    closeConnection();
    return;
  }

  // Send L2ping configuration to L2ping service
  if (!sendConfig(stat)) {
    closeConnection();
    return;
  }

  // Receive data from L2ping service
  if (!receiveData(stat)) {
    closeConnection();
    return;
  }

  // Close connection with L2ping service
  closeConnection();

  // Set error level of diagnostic status
  setErrorLevel(stat);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(BluetoothMonitor)
