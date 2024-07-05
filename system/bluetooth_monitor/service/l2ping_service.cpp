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

#include "bluetooth_monitor/service/l2ping_service.hpp"

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/process.hpp>

#include <sys/socket.h>
#include <syslog.h>

#include <memory>
#include <regex>
#include <sstream>
#include <string>

#define FMT_HEADER_ONLY
#include <fmt/format.h>

namespace bp = boost::process;

L2pingService::L2pingService(const int port) : port_(port), socket_(-1)
{
}

bool L2pingService::initialize()
{
  // Create a new socket
  socket_ = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_ < 0) {
    syslog(LOG_ERR, "Failed to create a new socket. %s\n", strerror(errno));
    return false;
  }

  // Allow address reuse
  int ret = 0;
  int opt = 1;
  ret = setsockopt(
    socket_, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<char *>(&opt), (socklen_t)sizeof(opt));
  if (ret < 0) {
    syslog(LOG_ERR, "Failed to set socket FD's option. %s\n", strerror(errno));
    return false;
  }

  // Give the socket FD the local address ADDR
  sockaddr_in address;
  memset(&address, 0, sizeof(sockaddr_in));
  address.sin_family = AF_INET;
  address.sin_port = htons(port_);
  address.sin_addr.s_addr = htonl(INADDR_ANY);
  // cppcheck-suppress cstyleCast
  ret = bind(socket_, (struct sockaddr *)&address, sizeof(address));
  if (ret < 0) {
    syslog(LOG_ERR, "Failed to give the socket FD the local address ADDR. %s\n", strerror(errno));
    return false;
  }

  // Prepare to accept connections on socket FD
  ret = listen(socket_, 5);
  if (ret < 0) {
    syslog(LOG_ERR, "Failed to prepare to accept connections on socket FD. %s\n", strerror(errno));
    return false;
  }

  return true;
}

void L2pingService::shutdown()
{
  close(socket_);
}

void L2pingService::run()
{
  sockaddr_in client;
  socklen_t len = sizeof(client);

  while (true) {
    // Await a connection on socket FD
    int new_sock = accept(socket_, reinterpret_cast<sockaddr *>(&client), &len);
    if (new_sock < 0) {
      syslog(
        LOG_ERR, "Failed to prepare to accept connections on socket FD. %s\n", strerror(errno));
      continue;
    }

    // Receive configuration of L2ping
    char buf[1024]{};
    int ret = recv(new_sock, buf, sizeof(buf) - 1, 0);
    if (ret < 0) {
      syslog(LOG_ERR, "Failed to receive. %s\n", strerror(errno));
      close(new_sock);
      continue;
    }
    // No data received
    if (ret == 0) {
      syslog(LOG_ERR, "No data received. %s\n", strerror(errno));
      close(new_sock);
      continue;
    }

    // Restore configuration of L2ping
    L2pingServiceConfig config;
    try {
      std::istringstream iss(buf);
      boost::archive::text_iarchive oa(iss);
      oa & config;
    } catch (const std::exception & e) {
      syslog(LOG_ERR, "Exception occurred. ! %s\n", e.what());
      close(new_sock);
      continue;
    }

    // If configuration is changed
    if (config_ != config) {
      // Stop all ping threads
      stop();
      config_ = config;
    }

    // Now communication with ros2 node successful

    // Build device list to ping
    if (buildDeviceList()) {
      // Clear list
      status_list_.clear();
      // Build status list
      for (const auto & object : objects_) {
        status_list_.emplace_back(object->getStatus());
      }
    }

    // Inform ros2 node if error occurred
    std::ostringstream out_stream;
    boost::archive::text_oarchive archive(out_stream);
    archive << status_list_;
    //  Write N bytes of BUF to FD
    ret = write(new_sock, out_stream.str().c_str(), out_stream.str().length());
    if (ret < 0) {
      syslog(LOG_ERR, "Failed to write N bytes of BUF to FD. %s\n", strerror(errno));
    }

    close(new_sock);
  }
}

void L2pingService::setFunctionError(
  const std::string & function_name, const std::string & error_message)
{
  // Clear list
  status_list_.clear();

  // Set error data
  L2pingStatus status{};
  status.status_code = StatusCode::FUNCTION_ERROR;
  status.function_name = function_name;
  status.error_message = error_message;

  status_list_.emplace_back(status);
}

void L2pingService::stop()
{
  for (auto & object : objects_) {
    object->stop();
    object.reset();
  }
  objects_.clear();
}

bool L2pingService::buildDeviceList()
{
  bp::pipe pipe;
  std::ostringstream os;

  // Get MAC address list of paired devices
  /*
   Output example of `bluetoothctl paired-devices`

   Device 01:02:03:04:05:06 Wireless Controller
   Device AA:BB:CC:DD:EE:FF bluetooth mouse4.0
  */
  {
    bp::ipstream is_err;
    bp::child c("bluetoothctl paired-devices", bp::std_out > pipe, bp::std_err > is_err);
    c.wait();
    if (c.exit_code() != 0) {
      is_err >> os.rdbuf();
      setFunctionError("bluetoothctl", os.str());
      syslog(LOG_ERR, "%s\n", os.str().c_str());
      return false;
    }
  }

  // Pick up MAC address
  std::vector<std::string> address_list;
  {
    bp::ipstream is_out;
    bp::ipstream is_err;
    bp::child c("cut -f 2 -d \" \"", bp::std_out > is_out, bp::std_err > is_err, bp::std_in < pipe);
    c.wait();
    if (c.exit_code() != 0) {
      is_err >> os.rdbuf();
      setFunctionError("cut", os.str());
      syslog(LOG_ERR, "%s\n", os.str().c_str());
      return false;
    }

    // Register device
    std::string line;
    while (std::getline(is_out, line) && !line.empty()) {
      // Skip if device not found and wild card not specified
      if (
        std::count(config_.addresses.begin(), config_.addresses.end(), line) == 0 &&
        std::count(config_.addresses.begin(), config_.addresses.end(), "*") == 0) {
        continue;
      }
      address_list.push_back(line);
    }
  }

  // Loop for registered devices
  for (const auto & address : address_list) {
    bool found = false;
    for (const auto & object : objects_) {
      // If device not registered
      if (object->getAddress() == address) {
        found = true;
        break;
      }
    }

    if (!found) {
      // Start ping thread
      objects_.emplace_back(std::make_unique<L2ping>(address, config_.l2ping));
      objects_.back().get()->run();
    }
  }

  return true;
}
