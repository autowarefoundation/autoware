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
//

#include "bluetooth_monitor/service/l2ping.hpp"

#include <boost/process.hpp>

#include <syslog.h>

#include <regex>
#include <string>

#define FMT_HEADER_ONLY
#include <fmt/format.h>

namespace bp = boost::process;

L2ping::L2ping(const std::string & address, const L2pingConfig & config)
: config_(config), status_{}
{
  status_.address = address;
}

bool L2ping::getDeviceInformation()
{
  std::ostringstream os;
  bp::ipstream is_out;
  bp::ipstream is_err;
  bp::child c(
    fmt::format("hcitool info {}", status_.address), bp::std_out > is_out, bp::std_err > is_err);
  c.wait();
  if (c.exit_code() != 0) {
    is_err >> os.rdbuf();
    syslog(LOG_ERR, "hcitool info: %s\n", os.str().c_str());
    return false;
  }

  std::string line;
  std::cmatch match;
  const std::regex filter_device_name("\tDevice Name: (.*)");
  const std::regex filter_manufacturer("\tManufacturer: (.*)");

  while (std::getline(is_out, line) && !line.empty()) {
    if (std::regex_match(line.c_str(), match, filter_device_name)) {
      status_.name = match[1].str();
    } else if (std::regex_match(line.c_str(), match, filter_manufacturer)) {
      status_.manufacturer = match[1].str();
    }
  }

  return true;
}

void L2ping::run()
{
  // Start thread loop
  stop_ = false;
  thread_ = std::thread(&L2ping::thread, this);
}

void L2ping::stop()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
  }

  thread_.join();
}

void L2ping::thread()
{
  while (true) {
    bool stop = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop = stop_;
    }
    if (stop) {
      break;
    }

    // Get device information if not provided
    if (status_.name.empty() || status_.manufacturer.empty()) {
      getDeviceInformation();
    }
    // Ping to remote device in loop
    ping();
  }
}

bool L2ping::ping()
{
  std::ostringstream os;
  bp::ipstream is_out;
  bp::ipstream is_err;

  bp::child c(
    fmt::format("l2ping -c 1 -t {} {}", config_.timeout, status_.address), bp::std_out > is_out,
    bp::std_err > is_err);

  /*
   Output example of `l2ping`

   Ping: AA:BB:CC:DD:EE:FF from 01:23:45:67:89:01 (data size 44) ...
   44 bytes from AA:BB:CC:DD:EE:FF id 0 time 23.08ms
   1 sent, 1 received, 0% loss
  */
  std::string line;
  // Skip header
  std::getline(is_out, line);
  // Get 2nd line
  std::getline(is_out, line);

  std::cmatch match;

  // Echo response received
  const std::regex filter_time(".*time (\\d+\\.\\d+)ms");

  if (std::regex_match(line.c_str(), match, filter_time)) {
    status_.time_difference = std::stof(match[1].str());
    // Check RTT if configured
    if (config_.rtt_warn != RTT_NO_WARN && status_.time_difference > config_.rtt_warn) {
      setStatusCode(StatusCode::RTT_WARNING);
      // Otherwise, ok
    } else {
      setStatusCode(StatusCode::OK);
    }
  }

  // No response
  const std::regex filter_no_response("^no response from .*");

  if (std::regex_match(line.c_str(), match, filter_no_response)) {
    setStatusCode(StatusCode::LOST);
  }

  c.wait();
  if (c.exit_code() != 0) {
    is_err >> os.rdbuf();
    // Remove return code
    std::string message = std::regex_replace(os.str(), std::regex("\n"), "");
    // Get stdout if stderr is empty
    if (message.empty()) {
      message = std::regex_replace(line, std::regex("\n"), "");
    }
    syslog(LOG_ERR, "l2ping: %s\n", message.c_str());
    setFunctionError("l2ping", message);
    return false;
  }

  return true;
}

void L2ping::setFunctionError(const std::string & function_name, const std::string & error_message)
{
  // Build error data
  status_.status_code = StatusCode::FUNCTION_ERROR;
  status_.function_name = function_name;
  status_.error_message = error_message;
}

void L2ping::setStatusCode(StatusCode code)
{
  // Build error data
  status_.status_code = code;
  status_.function_name = {};
  status_.error_message = {};
}

L2pingStatus L2ping::getStatus() const
{
  return status_;
}

const std::string & L2ping::getAddress() const
{
  return status_.address;
}
