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

#ifndef BLUETOOTH_MONITOR__SERVICE__L2PING_HPP_
#define BLUETOOTH_MONITOR__SERVICE__L2PING_HPP_

#include "bluetooth_monitor/service/l2ping_interface.hpp"

#include <mutex>
#include <string>
#include <thread>
#include <vector>

class L2ping
{
public:
  /**
   * @brief Constructor
   * @param [in]  address Bluetooth address of remote device
   * @param [in] config Configuration of L2ping
   */
  L2ping(const std::string & address, const L2pingConfig & config);

  /**
   * @brief Start ping thread
   */
  void run();

  /**
   * @brief Stop ping thread
   */
  void stop();

  /**
   * @brief Get status
   * @return Status
   */
  L2pingStatus getStatus() const;

  /**
   * @brief Get address of remote device
   * @return address of remote device
   */
  const std::string & getAddress() const;

protected:
  /**
   * @brief Get information from remote device
   * @return true on success, false on error
   */
  bool getDeviceInformation();

  /**
   * @brief Thread loop
   */
  void thread();

  /**
   * @brief Ping to remote device
   * @return true on success, false on error
   */
  bool ping();

  /**
   * @brief Set error data to inform ros2 node
   * @param [in] function_name Function name which error occurred
   * @param [in] error_message Error message to display
   */
  void setFunctionError(const std::string & function_name, const std::string & error_message);

  /**
   * @brief Set status code
   * @param [in] code Status code
   */
  void setStatusCode(StatusCode code);

  L2pingConfig config_;  //!< @brief Configuration of L2ping
  std::thread thread_;   //!< @brief Thread to L2ping
  L2pingStatus status_;  //!< @brief L2ping status
  std::mutex mutex_;     //!< @brief mutex for stop flag
  bool stop_;            //!< @brief Flag to stop thread
};

#endif  // BLUETOOTH_MONITOR__SERVICE__L2PING_HPP_
