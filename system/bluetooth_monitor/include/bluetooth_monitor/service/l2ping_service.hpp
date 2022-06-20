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

#ifndef BLUETOOTH_MONITOR__SERVICE__L2PING_SERVICE_HPP_
#define BLUETOOTH_MONITOR__SERVICE__L2PING_SERVICE_HPP_

#include "bluetooth_monitor/service/l2ping.hpp"
#include "bluetooth_monitor/service/l2ping_interface.hpp"

#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

class L2pingService
{
public:
  /**
   * @brief Constructor
   * @param [in] port Port number to access l2ping service
   */
  explicit L2pingService(const int port);

  /**
   * @brief Initialization
   * @return true on success, false on error
   */
  bool initialize();

  /**
   * @brief Shutdown
   */
  void shutdown();

  /**
   * @brief Main loop
   */
  void run();

protected:
  /**
   * @brief Set error data to inform ros2 node
   * @param [in] function_name Function name which error occurred
   * @param [in] error_message Error message to display
   */
  void setFunctionError(const std::string & function_name, const std::string & error_message);

  /**
   * @brief Stop all ping threads
   */
  void stop();

  /**
   * @brief Build device list to ping
   * @param [in] addresses List of bluetooth address
   * @return true on success, false on error
   */
  bool buildDeviceList();

  /**
   * @brief Build device list to ping from connected devices
   * @param [in] sock socket to bluetooth host controller interface(HCI)
   * @param [in] device_id Device ID
   * @param [in] addresses List of bluetooth address
   * @return true on success, false on error
   */
  bool buildDeviceListFromConnectedDevices(int sock, uint16_t device_id);

  int port_;                                      //!< @brief Port number to access l2ping service
  int socket_;                                    //!< @brief Socket to communicate with ros2 node
  L2pingServiceConfig config_;                    //!< @brief Configuration of L2ping service
  std::vector<std::unique_ptr<L2ping>> objects_;  //!< @brief List of l2ping object
  L2pingStatusList status_list_;                  //!< @brief List of l2ping status
};

#endif  // BLUETOOTH_MONITOR__SERVICE__L2PING_SERVICE_HPP_
