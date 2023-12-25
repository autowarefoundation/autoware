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

#ifndef SYSTEM_MONITOR__TRAFFIC_READER__TRAFFIC_READER_SERVICE_HPP_
#define SYSTEM_MONITOR__TRAFFIC_READER__TRAFFIC_READER_SERVICE_HPP_

#include "system_monitor/traffic_reader/traffic_reader_common.hpp"

#include <boost/archive/text_iarchive.hpp>
#include <boost/asio.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace traffic_reader_service
{

namespace local = boost::asio::local;

class TrafficReaderService
{
public:
  /**
   * @brief Constructor
   * @param[in] socket_path Path of UNIX domain socket
   */
  explicit TrafficReaderService(std::string socket_path);

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
   * @brief Handle message
   * @param[in] buffer Pointer to data received
   */
  void handle_message(const char * buffer);

  /**
   * @brief Start nethogs
   * @param[in] archive Archive object for loading
   */
  void start_nethogs(boost::archive::text_iarchive & archive);

  /**
   * @brief Get command line of process from nethogs output
   * @param[in] line nethogs output
   * @return Command line of process
   */
  static std::string get_command_line(const std::string & line);

  /**
   * @brief Get command line of process from PID
   * @param[in] pid PID
   * @return Command line of process
   */
  static std::string get_command_line_with_pid(pid_t pid);

  /**
   * @brief Return result of nethogs
   */
  void get_result();

  /**
   * @brief Execute nethogs
   */
  void execute_nethogs();

  std::string socket_path_;             //!< @brief Path of UNIX domain socket
  boost::asio::io_service io_service_;  //!< @brief Core I/O functionality
  std::unique_ptr<local::stream_protocol::acceptor> acceptor_;  //!< @brief UNIX domain acceptor
  std::unique_ptr<local::stream_protocol::socket> socket_;      //!< @brief UNIX domain socket
  std::thread thread_;                                          //!< @brief Thread to run nethogs
  std::mutex mutex_;                                            //!< @brief Mutex guard for the flag
  bool stop_;                                                   //!< @brief Flag to stop thread
  std::vector<std::string> devices_;                            //!< @brief List of devices
  std::string program_name_;                                    //!< @brief Filter by program name
  traffic_reader_service::Result result_;                       //!< @brief Result of nethogs
};

}  // namespace traffic_reader_service

#endif  // SYSTEM_MONITOR__TRAFFIC_READER__TRAFFIC_READER_SERVICE_HPP_
