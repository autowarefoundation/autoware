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

#ifndef VELODYNE_MONITOR__VELODYNE_MONITOR_HPP_
#define VELODYNE_MONITOR__VELODYNE_MONITOR_HPP_

/**
 * @file velodyne_monitor.hpp
 * @brief Velodyne monitor class
 */

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

// Include after diagnostic_updater because it causes errors
#include <cpprest/http_client.h>

namespace http = web::http;
namespace client = web::http::client;
namespace json = web::json;

class VelodyneMonitor : public rclcpp::Node
{
public:
  /**
   * @brief constructor
   */
  explicit VelodyneMonitor(const rclcpp::NodeOptions & options);

protected:
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

  /**
   * @brief obtain JSON-formatted diagnostic status and check connection
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkConnection(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief check the temperature of the top and bottom board
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkTemperature(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief check the motor rpm
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkMotorRpm(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief send an HTTP-GET request
   * @param [in] path_query string containing the path, query
   * @param [out] res an HTTP response
   * @param [out] err_msg diagnostic message passed directly to diagnostic publish calls
   * @return true on success, false on error
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  bool requestGET(const std::string & path_query, http::http_response & res, std::string & err_msg);

  /**
   * @brief convert raw diagnostic data to usable temperature value
   * @param [in] raw raw diagnostic data
   * @return usable temperature value
   */
  float convertTemperature(int raw);

  diagnostic_updater::Updater updater_;  //!< @brief updater class which advertises to /diagnostics
  std::unique_ptr<client::http_client> client_;  //!< @brief HTTP client class
  json::value info_json_;                        //!< @brief values of info.json
  json::value diag_json_;                        //!< @brief values of diag.json
  json::value status_json_;                      //!< @brief values of status.json
  json::value settings_json_;                    //!< @brief values of settings.json
  bool diag_json_received_;                      //!< @brief flag of diag.json received

  std::string ip_address_;  //!< @brief Network IP address of sensor
  double timeout_;          //!< @brief timeout parameter
  float temp_cold_warn_;    //!< @brief the cold temperature threshold to generate a warning
  float temp_cold_error_;   //!< @brief the cold temperature threshold to generate an error
  float temp_hot_warn_;     //!< @brief the hot temperature threshold to generate a warning
  float temp_hot_error_;    //!< @brief the hot temperature threshold to generate an error
  float rpm_ratio_warn_;    //!< @brief the rpm threshold(%) to generate a warning
  float rpm_ratio_error_;   //!< @brief the rpm threshold(%) to generate an error

  /**const ros::TimerEvent & event
   * @brief RPM status messages
   */
  const std::map<int, const char *> rpm_dict_ = {
    {DiagStatus::OK, "OK"}, {DiagStatus::WARN, "RPM low"}, {DiagStatus::ERROR, "RPM too low"}};
};

#endif  // VELODYNE_MONITOR__VELODYNE_MONITOR_HPP_
