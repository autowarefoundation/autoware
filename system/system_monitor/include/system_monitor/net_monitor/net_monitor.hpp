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

/**
 * @file net_monitor.h
 * @brief Net monitor class
 */

#ifndef SYSTEM_MONITOR__NET_MONITOR__NET_MONITOR_HPP_
#define SYSTEM_MONITOR__NET_MONITOR__NET_MONITOR_HPP_

#include "system_monitor/net_monitor/nl80211.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <climits>
#include <deque>
#include <map>
#include <string>
#include <vector>

#define toMbit(X) (static_cast<float>(X) / 1000000 * 8)

/**
 * @brief Bytes information
 */
typedef struct bytes
{
  unsigned int rx_bytes;  //!< @brief total bytes received
  unsigned int tx_bytes;  //!< @brief total bytes transmitted

  bytes() : rx_bytes(0), tx_bytes(0) {}
} bytes;

class NetMonitor : public rclcpp::Node
{
public:
  /**
   * @brief constructor
   * @param [in] options Options associated with this node.
   */
  explicit NetMonitor(const rclcpp::NodeOptions & options);
  /**
   * @brief destructor
   */
  ~NetMonitor();

  /**
   * @brief Shutdown nl80211 object
   */
  void shutdown_nl80211();

protected:
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

  /**
   * @brief check CPU usage
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkUsage(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief monitor traffic
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void monitorTraffic(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief check CRC error
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkCrcError(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief check IP packet reassembles failed
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @note NOLINT syntax is needed since diagnostic_updater asks for a non-const reference
   * to pass diagnostic message updated in this function to diagnostic publish calls.
   */
  void checkReassemblesFailed(
    diagnostic_updater::DiagnosticStatusWrapper & stat);  // NOLINT(runtime/references)

  /**
   * @brief get wireless speed
   * @param [in] ifa_name interface name
   * @return wireless speed
   */
  float getWirelessSpeed(const char * ifa_name);

  /**
   * @brief timer callback
   */
  void onTimer();

  /**
   * @brief update Network information list
   */
  void updateNetworkInfoList();

  /**
   * @brief check NetMonitor General Infomation
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @return check result
   */
  bool checkGeneralInfo(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /**
   * @brief Network information
   */
  struct NetworkInfo
  {
    int mtu_errno;               //!< @brief errno set by ioctl() with SIOCGIFMTU
    int ethtool_errno;           //!< @brief errno set by ioctl() with SIOCETHTOOL
    bool is_running;             //!< @brief resource allocated flag
    std::string interface_name;  //!< @brief interface name
    float speed;                 //!< @brief network capacity
    int mtu;                     //!< @brief MTU
    float rx_traffic;            //!< @brief traffic received
    float tx_traffic;            //!< @brief traffic transmitted
    float rx_usage;              //!< @brief network capacity usage rate received
    float tx_usage;              //!< @brief network capacity usage rate transmitted
    unsigned int rx_bytes;       //!< @brief total bytes received
    unsigned int rx_errors;      //!< @brief bad packets received
    unsigned int tx_bytes;       //!< @brief total bytes transmitted
    unsigned int tx_errors;      //!< @brief packet transmit problems
    unsigned int collisions;     //!< @brief number of collisions during packet transmissions

    NetworkInfo()
    : mtu_errno(0),
      ethtool_errno(0),
      is_running(false),
      interface_name(""),
      speed(0.0),
      mtu(0),
      rx_traffic(0.0),
      tx_traffic(0.0),
      rx_usage(0.0),
      tx_usage(0.0),
      rx_bytes(0),
      rx_errors(0),
      tx_bytes(0),
      tx_errors(0),
      collisions(0)
    {
    }
  };

  /**
   * @brief determine if it is a supported network
   * @param [in] net_info network infomation
   * @param [in] index index of network infomation index
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   * @param [out] error_str error string
   * @return result of determining whether it is a supported network
   */
  bool isSupportedNetwork(
    const NetworkInfo & net_info, int index, diagnostic_updater::DiagnosticStatusWrapper & stat,
    std::string & error_str);

  /**
   * @brief search column index of IP packet reassembles failed in /proc/net/snmp
   */
  void searchReassemblesFailedColumnIndex();

  /**
   * @brief get IP packet reassembles failed
   * @param [out] reassembles_failed IP packet reassembles failed
   * @return execution result
   */
  bool getReassemblesFailed(uint64_t & reassembles_failed);

  diagnostic_updater::Updater updater_;  //!< @brief Updater class which advertises to /diagnostics
  rclcpp::TimerBase::SharedPtr timer_;   //!< @brief timer to get Network information

  char hostname_[HOST_NAME_MAX + 1];        //!< @brief host name
  std::map<std::string, bytes> bytes_;      //!< @brief list of bytes
  rclcpp::Time last_update_time_;           //!< @brief last update time
  std::vector<std::string> device_params_;  //!< @brief list of devices
  NL80211 nl80211_;                         //!< @brief 802.11 netlink-based interface
  int getifaddrs_errno_;                    //!< @brief errno set by getifaddrs()
  std::vector<NetworkInfo> net_info_list_;  //!< @brief list of Network information

  /**
   * @brief CRC errors information
   */
  typedef struct crc_errors
  {
    std::deque<unsigned int> errors_queue;  //!< @brief queue that holds count of CRC errors
    unsigned int last_rx_crc_errors;  //!< @brief rx_crc_error at the time of the last monitoring

    crc_errors() : last_rx_crc_errors(0) {}
  } crc_errors;
  std::map<std::string, crc_errors> crc_errors_;  //!< @brief list of CRC errors

  std::deque<unsigned int>
    reassembles_failed_queue_;  //!< @brief queue that holds count of IP packet reassembles failed
  uint64_t last_reassembles_failed_;  //!< @brief IP packet reassembles failed at the time of the
                                      //!< last monitoring

  std::string monitor_program_;             //!< @brief nethogs monitor program name
  bool nethogs_all_;                        //!< @brief nethogs result all mode
  int traffic_reader_port_;                 //!< @brief port number to connect to traffic_reader
  unsigned int crc_error_check_duration_;   //!< @brief CRC error check duration
  unsigned int crc_error_count_threshold_;  //!< @brief CRC error count threshold
  unsigned int
    reassembles_failed_check_duration_;  //!< @brief IP packet reassembles failed check duration
  unsigned int
    reassembles_failed_check_count_;  //!< @brief IP packet reassembles failed check count threshold
  unsigned int reassembles_failed_column_index_;  //!< @brief column index of IP Reassembles failed
                                                  //!< in /proc/net/snmp

  /**
   * @brief Network usage status messages
   */
  const std::map<int, const char *> usage_dict_ = {
    {DiagStatus::OK, "OK"}, {DiagStatus::WARN, "high load"}, {DiagStatus::ERROR, "down"}};
};

#endif  // SYSTEM_MONITOR__NET_MONITOR__NET_MONITOR_HPP_
