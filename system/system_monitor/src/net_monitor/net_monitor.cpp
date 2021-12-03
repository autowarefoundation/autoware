// Copyright 2020 Autoware Foundation
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
 * @file net_monitor.cpp
 * @brief Net monitor class
 */

#include "system_monitor/net_monitor/net_monitor.hpp"

#include "system_monitor/system_monitor_utility.hpp"

#include <boost/range/algorithm.hpp>

#include <fmt/format.h>
#include <ifaddrs.h>
#include <linux/ethtool.h>
#include <linux/if_link.h>
#include <linux/sockios.h>
#include <net/if.h>
#include <sys/ioctl.h>

#include <algorithm>
#include <string>
#include <vector>

NetMonitor::NetMonitor(const rclcpp::NodeOptions & options)
: Node("net_monitor", options),
  updater_(this),
  last_update_time_{0, 0, this->get_clock()->get_clock_type()},
  device_params_(
    declare_parameter<std::vector<std::string>>("devices", std::vector<std::string>())),
  usage_warn_(declare_parameter<float>("usage_warn", 0.95))
{
  gethostname(hostname_, sizeof(hostname_));
  updater_.setHardwareID(hostname_);
  updater_.add("Network Usage", this, &NetMonitor::checkUsage);

  nl80211_.init();
}

NetMonitor::~NetMonitor() { shutdown_nl80211(); }

void NetMonitor::update() { updater_.force_update(); }

void NetMonitor::shutdown_nl80211() { nl80211_.shutdown(); }

void NetMonitor::checkUsage(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  if (device_params_.empty()) {
    stat.summary(DiagStatus::ERROR, "invalid device parameter");
    return;
  }

  const struct ifaddrs * ifa;
  struct ifaddrs * ifas = nullptr;

  rclcpp::Duration duration = this->now() - last_update_time_;

  // Get network interfaces
  if (getifaddrs(&ifas) < 0) {
    stat.summary(DiagStatus::ERROR, "getifaddrs error");
    stat.add("getifaddrs", strerror(errno));
    return;
  }

  int level = DiagStatus::OK;
  int whole_level = DiagStatus::OK;
  int index = 0;
  std::string error_str;
  float rx_traffic{0.0};
  float tx_traffic{0.0};
  float rx_usage{0.0};
  float tx_usage{0.0};
  std::vector<std::string> interface_names;

  for (ifa = ifas; ifa; ifa = ifa->ifa_next) {
    // Skip no addr
    if (!ifa->ifa_addr) {
      continue;
    }
    // Skip loopback
    if (ifa->ifa_flags & IFF_LOOPBACK) {
      continue;
    }
    // Skip non AF_PACKET
    if (ifa->ifa_addr->sa_family != AF_PACKET) {
      continue;
    }
    // Skip device not specified
    if (
      boost::find(device_params_, ifa->ifa_name) == device_params_.end() &&
      boost::find(device_params_, "*") == device_params_.end()) {
      continue;
    }

    int fd;
    struct ifreq ifrm;
    struct ifreq ifrc;
    struct ethtool_cmd edata;

    // Get MTU information
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    strncpy(ifrm.ifr_name, ifa->ifa_name, IFNAMSIZ - 1);
    if (ioctl(fd, SIOCGIFMTU, &ifrm) < 0) {
      if (errno == ENOTSUP) {
        stat.add(fmt::format("Network {}: status", index), "Not Supported");
      } else {
        stat.add(fmt::format("Network {}: status", index), "Error");
        error_str = "ioctl error";
      }

      stat.add(fmt::format("Network {}: interface name", index), ifa->ifa_name);
      stat.add("ioctl(SIOCGIFMTU)", strerror(errno));

      ++index;
      close(fd);
      interface_names.push_back(ifa->ifa_name);
      continue;
    }

    // Get network capacity
    float speed = 0.0;
    strncpy(ifrc.ifr_name, ifa->ifa_name, IFNAMSIZ - 1);
    ifrc.ifr_data = (caddr_t)&edata;
    edata.cmd = ETHTOOL_GSET;
    if (ioctl(fd, SIOCETHTOOL, &ifrc) < 0) {
      // possibly wireless connection, get bitrate(MBit/s)
      speed = nl80211_.getBitrate(ifa->ifa_name);
      if (speed <= 0) {
        if (errno == ENOTSUP) {
          stat.add(fmt::format("Network {}: status", index), "Not Supported");
        } else {
          stat.add(fmt::format("Network {}: status", index), "Error");
          error_str = "ioctl error";
        }

        stat.add(fmt::format("Network {}: interface name", index), ifa->ifa_name);
        stat.add("ioctl(SIOCETHTOOL)", strerror(errno));

        ++index;
        close(fd);
        interface_names.push_back(ifa->ifa_name);
        continue;
      }
    } else {
      speed = edata.speed;
    }

    level = (ifa->ifa_flags & IFF_RUNNING) ? DiagStatus::OK : DiagStatus::ERROR;

    auto * stats = (struct rtnl_link_stats *)ifa->ifa_data;
    if (bytes_.find(ifa->ifa_name) != bytes_.end()) {
      rx_traffic = toMbit(stats->rx_bytes - bytes_[ifa->ifa_name].rx_bytes) / duration.seconds();
      tx_traffic = toMbit(stats->tx_bytes - bytes_[ifa->ifa_name].tx_bytes) / duration.seconds();
      rx_usage = rx_traffic / speed;
      tx_usage = tx_traffic / speed;
      if (rx_usage >= usage_warn_ || tx_usage > usage_warn_) {
        level = std::max(level, static_cast<int>(DiagStatus::WARN));
      }
    }

    stat.add(fmt::format("Network {}: status", index), usage_dict_.at(level));
    stat.add(fmt::format("Network {}: interface name", index), ifa->ifa_name);
    stat.addf(fmt::format("Network {}: rx_usage", index), "%.2f%%", rx_usage * 1e+2);
    stat.addf(fmt::format("Network {}: tx_usage", index), "%.2f%%", tx_usage * 1e+2);
    stat.addf(fmt::format("Network {}: rx_traffic", index), "%.2f MBit/s", rx_traffic);
    stat.addf(fmt::format("Network {}: tx_traffic", index), "%.2f MBit/s", tx_traffic);
    stat.addf(fmt::format("Network {}: capacity", index), "%.1f MBit/s", speed);
    stat.add(fmt::format("Network {}: mtu", index), ifrm.ifr_mtu);
    stat.add(fmt::format("Network {}: rx_bytes", index), stats->rx_bytes);
    stat.add(fmt::format("Network {}: rx_errors", index), stats->rx_errors);
    stat.add(fmt::format("Network {}: tx_bytes", index), stats->tx_bytes);
    stat.add(fmt::format("Network {}: tx_errors", index), stats->tx_errors);
    stat.add(fmt::format("Network {}: collisions", index), stats->collisions);

    close(fd);

    bytes_[ifa->ifa_name].rx_bytes = stats->rx_bytes;
    bytes_[ifa->ifa_name].tx_bytes = stats->tx_bytes;
    whole_level = std::max(whole_level, level);
    ++index;

    interface_names.push_back(ifa->ifa_name);
  }

  freeifaddrs(ifas);

  // Check if specified device exists
  for (const auto & device : device_params_) {
    // Skip if all devices specified
    if (device == "*") {
      continue;
    }
    // Skip if device already appended
    if (boost::find(interface_names, device) != interface_names.end()) {
      continue;
    }

    stat.add(fmt::format("Network {}: status", index), "No Such Device");
    stat.add(fmt::format("Network {}: interface name", index), device);
    error_str = "no such device";
    ++index;
  }

  if (!error_str.empty()) {
    stat.summary(DiagStatus::ERROR, error_str);
  } else {
    stat.summary(whole_level, usage_dict_.at(whole_level));
  }

  last_update_time_ = this->now();

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(NetMonitor)
