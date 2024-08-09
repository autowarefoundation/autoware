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
 * @file msr_reader.cpp
 * @brief MSR read class
 */

#include "system_monitor/msr_reader/msr_reader.hpp"

#include <boost/archive/text_oarchive.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <syslog.h>
#include <unistd.h>

#include <algorithm>
#include <iostream>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

namespace fs = boost::filesystem;

// 7634-7647 Unassigned
constexpr int PORT = 7634;

/**
 * @brief Package Thermal Status Information
 * For details please see the documents below.
 * - Intel® 64 and IA-32 ArchitecturesSoftware Developer’s Manual
 *   https://software.intel.com/sites/default/files/managed/39/c5/325462-sdm-vol-1-2abcd-3abcd.pdf
 */
typedef struct
{
  uint64_t pkg_thermal_status_ : 1;  //!< @brief 0 Pkg Thermal Status (RO)
  // cppcheck-suppress unusedStructMember
  uint64_t pkg_thermal_status_log_ : 1;  //!< @brief 1 Pkg Thermal Status Log (R/W)
  // cppcheck-suppress unusedStructMember
  uint64_t pkg_prochot_event_ : 1;  //!< @brief 2 Pkg PROCHOT # event (RO)
  // cppcheck-suppress unusedStructMember
  uint64_t pkg_prochot_log_ : 1;  //!< @brief 3 Pkg PROCHOT # log (R/WC0)
  // cppcheck-suppress unusedStructMember
  uint64_t pkg_critical_temperature_status_ : 1;  //!< @brief 4 Pkg Critical Temperature Status (RO)
  uint64_t                                        //!< @brief 5 Pkg Critical Temperature
                                                  // cppcheck-suppress unusedStructMember
    pkg_critical_temperature_status_log_ : 1;     //!<   Status Log (R/WC0)
  // cppcheck-suppress unusedStructMember
  uint64_t pkg_thermal_threshold_1_status_ : 1;  //!< @brief 6 Pkg Thermal Threshold #1 Status (RO)
  // cppcheck-suppress unusedStructMember
  uint64_t pkg_thermal_threshold_1_log_ : 1;  //!< @brief 7 Pkg Thermal Threshold #1 log (R/WC0)
  // cppcheck-suppress unusedStructMember
  uint64_t pkg_thermal_threshold_2_status_ : 1;  //!< @brief 8 Pkg Thermal Threshold #2 Status (RO)
  // cppcheck-suppress unusedStructMember
  uint64_t pkg_thermal_threshold_2_log_ : 1;  //!< @brief 9 Pkg Thermal Threshold #2 log (R/WC0)
  // cppcheck-suppress unusedStructMember
  uint64_t pkg_power_limitation_status_ : 1;  //!< @brief 10 Pkg Power Limitation Status (RO)
  // cppcheck-suppress unusedStructMember
  uint64_t pkg_power_limitation_log_ : 1;  //!< @brief 11 Pkg Power Limitation log (R/WC0)
  // cppcheck-suppress unusedStructMember
  uint64_t reserved1_ : 4;  //!< @brief 15:12 Reserved
  // cppcheck-suppress unusedStructMember
  uint64_t pkg_digital_readout_ : 7;  //!< @brief 22:16 Pkg Digital Readout (RO)
  // cppcheck-suppress unusedStructMember
  uint64_t reserved2_ : 41;  //!< @brief 63:23 Reserved
} PackageThermalStatus;

/**
 * @brief print usage
 */
void usage()
{
  printf("Usage: msr_reader [options]\n");
  printf("  -h --help   : Display help\n");
  printf("  -p --port # : Port number to listen to.\n");
  printf("\n");
}

/**
 * @brief check CPU thermal throttling
 * @param [in] port port to listen
 * @param [in] list list of path to msr
 */
void run(int port, const std::vector<std::string> & list)
{
  // Create a new socket
  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    syslog(LOG_ERR, "Failed to create a new socket. %s\n", strerror(errno));
    return;
  }

  // Allow address reuse
  int ret = 0;
  int opt = 1;
  ret = setsockopt(
    sock, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<char *>(&opt), (socklen_t)sizeof(opt));
  if (ret < 0) {
    syslog(LOG_ERR, "Failed to set socket FD's option. %s\n", strerror(errno));
    close(sock);
    return;
  }

  // Give the socket FD the local address ADDR
  sockaddr_in addr;
  memset(&addr, 0, sizeof(sockaddr_in));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  // cppcheck-suppress cstyleCast
  ret = bind(sock, (struct sockaddr *)&addr, sizeof(addr));
  if (ret < 0) {
    syslog(LOG_ERR, "Failed to give the socket FD the local address ADDR. %s\n", strerror(errno));
    close(sock);
    return;
  }

  // Prepare to accept connections on socket FD
  ret = listen(sock, 5);
  if (ret < 0) {
    syslog(LOG_ERR, "Failed to prepare to accept connections on socket FD. %s\n", strerror(errno));
    close(sock);
    return;
  }

  sockaddr_in client;
  socklen_t len = sizeof(client);

  while (true) {
    // Await a connection on socket FD
    int new_sock = accept(sock, reinterpret_cast<sockaddr *>(&client), &len);
    if (new_sock < 0) {
      syslog(
        LOG_ERR, "Failed to prepare to accept connections on socket FD. %s\n", strerror(errno));
      close(sock);
      return;
    }

    ret = 0;
    std::ostringstream oss;
    boost::archive::text_oarchive oa(oss);
    MSRInfo msr{0, {}};

    for (auto itr = list.begin(); itr != list.end(); ++itr) {
      // Open a file
      int fd = open(itr->c_str(), O_RDONLY);
      if (fd < 0) {
        msr.error_code_ = errno;
        syslog(LOG_ERR, "Failed to open a file. %s\n", strerror(msr.error_code_));
        break;
      }

      // Read from a file descriptor
      PackageThermalStatus val;
      ret = pread(fd, &val, sizeof(uint64_t), 0x1b1);
      if (ret < 0) {
        msr.error_code_ = errno;
        syslog(LOG_ERR, "Failed to read from a file descriptor. %s\n", strerror(msr.error_code_));
        close(fd);
        break;
      }

      // Close the file descriptor FD
      ret = close(fd);
      if (ret < 0) {
        msr.error_code_ = errno;
        syslog(LOG_ERR, "Failed to close the file descriptor FD. %s\n", strerror(msr.error_code_));
        break;
      }

      msr.pkg_thermal_status_.push_back(val.pkg_thermal_status_);
    }

    oa << msr;
    // Write N bytes of BUF to FD
    ret = write(new_sock, oss.str().c_str(), oss.str().length());
    if (ret < 0) {
      syslog(LOG_ERR, "Failed to write N bytes of BUF to FD. %s\n", strerror(errno));
    }

    // Close the file descriptor FD
    ret = close(new_sock);
    if (ret < 0) {
      syslog(LOG_ERR, "Failed to close the file descriptor FD. %s\n", strerror(errno));
    }
  }

  close(sock);
}

int main(int argc, char ** argv)
{
  static struct option long_options[] = {
    {"help", no_argument, 0, 'h'}, {"port", required_argument, 0, 'p'}, {0, 0, 0, 0}};

  // Parse command-line options
  int c = 0;
  int option_index = 0;
  int port = PORT;
  while ((c = getopt_long(argc, argv, "hp:", long_options, &option_index)) != -1) {
    switch (c) {
      case 'h':
        usage();
        return EXIT_SUCCESS;

      case 'p':
        try {
          port = boost::lexical_cast<int>(optarg);
        } catch (const boost::bad_lexical_cast & e) {
          printf("Error: %s\n", e.what());
          return EXIT_FAILURE;
        }
        break;

      default:
        break;
    }
  }

  if (!fs::exists("/dev/cpu")) {
    printf("Failed to access /dev/cpu.\n");
    return EXIT_FAILURE;
  }

  std::vector<std::string> list;
  const fs::path root("/dev/cpu");

  for (const fs::path & path : boost::make_iterator_range(
         fs::recursive_directory_iterator(root), fs::recursive_directory_iterator())) {
    if (fs::is_directory(path)) {
      continue;
    }

    std::cmatch match;
    const char * msr = path.generic_string().c_str();

    // /dev/cpu/[0-9]/msr ?
    if (!std::regex_match(msr, match, std::regex(".*msr"))) {
      continue;
    }

    list.push_back(path.generic_string());
  }

  std::sort(list.begin(), list.end(), [](const std::string & c1, const std::string & c2) {
    std::cmatch match;
    const std::regex filter(".*/(\\d+)/msr");
    int n1 = 0;
    int n2 = 0;
    if (std::regex_match(c1.c_str(), match, filter)) {
      n1 = std::stoi(match[1].str());
    }
    if (std::regex_match(c2.c_str(), match, filter)) {
      n2 = std::stoi(match[1].str());
    }
    return n1 < n2;
  });  // NOLINT

  if (list.empty()) {
    printf("No msr found in /dev/cpu.\n");
    return EXIT_FAILURE;
  }

  // Put the program in the background
  if (daemon(0, 0) < 0) {
    printf("Failed to put the program in the background. %s\n", strerror(errno));
    return errno;
  }

  // Open connection to system logger
  openlog(nullptr, LOG_PID, LOG_DAEMON);

  run(port, list);

  // Close descriptor used to write to system logger
  closelog();

  return EXIT_SUCCESS;
}
