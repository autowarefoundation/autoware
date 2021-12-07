// Copyright 2021 Autoware Foundation
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
 * @file traffic_reader.cpp
 * @brief traffic information read class
 */

#include <traffic_reader/traffic_reader.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/process.hpp>

#include <getopt.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <syslog.h>

#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

namespace bp = boost::process;

/**
 * @brief thread safe nethogs result
 */
class NethogsResult
{
public:
  /**
   * @brief get nethogs result string
   * @param [out] str nethogs result string
   * @return error code
   */
  int get(std::string & str)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    str = str_;
    return err_;
  }
  /**
   * @brief set nethogs result string and error code
   * @param [in] str nethogs result string
   * @param [in] err error code
   * @return error code
   */
  void set(const std::string & str, int err)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    str_ = str;
    err_ = err;
  }

private:
  std::mutex mutex_;  //!< @brief lock for thread safe
  std::string str_;   //!< @brief nethogs result string
  int err_;           //!< @brief error code, 0 on success, otherwise error
};
static NethogsResult g_NethogsString;     //!< @brief nethogs result
static volatile bool g_Running = false;   //!< @brief run status, true running, false exit
static volatile int g_WatchDogCount = 0;  //!< @brief executeNethogs watchdog, 0 refresh

/**
 * @brief print usage
 */
void usage()
{
  printf("Usage: traffic_reader [options]\n");
  printf("  -h --help   : Display help\n");
  printf("  -p --port # : Port number to listen to.\n");
  printf("\n");
}

/**
 * @brief thread function to execute nethogs
 */
void executeNethogs()
{
  const std::string cmd = "nethogs -t";
  // Output format of `nethogs -t`
  //  Blank Line    |
  //  Start Line    | Refreshing:
  //  Result Line 1 | PROGRAM/PID/UID SENT RECEIVED
  //     ...        |   ...
  //  Result Line n | PROGRAM/PID/UID SENT RECEIVED
  //  Last Line     | unknown TCP/0/0
  std::string error_str;
  std::string log_str;

  g_WatchDogCount = 0;

  while (g_Running) {
    bp::ipstream is_out;
    bp::ipstream is_err;
    boost::process::child c;
    try {
      c = bp::child(cmd, bp::std_out > is_out, bp::std_err > is_err);
      std::string result_buf;
      std::string line;
      while (std::getline(is_out, line)) {
        if (!g_Running) {
          // run() exit
          return;
        }
        if (line.empty()) {
          // Blank Line
          if (!result_buf.empty()) {
            g_NethogsString.set(result_buf, 0);
            result_buf = "";
          }
        } else if (line.find("unknown TCP/0/0") != std::string::npos) {
          // Last Line
          result_buf.append(line);
          g_NethogsString.set(result_buf, 0);
          result_buf = "";
        } else if (line.find("/0/0\t") != std::string::npos) {
          // no-pid and root data skip.
        } else if (line == std::string("Refreshing:")) {
          // Start Line
          g_WatchDogCount = 0;
          if (!result_buf.empty()) {
            g_NethogsString.set(result_buf, 0);
            result_buf = "";
          }
          if (!log_str.empty()) {
            syslog(LOG_INFO, "[%s] command started.\n", cmd.c_str());
            log_str = "";
          }
        } else {
          // Result Line
          result_buf.append(line + "\n");
        }
      }  // while

      c.wait();
      std::ostringstream os;
      is_err >> os.rdbuf();
      error_str = "command terminate. " + os.str();
    } catch (boost::process::process_error & e) {
      error_str = e.what();
      c.terminate();
      signal(SIGCHLD, SIG_IGN);
    }

    int err_code = c.exit_code();
    // The err_code when killing a child process is usually 9. However,
    // Because there were times when it was 0, 0 is changed to an error value.
    if (err_code == 0) {
      err_code = -1;
    }
    g_NethogsString.set("[" + cmd + "] " + error_str, err_code);

    if (log_str != error_str) {
      log_str = error_str;
      syslog(LOG_ERR, "[%s] err(%d) %s\n", cmd.c_str(), err_code, error_str.c_str());
    }

    for (int cnt = 10; cnt > 0 && g_Running; --cnt) {
      g_WatchDogCount = 0;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }  // while

  std::terminate();
}

/**
 * @brief check NET temperature
 * @param [in] port port to listen
 */
void run(int port)
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
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
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

  sockaddr_in client{};
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

    // Receive list of device from a socket
    char buf[1024]{};
    ret = recv(new_sock, buf, sizeof(buf) - 1, 0);
    if (ret < 0) {
      syslog(LOG_ERR, "Failed to receive. %s\n", strerror(errno));
      close(new_sock);
      close(sock);
      return;
    }
    // No data received
    if (ret == 0) {
      syslog(LOG_ERR, "No data received. %s\n", strerror(errno));
      close(new_sock);
      close(sock);
      return;
    }

    buf[sizeof(buf) - 1] = '\0';
    const std::string program_name = std::string(buf);

    // set result data
    TrafficReaderResult result;
    result.error_code_ = g_NethogsString.get(result.str_);
    if (result.error_code_ == 0 && program_name != GET_ALL_STR) {
      std::stringstream lines{result.str_};
      std::string line;
      std::string result_str;
      while (std::getline(lines, line)) {
        if (line.find(program_name) != std::string::npos) {
          result_str.append(line + "\n");
        }
      }  // while
      result.str_ = result_str;
    }

    std::ostringstream oss;
    boost::archive::text_oarchive oa(oss);
    oa << result;
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

    if (++g_WatchDogCount >= 3) {
      syslog(LOG_ERR, "nethogs command thread error\n");
      close(sock);
      return;
    }
  }

  close(sock);
}

int main(int argc, char ** argv)
{
  static struct option long_options[] = {
    {"help", no_argument, nullptr, 'h'},
    {"port", required_argument, nullptr, 'p'},
    {nullptr, 0, nullptr, 0}};

  // Parse command-line options
  int c = 0;
  int option_index = 0;
  int port = TRAFFIC_READER_PORT;
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

  // Put the program in the background
  if (daemon(0, 0) < 0) {
    printf("Failed to put the program in the background. %s\n", strerror(errno));
    return errno;
  }

  // Open connection to system logger
  openlog(nullptr, LOG_PID, LOG_DAEMON);

  g_Running = true;
  std::thread th = std::thread(&executeNethogs);
  th.detach();

  run(port);

  g_Running = false;

  // Close descriptor used to write to system logger
  closelog();

  return EXIT_SUCCESS;
}
