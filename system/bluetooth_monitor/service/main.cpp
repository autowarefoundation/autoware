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

#include "bluetooth_monitor/service/l2ping_interface.hpp"
#include "bluetooth_monitor/service/l2ping_service.hpp"

#include <boost/lexical_cast.hpp>

#include <getopt.h>
#include <syslog.h>

/**
 * @brief print usage
 */
void usage()
{
  printf("Usage: l2ping_service [options]\n");
  printf("  -h --help   : Display help\n");
  printf("  -p --port # : Port number to listen to.\n");
  printf("\n");
}

/**
 * @brief Main loop
 * @param [in] argc Number of arguments
 * @param [in] argv Command line arguments
 * @return 0 on success, 1 on error
 */
int main(int argc, char ** argv)
{
  static struct option long_options[] = {
    {"help", no_argument, 0, 'h'}, {"port", required_argument, 0, 'p'}, {0, 0, 0, 0}};

  // Parse command-line options
  int c = 0;
  int option_index = 0;
  int port = DEFAULT_PORT;
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

  // Initialize l2ping service
  L2pingService service(port);

  if (!service.initialize()) {
    service.shutdown();
    closelog();
    return EXIT_FAILURE;
  }

  // Run main loop
  service.run();

  // Shutdown l2ping service
  service.shutdown();

  // Close descriptor used to write to system logger
  closelog();

  return EXIT_SUCCESS;
}
