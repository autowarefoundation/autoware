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
 * @file system_monitor_utility.h
 * @brief System Monitor Utility class
 */

#ifndef SYSTEM_MONITOR__SYSTEM_MONITOR_UTILITY_HPP_
#define SYSTEM_MONITOR__SYSTEM_MONITOR_UTILITY_HPP_

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/range.hpp>

#include <chrono>
#include <regex>
#include <string>
#include <vector>

namespace fs = boost::filesystem;

typedef struct thermal_zone
{
  std::string type_;   //!< @brief thermal zone name
  std::string label_;  //!< @brief thermal_zone[0-9]
  std::string path_;   //!< @brief sysfs path to temperature

  thermal_zone() : type_(), label_(), path_() {}
  thermal_zone(const std::string & type, const std::string & label, const std::string & path)
  : type_(type), label_(label), path_(path)
  {
  }
} thermal_zone;

class SystemMonitorUtility
{
public:
  /**
   * @brief get thermal zone information
   * @param [in] t thermal zone name
   * @param [in] pointer to thermal zone information
   */
  static void getThermalZone(const std::string & t, std::vector<thermal_zone> * therm)
  {
    if (therm == nullptr) {
      return;
    }

    therm->clear();

    const fs::path root("/sys/class/thermal");

    for (const fs::path & path :
         boost::make_iterator_range(fs::directory_iterator(root), fs::directory_iterator())) {
      if (!fs::is_directory(path)) {
        continue;
      }

      std::cmatch match;
      const char * therm_dir = path.generic_string().c_str();

      // not thermal_zone[0-9]
      if (!std::regex_match(therm_dir, match, std::regex(".*/thermal_zone(\\d+)"))) {
        continue;
      }

      std::string type;
      const fs::path type_path = path / "type";
      fs::ifstream ifs(type_path, std::ios::in);
      if (ifs) {
        std::string line;
        if (std::getline(ifs, line)) {
          type = line;
        }
      }
      ifs.close();

      if (type != t) {
        continue;
      }

      const fs::path temp_path = path / "temp";
      therm->emplace_back(t, path.filename().generic_string(), temp_path.generic_string());
    }
  }

  /**
   * @brief Remember start time to measure elapsed time
   * @return start time
   */
  static std::chrono::high_resolution_clock::time_point startMeasurement()
  {
    return std::chrono::high_resolution_clock::now();
  }

  /**
   * @brief Measure elapsed time since start time and report
   * @param [in] t_start start time
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   */
  static void stopMeasurement(
    const std::chrono::high_resolution_clock::time_point & start,
    diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    // Measure elapsed time since start time and report
    const auto t_end = std::chrono::high_resolution_clock::now();
    const float elapsed_ms = std::chrono::duration<float, std::milli>(t_end - start).count();
    stat.addf("execution time", "%f ms", elapsed_ms);
  }
};

#endif  // SYSTEM_MONITOR__SYSTEM_MONITOR_UTILITY_HPP_
