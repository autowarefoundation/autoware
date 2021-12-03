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
 * @file msr_reader.h
 * @brief MSR reader definitions
 */

#ifndef MSR_READER__MSR_READER_HPP_
#define MSR_READER__MSR_READER_HPP_

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>

#include <vector>

/**
 * @brief MSR information
 */
struct MSRInfo
{
  int error_code_;                        //!< @brief error code, 0 on success, otherwise error
  std::vector<bool> pkg_thermal_status_;  //!< @brief Pkg Thermal Status

  /**
   * @brief Load or save data members.
   * @param [inout] ar archive reference to load or save the serialized data members
   * @param [in] version version for the archive
   * @note NOLINT syntax is needed since this is an interface to serialization and
   * used inside boost serialization.
   */
  template <typename archive>
  void serialize(archive & ar, const unsigned /*version*/)  // NOLINT(runtime/references)
  {
    ar & error_code_;
    ar & pkg_thermal_status_;
  }
};

#endif  // MSR_READER__MSR_READER_HPP_
