// Copyright 2021 Tier IV, Inc.
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
 * @file traffic_reader.h
 * @brief traffic reader definitions
 */

#ifndef SYSTEM_MONITOR__TRAFFIC_READER__TRAFFIC_READER_COMMON_HPP_
#define SYSTEM_MONITOR__TRAFFIC_READER__TRAFFIC_READER_COMMON_HPP_

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/vector.hpp>

#include <string>

namespace traffic_reader_service
{

static constexpr char socket_path[] = "/tmp/traffic_reader";

enum Request {
  NONE = 0,
  START_NETHOGS,
  GET_RESULT,
};

/**
 * @brief Result of nethogs
 */
struct Result
{
  int error_code;      //!< @brief Error code, 0 on success, otherwise error
  std::string output;  //!< @brief Result output of nethogs

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
    ar & error_code;
    ar & output;
  }
};

// constexpr std::string_view GET_ALL_STR{"<All>"};  //!< @brief nethogs result all request string

}  // namespace traffic_reader_service

#endif  // SYSTEM_MONITOR__TRAFFIC_READER__TRAFFIC_READER_COMMON_HPP_
