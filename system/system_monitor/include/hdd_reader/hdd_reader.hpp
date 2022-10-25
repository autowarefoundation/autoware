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
 * @file hdd_reader.h
 * @brief HDD reader definitions
 */

#ifndef HDD_READER__HDD_READER_HPP_
#define HDD_READER__HDD_READER_HPP_

#include <boost/serialization/map.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/string.hpp>

#include <bitset>
#include <map>
#include <string>

/**
 * @brief Enumeration of Request ID to hdd_reader
 */
enum HddReaderRequestId {
  GetHddInfo,
  UnmountDevice,
};

/**
 * @brief HDD device
 */
struct HddDevice
{
  std::string name_;                     //!< @brief Device name
  uint8_t temp_attribute_id_;            //!< @brief S.M.A.R.T attribute ID of temperature
  uint8_t power_on_hours_attribute_id_;  //!< @brief S.M.A.R.T attribute ID of power on hours
  uint8_t
    total_data_written_attribute_id_;     //!< @brief S.M.A.R.T attribute ID of total data written
  uint8_t recovered_error_attribute_id_;  //!< @brief S.M.A.R.T attribute ID of recovered error

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
    ar & name_;
    ar & temp_attribute_id_;
    ar & power_on_hours_attribute_id_;
    ar & total_data_written_attribute_id_;
    ar & recovered_error_attribute_id_;
  }
};

/**
 * @brief HDD information
 */
struct HddInfo
{
  int error_code_;      //!< @brief error code, 0 on success, otherwise error
  std::string model_;   //!< @brief Model number
  std::string serial_;  //!< @brief Serial number
  uint8_t temp_;        //!< @brief temperature(DegC)
  // Lowest byte of the raw value contains the exact temperature value (Celsius degrees)
  // in S.M.A.R.T. information.
  uint64_t power_on_hours_;           //!< @brief power on hours count
  uint64_t total_data_written_;       //!< @brief total data written
  uint32_t recovered_error_;          //!< @brief recovered error count
  bool is_valid_temp_;                //!< @brief whether temp_ is valid value
  bool is_valid_power_on_hours_;      //!< @brief whether power_on_hours_ is valid value
  bool is_valid_total_data_written_;  //!< @brief whether total_data_written_ is valid value
  bool is_valid_recovered_error_;     //!< @brief whether recovered_error_ is valid value

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
    ar & model_;
    ar & serial_;
    ar & temp_;
    ar & power_on_hours_;
    ar & total_data_written_;
    ar & recovered_error_;
    ar & is_valid_temp_;
    ar & is_valid_power_on_hours_;
    ar & is_valid_total_data_written_;
    ar & is_valid_recovered_error_;
  }
};

/**
 * @brief unmount device information
 */
struct UnmountDeviceInfo
{
  std::string part_device_;  //!< @brief partition device

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
    ar & part_device_;
  }
};

/**
 * @brief HDD information list
 */
typedef std::map<std::string, HddInfo> HddInfoList;

#endif  // HDD_READER__HDD_READER_HPP_
