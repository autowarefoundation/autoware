// Copyright 2020 TierIV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

//lint -e537 pclint vs cpplint NOLINT

#include <lanelet2_io/io_handlers/Serialize.h>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "had_map_utils/had_map_conversion.hpp"


namespace autoware
{
namespace common
{
namespace had_map_utils
{

void toBinaryMsg(
  const std::shared_ptr<lanelet::LaneletMap> & map,
  autoware_auto_mapping_msgs::msg::HADMapBin & msg)
{
  std::stringstream ss;
  boost::archive::binary_oarchive oa(ss);
  oa << *map;
  auto id_counter = lanelet::utils::getId();
  oa << id_counter;

  std::string tmp_str = ss.str();
  msg.data.clear();
  msg.data.resize(tmp_str.size());
  msg.data.assign(tmp_str.begin(), tmp_str.end());
}

void fromBinaryMsg(
  const autoware_auto_mapping_msgs::msg::HADMapBin & msg,
  std::shared_ptr<lanelet::LaneletMap> & map)
{
  std::string data_str;
  data_str.assign(msg.data.begin(), msg.data.end());
  std::stringstream ss;
  ss << data_str;
  boost::archive::binary_iarchive oa(ss);
  oa >> *map;
  lanelet::Id id_counter;
  oa >> id_counter;
  lanelet::utils::registerId(id_counter);
}

}  // namespace had_map_utils
}  // namespace common
}  // namespace autoware
