// Copyright 2019 the Autoware Foundation
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include "autoware_auto_common/common/types.hpp"
#include "autoware_auto_common/helper_functions/byte_reader.hpp"

#include <gtest/gtest.h>

#include <vector>

using autoware::common::types::float64_t;

namespace
{
class ByteReader : public ::testing::Test
{
};
}  // namespace

// tests serial_driver_node's get_packet function which receives serial packages
TEST_F(ByteReader, Basic)
{
  std::vector<uint8_t> data = {0x00, 0x00, 0x00, 0x17, 0x40, 0x28, 0xAE, 0x14,
                               0x7A, 0xE1, 0x47, 0xAE, 0x00, 0x00, 0x08};

  autoware::common::helper_functions::ByteReader byte_reader(data);

  uint32_t a = 0;
  byte_reader.read(a);
  ASSERT_EQ(a, 23U);

  float64_t b = 0;
  byte_reader.read(b);
  ASSERT_EQ(b, 12.34);

  byte_reader.skip(1);

  int16_t c = 0;
  byte_reader.read(c);
  ASSERT_EQ(c, 8);
}
