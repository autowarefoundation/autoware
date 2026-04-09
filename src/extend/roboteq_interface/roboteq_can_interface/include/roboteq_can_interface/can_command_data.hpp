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

#ifndef ROBOTEQ_CAN_INTERFACE__CAN_COMMAND_DATA_HPP_
#define ROBOTEQ_CAN_INTERFACE__CAN_COMMAND_DATA_HPP_

#include <array>

class CanCommandData
{
public:
  CanCommandData() {}

  std::array<uint8_t, 8> get_ex() { return ex_cmd; }
  std::array<uint8_t, 8> get_mg() { return mg_cmd; }
  std::array<uint8_t, 8> get_bs() { return bs_cmd; }
  std::array<uint8_t, 8> get_bsr() { return bsr_cmd; }
  std::array<uint8_t, 8> get_cb() { return cb_cmd; }
  std::array<uint8_t, 8> get_e() { return e_cmd; }
  std::array<uint8_t, 8> get_f() { return f_cmd; }
  std::array<uint8_t, 8> get_ff() { return ff_cmd; }
  std::array<uint8_t, 8> get_fm() { return fm_cmd; }
  std::array<uint8_t, 8> get_fs() { return fs_cmd; }

private:
  // Runtime Commands
  std::array<uint8_t, 8> ex_cmd  // EX: Emergency Stop
    {0x2C, 0x0C, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00};
  std::array<uint8_t, 8> mg_cmd  // MG: Emergency Stop Release
    {0x2C, 0x0D, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00};

  // Runtime Queries
  std::array<uint8_t, 8> bs_cmd  // BS: Read BL Motor Speed in RPM
    {0x4C, 0x0A, 0x21, 0x01, 0x00, 0x00, 0x00, 0x00};
  std::array<uint8_t, 8> bsr_cmd  // BSR: Read BL Motor Speed as 1/1000 of Max RPM
    {0x4C, 0x0B, 0x21, 0x01, 0x00, 0x00, 0x00, 0x00};
  std::array<uint8_t, 8> cb_cmd  // CB: Read Absolute Brushless Counter
    {0x4C, 0x05, 0x21, 0x01, 0x00, 0x00, 0x00, 0x00};
  std::array<uint8_t, 8> e_cmd  // E: Read Closed Loop Error
    {0x4C, 0x14, 0x21, 0x01, 0x00, 0x00, 0x00, 0x00};
  std::array<uint8_t, 8> f_cmd  // F: Read Feedback
    {0x4C, 0x10, 0x21, 0x01, 0x00, 0x00, 0x00, 0x00};
  std::array<uint8_t, 8> ff_cmd  // FF: Read Fault Flags
    {0x4C, 0x12, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00};
  std::array<uint8_t, 8> fm_cmd  // FM: Read Runtime Status Flag
    {0x4C, 0x22, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00};
  std::array<uint8_t, 8> fs_cmd  // FS: Read Status Flags
    {0x4C, 0x11, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00};
};

#endif  // ROBOTEQ_CAN_INTERFACE__CAN_COMMAND_DATA_HPP_
