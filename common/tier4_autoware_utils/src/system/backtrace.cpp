// Copyright 2023 Tier IV, Inc.
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

#include "tier4_autoware_utils/system/backtrace.hpp"

#include "rclcpp/rclcpp.hpp"

#include <execinfo.h>

#include <iostream>
#include <memory>
#include <sstream>
#include <vector>

namespace tier4_autoware_utils
{

void print_backtrace()
{
  constexpr size_t max_frames = 100;
  void * addrlist[max_frames + 1];

  int addrlen = backtrace(addrlist, sizeof(addrlist) / sizeof(void *));

  if (addrlen == 0) {
    return;
  }

  char ** symbol_list = backtrace_symbols(addrlist, addrlen);

  std::stringstream ss;
  ss << "\n   @   ********** back trace **********" << std::endl;
  for (int i = 1; i < addrlen; i++) {
    ss << "   @   " << symbol_list[i] << std::endl;
  }
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("tier4_autoware_utils"), ss.str());

  free(symbol_list);
}

}  // namespace tier4_autoware_utils
