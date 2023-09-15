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

#ifndef TIER4_AUTOWARE_UTILS__SYSTEM__BACKTRACE_HPP_
#define TIER4_AUTOWARE_UTILS__SYSTEM__BACKTRACE_HPP_

#include <execinfo.h>

#include <iostream>
#include <memory>
#include <sstream>
#include <vector>

namespace tier4_autoware_utils
{

inline void print_backtrace()
{
  constexpr size_t max_frames = 100;
  void * addrlist[max_frames + 1];

  int addrlen = backtrace(addrlist, sizeof(addrlist) / sizeof(void *));

  if (addrlen == 0) {
    return;
  }

  char ** symbol_list = backtrace_symbols(addrlist, addrlen);

  std::stringstream ss;
  ss << "  ********** back trace **********" << std::endl;
  for (int i = 1; i < addrlen; i++) {
    ss << "   @   " << symbol_list[i] << std::endl;
  }
  std::cerr << ss.str() << std::endl;

  free(symbol_list);
}

}  // namespace tier4_autoware_utils

#endif  // TIER4_AUTOWARE_UTILS__SYSTEM__BACKTRACE_HPP_
