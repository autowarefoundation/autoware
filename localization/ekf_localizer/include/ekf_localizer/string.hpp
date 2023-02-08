// Copyright 2023 Autoware Foundation
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

#ifndef EKF_LOCALIZER__STRING_HPP_
#define EKF_LOCALIZER__STRING_HPP_

#include <string>

inline std::string eraseLeadingSlash(const std::string & s)
{
  std::string a = s;
  if (a.front() == '/') {
    a.erase(0, 1);
  }
  return a;
}

#endif  // EKF_LOCALIZER__STRING_HPP_
