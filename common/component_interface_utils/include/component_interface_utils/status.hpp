// Copyright 2022 TIER IV, Inc.
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

#ifndef COMPONENT_INTERFACE_UTILS__STATUS_HPP_
#define COMPONENT_INTERFACE_UTILS__STATUS_HPP_

namespace component_interface_utils::status
{

template <class T1, class T2>
void copy(const T1 & src, T2 & dst)  // NOLINT(build/include_what_you_use): cpplint false positive
{
  dst->status.success = src->status.success;
  dst->status.code = src->status.code;
  dst->status.message = src->status.message;
}

}  // namespace component_interface_utils::status

#endif  // COMPONENT_INTERFACE_UTILS__STATUS_HPP_
