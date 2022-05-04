// Copyright 2017-2019 the Autoware Foundation
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
/// \file
/// \brief This file includes common helper functions

#ifndef HELPER_FUNCTIONS__CRTP_HPP_
#define HELPER_FUNCTIONS__CRTP_HPP_

namespace autoware
{
namespace common
{
namespace helper_functions
{
template <typename Derived>
class crtp
{
protected:
  const Derived & impl() const
  {
    // This is the CRTP pattern for static polymorphism: this is related, static_cast is the only
    // way to do this
    // lint -e{9005, 9176, 1939} NOLINT
    return *static_cast<const Derived *>(this);
  }

  Derived & impl()
  {
    // This is the CRTP pattern for static polymorphism: this is related, static_cast is the only
    // way to do this
    // lint -e{9005, 9176, 1939} NOLINT
    return *static_cast<Derived *>(this);
  }
};
}  // namespace helper_functions
}  // namespace common
}  // namespace autoware

#endif  // HELPER_FUNCTIONS__CRTP_HPP_
