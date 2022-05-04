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

#ifndef HELPER_FUNCTIONS__BYTE_READER_HPP_
#define HELPER_FUNCTIONS__BYTE_READER_HPP_

#include <cstdint>
#include <cstring>
#include <vector>

namespace autoware
{
namespace common
{
namespace helper_functions
{
/// \brief A utility class to read byte vectors in big-endian order
class ByteReader
{
private:
  const std::vector<uint8_t> & byte_vector_;
  std::size_t index_;

public:
  /// \brief Default constructor, byte reader class
  /// \param[in] byte_vector A vector to read bytes from
  explicit ByteReader(const std::vector<uint8_t> & byte_vector)
  : byte_vector_(byte_vector), index_(0U)
  {
  }

  // brief Read bytes and store it in the argument passed in big-endian order
  /// \param[inout] value Read and store the bytes from the vector matching the size of the argument
  template <typename T>
  void read(T & value)
  {
    constexpr std::size_t kTypeSize = sizeof(T);
    union {
      T value;
      uint8_t byte_vector[kTypeSize];
    } tmp;

    for (std::size_t i = 0; i < kTypeSize; ++i) {
      tmp.byte_vector[i] = byte_vector_[index_ + kTypeSize - 1 - i];
    }

    value = tmp.value;

    index_ += kTypeSize;
  }

  void skip(std::size_t count) { index_ += count; }
};
}  // namespace helper_functions
}  // namespace common
}  // namespace autoware

#endif  // HELPER_FUNCTIONS__BYTE_READER_HPP_
