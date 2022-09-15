// Copyright 2022 The Autoware Contributors
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

#include "../src/pose_initializer/copy_vector_to_array.hpp"

#include <gmock/gmock.h>

TEST(CopyVectorToArray, CopyAllElements)
{
  const std::vector<int> vector{0, 1, 2, 3, 4};
  std::array<int, 5> array;
  copy_vector_to_array<int, 5>(vector, array);
  EXPECT_THAT(array, testing::ElementsAre(0, 1, 2, 3, 4));
}

TEST(CopyVectorToArray, CopyZeroElements)
{
  const std::vector<int> vector{};
  // just confirm that this works
  std::array<int, 0> array;
  copy_vector_to_array<int, 0>(vector, array);
}

TEST(CopyVectorToArray, ThrowsInvalidArgumentIfMoreElementsExpected)
{
  auto f = [] {
    const std::vector<int> vector{0, 1, 2, 3, 4};
    std::array<int, 6> array;
    copy_vector_to_array<int, 6>(vector, array);
  };

  EXPECT_THROW(
    try { f(); } catch (std::exception & e) {
      EXPECT_STREQ(
        e.what(), "Vector size (which is 5) is different from the copy size (which is 6)");
      throw;
    },
    std::invalid_argument);
}
