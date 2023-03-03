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

#include "map_tf_generator/uniform_random.hpp"

#include <gmock/gmock.h>

using testing::AllOf;
using testing::Each;
using testing::Ge;
using testing::Lt;

TEST(UniformRandom, UniformRandom)
{
  {
    const std::vector<size_t> random = UniformRandom(4, 0);
    ASSERT_EQ(random.size(), static_cast<size_t>(0));
  }

  // checks if the returned values are in range of [min, max)
  // note that the minimum range is always zero and the max value is exclusive
  {
    const size_t min_inclusive = 0;
    const size_t max_exclusive = 4;

    for (int i = 0; i < 50; i++) {
      const std::vector<size_t> random = UniformRandom(4, 10);
      ASSERT_EQ(random.size(), 10U);
      ASSERT_THAT(random, Each(AllOf(Ge(min_inclusive), Lt(max_exclusive))));  // in range [0, 4)
    }
  }
}
