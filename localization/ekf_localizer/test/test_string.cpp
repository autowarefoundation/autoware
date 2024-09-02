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

#include "ekf_localizer/string.hpp"

#include <gtest/gtest.h>

namespace autoware::ekf_localizer
{

TEST(erase_leading_slash, SmokeTest)
{
  EXPECT_EQ(erase_leading_slash("/topic"), "topic");
  EXPECT_EQ(erase_leading_slash("topic"), "topic");  // do nothing

  EXPECT_EQ(erase_leading_slash(""), "");
  EXPECT_EQ(erase_leading_slash("/"), "");
}

}  // namespace autoware::ekf_localizer
