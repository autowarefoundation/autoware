// Copyright 2023 The Autoware Contributors
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

#include "ekf_localizer/aged_object_queue.hpp"

#include <gtest/gtest.h>

TEST(AgedObjectQueue, DiscardsObjectWhenAgeReachesMaximum)
{
  AgedObjectQueue<std::string> queue(3);

  queue.push("a");
  EXPECT_EQ(queue.size(), 1U);

  queue.pop_increment_age();  // age = 1
  EXPECT_EQ(queue.size(), 1U);

  queue.pop_increment_age();  // age = 2
  EXPECT_EQ(queue.size(), 1U);

  queue.pop_increment_age();  // age = 3
  EXPECT_EQ(queue.size(), 0U);
}

TEST(AgedObjectQueue, MultipleObjects)
{
  AgedObjectQueue<std::string> queue(3);

  queue.push("a");
  EXPECT_EQ(queue.size(), 1U);
  EXPECT_EQ(queue.pop_increment_age(), std::string{"a"});  // age of a = 1
  EXPECT_EQ(queue.size(), 1U);
  EXPECT_EQ(queue.pop_increment_age(), std::string{"a"});  // age of a = 2

  queue.push("b");

  EXPECT_EQ(queue.pop_increment_age(), std::string{"a"});  // age of a = 3
  EXPECT_EQ(queue.size(), 1U);

  EXPECT_EQ(queue.pop_increment_age(), std::string{"b"});  // age of b = 1
  EXPECT_EQ(queue.size(), 1U);

  EXPECT_EQ(queue.pop_increment_age(), std::string{"b"});  // age of b = 2
  EXPECT_EQ(queue.size(), 1U);

  EXPECT_EQ(queue.pop_increment_age(), std::string{"b"});  // age of b = 3
  EXPECT_EQ(queue.size(), 0U);
}

TEST(AgedObjectQueue, Empty)
{
  AgedObjectQueue<std::string> queue(2);

  EXPECT_TRUE(queue.empty());

  queue.push("a");

  EXPECT_FALSE(queue.empty());

  queue.pop_increment_age();
  queue.pop_increment_age();

  EXPECT_TRUE(queue.empty());
}

TEST(AgedObjectQueue, Clear)
{
  AgedObjectQueue<std::string> queue(3);

  queue.push("a");
  queue.push("b");

  EXPECT_EQ(queue.size(), 2U);

  queue.clear();

  EXPECT_EQ(queue.size(), 0U);
}

TEST(AgedObjectQueue, Back)
{
  AgedObjectQueue<std::string> queue(3);

  queue.push("a");

  EXPECT_EQ(queue.back(), std::string{"a"});
  queue.push("b");

  EXPECT_EQ(queue.back(), std::string{"b"});
}
