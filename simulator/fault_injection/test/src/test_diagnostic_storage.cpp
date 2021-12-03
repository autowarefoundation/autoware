// Copyright 2021 Tier IV, Inc.
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

#include "fault_injection/diagnostic_storage.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

class TestDiagnosticStorage : public ::testing::Test
{
protected:
  TestDiagnosticStorage()
  {
    for (const auto & event_diag : diag_config) {
      storage_.registerEvent(event_diag);
    }
  }

  fault_injection::DiagnosticStorage storage_;
  const std::vector<fault_injection::DiagConfig> diag_config{
    {"foo", "foo_diag"},
  };
};

TEST_F(TestDiagnosticStorage, call_update_with_correct_key)
{
  auto diag = storage_.getDiag("foo");
  EXPECT_STREQ(diag.name.c_str(), "foo_diag");
  EXPECT_EQ(diag.level, 0);

  storage_.updateLevel("foo", 1);
  diag = storage_.getDiag("foo");
  EXPECT_STREQ(diag.name.c_str(), "foo_diag");
  EXPECT_EQ(diag.level, 1);
}

TEST_F(TestDiagnosticStorage, raise_exception_with_wrong_key)
{
  EXPECT_ANY_THROW(storage_.getDiag("invalid_name"));
}
