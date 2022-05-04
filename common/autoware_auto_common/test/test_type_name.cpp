// Copyright 2021 Apex.AI, Inc.
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
// Developed by Apex.AI, Inc.

#include <common/types.hpp>
#include <helper_functions/type_name.hpp>

#include <gtest/gtest.h>

namespace
{
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using autoware::helper_functions::get_type_name;

struct SomeStruct
{
};
}  // namespace

/// @test       Test that type names can be demangled.
TEST(TestTypeDemangling, Demangle)
{
  EXPECT_EQ(get_type_name<float32_t>(), "float");
  const float64_t val{42.0};
  EXPECT_EQ(get_type_name(val), "double");
  EXPECT_EQ(get_type_name<SomeStruct>(), "(anonymous namespace)::SomeStruct");
}
