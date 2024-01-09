// Copyright 2020 Mapless AI, Inc.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.

#include "autoware_auto_common/helper_functions/bool_comparisons.hpp"

#include <gtest/gtest.h>

// cppcheck does not like gtest macros inside of namespaces:
// https://sourceforge.net/p/cppcheck/discussion/general/thread/e68df47b/
// use a namespace alias instead of putting macros into the namespace
namespace comp = autoware::common::helper_functions::comparisons;

//------------------------------------------------------------------------------

TEST(HelperFunctionsComparisons, ExclusiveOr)
{
  EXPECT_TRUE(comp::exclusive_or(0, 1));
  EXPECT_TRUE(comp::exclusive_or(1, 0));
  EXPECT_FALSE(comp::exclusive_or(0, 0));
  EXPECT_FALSE(comp::exclusive_or(1, 1));

  EXPECT_TRUE(comp::exclusive_or(false, true));
  EXPECT_TRUE(comp::exclusive_or(true, false));
  EXPECT_FALSE(comp::exclusive_or(false, false));
  EXPECT_FALSE(comp::exclusive_or(true, true));
}

//------------------------------------------------------------------------------
