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

#ifndef HELPER_FUNCTIONS__BOOL_COMPARISONS_HPP_
#define HELPER_FUNCTIONS__BOOL_COMPARISONS_HPP_

#include "common/types.hpp"

namespace autoware
{
namespace common
{
namespace helper_functions
{
namespace comparisons
{

/**
 * @brief Convenience method for performing logical exclusive or ops.
 * @return True iff exactly one of 'a' and 'b' is true.
 */
template <typename T>
types::bool8_t exclusive_or(const T & a, const T & b)
{
  return static_cast<types::bool8_t>(a) != static_cast<types::bool8_t>(b);
}

}  // namespace comparisons
}  // namespace helper_functions
}  // namespace common
}  // namespace autoware

#endif  // HELPER_FUNCTIONS__BOOL_COMPARISONS_HPP_
