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

#ifndef GEOMETRY__INTERVAL_HPP_
#define GEOMETRY__INTERVAL_HPP_

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

#include "common/types.hpp"
#include "helper_functions/float_comparisons.hpp"

namespace autoware
{
namespace common
{
namespace geometry
{

/**
 * @brief Data structure to contain scalar interval bounds.
 *
 * @post The interval is guaranteed to be valid upon successful construction. An
 * interval [min, max] is "valid" if it is empty (min/max = NaN) or its bounds
 * are ordered (min <= max).
 */
template<typename T>
class Interval
{
  static_assert(
    std::is_floating_point<T>::value,
    "Intervals only support floating point types.");

public:
  /**
   * @brief Compute equality.
   *
   * Two intervals compare equal iff they are both valid and they are both
   * either empty or have equal bounds.
   *
   * @note This is defined inline because the class is templated.
   *
   * @return True iff the intervals compare equal.
   */
  friend bool operator==(const Interval & i1, const Interval & i2)
  {
    const auto min_eq = (Interval::min(i1) == Interval::min(i2));
    const auto max_eq = (Interval::max(i1) == Interval::max(i2));
    const auto bounds_equal = (min_eq && max_eq);
    const auto both_empty = (Interval::empty(i1) && Interval::empty(i2));
    return both_empty || bounds_equal;
  }

  /**
   * @brief Compute inequality and the logical negation of equality.
   * @note This is defined inline because the class is templated.
   */
  friend bool operator!=(const Interval & i1, const Interval & i2)
  {
    return !(i1 == i2);
  }

  /**
   * @brief Stream overload for Interval types.
   *
   * @note Output precision is fixed inside the function definition, and the
   * serialization is JSON compatible.
   *
   * @note The serialization is lossy. It is used for debugging and for
   * generating exception strings.
   */
  friend std::ostream & operator<<(std::ostream & os, const Interval & i)
  {
    constexpr auto PRECISION = 5;

    // Internal helper to format the output.
    const auto readable_extremum = [](const T & val) {
        if (val == std::numeric_limits<T>::lowest()) {
          return std::string("REAL_MIN");
        }
        if (val == std::numeric_limits<T>::max()) {
          return std::string("REAL_MAX");
        }

        std::stringstream ss;
        ss.setf(std::ios::fixed, std::ios::floatfield);
        ss.precision(PRECISION);
        ss << val;
        return ss.str();
      };

    // Do stream output.
    if (Interval::empty(i)) {
      return os << "{}";
    }
    return os << "{\"min\": " << readable_extremum(Interval::min(i)) <<
           ", \"max\": " << readable_extremum(Interval::max(i)) << "}";
  }

  /**
   * @brief Test whether the two intervals have bounds within epsilon of each
   * other.
   *
   * @note If both intervals are empty, this returns true. If only one is empty,
   * this returns false.
   */
  static bool abs_eq(const Interval & i1, const Interval & i2, const T & eps);

  /** @brief The minimum bound of the interval. */
  static T min(const Interval & i) {return i.min_;}

  /** @brief The maximum bound of the interval. */
  static T max(const Interval & i) {return i.max_;}

  /**
   * @brief Return the measure (length) of the interval.
   *
   * @note For empty or invalid intervals, NaN is returned. See Interval::empty
   * for note on distinction between measure zero and the emptiness property.
   */
  static T measure(const Interval & i);

  /**
   * @brief Utility for checking whether an interval has zero measure.
   *
   * @note For empty or invalid intervals, false is returned. See
   * Interval::empty for note on distinction between measure zero and the
   * emptiness property.
   *
   * @return True iff the interval has zero measure.
   */
  static bool zero_measure(const Interval & i);

  /**
   * @brief Whether the interval is empty or not.
   *
   * @note Emptiness refers to whether the interval contains any points and is
   * thus a distinct property from measure: an interval is non-empty if contains
   * only a single point even though its measure in that case is zero.
   *
   * @return True iff the interval is empty.
   */
  static bool empty(const Interval & i);

  /**
   * @brief Test for whether a given interval contains a given value within the given epsilon
   * @return True iff 'interval' contains 'value'.
   */
  static bool contains(
    const Interval & i, const T & value,
    const T & eps = std::numeric_limits<T>::epsilon());

  /**
   * @brief Test for whether 'i1' subseteq 'i2'
   * @return True iff i1 subseteq i2.
   */
  static bool is_subset_eq(const Interval & i1, const Interval & i2);

  /**
   * @brief Compute the intersection of two intervals as a new interval.
   */
  static Interval intersect(const Interval & i1, const Interval & i2);

  /**
   * @brief Clamp a scalar 'val' onto 'interval'.
   * @return If 'val' in 'interval', return 'val'; otherwise return the nearer
   * interval bound.
   */
  static T clamp_to(const Interval & i, T val);

  /**
   * @brief Constructor: initialize an empty interval with members set to NaN.
   */
  Interval();

  /**
   * @brief Constructor: specify exact interval bounds.
   *
   * @note An empty interval is specified by setting both bounds to NaN.
   * @note An exception is thrown if the specified bounds are invalid.
   *
   * @post Construction is successful iff the interval is valid.
   */
  Interval(const T & min, const T & max);

private:
  static constexpr T NaN = std::numeric_limits<T>::quiet_NaN();

  T min_;
  T max_;

  /**
   * @brief Verify that the bounds are valid in an interval.
   * @note This method is private because it can only be used in the
   * constructor. Once an interval has been constructed, its bounds are
   * guaranteed to be valid.
   */
  static bool bounds_valid(const Interval & i);
};  // class Interval

//------------------------------------------------------------------------------

typedef Interval<autoware::common::types::float64_t> Interval_d;
typedef Interval<autoware::common::types::float32_t> Interval_f;

//------------------------------------------------------------------------------

template<typename T>
constexpr T Interval<T>::NaN;

//------------------------------------------------------------------------------

template<typename T>
Interval<T>::Interval()
: min_(Interval::NaN), max_(Interval::NaN) {}

//------------------------------------------------------------------------------

template<typename T>
Interval<T>::Interval(const T & min, const T & max)
: min_(min), max_(max)
{
  if (!Interval::bounds_valid(*this)) {
    std::stringstream ss;
    ss << "Attempted to construct an invalid interval: " << *this;
    throw std::runtime_error(ss.str());
  }
}

//------------------------------------------------------------------------------

template<typename T>
bool Interval<T>::abs_eq(
  const Interval & i1, const Interval & i2, const T & eps)
{
  namespace comp = helper_functions::comparisons;

  const auto both_empty = Interval::empty(i1) && Interval::empty(i2);
  const auto both_non_empty = !Interval::empty(i1) && !Interval::empty(i2);

  const auto mins_equal = comp::abs_eq(Interval::min(i1), Interval::min(i2), eps);
  const auto maxs_equal = comp::abs_eq(Interval::max(i1), Interval::max(i2), eps);
  const auto both_non_empty_equal = both_non_empty && mins_equal && maxs_equal;

  return both_empty || both_non_empty_equal;
}

//------------------------------------------------------------------------------

template<typename T>
bool Interval<T>::zero_measure(const Interval & i)
{
  // An empty interval will return false because (NaN == NaN) is false.
  return Interval::min(i) == Interval::max(i);
}

//------------------------------------------------------------------------------

template<typename T>
bool Interval<T>::empty(const Interval & i)
{
  return std::isnan(Interval::min(i)) && std::isnan(Interval::max(i));
}

//------------------------------------------------------------------------------

template<typename T>
bool Interval<T>::bounds_valid(const Interval & i)
{
  const auto is_ordered = (Interval::min(i) <= Interval::max(i));

  // Check for emptiness expicitly because it requires both bounds to be NaN
  return Interval::empty(i) || is_ordered;
}

//------------------------------------------------------------------------------

template<typename T>
bool Interval<T>::is_subset_eq(const Interval & i1, const Interval & i2)
{
  const auto lower_contained = (Interval::min(i1) >= Interval::min(i2));
  const auto upper_contained = (Interval::max(i1) <= Interval::max(i2));
  return lower_contained && upper_contained;
}

//------------------------------------------------------------------------------

template<typename T>
bool Interval<T>::contains(const Interval & i, const T & value, const T & eps)
{
  return common::helper_functions::comparisons::abs_gte(value, Interval::min(i), eps) &&
         common::helper_functions::comparisons::abs_lte(value, Interval::max(i), eps);
}

//------------------------------------------------------------------------------

template<typename T>
T Interval<T>::measure(const Interval & i)
{
  return Interval::max(i) - Interval::min(i);
}

//------------------------------------------------------------------------------

template<typename T>
Interval<T> Interval<T>::intersect(const Interval & i1, const Interval & i2)
{
  // Construct intersection assuming an intersection exists.
  try {
    const auto either_empty = Interval::empty(i1) || Interval::empty(i2);
    const auto min = std::max(Interval::min(i1), Interval::min(i2));
    const auto max = std::min(Interval::max(i1), Interval::max(i2));
    return either_empty ? Interval() : Interval(min, max);
  } catch (...) {
  }

  // Otherwise, the intersection is empty.
  return Interval();
}

//------------------------------------------------------------------------------

template<typename T>
T Interval<T>::clamp_to(const Interval & i, T val)
{
  // clamp val to min
  val = std::max(val, Interval::min(i));

  // clamp val to max
  val = std::min(val, Interval::max(i));

  return Interval::empty(i) ? Interval::NaN : val;
}

//------------------------------------------------------------------------------

}  // namespace geometry
}  // namespace common
}  // namespace autoware

#endif  // GEOMETRY__INTERVAL_HPP_
