// Copyright 2017-2020 the Autoware Foundation
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
/// \brief This file contains a 1D linear lookup table implementation

#ifndef AUTOWARE_AUTO_GEOMETRY__LOOKUP_TABLE_HPP_
#define AUTOWARE_AUTO_GEOMETRY__LOOKUP_TABLE_HPP_

#include "autoware_auto_geometry/interval.hpp"
#include "common/types.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

namespace autoware
{
namespace common
{
namespace helper_functions
{

namespace
{

/**
 * @brief Compute a scaling between 'a' and 'b'
 * @note scaling is clamped to be in the interval [0, 1]
 */
template <typename T, typename U>
T interpolate(const T & a, const T & b, U scaling)
{
  static const geometry::Interval<U> UNIT_INTERVAL(static_cast<U>(0), static_cast<U>(1));
  scaling = geometry::Interval<U>::clamp_to(UNIT_INTERVAL, scaling);

  const auto m = (b - a);
  const auto offset = static_cast<U>(m) * scaling;
  return a + static_cast<T>(offset);
}

// TODO(c.ho) support more forms of interpolation as template functor
// Actual lookup logic, assuming all invariants hold:
// Throw if value is not finite
template <typename T>
T lookup_impl_1d(const std::vector<T> & domain, const std::vector<T> & range, const T value)
{
  if (!std::isfinite(value)) {
    throw std::domain_error{"Query value is not finite (NAN or INF)"};
  }
  if (value <= domain.front()) {
    return range.front();
  } else if (value >= domain.back()) {
    return range.back();
  } else {
    // Fall through to normal case
  }

  auto second_idx{0U};
  for (auto idx = 1U; idx < domain.size(); ++idx) {
    if (value < domain[idx]) {
      second_idx = idx;
      break;
    }
  }
  // T must be a floating point between 0 and 1
  const auto num = static_cast<double>(value - domain[second_idx - 1U]);
  const auto den = static_cast<double>(domain[second_idx] - domain[second_idx - 1U]);
  const auto t = num / den;
  const auto val = interpolate(range[second_idx - 1U], range[second_idx], t);
  return static_cast<T>(val);
}

// Check invariants for table lookup:
template <typename T>
void check_table_lookup_invariants(const std::vector<T> & domain, const std::vector<T> & range)
{
  if (domain.size() != range.size()) {
    throw std::domain_error{"Domain's size does not match range's"};
  }
  if (domain.empty()) {
    throw std::domain_error{"Empty domain or range"};
  }
  // Check if sorted: Can start at 1 because not empty
  for (auto idx = 1U; idx < domain.size(); ++idx) {
    if (domain[idx] <= domain[idx - 1U]) {  // This is safe because indexing starts at 1
      throw std::domain_error{"Domain is not sorted"};
    }
  }
}
}  // namespace

/// Do a 1D table lookup: Does some semi-expensive O(N) error checking first.
/// If query value fall out of the domain, then the value at the corresponding edge of the domain is
/// returned.
/// \param[in] domain The domain, or set of x values
/// \param[in] range The range, or set of y values
/// \param[in] value The point in the domain to query, x
/// \return A linearly interpolated value y, corresponding to the query, x
/// \throw std::domain_error If domain or range is empty
/// \throw std::domain_error If range is not the same size as domain
/// \throw std::domain_error If domain is not sorted
/// \throw std::domain_error If value is not finite (NAN or INF)
/// \tparam T The type of the function, must be interpolatable
template <typename T>
T lookup_1d(const std::vector<T> & domain, const std::vector<T> & range, const T value)
{
  check_table_lookup_invariants(domain, range);

  return lookup_impl_1d(domain, range, value);
}

/// A class wrapping a 1D lookup table. Intended for more frequent lookups. Error checking is pushed
/// into the constructor and not done in the lookup function call
/// \tparam T The type of the function, must be interpolatable
template <typename T>
class LookupTable1D
{
public:
  /// Constructor
  /// \param[in] domain The domain, or set of x values
  /// \param[in] range The range, or set of y values
  /// \throw std::domain_error If domain or range is empty
  /// \throw std::domain_error If range is not the same size as domain
  /// \throw std::domain_error If domain is not sorted
  LookupTable1D(const std::vector<T> & domain, const std::vector<T> & range)
  : m_domain{domain}, m_range{range}
  {
    check_table_lookup_invariants(m_domain, m_range);
  }

  /// Move constructor
  /// \param[in] domain The domain, or set of x values
  /// \param[in] range The range, or set of y values
  /// \throw std::domain_error If domain or range is empty
  /// \throw std::domain_error If range is not the same size as domain
  /// \throw std::domain_error If domain is not sorted
  LookupTable1D(std::vector<T> && domain, std::vector<T> && range)
  : m_domain{domain}, m_range{range}
  {
    check_table_lookup_invariants(m_domain, m_range);
  }

  /// Do a 1D table lookup
  /// If query value fall out of the domain, then the value at the corresponding edge of the domain
  /// is returned.
  /// \param[in] value The point in the domain to query, x
  /// \return A linearly interpolated value y, corresponding to the query, x
  /// \throw std::domain_error If value is not finite
  T lookup(const T value) const { return lookup_impl_1d(m_domain, m_range, value); }
  /// Get the domain table
  const std::vector<T> & domain() const noexcept { return m_domain; }
  /// Get the range table
  const std::vector<T> & range() const noexcept { return m_range; }

private:
  std::vector<T> m_domain;
  std::vector<T> m_range;
};  // class LookupTable1D

}  // namespace helper_functions
}  // namespace common
}  // namespace autoware

#endif  // AUTOWARE_AUTO_GEOMETRY__LOOKUP_TABLE_HPP_
