# Interval

The interval is a standard 1D real-valued interval.
The class implements a representation and operations on the interval type and guarantees interval validity on construction.
Basic operations and accessors are implemented, as well as other common operations.
See 'Example Usage' below.

## Target use cases

- Range or containment checks.
  The interval class simplifies code that involves checking membership of a value to a range, or intersecting two ranges.
  It also provides consistent behavior and consistent handling of edge cases.

## Properties

- **empty**: An empty interval is equivalent to an empty set.
  It contains no elements.
  It is a valid interval, but because it is empty, the notion of measure (length) is undefined; the measure of an empty interval is _not_ zero.
  The implementation represents the measure of an empty interval with `NaN`.
- **zero measure**: An interval with zero measure is an interval whose bounds are exactly equal.
  The measure is zero because the interval contains only a single point, and points have zero measure.
  However, because it does contain a single element, the interval is _not_ empty.
- **valid**: A valid interval is either empty or has min/max bounds such that (min <= max). On construction, interval objects are guaranteed to be valid.
  An attempt to construct an invalid interval results in a runtime_error exception being thrown.
- **pseudo-immutable**: Once constructed the only way to change the value of an interval is to overwrite it with a new one; an existing object cannot be modified.

## Conventions

- All operations on interval objects are defined as static class methods on the interval class.
  This is a functional-style of programming that basically turns the class into a namespace that grants functions access to private member variables of the object they operate on.

## Assumptions

- The interval is only intended for floating point types.
  This is enforced via static assertion.
- The constructor for non-empty intervals takes two arguments 'min' and 'max', and they must be ordered (i.e., min <= max).
  If this assumption is violated, an exception is emitted and construction fails.

## Example Usage

```c++
#include "geometry/interval.hpp"

#include <iostream>

// using-directive is just for illustration; don't do this in practice
using namespace autoware::common::geometry;

// bounds for example interval
constexpr auto MIN = 0.0;
constexpr auto MAX = 1.0;

//
// Try to construct an invalid interval. This will give the following error:
// 'Attempted to construct an invalid interval: {"min": 1.0, "max": 0.0}'
//

try {
  const auto i = Interval_d(MAX, MIN);
} catch (const std::runtime_error& e) {
  std::cerr << e.what();
}

//
// Construct a double precision interval from 0 to 1
//

const auto i = Interval_d(MIN, MAX);

//
// Test accessors and properties
//

std::cout << Interval_d::min(i) << " " << Interval_d::max(i) << "\n";
// Prints: 0.0 1.0

std::cout << Interval_d::empty(i) << " " << Interval_d::length(i) << "\n";
// Prints: false 1.0

std::cout << Interval_d::contains(i, 0.3) << "\n";
// Prints: true

std::cout << Interval_d::is_subset_eq(Interval_d(0.2, 0.4), i) << "\n";
// Prints: true

//
// Test operations.
//

std::cout << Interval_d::intersect(i, Interval(-1.0, 0.3)) << "\n";
// Prints: {"min": 0.0, "max": 0.3}

std::cout << Interval_d::project_to_interval(i, 0.5) << " "
          << Interval_d::project_to_interval(i, -1.3) << "\n";
// Prints: 0.5 0.0

//
// Distinguish empty/zero measure
//

const auto i_empty = Interval();
const auto i_zero_length = Interval(0.0, 0.0);

std::cout << Interval_d::empty(i_empty) << " "
          << Interval_d::empty(i_zero_length) << "\n";
// Prints: true false

std::cout << Interval_d::zero_measure(i_empty) << " "
          << Interval_d::zero_measure(i_zero_length) << "\n";
// Prints: false false
```
