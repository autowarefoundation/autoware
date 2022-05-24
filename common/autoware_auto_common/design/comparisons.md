# Comparisons

The `float_comparisons.hpp` library is a simple set of functions for performing approximate numerical comparisons.
There are separate functions for performing comparisons using absolute bounds and relative bounds. Absolute comparison checks are prefixed with `abs_` and relative checks are prefixed with `rel_`.

The `bool_comparisons.hpp` library additionally contains an XOR operator.

The intent of the library is to improve readability of code and reduce likelihood of typographical errors when using numerical and boolean comparisons.

## Target use cases

The approximate comparisons are intended to be used to check whether two numbers lie within some absolute or relative interval.
The `exclusive_or` function will test whether two values cast to different boolean values.

## Assumptions

- The approximate comparisons all take an `epsilon` parameter.
  The value of this parameter must be >= 0.
- The library is only intended to be used with floating point types.
  A static assertion will be thrown if the library is used with a non-floating point type.

## Example Usage

```c++
#include "common/bool_comparisons.hpp"
#include "common/float_comparisons.hpp"

#include <iostream>

// using-directive is just for illustration; don't do this in practice
using namespace autoware::common::helper_functions::comparisons;

static constexpr auto epsilon = 0.2;
static constexpr auto relative_epsilon = 0.01;

std::cout << exclusive_or(true, false) << "\n";
// Prints: true

std::cout << rel_eq(1.0, 1.1, relative_epsilon)) << "\n";
// Prints: false

std::cout << approx_eq(10000.0, 10010.0, epsilon, relative_epsilon)) << "\n";
// Prints: true

std::cout << abs_eq(4.0, 4.2, epsilon) << "\n";
// Prints: true

std::cout << abs_ne(4.0, 4.2, epsilon) << "\n";
// Prints: false

std::cout << abs_eq_zero(0.2, epsilon) << "\n";
// Prints: false

std::cout << abs_lt(4.0, 4.25, epsilon) << "\n";
// Prints: true

std::cout << abs_lte(1.0, 1.2, epsilon) << "\n";
// Prints: true

std::cout << abs_gt(1.25, 1.0, epsilon) << "\n";
// Prints: true

std::cout << abs_gte(0.75, 1.0, epsilon) << "\n";
// Prints: false
```
