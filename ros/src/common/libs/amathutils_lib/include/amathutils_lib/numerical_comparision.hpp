#pragma once
#include <cmath>
#include <limits>

namespace amathutils
{
template <typename NumericType>
static bool approximatelyEqual(NumericType a, NumericType b, NumericType tolerance = std::numeric_limits<NumericType>::epsilon() * 10.0)
{
    NumericType diff = std::abs(a - b);
    if (diff <= tolerance)
        return true;

    if (diff < std::fmax(std::abs(a), std::abs(b)) * tolerance)
        return true;

    return false;
}

template <typename NumericType>
static bool approximatelyZero(NumericType a, NumericType tolerance = std::numeric_limits<NumericType>::epsilon() * 10.0)
{
    if (std::abs(a) <= tolerance)
        return true;
    return false;
}

template <typename NumericType>
static bool definitelyLessThan(NumericType a, NumericType b, NumericType tolerance = std::numeric_limits<NumericType>::epsilon() * 10.0)
{
    NumericType diff = a - b;
    if (diff < tolerance)
        return true;

    if (diff < std::fmax(std::abs(a), std::abs(b)) * tolerance)
        return true;

    return false;
}

template <typename NumericType>
static bool definitelyGreaterThan(NumericType a, NumericType b, NumericType tolerance = std::numeric_limits<NumericType>::epsilon() * 10.0)
{
    NumericType diff = a - b;
    if (diff > tolerance)
        return true;

    if (diff > std::fmax(std::abs(a), std::abs(b)) * tolerance)
        return true;

    return false;

}
} // namespace amathutils