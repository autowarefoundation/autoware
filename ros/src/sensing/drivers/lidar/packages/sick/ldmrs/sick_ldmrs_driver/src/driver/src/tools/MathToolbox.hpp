//
// MathToolbox.hpp
//
//
// Math functions and definitions.
//
//////////////////////////////////////////////////////////////////////

#if !defined(MATHTOOLBOX_HPP)
#define MATHTOOLBOX_HPP

#include <math.h>
#include <cmath>	// for abs()
#include "../BasicDatatypes.hpp"


extern double hypot(double x, double y, double z);

/// Not-a-Number in double precision
extern const double NaN_double;

/// Tests if two \c double values are nearly equal
/**
 * \return \c true if the two \c double numbers are equal in
 * terms of the machine precision, which means their difference
 * must be less than 1E-11.
*/
inline bool fuzzyCompare (double a, double b)
{
	return std::abs(a - b) < 1E-11;
}

//
// Square
//
inline double sqr(double val)
{
	return val * val;
}

/// Tests if two \c float values are nearly equal
/**
 * \return \c true if the two \c float numbers are equal in
 * terms of the machine precision, which means their difference
 * must be less than 1E-6.
 */
inline bool fuzzyCompare(float a, float b)
{
	return std::abs(a - b) < 1E-6f;
}


// Normalizes an angle in radians to the interval [-pi,+pi)
double normalizeRadians (double radians);

/// Checks if a floating point value is Not-a-Number (NaN)
/**
 * The floating-point standard IEC 559 (a.k.a. IEEE 754) specifies that
 * a floating-point value \a x represents NaN if \a x != \a x.
 * \sa NaN, NaN_double, http://en.wikipedia.org/wiki/IEC_559
 */
template<typename floatT>
inline bool isNaN (floatT x)
{
	return (x != x);
}

/// Round to the closest integer
/**
 * \param floatValue The float value that shall be rounded
 * \return \a floatValue rounded to the closest integer
 */
template<typename IntT>
inline IntT round_to_int (float floatValue)
{

	//	assert (std::numeric_limits<float>::round_style == std::round_toward_zero);
	//	static_cast<IntT>() rounds toward zero
	return IntT (floatValue + (floatValue >= 0.0f ? + 0.5f : -0.5f));
}

/// Round to the closest integer
/**
 * \param floatValue The float value that shall be rounded
 * \return \a floatValue rounded to the closest integer
 */
template<typename IntT>
inline IntT round_to_int (double floatValue)
{

	//	assert (std::numeric_limits<double>::round_style == std::round_toward_zero);
	//	static_cast<IntT>() rounds toward zero
	return IntT (floatValue + (floatValue >= 0.0 ? + 0.5 : -0.5));
}


#endif // MATHTOOLBOX
