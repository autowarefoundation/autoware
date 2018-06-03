//
// MathToolbox.cpp
//
//
// Math functions.
//
//////////////////////////////////////////////////////////////////////

#include "MathToolbox.hpp"
#include <limits>


const double NaN_double = std::numeric_limits<double>::quiet_NaN();
//const float NaN = std::numeric_limits<float>::quiet_NaN();

/**
 * Berechne die Laenge der Hypothenuse
 */
double hypot(double x, double y, double z)
{
	return sqrt(x*x + y*y + z*z);
}


/**
 * Normalizes an angle given in radians by adding or subtracting an integer
 * multiple of 2*pi so that the resulting angle is in the half-open interval
 * [-pi,+pi). The current implementation takes O(1) time, i.e. the time of
 * execution has a fixed upper boundary independend from the angle.
 */
double normalizeRadians (double radians)
{
	if (std::abs(radians) > PI)
	{
		// For numerical stability we must use this sin/cos/atan2
		// implementation even though it might consume more cycles.
		// Note that radians = -pi converts to atan2(0,-1) = +pi!
		radians = std::atan2 (std::sin(radians), std::cos(radians));
		// radians now in (-pi,+pi]
	}
	if (radians == PI)	// equality of doubles ... just for the sake of completeness
	{
		// Convert half-open interval from  (-pi,+pi]  to  [-pi,+pi)
		radians = -PI;
	}
	return radians;
}
