//
// Point2D.cpp
// A point in the 2-D plane.
//

#include <cmath>
#include "Point2D.hpp"
#include "../BasicDatatypes.hpp"

namespace datatypes
{
	
Point2D::value_type Point2D::dist( const Point2D & point ) const
{
	return hypot(m_x - point.m_x, m_y - point.m_y);
}

Point2D::value_type Point2D::distSquare( const Point2D & point ) const
{
	const value_type x = getX() - point.getX();
	const value_type y = getY() - point.getY();
	return x * x + y * y;
}

Point2D Point2D::fromPolar(value_type r, value_type angle)
{
	Point2D p;
	p.setPolar(r, angle);
	return p;
}


void Point2D::rotate(value_type angle)
{
	*this = rotated(angle);
}

Point2D Point2D::rotated(value_type angle_rad) const
{
	value_type dCos = cos(angle_rad);
	value_type dSin = sin(angle_rad);
	return Point2D(m_x * dCos - m_y * dSin,
				   m_x * dSin + m_y * dCos);
}

Point2D Point2D::normalized() const
{
	Point2D result(*this);
	result.normalize();
	return result;
}

void Point2D::normalize()
{
	if (isZero())
		// Vector has zero length. Such a vector will be left unchanged.
		return;

	// The division below will be done in float, not in double! Hence,
	// we must retrieve the length in float already as well.
	value_type len = dist();

	// If isZero() was false above, the distance cannot be zero
	// anyway, so checking for this would only result in unreachable
	// code.
	assert (!fuzzyCompare(len, value_type(0.0)));

	*this /= len;
}

/**
 * Wandlung in einen String.
 * digits = Anzahl der Nachkommastellen.
 */
std::string Point2D::toString(UINT16 digits) const
{
	std::string text = "(" + ::toString(getX(), (int)digits) + ", " + ::toString(getY(), (int)digits) + ")";
	return text;
}


}	// namespace datatypes
