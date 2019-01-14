//
// Ellipse2D.cpp
//

#define _USE_MATH_DEFINES
#include <cmath>

#include "Ellipse2D.hpp"
#include "Circle2D.hpp"
#include "../tools/MathToolbox.hpp"
#include "../tools/errorhandler.hpp"

namespace datatypes
{

// ////////////////////////////////////////////////////////////

Ellipse2D::Ellipse2D()
	: m_center(0, 0)
	, m_radius(0, 0)
	, m_rotation(0)
{
	m_datatype = Datatype_Circle2D;
}

Ellipse2D::Ellipse2D(const Point2D& center, const Point2D& radius, value_type rotation)
	: m_center(center)
	, m_radius(radius)
	, m_rotation(rotation)
{
	assert((m_radius.getX() >= 0.0) || isNaN(m_radius.getX()));
	assert((m_radius.getY() >= 0.0) || isNaN(m_radius.getY()));
	verifyNumericRanges();
	m_datatype = Datatype_Circle2D;
}

Ellipse2D::Ellipse2D(value_type x_center, value_type y_center, value_type x_radius, value_type y_radius, value_type rotation)
	: m_center(x_center, y_center)
	, m_radius(x_radius, y_radius)
	, m_rotation(rotation)
{
	assert((m_radius.getX() >= 0.0) || isNaN(m_radius.getX()));
	assert((m_radius.getY() >= 0.0) || isNaN(m_radius.getY()));
	verifyNumericRanges();
	m_datatype = Datatype_Circle2D;
}


void Ellipse2D::setRadius(const Point2D& p)
{
	m_radius = p;
	verifyNumericRanges();
}
void Ellipse2D::setRadius(value_type x_length, value_type y_width)
{
	m_radius.setXY(x_length, y_width);
	verifyNumericRanges();
}

void Ellipse2D::setRotation(value_type r)
{
	m_rotation = r;
	verifyNumericRanges();
}

bool Ellipse2D::containsPoint(const Point2D& point) const
{
	if (fuzzyCompare(m_rotation, 0.0))
	{
		// Rotation is zero, hence we can directly check this in
		// cartesian coordinates.
		value_type pointX = point.getX() - m_center.getX();
		value_type pointY = point.getY() - m_center.getY();

		// compare position to radius
		return (sqr(pointX / m_radius.getX()) + sqr(pointY / m_radius.getY())) < 1;
	}
	else
	{
		// 2D coordinate transformation as proposed by Uni Ulm.

		// Move coordinate system to center of the ellipse
		value_type deltaX = point.getX() - m_center.getX();
		value_type deltaY = point.getY() - m_center.getY();

		// If any of the X or Y components are outside of the
		// "manhatten" diagonal bounding box, the point cannot be
		// inside the ellipse (and we don't even have to calculate the
		// rotated point).
		if (std::max(std::abs(deltaX), std::abs(deltaY)) > m_radius.getX() + m_radius.getY())
			return false;

		// Rotate by -psi
		value_type dCos = std::cos(m_rotation);
		value_type dSin = std::sin(m_rotation);
		value_type pointX =  dCos * deltaX  +  dSin * deltaY;
		value_type pointY = -dSin * deltaX  +  dCos * deltaY;

		// compare position to radius
		return (sqr(pointX / m_radius.getX()) + sqr(pointY / m_radius.getY())) < 1;
	}
}

// ////////////////////////////////////////////////////////////


void Ellipse2D::verifyNumericRanges()
{
	// Check our assumptions about the m_radius
	if (m_radius.getX() < 0 || m_radius.getY() < 0)
	{
		m_radius.setX(std::abs(m_radius.getX()));
		m_radius.setY(std::abs(m_radius.getY()));
		printWarning("Radius of Ellipse2D was given as negative value - silently using the absolute value instead.");
		// This is probably better than throwing an exception.
		//throw std::out_of_range("Radius of Ellipse2D negative - this is an invalid Ellipse2D.");
	}

	if (isNaN(m_radius.getX()))
	{
		m_radius.setX(0);
		printWarning("Radius.getX() of Ellipse2D was given as NaN value - silently using Zero instead.");
	}
	if (isNaN(m_radius.getY()))
	{
		m_radius.setY(0);
		printWarning("Radius.getY() of Ellipse2D was given as NaN value - silently using Zero instead.");
	}
	if (isNaN(m_rotation))
	{
		m_rotation = 0;
		printWarning("Rotation of Ellipse2D was given as NaN value - silently using Zero instead.");
	}
	value_type normd_rot = normalizeRadians(m_rotation);
	if (!fuzzyCompare(normd_rot, m_rotation))
	{
		printWarning("Rotation of Ellipse2D (" + ::toString(m_rotation, 2) +
					") was outside of its [-pi,pi] definition range - silently normalizing it back into that range (" +
					::toString(normd_rot, 2) + ").");
		m_rotation = normd_rot;
	}
}


std::string Ellipse2D::toString() const
{
	std::string text;
	
	text = "[ Center=" + getCenter().toString() +
			" Radius=" + getRadius().toString() +
			" Rotation=" + ::toString(getRotation(), 2) +
			"]";
	return text;
}

}	// namespace datatypes
