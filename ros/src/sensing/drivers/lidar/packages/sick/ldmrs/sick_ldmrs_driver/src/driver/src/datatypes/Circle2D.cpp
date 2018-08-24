//
// Circle2D.cpp
//

#include <cmath>

#include "Circle2D.hpp"
#include "../tools/errorhandler.hpp"

namespace datatypes
{


Circle2D::Circle2D()
	: base_class()
{
	m_datatype = Datatype_Circle2D;
}

Circle2D::Circle2D(const Point2D& center, value_type radius)
	: base_class(center, Point2D(radius, radius), 0)
{
	m_datatype = Datatype_Circle2D;
}

Circle2D::Circle2D(value_type x_center, value_type y_center, value_type radius)
	: base_class(x_center, y_center, radius, radius, 0)
{
	m_datatype = Datatype_Circle2D;
}

void Circle2D::setRadius(value_type radius)
{
	base_class::setRadius(radius, radius);
}

bool Circle2D::containsPoint(const Point2D& point) const
{
	// Move coordinate system to center of the ellipse
	value_type deltaX = point.getX() - m_center.getX();
	value_type deltaY = point.getY() - m_center.getY();

	// If any of the X or Y components are outside of the radius, the
	// point cannot be inside the circle.
	if (std::max(std::abs(deltaX), std::abs(deltaY)) > m_radius.getX())
		return false;

	return hypot(deltaX, deltaY) < m_radius.getX();
}

}	// namespace datatypes

