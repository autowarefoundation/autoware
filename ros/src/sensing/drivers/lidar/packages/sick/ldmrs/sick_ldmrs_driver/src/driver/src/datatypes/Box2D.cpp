//
// Box2D.cpp
//

#define _USE_MATH_DEFINES
#include <cmath>

#include "Box2D.hpp"
#include "../tools/errorhandler.hpp"
#include "Polygon2D.hpp"
#include <sstream>

namespace datatypes
{


// ////////////////////////////////////////////////////////////

// Define this macro so that checking of correct numeric ranges will
// be done at construction time of the box values.
//#define ASSERT_NUMERIC_RANGES

Box2D::Box2D()
	: m_center(0, 0)
	, m_size(0, 0)
	, m_rotation(0)
{
	m_datatype = Datatype_Box2D;
}

Box2D::Box2D(const Point2D& center, const Point2D& size, value_type rotation)
	: m_center(center)
	, m_size(size)
	, m_rotation(rotation)
{
	assert((m_size.getX() >= 0.0) || isNaN(m_size.getX()));
	assert((m_size.getY() >= 0.0) || isNaN(m_size.getY()));
#ifdef ASSERT_NUMERIC_RANGES
	verifyNumericRanges();
#endif
	m_datatype = Datatype_Box2D;
}

Box2D::Box2D(value_type x_center, value_type y_center, value_type x_size, value_type y_size, value_type rotation)
	: m_center(x_center, y_center)
	, m_size(x_size, y_size)
	, m_rotation(rotation)
{
	assert((m_size.getX() >= 0.0) || isNaN(m_size.getX()));
	assert((m_size.getY() >= 0.0) || isNaN(m_size.getY()));
#ifdef ASSERT_NUMERIC_RANGES
	verifyNumericRanges();
#endif
	m_datatype = Datatype_Box2D;
}


void Box2D::setSize(const Point2D& p)
{
	m_size = p;
	verifyNumericRanges(); // not yet active to not break existing code!
}
void Box2D::setSize(value_type x_length, value_type y_width)
{
	m_size.setXY(x_length, y_width);
#ifdef ASSERT_NUMERIC_RANGES
	verifyNumericRanges();
#endif
}

void Box2D::setRotation(value_type r)
{
	m_rotation = r;
#ifdef ASSERT_NUMERIC_RANGES
	verifyNumericRanges();
#endif
}

void Box2D::moveBy(const Point2D& pointvalues)
{
	m_center += pointvalues;
}

Box2D Box2D::movedBy(const Point2D& pointvalues) const
{
	return Box2D(m_center + pointvalues, m_size, m_rotation);
}

Polygon2D Box2D::toPolygon() const
{
	// This is the delta between center and edges
	value_type deltaX = m_size.getX() * 0.5f;
	value_type deltaY = m_size.getY() * 0.5f;

	// The cos/sin rotation factors
	value_type dCos = std::cos(m_rotation);
	value_type dSin = std::sin(m_rotation);

	// The dimensions after the rotation
	value_type XCos = deltaX * dCos, XSin = deltaX * dSin;
	value_type YCos = deltaY * dCos, YSin = deltaY * dSin;

	// Create the resulting points
	Polygon2D poly;
	poly.resize(5);
	// Rotate each edge according to the m_rotation
	poly[0] = m_center + Point2D(   XCos - YSin,    XSin + YCos);
	poly[1] = m_center + Point2D(   XCos + YSin,    XSin - YCos);
	poly[2] = m_center + Point2D( - XCos + YSin,  - XSin - YCos);
	poly[3] = m_center + Point2D( - XCos - YSin,  - XSin + YCos);
	poly[4] = poly[0];
	return poly;
}

Box2D Box2D::toBoundingBox() const
{
	// This is the delta between center and edges
	value_type deltaX = m_size.getX() * 0.5f;
	value_type deltaY = m_size.getY() * 0.5f;

	// The absolute values of the cos/sin rotation
	value_type dCos = std::abs(std::cos(m_rotation));
	value_type dSin = std::abs(std::sin(m_rotation));

	// The maximum x- and y-size of the rotated points. We use only
	// absolute values because we want the maximum anyway.
	value_type xmax = deltaX * dCos + deltaY * dSin;
	value_type ymax = deltaX * dSin + deltaY * dCos;

	// The resulting size of the bounding box
	Point2D size(2.0f * xmax, 2.0f * ymax);

	// Center stays identical, only size is changed
	return Box2D(m_center, size, 0.0f);
}

std::pair<Box2D::value_type, Box2D::value_type>
Box2D::getBoundingAngles() const
{
	// This function calculates a low and a high boundary angle for
	// all edges of the given (rotated) Box2D. The returned FloatPair
	// has the component "first" for the lower bounding angle, and
	// "second" for the upper bounding angle. (Note: This ordering is
	// swapped compared to the scan point ordering!)

	// Need to check whether the origin is inside the box
	if (containsPoint(Point2D(0, 0)))
	{
		// Origin is inside. Then return the full interval.
		return std::make_pair(-PI, PI);
	}

	// The usual case: The box does not contain the origin. Then we
	// look for the min and max angles of the edges.

	Polygon2D poly = toPolygon();
	value_type minangle = poly[0].angle();
	value_type maxangle = minangle;
	for (unsigned k = 1; k < 4; ++k)
	{
		value_type pointangle = poly[k].angle();

		if (pointangle < minangle)
			minangle = pointangle;

		if (pointangle > maxangle)
			maxangle = pointangle;
	}

	return std::make_pair(minangle, maxangle);
}

bool Box2D::containsPoint(const Point2D& point) const
{
	if (fuzzyCompare(m_rotation, 0.0))
	{
		// Rotation is zero, hence we can directly check this in
		// cartesian coordinates.
		value_type pointX = point.getX() - m_center.getX();
		value_type pointY = point.getY() - m_center.getY();

		// compare position to half size
		return (std::abs(pointX) <=  0.5f * m_size.getX())
			   && (std::abs(pointY) <=  0.5f * m_size.getY());
	}
	else
	{
		// 2D coordinate transformation as proposed by Uni Ulm.

		// Move coordinate system to center of the box
		value_type deltaX = point.getX() - m_center.getX();
		value_type deltaY = point.getY() - m_center.getY();

		// If any of the X or Y components are outside of the
		// "manhatten" diagonal bounding box, the point cannot be
		// inside the box (and we don't even have to calculate the
		// rotated point).
		value_type half_sizeX = 0.5f * m_size.getX();
		value_type half_sizeY = 0.5f * m_size.getY();
		if (std::max(std::abs(deltaX), std::abs(deltaY)) > half_sizeX + half_sizeY)
			return false;

		// Rotate by -psi
		value_type dCos = std::cos(m_rotation);
		value_type dSin = std::sin(m_rotation);
		value_type pointX =  dCos * deltaX  +  dSin * deltaY;
		value_type pointY = -dSin * deltaX  +  dCos * deltaY;

		// compare position to half size
		return ((std::abs(pointX) <=  half_sizeX)
				&& (std::abs(pointY) <=  half_sizeY));
	}
}


// ////////////////////////////////////////////////////////////

template<typename floatT>
static inline floatT
distanceFromStraightBox(floatT pointX, floatT pointY, floatT boxSizeX, floatT boxSizeY)
{
	// Rotation is zero, hence we can directly check this in
	// cartesian coordinates.
	floatT pointXAbs = std::abs(pointX);
	floatT pointYAbs = std::abs(pointY);

	// Above and right of the box?
	if (pointXAbs > boxSizeX && pointYAbs > boxSizeY)
	{
		// Yes, return distance to edge
		return hypot(pointXAbs - boxSizeX, pointYAbs - boxSizeY);
	}
	// Right of the box?
	if (pointXAbs > boxSizeX)
		// Yes, return the x-coordinate difference
		return pointXAbs - boxSizeX;
	// Above the box?
	if (pointYAbs > boxSizeY)
		// Yes, return the y-coordinate difference
		return pointYAbs - boxSizeY;

	// We are obviously inside the box
	return std::min(boxSizeX - pointXAbs, boxSizeY - pointYAbs);
}

// Implementation of the distanceFromOutline algorithm.
template<class PointT>
static inline Box2D::value_type distanceFromOutline(const Box2D& box, const PointT& point)
{
	Box2D::value_type boxSizeX = box.getSize().getX() * 0.5;
	Box2D::value_type boxSizeY = box.getSize().getY() * 0.5;

	if (fuzzyCompare(box.getRotation(), 0.0))
	{
		// Rotation is zero, hence we can directly check this in
		// cartesian coordinates.
		return distanceFromStraightBox(point.getX() - box.getCenter().getX(),
									   point.getY() - box.getCenter().getY(),
									   boxSizeX, boxSizeY);
	}
	else
	{
		// 2D coordinate transformation

		// Move coordinate system to center of the box
		Box2D::value_type deltaX = point.getX() - box.getCenter().getX();
		Box2D::value_type deltaY = point.getY() - box.getCenter().getY();

		// Rotate by -psi
		Box2D::value_type dCos = std::cos(box.getRotation());
		Box2D::value_type dSin = std::sin(box.getRotation());
		// and continue as if this were a straight box
		return distanceFromStraightBox( dCos * deltaX  +  dSin * deltaY,
										-dSin * deltaX  +  dCos * deltaY,
										boxSizeX, boxSizeY);
	}
}


// Calculate the mean of many distances. This is a template so that it
// works for both Point2D and Scanpoints.
template<class IteratorT>
static inline Box2D::value_type
distanceFromOutline(const Box2D& box, const IteratorT& begin, const IteratorT& end)
{
	std::vector<Point2D>::size_type numPoints = 0;
	Box2D::value_type result = 0.0;
	for (IteratorT iter = begin; iter != end; ++iter)
	{
		numPoints++;
		result += distanceFromOutline(box, *iter);
	}
	if (numPoints > 0)
	{
		return result / Box2D::value_type(numPoints);
	}
	else
	{
		return 0.0;
	}
}



// ////////////////////////////////////////////////////////////

// Template function for the x-component of a rotational
// transform. This is a template because this works for both the
// Point2D and a ScanPoint.
template<class PointT>
static inline double rotate_x(const PointT& p, double dCos, double dSin)
{
	return dCos * p.getX() + dSin * p.getY();
}
// Template function for the y-component of a rotational
// transform. This is a template because this works for both the
// Point2D and a ScanPoint.
template <class PointT>
static inline double rotate_y(const PointT& p, double dCos, double dSin)
{
	return -dSin * p.getX() + dCos * p.getY();
}

// Template function that implements the three overloaded methods
// below. A template function makes it very easy to enable using the
// vector<ScanPoint> as well, which is useful if we process actual
// scanpoints.
template<class IterT> static Box2D
calcOrientatedBox(double orientation, const IterT& begin, const IterT& end)
{
	Box2D result;

	float dCos = cos(orientation);
	float dSin = sin(orientation);

	// Is the sequence empty? If assertions are active, throw the
	// assertion here. Otherwise silently return a null box.
	assert(begin != end);
	if (begin == end)
	{
		return result;
	}

	// Set the min/max values to the coordinates of the first point
	float minX = rotate_x(*begin, dCos, dSin);
	float maxX = minX;
	float minY = rotate_y(*begin, dCos, dSin);
	float maxY = minY;

	// Iterate through the rest of the points to find the actual min
	// and max values.
	for (IterT iter = begin + 1; iter != end; iter++)
	{
		float x = rotate_x(*iter, dCos, dSin);
		float y = rotate_y(*iter, dCos, dSin);
		if (x < minX) minX = x;
		if (x > maxX) maxX = x;
		if (y < minY) minY = y;
		if (y > maxY) maxY = y;
	}

	// The center point is the mean of the bounds.
	float rotated_center_x = 0.5 * (maxX + minX);
	float rotated_center_y = 0.5 * (maxY + minY);

	// Rotate the center point back to the original coordinate system.
	result.setCenter(dCos * rotated_center_x - dSin * rotated_center_y,
					 dSin * rotated_center_x + dCos * rotated_center_y);
	// Size is even easier: Difference between max and min.
	result.setSize(maxX - minX,
				   maxY - minY);
	// Orientation was already given by the caller.
	result.setRotation(orientation);

	return result;
}

// Returns an orientated bounding box for the given list of points.
Box2D Box2D::orientatedBox(value_type orientation, const Polygon2D& poly)
{
	return calcOrientatedBox(orientation, poly.begin(), poly.end());
}

// Returns an orientated bounding box for the given list of points.
Box2D Box2D::orientatedBox(value_type orientation,
						   const std::vector<Point2D>::const_iterator& begin,
						   const std::vector<Point2D>::const_iterator& end)
{
	return calcOrientatedBox(orientation, begin, end);
}


Box2D Box2D::orientatedBox(value_type orientation_rad,
						   const std::vector<Point2D>& points)
{
	return calcOrientatedBox(orientation_rad, points.begin(), points.end());
}

// Returns an orientated bounding box for the given list of points.
// Box2D Box2D::orientatedBox(value_type orientation,
// 						   const std::vector<ScanPoint>::const_iterator& begin,
// 						   const std::vector<ScanPoint>::const_iterator& end)
// {
// 	return calcOrientatedBox(orientation, begin, end);
// }
// 
// Box2D Box2D::orientatedBox(value_type orientation_rad,
// 						   const SegmentIterator& begin,
// 						   const SegmentIterator& end)
// {
// 	return calcOrientatedBox(orientation_rad, begin, end);
// }





void Box2D::verifyNumericRanges()
{
#ifdef __GNUC__
	// Allow correctRange to be set but unused in case everything works well.
	// This is needed for gcc v4.6 and up when using -Wall
	bool __attribute__((unused)) correctRange = true;
#else
	bool correctRange = true;
#endif
	// Check our assumptions about the m_size
	if (m_size.getX() < 0 || m_size.getY() < 0)
	{
		printWarning("Size of Box2D was given as negative value (" +
						 ::toString(m_size.getX(), 2) + "," + ::toString(m_size.getY(), 2) +
						 ") - silently using the absolute value instead.");
		m_size.setX(std::abs(m_size.getX()));
		m_size.setY(std::abs(m_size.getY()));
		correctRange = false;
		// This is probably better than throwing an exception.
		//throw std::out_of_range("Size of Box2D negative - this is an invalid Box2D.");
	}

	if (isNaN(m_size.getX()))
	{
		m_size.setX(0);
		printWarning("Size.getX() of Box2D was given as NaN value - silently using Zero instead.");
		correctRange = false;
	}
	if (isNaN(m_size.getY()))
	{
		m_size.setY(0);
		printWarning("Size.getY() of Box2D was given as NaN value - silently using Zero instead.");
		correctRange = false;
	}
	if (isNaN(m_rotation))
	{
		m_rotation = 0;
		printWarning("Rotation of Box2D was given as NaN value - silently using Zero instead.");
		correctRange = false;
	}
	value_type normd_rot = normalizeRadians(m_rotation);
	if (!fuzzyCompare(normd_rot, m_rotation))
	{
		printWarning("Rotation of Box2D (" + ::toString(m_rotation, 2) + ") was outside of its [-pi,pi] definition range - silently normalizing it back into that range (" +
							::toString(normd_rot, 2) + ").");
		m_rotation = normd_rot;
		correctRange = false;
	}
#ifdef ASSERT_NUMERIC_RANGES
	assert(correctRange);
#endif
}

std::string Box2D::toString() const
{
	std::string text;
	
	text = "[ Center=" + getCenter().toString() +
			" Size=" + getSize().toString() +
			" Rotation=" + ::toString(getRotation(), 2) +
			"]";
	
	return text;
}

}	// namespace datatypes

