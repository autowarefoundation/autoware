// Polygon2D
//
//


#include <cmath>

#include "Polygon2D.hpp"

#include "Box2D.hpp"
#include "Circle2D.hpp"
#include "Ellipse2D.hpp"
#include "Line2D.hpp"
#include <iterator>
#include <sstream>
#include <stdexcept>	// for throw

namespace datatypes
{

// ////////////////////////////////////////////////////////////

Polygon2D::Polygon2D()
{
}

Polygon2D::Polygon2D(const Point2D& p1)
{
	push_back(p1);
}

Polygon2D::Polygon2D(const Point2D& p1, const Point2D& p2)
{
	push_back(p1);
	push_back(p2);
}
Polygon2D::Polygon2D(const Point2D& p1, const Point2D& p2, const Point2D& p3)
{
	push_back(p1);
	push_back(p2);
	push_back(p3);
}

Polygon2D::Polygon2D(const Point2D& p1, const Point2D& p2, const Point2D& p3, const Point2D& p4)
{
	push_back(p1);
	push_back(p2);
	push_back(p3);
	push_back(p4);
}

/*
Polygon2D::Polygon2D(const Line2D& p)
{
	push_back(p.getP1());
	push_back(p.getP2());
}
*/

Polygon2D::Polygon2D(const base_class& other_vector)
	: base_class(other_vector)
{
}

/*
Polygon2D::Polygon2D(const std::string& polygonAsString)
{
	fromString(polygonAsString);
}
*/

Polygon2D& Polygon2D::append (const Polygon2D& other )
{
	base_class::insert(base_class::end(), other.begin(), other.end());
	return *this;
}

Polygon2D& Polygon2D::append(const Point2D& point)
{
	push_back(point);
	return *this;
}

Polygon2D& Polygon2D::append(floatingpoint_type x, floatingpoint_type y)
{
	push_back(Point2D(x, y));
	return *this;
}

double Polygon2D::getArea() const
{
	if (empty())
		return 0.0;

	double result(0.0);
	for (size_type i = 0; i < size() - 1; ++i)
	{
		result += at(i).getX() * at(i + 1).getY() - at(i + 1).getX() * at(i).getY();
	}

	// Automatically close polygon. This won't harm, if polygon is already closed.
	result += at(size() - 1).getX() * at(0).getY() - at(0).getX() * at(size() - 1).getY();

	// Calculate absolute value in order to work with counter-clockwise polygon description.
	return std::abs(0.5 * result);
}

Point2D Polygon2D::getCenterOfGravity() const
{
	Point2D centerOfGravity;

	if (empty())
	{
		return centerOfGravity;
	}

	for (Polygon2D::const_iterator edgeIter = begin(); edgeIter != end(); ++edgeIter)
	{
		centerOfGravity += *edgeIter;
	}

	centerOfGravity /= size();

	return centerOfGravity;
}

bool Polygon2D::isClosed() const
{
	return !empty() && (front() == back());
}

////////////////////////////////////////////////////////////////////////////
//
// static functions
//
////////////////////////////////////////////////////////////////////////////

Polygon2D Polygon2D::fromCircle( const Point2D& center, const floatingpoint_type radius, const UINT32 samplingPoints )
{
	const Circle2D circle(center, radius);
	return fromEllipse(circle, samplingPoints);
}

Polygon2D Polygon2D::fromEllipse( const Point2D& center, const floatingpoint_type a, const floatingpoint_type b, const floatingpoint_type angle, const UINT32 samplingPoints )
{
	const Ellipse2D ellipse(center, Point2D(a, b), angle);
	return fromEllipse(ellipse, samplingPoints);
}

Polygon2D Polygon2D::fromEllipse(const Ellipse2D& ellipse, const UINT32 samplingPoints)
{
	Polygon2D result = fromArc(ellipse, 0, 2.0 * M_PI, samplingPoints);

	// Since we know the result should be a closed circle, we can
	// force the final point to be identical to the first one here.
	if (result.size() > 1)
		result.back() = result.front();

	return result;
}

Polygon2D Polygon2D::fromArc(const Ellipse2D& ellipse,
							 const floatingpoint_type startAngle, const floatingpoint_type endAngle,
							 const UINT32 samplingPoints, const bool clockwise)
{
	floatingpoint_type end = endAngle;

	// Modify the end angle so that we get the expected direction
	if (clockwise == true)
	{
		// clockwise direction (descending angle values from start to end)
		while (startAngle < end)
		{
			end -= 2.0 * PI;
		}
	}
	else
	{
		// counter-clockwise direction (ascending angle values from start to end)
		while (end < startAngle)
		{
			end += 2.0 * PI;
		}
	}

	return fromArc(ellipse, startAngle, end, samplingPoints);
}

Polygon2D Polygon2D::fromArc(const Ellipse2D& ellipse,
							 const floatingpoint_type startAngle, const floatingpoint_type endAngle,
							 const UINT32 samplingPoints)
{
	Polygon2D polygon;

	// Check if we have at least 2 sampling points
	if (samplingPoints < 2)
	{
		// Not enough points! Return an empty output.
		return polygon;
	}

	if (fabs(endAngle - startAngle) > double(2.0 * M_PI + 1e-3)) // allow 10^-3 epsilon
	{
		throw std::invalid_argument("Polygon2D::fromArc: Angle difference is too large! Is " + ::toString(fabs(endAngle - startAngle), 2) + " but must be at most 2*pi.");
	}

	const floatingpoint_type sinAngle(std::sin(ellipse.getRotation()));
	const floatingpoint_type cosAngle(std::cos(ellipse.getRotation()));
	const floatingpoint_type step = (endAngle - startAngle) / floatingpoint_type(samplingPoints - 1);

	floatingpoint_type t = startAngle;
	for (UINT32 p = 0 ; p < samplingPoints; p++)
	{
		// Last point? Then force the angle exactly on the endAngle
		if (p == (samplingPoints - 1))
		{
			t = endAngle;
		}

		Point2D point;
		point.setX(ellipse.getRadius().getX() * std::cos(t) * cosAngle - ellipse.getRadius().getY() * std::sin(t) * sinAngle);
		point.setY(ellipse.getRadius().getX() * std::cos(t) * sinAngle + ellipse.getRadius().getY() * std::sin(t) * cosAngle);
		point += ellipse.getCenter();
		polygon.push_back(point);
		t += step;
	}

	return polygon;
}

const char* Polygon2D::getSeparatorCharacters()
{
	return "()[],; \t\"'";
}

Polygon2D Polygon2D::rhombus(const Point2D& center, const floatingpoint_type radius)
{
	Polygon2D contour;
	contour.push_back(Point2D(center.getX() - radius, center.getY()));
	contour.push_back(Point2D(center.getX(), center.getY() - radius));
	contour.push_back(Point2D(center.getX() + radius, center.getY()));
	contour.push_back(Point2D(center.getX(), center.getY() + radius));
	contour.push_back(Point2D(center.getX() - radius, center.getY()));

	return contour;
}

Polygon2D Polygon2D::createRectangle(const Point2D& lowerLeft, const Point2D& upperRight)
{
	return Polygon2D()
		   .append(lowerLeft)
		   .append(Point2D(lowerLeft.getX(), upperRight.getY()))
		   .append(upperRight)
		   .append(Point2D(upperRight.getX(), lowerLeft.getY()));
}

/*
void Polygon2D::fromString(const std::string& polyString)
{
	clear();

	typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
	tokenizer listOfTokens(polyString, boost::char_separator<char>(getSeparatorCharacters()));
	for (tokenizer::const_iterator iter = listOfTokens.begin();
		 iter != listOfTokens.end(); ++iter)
	{
		floatingpoint_type x = boost::lexical_cast<floatingpoint_type>(*iter);
		++iter;
		if (iter == listOfTokens.end())
			throw std::runtime_error("When parsing a Polygon2D string, the last point only has a x component.");
		floatingpoint_type y = boost::lexical_cast<floatingpoint_type>(*iter);
		this->push_back(Point2D(x, y));
	}
}
*/

Box2D Polygon2D::getBoundingBox() const
{
	if (empty())
	{
		return Box2D();
	}

	Polygon2D::const_iterator iter = begin();
	floatingpoint_type minX = iter->getX();
	floatingpoint_type minY = iter->getY();
	floatingpoint_type maxX = minX;
	floatingpoint_type maxY = minY;

	++iter;
	for ( ; iter != end(); ++iter)
	{
		const floatingpoint_type x = iter->getX();
		const floatingpoint_type y = iter->getY();
		if (x < minX) minX = x;
		if (x > maxX) maxX = x;
		if (y < minY) minY = y;
		if (y > maxY) maxY = y;
	}
	return Box2D(0.5 * (maxX + minX),
				 0.5 * (maxY + minY),
				 maxX - minX,
				 maxY - minY);
}

std::pair<Polygon2D::floatingpoint_type, Polygon2D::floatingpoint_type> Polygon2D::getBoundingAngles() const
{
	if (empty())
		return std::make_pair(0, 0);

	// Need to check whether the origin is inside the polygon. BUT: If
	// the origin is directly on the boundary, containsPoint() may or
	// may not return true. For this reason we must watch out for the
	// case of points on the negative x axis below (if this test
	// returned false here).
	if (containsPoint(Point2D(0, 0)))
	{
		// Origin is inside. Then return the full interval.
		return std::make_pair(-PI, PI);
	}

	// The usual case: The polygon does not contain the origin. Then we
	// look for the min and max angles of the edges.

	Polygon2D::const_iterator iter = begin();
	floatingpoint_type minangle = iter->angle();
	floatingpoint_type maxangle = minangle;

	bool gotPointOnNegativeX = false;
	for ( ; iter != end(); ++iter)
	{
		if (fuzzyCompare(iter->getY(), 0.0)
			&& (iter->getX() < 0.0))
		{
			// Special case for points on the negative x axis: We do
			// not know (yet) whether we should record this as +pi or
			// -pi. We will decide on this afterwards.
			gotPointOnNegativeX = true;
		}
		else
		{
			const floatingpoint_type pointangle = iter->angle();

			if (pointangle < minangle)
				minangle = pointangle;

			if (pointangle > maxangle)
				maxangle = pointangle;
		}
	}

	// Did we also have at least one point on the negative X axis?
	// Either add this as -pi or +pi, or set both angles to -pi/pi
	// because we are not sure.
	if (gotPointOnNegativeX)
	{
		if (std::max(minangle, maxangle) <= 0.0f)
			minangle = -M_PI;
		else if (std::min(minangle, maxangle) >= 0.0f)
			maxangle = M_PI;
		else
		{
			minangle = -M_PI;
			maxangle = M_PI;
		}
	}

	return std::make_pair(minangle, maxangle);
}

/**
 * This is a Point-in-polygon test using ray-casting algorithm (Jordan
 * curve theorem). It is really fast and also works for non-convex and
 * self-intersecting polygons.
 *
 * However, Points which are directly on one boundary are classified
 * as either inside or outside, depending on which edge of the polygon
 * this is. In other words, this method cannot be used to find points
 * which are exactly on the edge of a polygon! (Citation from the URL
 * below: &quot;If you want to know when a point is exactly on the
 * boundary, you need another program. This is only one of many
 * functions that PNPOLY lacks; it also doesn't predict tomorrow's
 * weather. You are free to extend PNPOLY's source code.&quot;)
 *
 * Algorithm: From the test point a semi-finite ray is run through the
 * polygon. While doing so the number of edge crossings are
 * counted. If the number of crossings is even, the test point is
 * located outside of the polygon. If the number is odd, the point is
 * inside. (Jordan curve theorem)
 *
 * The polygon may be closed explicitly or implicitly, both will
 * work. I.e., if the polygon contains an additional final point
 * identical to the first one, this algorithms works correctly, but it
 * also works correctly without such an additional final point.
 *
 * As for multiple components and holes:
 *
 * - The polygon may contain multiple separate components, and/or
 * holes, provided that you separate the components and holes with a
 * (0,0) point, as follows.
 *
 *   -# First, include a (0,0) point.
 *   -# Then include the first component' points, repeating its first point after the last point.
 *   -# Include another (0,0) point.
 *   -# Include another component or hole, repeating its first point after the last point.
 *   -# Repeat the above two steps for each component and hole.
 *   -# Include a final (0,0) point.
 *
 * - Each component or hole's points may be listed either clockwise
 * or counter-clockwise.
 *
 * - If there is only one connected component, then it is optional to
 * repeat the first point at the end. It's also optional to surround
 * the component with zero points.
 *
 * C-Code was borrowed from:
 * http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
 * See there for further explanation of the border cases.
 */
bool Polygon2D::containsPoint(const Point2D& point) const
{
	// Point-in-polygon test using ray-casting algorithm
	//
	// C-Code was borrowed from:
	// http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
	//
	// Be careful: Don't fiddle with the code below unless you know
	// exactly what you're doing!
	//
	// Algorithm: From the test point a semi-finite ray is run through
	// the polygon. While doing so the number of edge crossings are
	// counted. If the number of crossings is even, the test point is
	// located outside of the polygon. If the number is odd, the point
	// is inside. (Jordan curve theorem)
	//
	// Also see: http://en.wikipedia.org/wiki/Point_in_polygon

	if (empty())
	{
		return false;
	}

	bool isInside(false);
	const Polygon2D& p = *this; // alias for shorter writing

	UINT32 i = 0;
	UINT32 j = (UINT32)(size() - 1);
	for (i = 0; i < size(); j = i++)
	{
		if (
			( (p[i].getY() > point.getY()) != (p[j].getY() > point.getY()) )
			&&
			( point.getX() < ( (p[j].getX() - p[i].getX())
							   * (point.getY() - p[i].getY())
							   / (p[j].getY() - p[i].getY())
							   + p[i].getX() ) )
		)
		{
			isInside = !isInside;
		}
	}

	return isInside;
}

void projectOntoAxis(const Point2D& axis, const Polygon2D& polygon,
					 Polygon2D::floatingpoint_type& min, Polygon2D::floatingpoint_type& max)
{
	min = max = (axis * polygon[0]);

	for (Polygon2D::const_iterator edge = polygon.begin(); edge != polygon.end(); ++edge)
	{
		const Polygon2D::floatingpoint_type dotProduct(axis * (*edge));
		min = std::min(min, dotProduct);
		max = std::max(max, dotProduct);
	}
}

Polygon2D::floatingpoint_type intervalDistance(const Polygon2D::floatingpoint_type minA, const Polygon2D::floatingpoint_type maxA,
		const Polygon2D::floatingpoint_type minB, const Polygon2D::floatingpoint_type maxB)
{
	if (minA < minB)
	{
		return minB - maxA;
	}
	else
	{
		return minA - maxB;
	}
}

bool Polygon2D::isColliding(const Polygon2D& p2) const
{
	Polygon2D both(*this);
	both.reserve(both.size() + p2.size());
	both.insert(both.end(), p2.begin(), p2.end());

	for (Polygon2D::const_iterator edge = both.begin(); edge != both.end(); ++edge)
	{
		Point2D axis(-edge->getY(), edge->getX());
		axis.normalize();

		floatingpoint_type minA, minB, maxA, maxB;
		projectOntoAxis(axis, *this, minA, maxA);
		projectOntoAxis(axis, p2, minB, maxB);

		if (intervalDistance(minA, maxA, minB, maxB) > 0)
		{
			return false;
		}
	}

	return true;
}


Polygon2D::base_class Polygon2D::isIntersecting(const Line2D& other) const
{
	base_class result;
	if (size() < 2 || other.isZero())
	{
		return result;
	}
	for (size_t k = 1; k < size(); ++k)
	{
		Point2D isect;
		Line2D currentline(operator[](k - 1), operator[](k));
		if (currentline.isIntersecting(other, &isect)
			== Line2D::LineIntersecting)
			result.push_back(isect);
	}
	return result;
}

Point2D::value_type Polygon2D::distanceToPoint(const Point2D& point) const
{
	if (size() < 2)
		return empty() ? 0 : point.dist(front());

	assert(size() >= 2);
	Point2D::value_type result = point.dist(front());
	for (size_t k = 1; k < size(); ++k)
	{
		Line2D line(operator[](k - 1), operator[](k));
		Point2D::value_type dist = line.distanceFromLineSegment(point);
		if (dist < result)
			result = dist;
	}
	return result;
}

Polygon2D Polygon2D::getSimplified() const
{
	if (size() < 3)
		return *this;
	Polygon2D result;
	result.reserve(size());

	// First point is always in the result
	result.push_back(front());

	for (size_type k = 1; k < size() - 1; ++k)
	{
		const Point2D& previous = result.back();
		const Point2D& current = operator[](k);
		const Point2D& next = operator[](k + 1);
		Line2D line(previous, next);
		// Only add the current point to the result if it doesn't lie
		// directly on the connection line between the previous output
		// point and the next one.
		if (!line.containsPoint(current))
			result.push_back(current);
	}

	// Last point is always in the result
	result.push_back(back());

	return result;
}

// ////////////////////////////////////////////////////////////


std::string Polygon2D::toString() const
{
	std::string text = "[ ";
	if (empty() == false)
	{
		for (size_type k = 0; k < size(); k++)
		{
			const Point2D& current = operator[](k);
			text += current.toString() + " ";
		}
	}
	text += "]";
	return text;
}

}	// namespace datatypes
