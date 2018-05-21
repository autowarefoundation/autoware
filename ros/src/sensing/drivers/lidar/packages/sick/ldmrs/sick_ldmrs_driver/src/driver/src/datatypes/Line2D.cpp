//
// Line2D.cpp
//

#include <cmath>

#include "Line2D.hpp"

#include "Point2D.hpp"
#include "Polygon2D.hpp"


namespace datatypes
{
	

Polygon2D Line2D::toPolygon2D() const
{
	return Polygon2D(first, second);
}

Polygon2D Line2D::toPolygon2D(unsigned samplePoints) const
{
	assert(samplePoints > 1);
	if (samplePoints <= 1)
		return Polygon2D(); // sanity check for when NDEBUG disables the assert() check

	Polygon2D result;
	result.resize(samplePoints);
	result.front() = first;
	result.back() = second;

	Point2D step = getDiff() / double(samplePoints - 1);
	for (unsigned k = 1; k < samplePoints - 1; ++k)
	{
		result[k] = first + step * double(k);
	}

	return result;
}

double Line2D::getInclination() const
{
	Point2D diff = getDiff();
	if (fuzzyCompare(diff.getX(), 0.0))
	{
		// vertical line
		if (first.getY() < second.getY())
			return M_PI_2; // pi/2
		else
			return -M_PI_2; // -pi/2
	}
	return atan2(diff.getY(), diff.getX());
}

Line2D Line2D::getUnitVector() const
{
	Point2D diff = getDiff();
	double thislength = getLength();

	return Line2D(first,
				  Point2D(first.getX() + diff.getX() / thislength,
						  first.getY() + diff.getY() / thislength));
}

std::string Line2D::toString()
{
	std::string text;
	
	text = "[" + getP1().toString() + " -> " +getP2().toString() + "]";
	
	return text;
}

bool Line2D::containsPoint(const Point2D& point) const
{
	double d = distanceFromLineSegment(point);
	return fuzzyCompare(d, 0.0);
}

double Line2D::distanceToPoint(const Point2D& point) const
{
	Point2D diff = point - projectOntoLine(point);
	return hypot(diff.getX(), diff.getY());
}

double dot_product(const Point2D& p1, const Point2D& p2)
{
	return p1.getX() * p2.getX() + p1.getY() * p2.getY();
}
Point2D Line2D::projectOntoLine(const Point2D& point) const
{
	Point2D line_vec = getDiff();
	double line_length = length();
	if (!fuzzyCompare(line_length, 0.0))
	{
		Point2D line_unitvec = line_vec / line_length;
		Point2D vec_to_point = point - first;
		double length_of_projection =
			dot_product(line_unitvec, vec_to_point);
		return first + line_unitvec * length_of_projection;
	}
	else
	{
		// Zero-length line; what should we do?
		return point;
	}
}

double Line2D::distanceFromLineSegment(const Point2D& point) const
{
// How do I find the distance from a point to a line?
//
//    Let the point be C (Cx,Cy) and the line be AB (Ax,Ay) to (Bx,By).
//    Let P be the point of perpendicular projection of C on AB.  The parameter
//    r, which indicates P's position along AB, is computed by the dot product
//    of AC and AB divided by the square of the length of AB:
//
//    (1)     AC dot AB
//        r = ---------
//            ||AB||^2
//
//    r has the following meaning:
//
//        r=0      P = A
//        r=1      P = B
//        r<0      P is on the backward extension of AB
//        r>1      P is on the forward extension of AB
//        0<r<1    P is interior to AB
//
//    The length of a line segment in d dimensions, AB is computed by:
//
//        L = sqrt( (Bx-Ax)^2 + (By-Ay)^2 + ... + (Bd-Ad)^2)
//
//    so in 2D:
//
//        L = sqrt( (Bx-Ax)^2 + (By-Ay)^2 )
//
//    and the dot product of two vectors in d dimensions, U dot V is computed:
//
//        D = (Ux * Vx) + (Uy * Vy) + ... + (Ud * Vd)
//
//    so in 2D:
//
//        D = (Ux * Vx) + (Uy * Vy)
//
//    So (1) expands to:
//
//            (Cx-Ax)(Bx-Ax) + (Cy-Ay)(By-Ay)
//        r = -------------------------------
//                          L^2
//
//    The point P can then be found:
//
//        Px = Ax + r(Bx-Ax)
//        Py = Ay + r(By-Ay)
//
//    And the distance from A to P = r*L.
//
//    Use another parameter s to indicate the location along PC, with the
//    following meaning:
//           s<0      C is left of AB
//           s>0      C is right of AB
//           s=0      C is on AB
//
//    Compute s as follows:
//
//            (Ay-Cy)(Bx-Ax)-(Ax-Cx)(By-Ay)
//        s = -----------------------------
//                        L^2
//
//
//    Then the distance from C to P = |s|*L.

	const double cx = point.getX();
	const double cy = point.getY();

	const double ax = getP1().getX();
	const double ay = getP1().getY();
	const double bx = getP2().getX();
	const double by = getP2().getY();

	const double r_numerator = (cx - ax) * (bx - ax) + (cy - ay) * (by - ay);
	const double r_denomenator = (bx - ax) * (bx - ax) + (by - ay) * (by - ay);
	const double r = r_numerator / r_denomenator;

// 	const double px = ax + r * (bx - ax);
// 	const double py = ay + r * (by - ay);

	const double s =  ((ay - cy) * (bx - ax) - (ax - cx) * (by - ay) ) / r_denomenator;

	const double distanceLine = std::abs(s) * sqrt(r_denomenator);
	double distanceSegment = -1.0;

	// (xx,yy) is the point on the lineSegment closest to (cx,cy)

// 	double xx = px;
// 	double yy = py;

	if ( (r >= 0) && (r <= 1) )
	{
		distanceSegment = distanceLine;
	}
	else
	{
		double dist1 = (cx - ax) * (cx - ax) + (cy - ay) * (cy - ay);
		double dist2 = (cx - bx) * (cx - bx) + (cy - by) * (cy - by);
		if (dist1 < dist2)
		{
// 			xx = ax;
// 			yy = ay;
			distanceSegment = sqrt(dist1);
		}
		else
		{
// 			xx = bx;
// 			yy = by;
			distanceSegment = sqrt(dist2);
		}
	}

	return distanceSegment;
}


// Line intersection algorithm, copied from Graphics Gems II
#define SAME_SIGNS(a, b) ((a) * (b) >= 0)
template<typename FloatT>
static bool line2d_intersect(FloatT x1, FloatT y1, FloatT x2, FloatT y2,
							 FloatT x3, FloatT y3, FloatT x4, FloatT y4)
{
	FloatT a1, a2, b1, b2, c1, c2; /* Coefficients of line eqns. */
	FloatT r1, r2, r3, r4;         /* 'Sign' values */

	a1 = y2 - y1;
	b1 = x1 - x2;
	c1 = x2 * y1 - x1 * y2;

	r3 = a1 * x3 + b1 * y3 + c1;
	r4 = a1 * x4 + b1 * y4 + c1;

	if ( r3 != 0 && r4 != 0 && SAME_SIGNS( r3, r4 ))
		return false;

	a2 = y4 - y3;
	b2 = x3 - x4;
	c2 = x4 * y3 - x3 * y4;

	r1 = a2 * x1 + b2 * y1 + c2;
	r2 = a2 * x2 + b2 * y2 + c2;

	if ( r1 != 0 && r2 != 0 && SAME_SIGNS( r1, r2 ))
		return false;

	return true;
}

Line2D::IntersectingType Line2D::isIntersecting(const Line2D& other, Point2D* point) const
{
	if (isZero() || other.isZero())
		return NotIntersecting;

	bool has_inside_intersection =
		line2d_intersect(first.getX(), first.getY(),
						 second.getX(), second.getY(),
						 other.first.getX(), other.first.getY(),
						 other.second.getX(), other.second.getY());

	bool is_vertical = fuzzyCompare(getDiff().getX(), 0.0);
	bool other_is_vertical = fuzzyCompare(other.getDiff().getX(), 0.0);
	IntersectingType result =
		has_inside_intersection ? LineIntersecting : OutsideIntersecting;
	Point2D intersectPoint;

	if (is_vertical && other_is_vertical)
		result = NotIntersecting;
	else if (is_vertical)
	{
		Point2D otherdiff = other.getDiff();
		double oa = otherdiff.getY() / otherdiff.getX();
		intersectPoint.setXY(first.getX(),
							 oa * first.getX() + other.first.getY() - oa * other.first.getX());
	}
	else if (other_is_vertical)
	{
		Point2D diff = getDiff();
		double a = diff.getY() / diff.getX();
		intersectPoint.setXY(other.first.getX(),
							 a * other.first.getX() + first.getY() - a * first.getX());
	}
	else
	{
		Point2D otherdiff = other.getDiff();
		double oa = otherdiff.getY() / otherdiff.getX();
		Point2D diff = getDiff();
		double ta = diff.getY() / diff.getX();
		if (oa == ta) // parallel lines
			return NotIntersecting;
		double x = ( - other.first.getY() + oa * other.first.getX()
					+ first.getY() - ta * first.getX() ) / (oa - ta);
		intersectPoint.setXY(x, ta * (x - first.getX()) + first.getY());
	}

	if (point)
		*point = intersectPoint;
	return result;
}

Line2D Line2D::fromLinearRegression(const Polygon2D &points)
{
	if (points.size() >= 2)
	{
		// y = a + b*x

		const Point2D mean(points.getCenterOfGravity());

		double SSxy = 0;
		double SSxx = 0;
		double minX = points.front().getX();
		double maxX = minX;

		for (Polygon2D::const_iterator point = points.begin(); point != points.end(); ++point)
		{
			minX = std::min(minX, double(point->getX()));
			maxX = std::max(maxX, double(point->getX()));
			SSxy += (point->getX() - mean.getX()) * (point->getY() - mean.getY());
			SSxx += (point->getX() - mean.getX()) * (point->getX() - mean.getX());
		}

		const double b(SSxy / SSxx);
		const double a(mean.getY() - b * mean.getX());

		return Line2D(Point2D(minX, a + b * minX), Point2D(maxX, a + b * maxX));
	}

	return Line2D();
}

Point2D Line2D::getCenterPoint() const
{
	return getP1() + (0.5  * getDiff());
}

}	// namespace datatypes
