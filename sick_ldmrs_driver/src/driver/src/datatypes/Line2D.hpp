//
// Line2D.hpp
//

#ifndef LINE2D_HPP
#define LINE2D_HPP

#include <iosfwd>
#include "Point2D.hpp"
#include "../BasicDatatypes.hpp"

namespace datatypes
{
	
class Polygon2D;

/// A line in the two-dimensional plane, composed out of two points.
/**
 * This class is a line composed out of two points and offers some
 * helper methods for simple geometrical calculations.
 *
 * \sa Point2D, Polygon2D
 */
class Line2D : public BasicData
{
public:
	/// Typedef for STL compatibility
	typedef Point2D value_type;
	/// Typedef for STL std::pair compatibility
	typedef Point2D first_type;
	/// Typedef for STL std::pair compatibility
	typedef Point2D second_type;

	/// Describes how two lines can be intersecting. \sa isIntersecting()
	enum IntersectingType
	{
		NotIntersecting ///< The lines are not intersecting, i.e. they are parallel or zero
		, LineIntersecting ///< The lines are intersecting within their line segments
		, OutsideIntersecting ///< The lines are intersecting, but outside of their line segments
	};


	/// Empty constructor
	Line2D()
		: first()
		, second()
	{	m_datatype = Datatype_Line2D; }

	/// Constructor with two points
	Line2D(const Point2D&a, const Point2D& b)
		: first(a)
		, second(b)
	{ m_datatype = Datatype_Line2D; }

	/// Constructor with x/y coordinates of the two points given explicitly
	Line2D(float x1, float y1, float x2, float y2)
		: first(x1, y1)
		, second(x2, y2)
	{ m_datatype = Datatype_Line2D; }

	/// Constructor from a std::pair
	Line2D(const std::pair<Point2D, Point2D>& p)
		: first(p.first)
		, second(p.second)
	{ m_datatype = Datatype_Line2D; }

	// Estimate the memory usage of this object
	inline virtual const UINT32 getUsedMemory() const {return (sizeof(*this));};
	
	
	std::string toString();	// Inhalt als lesbarer String
	
	/// Returns a Line2D from several points using linear regression.
	static Line2D fromLinearRegression(const Polygon2D &points);

	/** \name Accessor methods for information about this line */
	//\{

	/// Returns true if both points are zero
	bool isZero() const { return first.isZero() && second.isZero(); }

	/// Returns the first point
	const Point2D& getP1() const { return first; }

	/// Returns the second point
	const Point2D& getP2() const { return second; }

	/// Returns the point in the middle between first and second point
	Point2D getCenterPoint() const;

	/// Returns the length of this line, i.e. the distance of the two
	/// points.
	double getLength() const { return getDiff().dist(); }

	/// Deprecated. Returns the length of this line, i.e. the distance
	/// of the two points.
	double length() const { return getLength(); }

	/// Calculates the rising angle of a line: the inclination
	/// angle. Returns a value in [-PI,PI] in radians.
	double getInclination() const;

	/// Deprecated. Calculates the rising angle of a line: the
	/// inclination angle. Returns a value in [-PI,PI] in radians.
	double inclination() const { return getInclination(); }

	/// Returns the difference between line end and line start as a Point2D.
	// The returned value is getP2() - getP1().
	Point2D getDiff() const { return second - first; }

	/// Returns a unit vector for this line
	/**
	 * The returned unit vector has the same starting point as this
	 * line (which is getP1()) and it has a length of 1.0.
	 */
	Line2D getUnitVector() const;

	/// Conversion to Polygon2D
	Polygon2D toPolygon2D() const;

	/// Conversion to Polygon2D with more than 2 sampling points
	Polygon2D toPolygon2D(unsigned samplePoints) const;

	//\}


	/** \name Setter methods for changing this line */
	//\{

	/// Sets the first point
	void setP1(const Point2D& p1) { first = p1; }

	/// Sets the second point
	void setP2(const Point2D& p2) { second = p2; }

	//\}


	/** \name Geometrical relations to other objects */
	//\{


	/**
	 * \brief Returns the distance of the given point to its
	 * orthogonal projection onto this line.
	 *
	 * The calculated distance is always the distance from the point
	 * to the point's projection onto the line, even if the point's
	 * projection is not between the end points of the
	 * line. Alternatively, distanceFromLineSegment() returns the
	 * distance to the line's end point in that case.
	 */
	double distanceToPoint(const Point2D& point) const;

	/**
	 * \brief Returns the distance of a point to this line segment.
	 *
	 * If the point's projection onto the line is between the end
	 * points of this line, the distance to the projected point is
	 * returnd. If the projection is not on the line segment, the
	 * distance to the closest end point is returned. Alternatively,
	 * distanceToPoint() returns the distance to the point's
	 * projection in all cases.
	 */
	double distanceFromLineSegment(const Point2D& point) const;

	/// Calculates and returns the point that results when projecting
	/// the given point onto this line orthogonally.
	Point2D projectOntoLine(const Point2D& point) const;

	/**
	 * \brief Returns true if this line "contains" the given point.
	 *
	 * Computes the distance from the point to the line, and if it is
	 * zero, the point lies on the line and this method returns true.
	 */
	bool containsPoint(const Point2D& point) const;

	/// Calculates the intersection point between two lines
	/**
	 * The returned value describes whether the two lines intersect
	 * within their line segments, or outside of their line segments,
	 * or not at all. The actual intersection point is written to
	 * intersectingPoint if that pointer is non-NULL.
	 */
	IntersectingType isIntersecting(const Line2D& other,
									Point2D* intersectingPoint = NULL) const;

	//\}

private:
	/// The first point of this line
	Point2D first;
	/// The second point of this line
	Point2D second;

};

// Text output for debugging
std::ostream& operator<<(std::ostream& os, const Line2D& l);

}	// namespace datatypes

#endif
