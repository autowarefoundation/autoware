// This is -*-c++-*-
//

#ifndef POLYGON2D_HPP
#define POLYGON2D_HPP

#include "../BasicDatatypes.hpp"
#include "Point2D.hpp"
#include <iosfwd> // for istream, ostream
#include <vector>
#include <sstream>


namespace datatypes
{

class Line2D;
class Ellipse2D;
class Box2D;


/** \brief A polygon of 2D-points.
 *
 * The points can be accessed through the std::vector interface. Basic
 * information available through the std::vector base class include:
 *
 * - empty() returns true if the polygon contains zero points
 * - size() returns the number of points in the polygon
 * - front() returns a reference to the first point in a non-empty polygon
 * - back() returns a reference to the last point
 *
 * To access all points, use operator[]() or begin() and end() with a
 * Polygon2D::const_iterator or Polygon2D::iterator.
 *
 * The polygon can be interpreted as closed polygon either implicitly
 * or explicitly. For explicitly closed polygons, the final Point2D is
 * identical to the first one and isClosed() returns true. On the
 * other hand, several use cases of the Polygon2D assume the polygon
 * to be closed without the explicit final point identical to the
 * first one, so even without the additional final point the polygon
 * is probably fine most of the time.
 */
class Polygon2D : public std::vector<Point2D>,
					public BasicData
{
public:
	/// The base type. (Naming according to boost convention.)
	typedef std::vector<Point2D> base_class;

	/// The type of the stored x, y coordinates of each of the points
	/// in this polygon.
	typedef Point2D::value_type floatingpoint_type;

	/// Constructor for an empty polygon.
	Polygon2D();

	/// Convenience constructor for a polygon with one Point
	Polygon2D(const Point2D&);

	/// Convenience constructor for a polygon with two Points
	Polygon2D(const Point2D&, const Point2D&);

	/// Convenience constructor for a polygon with three Points
	Polygon2D(const Point2D&, const Point2D&, const Point2D&);

	/// Convenience constructor for a polygon with four Points
	Polygon2D(const Point2D&, const Point2D&, const Point2D&, const Point2D&);

	/// Convenience constructor for a polygon with the two points of a line
	Polygon2D(const Line2D&);

	/// Copy constructor from a std::vector.
	Polygon2D(const base_class&);

	/// Constructor for reading from a string using fromString()
	Polygon2D(const std::string& polygonAsString);

	// Estimate the memory usage of this object
	inline virtual const UINT32 getUsedMemory() const {return ((sizeof(*this)) + sizeof(Point2D)*size());};
	
	/// Returns a string containing a list of separator characters
	/// that is accepted when parsing a string in fromString().
	static const char* getSeparatorCharacters();

	/// Static function to get Polygon2D from circle parameters
	static Polygon2D fromCircle( const Point2D& center, const floatingpoint_type radius, const UINT32 samplingPoints = 32 );

	/// Static function to create a Polygon2D from ellipse parameters
	static Polygon2D fromEllipse( const Point2D& center, const floatingpoint_type a, const floatingpoint_type b, const floatingpoint_type angle, const UINT32 samplingPoints = 32 );

	/// Static function to create a Polygon2D from ellipse parameters
	static Polygon2D fromEllipse(const Ellipse2D& ellipse, const UINT32 samplingPoints = 32 );

	/// (DEPRECATED) Create a Polygon2D approximation of the arc of an ellipse
	/**
	 * DEPRECATED: Use the other fromArc() function instead, because
	 * the "direction" argument is difficult to understand correctly.
	 *
	 * Function starts at startAngle and moves to endAngle in a
	 * selectable direction.
	 *
	 * The number of sampling points must be at least 2 (one at the
	 * start, one at the end), otherwise an empty Polygon will be
	 * returned.
	 *
	 * \param ellipse The ellipse center, axis length, and rotation
	 *
	 * \param startAngle The start angle from which the arc will be drawn
	 *
	 * \param endAngle The end angle until which the arc will be drawn
	 *
	 * \param samplingPoints Number of points which the resulting
	 * polygon will contain. If less then 2, an empty Polygon2D will
	 * be returned.
	 *
	 * \param clockwise If true, the arc should be plotted from the
	 * startAngle in clockwise direction to the endAngle
	 * (mathematically negative). If false, the arc is plotted from
	 * the start angle in counter-clockwise direction to the end angle
	 * (mathematically positive).
	 */
	static Polygon2D fromArc(const Ellipse2D& ellipse,
							 const floatingpoint_type startAngle, const floatingpoint_type endAngle,
							 const UINT32 samplingPoints, const bool clockwise);

	/// Create a Polygon2D approximation of the arc of an ellipse
	/**
	 * The function starts at startAngle and calculates sampling
	 * points on the way to the endAngle in mathematically positive
	 * direction (counter-clockwise), where the startAngle is lesser
	 * than the endAngle.
	 *
	 * When an arc in the other direction is needed, simply specify a
	 * startAngle greater than the endAngle, which will result in an
	 * arc in mathematically negative direction (clockwise).
	 *
	 * The resulting polygon is explicitly closed (except for rounding errors),
	 * that is, the first sampling point will be located exactly at
	 * startAngle and the last sampling point exactly at endAngle. If
	 * the angles are 0 and 2*pi, the last point will be approximately
	 * identical to the first one except for rounding errors, which
	 * means the polygon is appoximately closed but isClosed() might
	 * not be true.
	 *
	 * (Successor of the other fromArc() function because the
	 * "direction" argument is difficult to understand correctly.)
	 *
	 * The number of sampling points must be at least 2 (one at the
	 * start, one at the end), otherwise an empty Polygon will be
	 * returned.
	 *
	 * \param ellipse The ellipse center, axis length, and rotation
	 *
	 * \param startAngle The start angle from which the arc will be drawn
	 *
	 * \param endAngle The end angle until which the arc will be
	 * drawn. For an arc in mathematically positive direction
	 * (counter-clockwise), this angle needs to be greater than the
	 * startAngle, otherwise lesser. The absolute difference between
	 * start and end angle must be at most 2*pi, otherwise an
	 * exception will be thrown.
	 *
	 * \param samplingPoints Number of points which the resulting
	 * polygon will contain. If less then 2, an empty Polygon2D will
	 * be returned.
	 *
	 * \return An approximation polygon of the arc of an ellipse.
	 */
	static Polygon2D fromArc(const Ellipse2D& ellipse,
							 const floatingpoint_type startAngle, const floatingpoint_type endAngle,
							 const UINT32 samplingPoints = 32);

	/// Static function to create a rhombus.
	static Polygon2D rhombus(const Point2D& center, const floatingpoint_type radius);

	/** Static function to create a four-point rectangle from a lower
	 * left and an upper right point.
	 *
	 * Note: The returned polygon contains the four points of the
	 * rectangle. This means isClosed() does not return true.  To be
	 * closed in the sense of isClosed(), the polygon manually needs
	 * to have the first point added as the last (fifth) point as
	 * well. On the other hand, several use cases of the Polygon2D
	 * assume the polygon to be closed without the explicit final
	 * point identical to the first one, so this four-point rectangle
	 * is probably fine most of the time.
	 *
	 * \return A polygon with four points representing the edge
	 * points of a rectangle.
	 */
	static Polygon2D createRectangle(const Point2D& lowerLeft, const Point2D& upperRight);

	/** \name Set functions
	 *
	 * \sa base_class::push_back, base_class::insert
	 */
	//\{
	/** \brief Appends the points of the \c other polygon to this polygon.
	 *
	 * This is identical to the std::vector method
	 * p.insert(p.end(),other.begin(),other.end());
	 *
	 * \sa base_class::push_back, base_class::insert
	 */
	Polygon2D& append(const Polygon2D& other);

	/** \brief Appends one point \c point to this polygon.
	 *
	 * This is identical to the std::vector method
	 * push_back() but returns a reference to this.
	 *
	 * \sa base_class::push_back
	 */
	Polygon2D& append(const Point2D& point);

	/** \brief Appends one point with coordinates \c x and \c y to this polygon.
	 *
	 * This is a shortcut for creating a new point with the coordinates x and y
	 * and using std::vector's method push_back() to append the point.
	 *
	 * \sa base_class::push_back
	 */
	Polygon2D& append(floatingpoint_type x, floatingpoint_type y);
	//\}

	/** \name Accessor methods for information about this polygon */
	//\{

	/** \brief Returns true if this is explicitly a closed polygon.
	 *
	 * This simply means the last point back() is equal to the first
	 * point front() of the polygon.
	 *
	 * On the other hand, several use cases of the Polygon2D assume
	 * the polygon to be closed even without the explicit final point
	 * identical to the first one, so polygons with the return value
	 * false here are probably still fine most of the time.
	 *
	 * An empty polygon (where empty() returns true) is not closed by
	 * definition and isClosed() will return false.
	 */
	bool isClosed() const;

	/** \brief Returns the area enclosed by this polygon
	 *
	 * Note: The polyon must be a non-self-intersecting polygon,
	 * i.e. a simple polygon.
	 */
	double getArea() const;

	/** \brief Returns the center of gravity of this polygon.
	 *
	 * Based on the edges of this polygon, this function returns the
	 * center of gravity.  (The returned value is simply the
	 * arithmetic mean of all edges.)
	 */
	Point2D getCenterOfGravity() const;

	/** \brief Returns a polygon with potentially less edge points
	 *
	 * This method checks the polygon for "unnecessary" edges and
	 * returns a polygon with those edges removed.  For each edge, it
	 * is checked whether they are located directly on the connection
	 * line between the edge before and after the middle edge.  If
	 * this is the case, the middle edge can be removed without
	 * changing the shape of the polygon.
	 */
	Polygon2D getSimplified() const;

	/// Returns a Box in parallel to the coordinate system that bounds this polygon
	/**
	 * This function calculates a bounding box to the given
	 * polygon. The returned box will have zero rotation and will be
	 * in parallel to the coordinate system.
	 */
	Box2D getBoundingBox() const;

	/** This function calculates a low and a high boundary angle for
	 * all edges of the given polygon. The returned FloatPair
	 * has the component "first" for the lower bounding angle, and
	 * "second" for the upper bounding angle.
	 *
	 * (Note: This ordering is swapped compared to the scan point
	 * ordering!)
	 */
	std::pair<floatingpoint_type, floatingpoint_type> getBoundingAngles() const;

	//\}

	/** \name Geometrical relations to other objects */
	//\{

	/** \brief Returns true if the given Point2D is inside this polygon.
	 */
	bool containsPoint(const Point2D& point) const;

	/// Returns the distance of a point to this polyon.
	/**
	 * For each line segment of this polygon, this function calculates
	 * three distances: The distances to both end points of the
	 * segment and the distance of the point to its projection onto
	 * the line segment. The overall minimum distance over these three
	 * possibilities and over all line segments will be returned.
	 *
	 * Internally, this will use Line2D::distanceFromLineSegment() for
	 * one segment and return the minimum of that over all segments.
	 */
	Point2D::value_type distanceToPoint(const Point2D& point) const;

	/** \brief Returns true if the given polygon collides with this polygon.
	 *
	 * In other words, there exists an intersection region or at least
	 * one intersecting point between these two polygons.
	 *
	 * Note: This might only work for convex polygons!
	 */
	bool isColliding(const Polygon2D& p2) const;

	/// Calculates all intersection points between a line and this polygon
	/**
	 * A line can have zero, one, or up to size() intersection points
	 * with this polygon. The returned list of points contains all
	 * those intersection points between the given line and all line
	 * segments of this polygon.
	 *
	 * In contrast to Line2D::isIntersecting() this method considers
	 * and returns only those intersection points which are within the
	 * line segments.
	 */
	base_class isIntersecting(const Line2D& other) const;

	//\}


	/// Text output for debugging
	/**
	 * This string can be parsed into a polygon again by using
	 * fromString().
	 */
	std::string toString() const;


};




/// Text output for debugging
std::ostream& operator<<(std::ostream& os, const Polygon2D& point);


}	// namespace datatypes

#endif
