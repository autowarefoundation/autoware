//
// Box2D.hpp
// A rotated 2-dimensional box.
//

#ifndef BOX2D_H
#define BOX2D_H

#include "Point2D.hpp"
#include "../BasicDatatypes.hpp"
#include <iosfwd> // for istream, ostream
#include <vector>

namespace datatypes
{

// Forward declarations
class Polygon2D;

/// A rotated 2-dimensional box in the plane
/**
 * This box (rectangle) is defined by its center point, its size in x-
 * and y-direction, and its rotation.
 *
 * The rotation denotes the rotation of this box' x-axis compared to
 * the original x-axis. The size denotes the total length and width of
 * this box in the box' coordinate system, i.e. the distance from the
 * center point to each outline is half that size.
 *
 * The size argument must be non-negative. The rotation argument must
 * be in the range [-pi,pi], which can be obtained by using
 * ::normalizeRadians().
 */
class Box2D : public BasicData
{
public:
	/// The type of the stored x, y coordinates, and the rotation.
	typedef Point2D::value_type value_type;
private:
	Point2D m_center;
	Point2D m_size;
	value_type m_rotation;
public:
	/// Constructor for an all-zero Box2D.
	Box2D();

	/// Constructor with specified center point, size, and rotation.
	/**
	 * \note The size components must be non-negative, otherwise an assertion will fail.
	 *
	 * \param center The center point
	 * \param size The size of the box in the box' coordinate system. Must be non-negative.
	 *
	 * \param rotation The rotation of the box' coordinate system
	 * around its center point in [radians]. Must be in the interval
	 * [-pi,pi], which can be obtained by using
	 * ::normalizeRadians().
	 */
	Box2D(const Point2D& center, const Point2D& size, value_type rotation = 0.0);

	/// Constructor with all values given.
	/**
	 * Constructor with all values given: x/y of center point, x/y of
	 * size (where the x coordinate of the size is in the same
	 * direction as the x axis of the coordinate system, rotated by
	 * the rotation argument; the y coordinate accordingly), and
	 * rotation.
	 *
	 * \note The x_size and y_size must be non-negative.
	 *
	 * \param x_center X-coordinate of center point
	 * \param y_center Y-coordinate of center point
	 *
	 * \param x_size The size of the box in X-direction in the box' coordinate system. Must be non-negative.
	 * \param y_size The size of the box in Y-direction in the box' coordinate system. Must be non-negative.
	 *
	 * \param rotation The rotation of the box' coordinate system
	 * around its center point in [radians]. Must be in the interval
	 * [-pi,pi], which can be obtained by using
	 * ::normalizeRadians().
	 */
	Box2D(value_type x_center, value_type y_center, value_type x_size, value_type y_size, value_type rotation = 0.0);

	// Estimate the memory usage of this object
	virtual const UINT32 getUsedMemory() const { return sizeof(*this); };
	
	
	/** \name Accessor methods for information about this box */
	//\{

	/// Returns the center point of this Box
	const Point2D& getCenter() const { return m_center; }

	/// Returns the size of this Box
	/**
	 * The returned size denotes the size of the box in x-direction
	 * Point2D::getX() and y-direction Point2D::getY(), where the
	 * x-direction is rotated against the original x-axis by
	 * getRotation().
	 */
	const Point2D& getSize() const { return m_size; }

	/** Returns the rotation angle of this Box2D in [radians], counter clock wise.
	 *
	 * The rotation of the box' coordinate system around its center
	 * point. Must be in the interval [-pi,pi]
	 */
	value_type getRotation() const { return m_rotation; }

	/// Converts this Box2D to a closed polygon
	/**
	 * Converts this Box2D to a closed polygon with 5 points, where
	 * the last point is identical to the first one to make it
	 * closed.  */
	Polygon2D toPolygon() const;

	/// Returns a Box in parallel to the coordinate system that bounds this box
	/**
	 * This function calculates a bounding box to the given box,
	 * because the given one might be rotated into some other
	 * direction. In contrast to this, the returned box will have zero
	 * rotation and will be in parallel to the coordinate system.
	 */
	Box2D toBoundingBox() const;

	/** \brief Returns boundary angles for this box.
	 *
	 * This function calculates a low and a high boundary angle for
	 * all edges of the given (rotated) Box2D. The returned FloatPair
	 * has the component "first" for the lower bounding angle, and
	 * "second" for the upper bounding angle.
	 *
	 * (Note: This ordering is swapped compared to the scan point
	 * ordering!)
	 */
	std::pair<value_type, value_type> getBoundingAngles() const;

	/** \brief Returns a Box that is copied from this one but with its center point moved
	 */
	Box2D movedBy(const Point2D& centerMovement) const;
	//\}


	/** \name Setter methods for changing this box */
	//\{

	/// Sets the center point of this Box2D
	void setCenter(const Point2D& p) { m_center = p; }

	/// Sets the center point of this Box2D
	void setCenter(value_type x, value_type y) { m_center.setXY(x, y); }

	/// Sets the size of this Box. Must be non-negative.
	void setSize(const Point2D& p);

	/// Sets the size of this Box2D. Must be non-negative.
	void setSize(value_type x_length, value_type y_width);

	/** \brief Sets the rotation angle of this Box in [radians], counter
	 * clock wise.
	 *
	 * The rotation of the box' coordinate system around its center
	 * point. Must be in the interval [-pi,pi], which can be obtained
	 * by using ::normalizeRadians().
	 */
	void setRotation(value_type r);

	/// Move the center point of this box by the given point values.
	void moveBy(const Point2D& centerMovement);
	//\}

	/** \name Geometrical relations to other objects */
	//\{

	/// Returns true if the given Point2D is inside this box or on its outline.
	/**
	 * (Note: This function is relatively cheap - it needs two sinus
	 * operations, four multiplications and a bunch of
	 * comparisons.)
	 */
	bool containsPoint(const Point2D& point) const;

	/// Returns the distance from the outline of this Box2D to the given point.
	/**
	 * This function calculates the minimum distance over the
	 * distances from the given point to each of the four outside
	 * lines of this Box2D.
	 *
	 * Internally, this might be implemented using
	 * Polygon2D::distanceToPoint and
	 * Line2D::distanceFromLineSegment(), but maybe the implementation
	 * will be optimized to work in some other way.
	 */
	Point2D::value_type distanceFromOutline(const Point2D& point) const;

	/// Returns the mean distance from the outline of this Box2D to all points of the point vector.
	/**
	 * For each given point, this function calculates the minimum
	 * distance over the distances from the given point to each of the
	 * four outside lines of this Box2D and returns that minimum
	 * distance. The returned value is the mean value of all
	 * distances.
	 *
	 * This is an overloaded version of distanceFromOutline(const
	 * Point2D&) for your convenience.
	 */
	Point2D::value_type distanceFromOutline(const std::vector<Point2D>& points) const;

	/// Returns the mean distance from the outline of this Box2D to all points of the given point iterator range.
	/**
	 * For each given point, this function calculates the minimum
	 * distance over the distances from the given point to each of the
	 * four outside lines of this Box2D and returns that minimum
	 * distance. The returned value is the mean value of all
	 * distances.
	 *
	 * This is an overloaded version of distanceFromOutline(const
	 * Point2D&) for your convenience.
	 */
	Point2D::value_type distanceFromOutline(const std::vector<Point2D>::const_iterator& begin,
											const std::vector<Point2D>::const_iterator& end) const;


	/// Returns an orientated bounding box for the given list of points.
	/**
	 * Given a list of points and a fixed orientation, this function
	 * will calculate a bounding box for the points that has the given
	 * orientation.
	 */
	static Box2D orientatedBox(value_type orientation_rad, const Polygon2D& poly);

	/// Returns an orientated bounding box for the given list of points.
	/**
	 * Given a list of points and a fixed orientation, this function
	 * will calculate a bounding box for the points that has the given
	 * orientation.
	 *
	 * This is an overloaded version of orientatedBox(const
	 * Polygon2D&) for your convenience.
	 */
	static Box2D orientatedBox(value_type orientation_rad, const std::vector<Point2D>& points);

	/// Returns an orientated bounding box for the given list of points.
	/**
	 * Given a list of points and a fixed orientation, this function
	 * will calculate a bounding box for the points that has the given
	 * orientation.
	 *
	 * This is an overloaded version of orientatedBox(const
	 * Polygon2D&) for your convenience.
	 */
	static Box2D orientatedBox(value_type orientation_rad,
							   const std::vector<Point2D>::const_iterator& begin,
							   const std::vector<Point2D>::const_iterator& end);


	std::string toString() const;		// Konvertierung in String


	friend inline bool operator==(const Box2D &, const Box2D &);
	friend inline bool operator!=(const Box2D &, const Box2D &);

private:
	void verifyNumericRanges();

};


inline bool operator==(const Box2D &b1, const Box2D &b2)
{
	return (b1.m_center == b2.m_center)
		   && (b1.m_size == b2.m_size)
		   && (fuzzyCompare(b1.m_rotation, b2.m_rotation)
			   || (isNaN(b1.m_rotation) && isNaN(b2.m_rotation)));
}

inline bool operator!=(const Box2D &b1, const Box2D &b2)
{
	return ! operator==(b1, b2);
}


}	// namespace datatypes

#endif // BOX2D_H
