// This is -*-c++-*-
//

#ifndef ELLIPSE2D_H
#define ELLIPSE2D_H

#include "../BasicDatatypes.hpp"
#include "Point2D.hpp"
#include <iosfwd> // for istream, ostream
#include <sstream>

namespace datatypes
{

/// A rotated 2-dimensional ellipse in the plane
/**
 * This ellipse is defined by its center point, the rotation of the
 * ellipse's main (x) axis measured from the global x-axis, and the
 * radius of the ellipse's x- and y-axis.
 *
 * The rotation denotes the rotation of this ellipse' x-axis compared
 * to the original x-axis. The two radii denote the radius length of
 * this ellipse in the ellipse' coordinate system, i.e. the distance
 * from the center point to each outline.
 *
 * The size argument must be non-negative. The rotation argument must
 * be in the range [-pi,pi], which can be obtained by using
 * ::normalizeRadians().
 */
class Ellipse2D : public BasicData
{
public:
	/// The type of the stored x, y coordinates, and the rotation.
	typedef Point2D::value_type value_type;
protected:
	Point2D m_center;
	Point2D m_radius;
	value_type m_rotation;
public:
	/// Constructor for an all-zero Ellipse2D.
	Ellipse2D();

	/// Constructor with specified center point, radius, and rotation.
	/**
	 * \note The radius components must be non-negative, otherwise an assertion will fail.
	 *
	 * \param center The center point
	 * \param radius The size of the ellipse in the ellipse' coordinate system. Must be non-negative.
	 *
	 * \param rotation The rotation of the ellipse' coordinate system
	 * around its center point in [radians]. Must be in the interval
	 * [-pi,pi], which can be obtained by using
	 * ::normalizeRadians().
	 */
	Ellipse2D(const Point2D& center, const Point2D& radius, value_type rotation = 0.0);

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
	 * \param x_radius The radius of the ellipse in X-direction in the ellipse' coordinate system. Must be non-negative.
	 * \param y_radius The radius of the ellipse in Y-direction in the ellipse' coordinate system. Must be non-negative.
	 *
	 * \param rotation The rotation of the ellipse' coordinate system
	 * around its center point in [radians]. Must be in the interval
	 * [-pi,pi], which can be obtained by using
	 * ::normalizeRadians().
	 */
	Ellipse2D(value_type x_center, value_type y_center, value_type x_radius, value_type y_radius, value_type rotation = 0.0);


	// Estimate the memory usage of this object
	virtual const UINT32 getUsedMemory() const { return sizeof(*this); };
	
	/** \name Accessor methods for information about this ellipse */
	//\{

	/// Returns the center point of this Ellipse
	const Point2D& getCenter() const { return m_center; }

	/// Returns the radius of this Ellipse
	/**
	 * The returned radius denotes the radius of the ellipse in x-direction
	 * Point2D::getX() and y-direction Point2D::getY(), where the
	 * x-direction is rotated against the original x-axis by
	 * getRotation().
	 */
	const Point2D& getRadius() const { return m_radius; }

	/** Returns the rotation angle of this Ellipse2D in [radians], counter clock wise.
	 *
	 * The rotation of the ellipse' coordinate system around its center
	 * point. Must be in the interval [-pi,pi]
	 */
	value_type getRotation() const { return m_rotation; }

	//\}


	/** \name Setter methods for changing this ellipse */
	//\{

	/// Sets the center point of this Ellipse2D
	void setCenter(const Point2D& p) { m_center = p; }

	/// Sets the center point of this Ellipse2D
	void setCenter(value_type x, value_type y) { m_center.setXY(x, y); }

	/// Sets the radius of this Ellipse. Must be non-negative.
	void setRadius(const Point2D& p);

	/// Sets the radius of this Ellipse2D. Must be non-negative.
	void setRadius(value_type x_length, value_type y_width);

	/** Sets the rotation angle of this Ellipse in [radians], counter
	 * clock wise.
	 *
	 * The rotation of the ellipse' coordinate system around its center
	 * point. Must be in the interval [-pi,pi], which can be obtained
	 * by using ::normalizeRadians().
	 */
	void setRotation(value_type r);

	//\}


	/** \name Geometrical relations to other objects */
	//\{

	/// Returns true if the given Point2D is inside this ellipse or on its outline.
	/**
	 * (Note: This function is relatively cheap - it needs two sinus
	 * operations, four multiplications and a bunch of
	 * comparisons.)
	 */
	bool containsPoint(const Point2D& point) const;

	//\}

	std::string toString() const;		// Konvertierung zu string
	

	friend inline bool operator==(const Ellipse2D &, const Ellipse2D &);
	friend inline bool operator!=(const Ellipse2D &, const Ellipse2D &);

private:
	void verifyNumericRanges();

};


inline bool operator==(const Ellipse2D &b1, const Ellipse2D &b2)
{
	return (b1.m_center == b2.m_center)
		   && (b1.m_radius == b2.m_radius)
		   && (fuzzyCompare(b1.m_rotation,
								  b2.m_rotation)
			   || (isNaN(b1.m_rotation) && isNaN(b2.m_rotation)));
}

inline bool operator!=(const Ellipse2D &b1, const Ellipse2D &b2)
{
	return ! operator==(b1, b2);
}

}	// namespace datatypes

#endif // ELLIPSE2D_H
