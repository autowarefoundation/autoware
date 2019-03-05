// This is -*-c++-*-
//

#ifndef CIRCLE2D_H
#define CIRCLE2D_H

#include "Ellipse2D.hpp"

// ////////////////////////////////////////////////////////////

namespace datatypes
{


/// A circle in the plane
/**
 * This circle is defined by its center point and its radius.
 *
 * The radius argument must be non-negative.
 */
class Circle2D : public Ellipse2D
{
public:
	/// Base class
	typedef Ellipse2D base_class;

	/// Constructor for an all-zero Circle2D.
	Circle2D();

	/// Constructor with specified center point and radius.
	/**
	 * \note The radius components must be non-negative, otherwise an assertion will fail.
	 *
	 * \param center The center point
	 * \param radius The radius of the circle.
	 */
	Circle2D(const Point2D& center, value_type radius);

	/// Constructor with all values given.
	/**
	 * Constructor with all values given: x/y of center point, x/y of
	 * size (where the x coordinate of the size is in the same
	 * direction as the x axis of the coordinate system, rotated by
	 * the rotation argument; the y coordinate accordingly), and
	 * rotation.
	 *
	 * \note The radius components must be non-negative, otherwise an assertion will fail.
	 *
	 * \param x_center X-coordinate of center point
	 * \param y_center Y-coordinate of center point
	 *
	 * \param radius The radius of the circle. Must be non-negative.
	 */
	Circle2D(value_type x_center, value_type y_center, value_type radius);

	// Estimate the memory usage of this object
	virtual const UINT32 getUsedMemory() const { return sizeof(*this); };
	
	/// Returns the radius of this circle
	value_type getRadius() const { return m_radius.getX(); }

	/// Sets the radius of this circle. Must be non-negative.
	void setRadius(value_type radius);

	/// Returns true if the given Point2D is inside this ellipse or on its outline.
	/**
	 * (Note: This function is relatively cheap.)
	 */
	bool containsPoint(const Point2D& point) const;

};


}	// namespace datatypes

#endif // CIRCLE2D_HPP
