//
// Point2D.hpp
// A 2-dimensional point (float)
//


#ifndef POINT2D_HPP
#define POINT2D_HPP

#include <cassert>
#include <iosfwd> // for istream, ostream
#include <ios>
#include <utility> // for std::pair<T,T>
#include <vector>
#include "../tools/MathToolbox.hpp"
#include "../tools/toolbox.hpp"
#include "../BasicDatatypes.hpp"


namespace datatypes
{
	

//
//This class defines a point in the two-dimensional plane.
//
class Point2D : public BasicData
{
public:
	/// The type of the stored x and y coordinates.
	typedef double value_type;

	/// The type of the stored x and y coordinates. An alias for value_type.
	typedef value_type floatingpoint_type;

private:
	value_type m_x;
	value_type m_y;
public:
	/** Constructs a null point, i.e. with coordinates (0.0, 0.0) */
	Point2D()
		: m_x(0)
		, m_y(0)
	{ m_datatype = Datatype_Point2D; }

	/** Constructs a point with the given coordinates (x, y). */
	Point2D(value_type x, value_type y)
		: m_x(x)
		, m_y(y)
	{ m_datatype = Datatype_Point2D; }

	/** Constructs a point from the given ScanPoint */
//	Point2D(const ScanPoint& p);

	/** Constructs a point from the given Point3D. Its z-coordinate will be ignored. */
//	explicit Point2D(const Point3D& p);

	// Estimate the memory usage of this object
	inline virtual const UINT32 getUsedMemory() const {return sizeof(*this);};

	/** \name Accessor methods for information about this point */
	//\{

	/** Returns true if this point is zero in terms of the machine
	 * precision, that is, its value is exactly zero or "almost
	 * zero". */
	bool isZero() const;

	/** Returns the x-coordinate of this point. */
	value_type getX() const { return m_x; }

	/** Returns the y-coordinate of this point. */
	value_type getY() const { return m_y; }

	/** Returns the Euclidian distance to the origin,
	 * sqrt(x^2+y^2). An alias for getDist(). */
	value_type dist() const;

	/** Returns the Euclidian distance to the origin,
	 * sqrt(x^2+y^2). */
	value_type getDist() const { return dist(); }

	/** Same as dist() */
	value_type length() const { return dist(); }

	/** Returns the polar angle of this point, which is the angle from
	 * the x-axis to this point. An alias for getAngle(). */
	value_type angle() const;

	/** Returns the polar angle of this point, which is the angle from
	 * the x-axis to this point. */
	value_type getAngle() const { return angle(); }

	/** Calculates the polar coordinates of this point and writes them
	 * to the given arguments. dist will contain the distance of this
	 * point to the origin, and angle will contain the polar angle
	 * from the x-axis to this point. */
	void toPolar(value_type& dist, value_type& angle) const;

	/** Returns the polar coordinates of this point as a pair of
	 * distance and angle. The returned value's component "first" is
	 * the distance of this point to the origin, and the returned
	 * value's component "second" is the polar angle from the x-axis
	 * to this point. You can use this as follows:
	 *
	 * std::pair<value_type,value_type> x = p.toPolar();
	 * cout << "Distance=" << x.first << "; Angle=" << x.second;
	 *
	 */
	std::pair<value_type, value_type> toPolar() const;

	/**
	 * Calculates the normalized form of this vector (point is
	 * considered as a vector here) and returns it.  If the vector has
	 * zero length (isZero() returns true), it will be left unchanged.
	 */
	Point2D normalized() const;

	/** Rotate the given point by the given angle around (0,0) and returns the
	 * rotated point.
	 *
	 * This method implements an "active rotation matrix".
	 *
	 * In other words: A positive rotation of the point (1,0) by the
	 * angle +pi/2 results in the point (0,1). Watch out: If the
	 * rotation is defined differently, another point (0,-1) might
	 * have been expected, but this function is implemented to return
	 * (0,1) here. */
	Point2D rotated(value_type angle_rad) const;

	/// Returns the x/y components of this class, converted into a Point3D object with zero z-component.
//	Point3D toPoint3D() const;

	//\}

	/** \name Setter methods for changing this point */
	//\{

	/** Sets the x-coordinate of this point to the given x coordinate. */
	void setX(value_type x) { m_x = x; }

	/** Sets the y-coordinate of this point to the given y coordinate. */
	void setY(value_type y) { m_y = y; }

	/** Sets the x- and y-coordinates of this point to the given
	 * coordinates. */
	void setXY(value_type x, value_type y) { m_x = x; m_y = y; }

	/**
	 * Normalizes this vector (point is treated as a vector here) to length 1.0.
	 * If the vector has zero length (isZero() returns true), it will be left unchanged.
	 */
	void normalize();

	/**
	 * Rotates this point around the orign (0,0)
	 * Same as rotated, but modifies the content.
	 */
	void rotate(value_type angle);

	/** Sets the coordinates of this point from the given polar
	 * coordinates. */
	void setPolar(value_type dist, value_type angle);

	/** Multiplies this point's coordinates by the given factor, and
	 * returns a reference to this point. */
	Point2D & operator*= ( value_type factor );

	/** Adds the given point to this point and returns a reference to
	 * this point. */
	Point2D & operator+= ( const Point2D & point );

	/** Subtracts the given point from this point and returns a
	 * reference to this point. */
	Point2D & operator-= ( const Point2D & point );

	/** Divides both x and y by the given divisor, and returns a
	 * reference to this point. */
	Point2D & operator/= ( value_type divisor );

	//\}


	/** \name Geometrical relations to other objects */
	//\{

	/** Returns the Euclidian distance to the given point,
	 * sqrt( (x_1-x_2)^2 + (y_1-y_2)^2 ). */
	value_type dist( const Point2D & point ) const;

	/** Returns the Euclidian distance to the given point,
	 * sqrt( (x_1-x_2)^2 + (y_1-y_2)^2 ). */
//	value_type dist( const ScanPoint & point ) const;

	/** Returns the square of the Euclidian distance to the given Point,
	 * (x_1-x_2)^2 + (y_1-y_2)^2.
	 */
	value_type distSquare ( const Point2D & point ) const;

	/** Returns the angle between this and another vector */
	value_type angle(const Point2D& point) const;

	//\}

	/** \name Serialization */
	//\{

	/// Reads a Point2D from an input stream
	/**
	 * \param is The input stream
	 * \param version 1 == compressed meter values, 4 bytes total; 2 == float values, 8 bytes total
	 */
	std::istream& read (std::istream& is, UINT32 version);

	/// Reads a Point2D from a memory buffer and increments the buffer pointer
	/**
	 * \param buf The memory buffer
	 * \param version 1 == compressed meter values, 4 bytes total; 2 == float values, 8 bytes total
	 */
	void read (const BYTE*& buf, UINT32 version);

	/// Writes a Point2D to an output stream
	/**
	 * \param os The output stream
	 * \param version 1 == compressed meter values, 4 bytes total; 2 == float values, 8 bytes total
	 */
	std::ostream& write (std::ostream& os, UINT32 version) const;

	/// Writes a Point2D to a memory buffer and increments the buffer pointer
	/**
	 * \param buf The memory buffer
	 * \param version 1 == compressed meter values, 4 bytes total; 2 == float values, 8 bytes total
	 */
	void write (BYTE*& buf, UINT32 version) const;

	/// Text output for debugging
	std::string toString(UINT16 digits = 2) const;

	//\}

	/** Returns a newly constructed Point2D that is a cartesian
	 * representation of the given polar coordinates. */
	static Point2D fromPolar(value_type dist, value_type angle);

	friend inline bool operator==(const Point2D &, const Point2D &);
	friend inline bool operator!=(const Point2D &, const Point2D &);
	friend inline const Point2D operator+(const Point2D &, const Point2D &);
	friend inline const Point2D operator-(const Point2D &, const Point2D &);
	friend inline const Point2D operator*(value_type, const Point2D &);
	friend inline const Point2D operator*(const Point2D &, value_type);
	friend inline Point2D::value_type operator*(const Point2D &p1, const Point2D &p2);
	friend inline const Point2D operator-(const Point2D &);
	friend inline const Point2D operator/(const Point2D &, value_type);
};

// Text output for debugging
std::ostream& operator<<(std::ostream& os, const Point2D& point);

// Inline functions

inline void Point2D::setPolar(value_type r, value_type angle)
{
	m_x = r * std::cos(angle);
	m_y = r * std::sin(angle);
}

inline Point2D & Point2D::operator*= ( value_type factor )
{
	m_x *= factor;
	m_y *= factor;
	return *this;
}

inline Point2D & Point2D::operator+= ( const Point2D & point )
{
	m_x += point.m_x;
	m_y += point.m_y;
	return *this;
}

inline Point2D & Point2D::operator-= ( const Point2D & point )
{
	m_x -= point.m_x;
	m_y -= point.m_y;
	return *this;
}

inline Point2D & Point2D::operator/= ( value_type divisor )
{
	assert(!fuzzyCompare(divisor, value_type(0.0)));
	m_x /= divisor;
	m_y /= divisor;
	return *this;
}

inline bool Point2D::isZero() const
{
	return (fuzzyCompare(m_x, value_type(0.0)) && fuzzyCompare(m_y, value_type(0.0)));
}

// Inline operators

inline bool operator==(const Point2D &p1, const Point2D &p2)
{
	return (fuzzyCompare(p1.m_x, p2.m_x)
			|| (isNaN(p1.m_x) && isNaN(p2.m_x)))
		   && (fuzzyCompare(p1.m_y, p2.m_y)
			   || (isNaN(p1.m_y) && isNaN(p2.m_y)));
}

inline bool operator!=(const Point2D &p1, const Point2D &p2)
{
	return ! operator==(p1, p2);
}

inline const Point2D operator+(const Point2D &p1, const Point2D &p2)
{
	return Point2D(p1.m_x + p2.m_x, p1.m_y + p2.m_y);
}

inline const Point2D operator-(const Point2D &p1, const Point2D &p2)
{
	return Point2D(p1.m_x - p2.m_x, p1.m_y - p2.m_y);
}

inline const Point2D operator*(const Point2D &p, Point2D::value_type factor)
{
	return Point2D(p.m_x * factor, p.m_y * factor);
}

inline const Point2D operator*(Point2D::value_type factor, const Point2D &p)
{
	return Point2D(p.m_x * factor, p.m_y * factor);
}

inline Point2D::value_type operator*(const Point2D &p1, const Point2D &p2)
{
	return p1.m_x * p2.m_x + p1.m_y * p2.m_y;
}

inline const Point2D operator-(const Point2D &p)
{
	return Point2D(-p.m_x, -p.m_y);
}

inline const Point2D operator/(const Point2D &p, Point2D::value_type divisor)
{
	assert(!fuzzyCompare(divisor, Point2D::value_type(0.0)));
	return Point2D(p.m_x / divisor, p.m_y / divisor);
}


inline Point2D::value_type Point2D::dist() const
{
	return hypot(m_x, m_y);
}

inline Point2D::value_type Point2D::angle() const
{
	return atan2(m_y, m_x);
}

inline Point2D::value_type Point2D::angle(const Point2D& point) const
{
	const double divisor(dist() * point.dist());
	assert(!fuzzyCompare(divisor, 0.0));

	return acos( (*this * point) / divisor );
}

inline void Point2D::toPolar(Point2D::value_type& r, Point2D::value_type& psi) const
{
	r = dist();
	psi = angle();
}

inline std::pair<Point2D::value_type, Point2D::value_type> Point2D::toPolar() const
{
	return std::make_pair(dist(), angle());
}

}	// namespace datatypes

#endif
