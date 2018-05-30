//
// Point3D.hpp
// A 3-dimensional cartesian point.
//


#ifndef POINT3D_HPP
#define POINT3D_HPP

#include <cassert>
#include <sstream>
#include <iosfwd> // for istream, ostream
#include <utility> // for std::pair<T,T>
#include "../BasicDatatypes.hpp"
#include "../tools/MathToolbox.hpp"		// for fuzzyCompare

namespace datatypes
{
	
class Point2D;


/** \brief This class defines a point in the three-dimensional plane.
 */
class Point3D : public BasicData
{
public:
	/// The type of the stored x and y coordinates.
	typedef double floatingpoint_type;

private:
	double m_x;
	double m_y;
	double m_z;

public:
	/** Constructs a point with the given coordinates (x, y, z). */
	Point3D(double x, double y, double z)
		: m_x(x)
		, m_y(y)
		, m_z(z)
	{m_datatype = Datatype_Point3D;}

	/** Constructs a null point, i.e. with coordinates (0.0, 0.0, 0.0) */
	Point3D()
		: m_x(0.0)
		, m_y(0.0)
		, m_z(0.0)
	{m_datatype = Datatype_Point3D;}

	/// Constructs a point from the given Point2D. Z-component will be zero.
	explicit Point3D(const Point2D& p);
	
	// Estimate the memory usage of this object
	inline virtual const UINT32 getUsedMemory() const {return sizeof(*this);};



	/** \name Accessor methods for information about this point */
	//\{

	bool isZero() const;				///< Check against (near-)zero
	double getX() const { return m_x; }	///< Returns the x-coordinate of this point.
	double getY() const { return m_y; }	///< Returns the y-coordinate of this point.
	double getZ() const { return m_z; }	///< Returns the z-coordinate of this point.

	double length() const;				///< Length of the vector (identical to distFromOrigin())
	double distFromOrigin() const;		///< Dist from the point (0,0,0) to this point

	double getAngleAroundX() const;	///< Returns the rotation angle around x (in the z-y-plane)
	double getAngleAroundY() const;	///< Returns the rotation angle around y (in the z-x-plane)
	double getAngleAroundZ() const;	///< Returns the rotation angle around z (in the x-y-plane)

	/// Returns the x/y components of this class, converted into a Point2D object.
	Point2D toPoint2D() const;
	//\}

	/** \name Setter methods for changing this point */
	//\{

	void setX(double x) { m_x = x; }	///< Sets the x-coordinate of this point to the given value.
	void setY(double y) { m_y = y; }	///< Sets the y-coordinate of this point to the given value.
	void setZ(double z) { m_z = z; }	///< Sets the z-coordinate of this point to the given value.
	void setXYZ(double x, double y, double z) { m_x = x; m_y = y; m_z = z; } ///< Sets the coordinates of this point to the given values

	void rotateAroundX(double rollAngle);	///< Rotate the point around the X-axis ("Roll angle")
	void rotateAroundY(double pitchAngle);	///< Rotate the point around the Y-axis ("Pitch angle")
	void rotateAroundZ(double dYawAngle);	///< Rotate the point around the Z-axis ("Yaw angle")

	void normalize();	///< Normalizes this vector (point) to length 1.0

	Point3D & operator+= ( const Point3D & point );	///< Adds the given point to this point and returns a reference to this point.
	Point3D & operator-= ( const Point3D & point );	///< Subtracts the given point to this point and returns a reference to this point.
	Point3D & operator/= ( double divisor );
	//\}


	/** \name Geometrical relations to other objects */
	//\{
	double dist( const Point3D& point ) const;		///< Calculates the distance to the given point
	//\}


	/** \name Geometrical relations to other objects */
	//\{
	static Point3D vectorProduct(const Point3D& v1, const Point3D& v2);	///< Returns the vector product ("Kreuzprodukt") of the two vectors
	static Point3D calcIntersectionPointOfVectorWithPlane(const Point3D& PlaneStartpoint,
			const Point3D& PlaneNormal,
			const Point3D& VectorStartpoint,
			const Point3D& VectorDirection);
	static double getDistanceBetweenPoints(const Point3D& pt1, const Point3D& pt2);	///< Returns the distance between the two point coordinates
	//\}


	/** \name Serialization */
	//\{

	/// Reads a Point3D from an input stream
	/**
	 * \param is The input stream
	 */
	std::istream& read (std::istream& is);	// , UINT32 version);	///< Reads a Point3D from an input stream
	void read (const BYTE*& buf);	// , UINT32 version);			///< Reads a Point3D from a memory buffer and increments the buffer pointer
	std::ostream& write (std::ostream& os) const;	// , UINT32 version) const;	///< Writes a Point3D to an output stream
	void write (BYTE*& buf) const;	//, UINT32 version) const;			///< Writes a Point2D to a memory buffer and increments the buffer pointer
	std::string toString() const;							///< Text output for debugging

	//\}

	static std::streamsize getSerializedSize();	// UINT32 version);	///< Returns the number of bytes this object needs in serialized form

	friend inline bool operator==(const Point3D &, const Point3D &);
	friend inline bool operator!=(const Point3D &, const Point3D &);
	friend inline const Point3D operator+(const Point3D &, const Point3D &);
	friend inline const Point3D operator-(const Point3D &, const Point3D &);
	friend inline double operator*(const Point3D &, const Point3D &);
	friend inline const Point3D operator*(double, const Point3D &);
	friend inline const Point3D operator*(const Point3D &, double);
	friend inline const Point3D operator-(const Point3D &);
	friend inline const Point3D operator/(const Point3D &, double);
};

std::ostream& operator<<(std::ostream& os, const Point3D& point);	///< Text output for debugging

// Inline functions

inline Point3D & Point3D::operator+= ( const Point3D & point )
{
	m_x += point.m_x;
	m_y += point.m_y;
	m_z += point.m_z;
	return *this;
}

inline Point3D & Point3D::operator-= ( const Point3D & point )
{
	m_x -= point.m_x;
	m_y -= point.m_y;
	m_z -= point.m_z;
	return *this;
}

inline Point3D & Point3D::operator/= ( double divisor )
{
	assert(!fuzzyCompare(divisor, 0.0));
	m_x /= divisor;
	m_y /= divisor;
	m_z /= divisor;

	return *this;
}

inline bool operator==(const Point3D &p1, const Point3D &p2)
{
	return (fuzzyCompare(p1.m_x, p2.m_x)
			|| (isNaN(p1.m_x) && isNaN(p2.m_x)))
		   && (fuzzyCompare(p1.m_y, p2.m_y)
			   || (isNaN(p1.m_y) && isNaN(p2.m_y)))
		   && (fuzzyCompare(p1.m_z, p2.m_z)
			   || (isNaN(p1.m_z) && isNaN(p2.m_z)));
}
inline bool operator!=(const Point3D &p1, const Point3D &p2)
{
	return ! operator==(p1, p2);
}

inline const Point3D operator+(const Point3D &p1, const Point3D &p2)
{
	return Point3D(p1.m_x + p2.m_x, p1.m_y + p2.m_y, p1.m_z + p2.m_z);
}


inline const Point3D operator-(const Point3D &p1, const Point3D &p2)
{
	return Point3D(p1.m_x - p2.m_x, p1.m_y - p2.m_y, p1.m_z - p2.m_z);
}

// wert = v1 * v2
inline double operator*(const Point3D &p1, const Point3D &p2)
{
	return ((p1.m_x * p2.m_x) + (p1.m_y * p2.m_y) + (p1.m_z * p2.m_z));
}


inline const Point3D operator*(const Point3D &p, double factor)
{
	return Point3D(p.m_x * factor, p.m_y * factor, p.m_z * factor);
}

inline const Point3D operator*(double factor, const Point3D &p)
{
	return Point3D(p.m_x * factor, p.m_y * factor, p.m_z * factor);
}


inline const Point3D operator-(const Point3D &p)
{
	return Point3D(-p.m_x, -p.m_y, -p.m_z);
}

inline const Point3D operator/(const Point3D &p, double divisor)
{
	assert(fuzzyCompare(divisor, 0.0) == false);
	return Point3D(p.m_x / divisor, p.m_y / divisor, p.m_z / divisor);
}

// ////////////////////////////////////////////////////////////

}	// namespace datatypes


#endif

