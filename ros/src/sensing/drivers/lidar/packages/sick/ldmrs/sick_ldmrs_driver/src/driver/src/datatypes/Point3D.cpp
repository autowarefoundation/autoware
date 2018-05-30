//
// Point3D.cpp
// A point in the 3-D cartesian plane. All data is stored in 64-bit-"double"-precision.
//

#define _USE_MATH_DEFINES // for MSVC
#include <cmath>
#include "Point2D.hpp"
#include "Point3D.hpp"

namespace datatypes
{
	
//
// Some of the non-trivial member functions
//

//
// Constructs a point from the given ScanPoint
//
Point3D::Point3D(const Point2D& p)
	: m_x(double(p.getX()))
	, m_y(double(p.getY()))
	, m_z(0.0)
{
	m_datatype = Datatype_Point3D;
}


/**
 * Returns true if this point is zero in terms of the machine
 * precision, that is, its value is exactly zero or "almost
 * zero".
 */
bool Point3D::isZero() const
{
	return (fuzzyCompare(m_x, 0.0) &&
			fuzzyCompare(m_y, 0.0) &&
			fuzzyCompare(m_z, 0.0));
}


double Point3D::dist( const Point3D& point ) const
{
	return hypot(m_x - point.getX(), m_y - point.getY(), m_z - point.getZ());
}

Point2D Point3D::toPoint2D() const
{
	Point2D result(m_x, m_y);
	return result;
}

/**
 * Returns the vector product ("Kreuzprodukt") of the two vectors.
 * Note that the returned vector product is not normalized (Hint: use normalize() to do this).
 */
Point3D Point3D::vectorProduct(const Point3D& v1, const Point3D& v2)
{
	Point3D kreuz;

	kreuz.m_x = + ((v1.getY() * v2.getZ()) - (v1.getZ() * v2.getY()));
	kreuz.m_y = -((v1.getX() * v2.getZ()) - (v1.getZ() * v2.getX()));
	kreuz.m_z = + ((v1.getX() * v2.getY()) - (v1.getY() * v2.getX()));

	return kreuz;
}

/**
 * Normalizes this vector (point is treated as a vector here) to length 1.0.
 *
 * If the vector has zero length (isZero() returns true), it will be left unchanged.
 */
void Point3D::normalize()
{
	if (isZero())
		// Vector has zero length. Such a vector will be left unchanged.
		return;

	double len = length();
	// To be really sure about avoiding division-by-zero, we check again here
	if (fuzzyCompare(len, 0.0))
	{
		// Vector has zero length. Such a vector will be left unchanged.
		return;
	}

	*this /= len;
}


/**
 * Rotate the point around the Z axis (with a yaw angle). The input is an angle in [rad].
 * Positive angles rotate counterclockwise.
 */
void Point3D::rotateAroundZ(double yawAngle)
{
	double dCos = cos(yawAngle);
	double dSin = sin(yawAngle);

	double x = m_x * dCos - m_y * dSin;
	double y = m_x * dSin + m_y * dCos;

	m_x = x;
	m_y = y;
}


/**
 * Rotate the point around the X axis (with a roll angle). The input is an angle in [rad].
 * Positive angles rotate clockwise (seen from the origin).
 */
void Point3D::rotateAroundX(double rollAngle)
{
	double dCos = cos(rollAngle);
	double dSin = sin(rollAngle);

	double y = -m_z * dSin + m_y * dCos;
	double z = m_z * dCos + m_y * dSin;

	m_z = z;
	m_y = y;
}

/**
 * Rotate the point around the Y axis (with a pitch angle). The input is an angle in [rad].
 * Positive angles rotate clockwise (seen from the origin).
 */
void Point3D::rotateAroundY(double pitchAngle)
{
	double dCos = cos(pitchAngle);
	double dSin = sin(pitchAngle);

	double x = m_z * dSin + m_x * dCos;
	double z = m_z * dCos - m_x * dSin;

	m_z = z;
	m_x = x;
}


/**
 * Calculates the dist from the origin (0,0,0) to the point. Assuming the point is a vector,
 * this is the length of the vector (see also length()).
 */
double Point3D::distFromOrigin() const
{
	return hypot(m_x, m_y, m_z);
}

/**
 * Calculates the dist from the origin (0,0,0) to the point. Assuming the point is a vector,
 * this is the length of the vector.
 */
double Point3D::length() const
{
	return distFromOrigin();
}


/**
 * Returns the angle that the projection of the point onto the x-y-plane has. This angle
 * is measured against the 0-degree-direction (x axis).
 * Note that the z-coordinate has no effect here.
 *
 *           ^ x
 *           |
 *           |
 *           |
 *  <--------+---------
 *  y        |
 *           |  (z points upwards)
 *           |
 *
 * The given angle is measured against the x axis, positive angles are
 * counterclockwise ("to the left").
 *
 */
double Point3D::getAngleAroundZ() const
{
	double angle;

	// atan2(y,x) returns the angle against the x-axis
	angle = std::atan2(m_y, m_x);

	return angle;
}

/**
 * Returns the angle that the projection of the point onto the x-z-plane has. This angle
 * is measured against the 0-degree-direction (z axis).
 * Note that the y-coordinate has no effect here.
 *
 *           ^ z
 *           |
 *           |
 *           |
 *  <--------+---------
 *  x        |
 *           |  (y points upwards)
 *           |
 *
 * The given angle is measured against the z axis, positive angles are
 * counterclockwise ("to the left").

 */
double Point3D::getAngleAroundY() const
{
	double angle;

	// atan2(y,x) returns the angle against the x-axis
	angle = std::atan2(m_x, m_z);

	return angle;
}


/**
 * Returns the angle that the projection of the point onto the z-y-plane has. This angle
 * is measured against the 0-degree-direction (y axis).
 * Note that the z-coordinate has no effect here.
 *
 *           ^ y
 *           |
 *           |
 *           |
 *  <--------+---------
 *  z        |
 *           |  (x points upwards)
 *           |
 *
 * The given angle is measured against the y axis, positive angles are
 * counterclockwise ("to the left").
 */
double Point3D::getAngleAroundX() const
{
	double angle;

	// atan2(z,y) returns the angle against the y-axis
	angle = std::atan2(m_z, m_y);

	return angle;
}



/**
 * Returns the distance between the two 3d-point coordinates.
 */
double Point3D::getDistanceBetweenPoints(const Point3D& pt1, const Point3D& pt2)
{
	return pt1.dist(pt2);
}





/**
 * Convert point to text string for debugging
 */
std::string Point3D::toString() const
{
//	std::string text;
	std::ostringstream ostr;
	ostr << *this;
	return ostr.str();
}

/**
 * Text output for debugging
 */
std::ostream& operator<<(std::ostream& os, const Point3D& point)
{
	os << "(" << point.getX() << ", " << point.getY() << ", " << point.getZ() << ")";
	return os;
}


/**
 * Calculates the intersection point between a plane and a vector.
 * The vector is given with a start point (Deutsch: Aufpunkt) and a direction vector
 * (Deutsch: Richtungsvektor).
 * The plane is given as a start point (Deutsch: Aufpunkt) and a normal vector (Deutsch: Normalenvektor).
 *
 * Note that the plane has infinite size, so the intersection point may not be where you expect
 * it to be, e.g. "behind" the vector start point, that is, in negative vector direction.
 * Note also that there may not be an intersection point. In this case, the resulting point will
 * be NaN in all components.
 *
 * Is this function better located in geom3D?
 */
Point3D Point3D::calcIntersectionPointOfVectorWithPlane(const Point3D& PlaneStartpoint,
		const Point3D& PlaneNormal,
		const Point3D& VectorStartpoint,
		const Point3D& VectorDirection)
{
	Point3D intersectionPoint;
	double nenner;

	nenner = PlaneNormal * VectorDirection;
	if (fuzzyCompare(nenner, 0.0) == false)
	{
		// Calc intersection point
		intersectionPoint = VectorStartpoint + ((PlaneNormal * (PlaneStartpoint - VectorStartpoint)) / nenner) * VectorDirection;
	}
	else
	{
		// Vector points "along" the plane, so there is no intersection point.
		intersectionPoint.setXYZ(NaN_double, NaN_double, NaN_double);
	}

	return intersectionPoint;
}

}	// namespace datatypes
