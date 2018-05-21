//
// ScanPoint.cpp
//

#include "ScanPoint.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory.h>	// for memset()
#include "../tools/errorhandler.hpp"
#include "ScannerInfo.hpp"
#include "Point3D.hpp"
#include "Point2D.hpp"


//////////////////////////////////////////////////////////////////////////
// ScanPoint
//////////////////////////////////////////////////////////////////////////
namespace datatypes
{

ScanPoint::ScanPoint()
{
}

void ScanPoint::clear()
{
	memset (this, 0, sizeof(*this));

	m_x = NaN_double;
	m_y = NaN_double;
	m_z = NaN_double;
	m_dist = NaN_double;
	m_hAngle = NaN_double;
	m_vAngle = NaN_double;
}

/**
 * Equivalent polar coordinates are computed on the fly.
 *
 * \param x x-coordinate in meters
 * \param y y-coordinate in meters
 * \param z z-coordinate in meters
 */
void ScanPoint::setCartesian (double x, double y, double z)
{
	m_x = x;
	m_y = y;
	m_z = z;

	updatePolar();		// compute equivalent polar coordinates
}

/**
 * Equivalent polar coordinates are computed on the fly.
 *
 * \param pt Point coordinates in [m]
 */
void ScanPoint::setPoint3D(const Point3D& pt)
{
	setCartesian(double(pt.getX()), double(pt.getY()), double(pt.getZ()));
}


/**
 * Returns the distance, in [m], between the two 3d-scanpoint coordinates.
 */
double ScanPoint::getDistanceBetweenScanpoints(const ScanPoint& pt1, const ScanPoint& pt2)
{
	return pt1.getDist(pt2);
}

double ScanPoint::getDist(const ScanPoint& other) const
{
	// We use "double" for the temporaries because we hope this might
	// give better precision when squaring. Who knows.
	const double x = other.m_x - m_x;
	const double y = other.m_y - m_y;
	const double z = other.m_z - m_z;
	return double(::hypot(x, y, z));
}

double ScanPoint::getDist2D(const ScanPoint& other) const
{
	// We use "double" for the temporaries because we hope this might
	// give better precision when squaring. Who
	// knows. ::hypot_precise() will for sure have better
	// precision, but it might be slower. We stick with this
	// workaround so far.
	const double x = other.m_x - m_x;
	const double y = other.m_y - m_y;
	return double(::hypot(x, y));
}

/**
 * Equivalent Cartesian coordinates are computed on the fly.
 *
 * \param dist Radial distance in meters
 *
 * \param hAngle Horizontal/azimuth angle in radians. The angle will
 * be normalized into the interval [-pi,+pi).
 *
 * \param vAngle Vertical/elevation angle in radians. The angle will
 * be normalized into the interval [-pi,+pi).
 */
void ScanPoint::setPolar (double dist, double hAngle, double vAngle)
{
	m_dist   = dist;
	m_hAngle = normalizeRadians(hAngle);
	m_vAngle = normalizeRadians(vAngle);

	updateCartesian();
}


Point3D ScanPoint::toPoint3D() const
{
	Point3D pt;
	pt.setXYZ(m_x, m_y, m_z);
	return pt;
}

Point2D ScanPoint::toPoint2D() const
{
	Point2D pt;
	pt.setXY(m_x, m_y);
	return pt;
}

//
// Useful e.g. to add the offset of the mounting position of a laserscanner.
//
// \param xOffset x-offset in meters
// \param yOffset y-offset in meters
// \param zOffset z-offset in meters
//
void ScanPoint::addCartesianOffset (double xOffset, double yOffset, double zOffset)
{
	m_x += xOffset;
	m_y += yOffset;
	m_z += zOffset;

	updatePolar();		// compute equivalent polar coordinates
}

/**
 * Useful e.g. to add the horizontal and vertical angle offsets of the
 * mounting position of a laserscanner.
 *
 * \param distOffset   Radial offset in meters
 *
 * \param hAngleOffset Horizontal (yaw) angle offset in radians. The
 * resulting angle will be normalized into the interval [-pi,+pi).
 *
 * \param vAngleOffset Vertical (pitch) angle offset in radians. The
 * resulting angle will be normalized into the interval [-pi,+pi).
 */
void ScanPoint::addPolarOffset (double distOffset, double hAngleOffset, double vAngleOffset)
{
	m_dist   += distOffset;
	m_hAngle = normalizeRadians(m_hAngle + hAngleOffset);
	m_vAngle = normalizeRadians(m_vAngle + vAngleOffset);

	updateCartesian();		// compute equivalent Cartesian coordinates
}

/**
 * This method is called when the Cartesian coordinates have changed,
 * to keep polar and Cartesian coordinates consistent.
 *
 * Moreover, since polar coordinates are redundant information, they are not
 * part of the external representation of a scan point. Upon input, the polar
 * coordinates are reconstructed using this method.
 *
 * \note Note that up to 6. June 2009, resulting vertical (pitch)
 * angles from negative \a z components erroneously had a negative
 * sign. Now, the resulting pitch angles are positive to comply with DIN
 * 70000, i.e. positive pitch angles are pointing "downwards".
 */
void ScanPoint::updatePolar()
{
	// diag^2 = x^2 + y^2
	const double diag2 = m_x * m_x + m_y * m_y;

	m_dist   = sqrt (diag2 + m_z * m_z);

	// Normalize v/hAngle here as well to maintain the invariant that
	// we're always strictly inside the interval of
	// normalizeRadians().
	m_hAngle = normalizeRadians( atan2 (m_y, m_x));
	m_vAngle = normalizeRadians(-atan2 (m_z, sqrt(diag2)));

	// Note: This calculation is expensive - two atan2() for each scan
	// point. We already checked whether we can get rid of some of
	// these steps by instead using a "bool m_polarIsValid" cache flag
	// and calculate the m_hAngle and m_vAngle only when getHAngle()
	// etc is called. However, it turned out almost all appbase graphs
	// use the polar coordinates in any coordinate system (e.g. dirt
	// detection in scanner coordinates, distance thresholds in
	// vehicle coordinates). For this reason, the potential savings
	// are more than offset by the overhead of the additional check of
	// the cache flag, and we removed any such cache implementation
	// again because valgrind told us the overhead increased the
	// overall complexity instead of reducing it. There might be an
	// exception if only the m_vAngle is calculated on-demand, but on
	// the other hand the saving is probably not that much anymore, so
	// we just leave the implementation as-is.
}

/**
 * This method is called when the polar coordinates have changed,
 * to keep polar and Cartesian coordinates consistent.
 *
 * \note Note that up to 6. June 2009, a negative vertical angle resulted in a neg. \a z component which was incorrect. Now,
 * the resulting \a z component is positive to comply with DIN 70000.
 */
void ScanPoint::updateCartesian()
{
	// diag = m_dist projected onto x-y-plane
	const double diag = m_dist * cos(m_vAngle);

	// Pitch and Yaw transformation:
	// x = dist *  cos(pitch) * cos(yaw)
	// y = dist *  cos(pitch) * sin(yaw)
	// z = dist * -sin(pitch)

	m_x =   diag * cos(m_hAngle);
	m_y =   diag * sin(m_hAngle);
	m_z = m_dist * -sin(m_vAngle);
}

/**
 * The resolution of the echo width is reduced in the same way as
 * compressMeters() does. This is to ensure that compressed replay data
 * is the same as uncompressed live data.
 */
void ScanPoint::setEchoWidth (double echoWidth)
{
	m_echoWidth = echoWidth;
}


std::string ScanPoint::toString() const
{
	std::ostringstream ostr;
	ostr << *this;
	return ostr.str();
}

// Text output for debugging
std::ostream& operator<<(std::ostream& os, const ScanPoint& point)
{
	os << "[(" << point.getX() << ", " << point.getY() << ", " << point.getZ() << ")"
	   << " channel " << int(point.getLayer())
	   << " subch " << int(point.getEchoNum())
	   << " devID " << int(point.getSourceId())
	   << "]"
	   ;
	return os;
}

bool operator==(const ScanPoint &p1, const ScanPoint &p2)
{
	return (p1.getSourceId() == p2.getSourceId())
		   && (p1.getLayer() == p2.getLayer())
		   && (p1.getEchoNum() == p2.getEchoNum())
		   && (p1.getX() == p2.getX() || (::isNaN(p1.getX()) && ::isNaN(p2.getX())))
		   && (p1.getY() == p2.getY() || (::isNaN(p1.getY()) && ::isNaN(p2.getY())))
		   && (p1.getZ() == p2.getZ() || (::isNaN(p1.getZ()) && ::isNaN(p2.getZ())))
		   ;
}

bool operator!=(const ScanPoint &p1, const ScanPoint &p2)
{
	return !(p1 == p2);
}


}	// namespace datatypes

