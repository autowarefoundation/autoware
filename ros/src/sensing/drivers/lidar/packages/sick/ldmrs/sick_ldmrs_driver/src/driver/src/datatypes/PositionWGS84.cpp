// PositionWGS84.cpp
//
//
#include <cmath>

#include "PositionWGS84.hpp"
#include "../tools/MathToolbox.hpp"
#include <sstream>
#include <iomanip>	// for setprecision
#include "../tools/toolbox.hpp"
#include "../tools/errorhandler.hpp"

namespace datatypes
{

/**
* \brief Constructor.
*
* All values are initialized with default values: latitude, longitude, and
* altitude are set to zero, all sigma values are set to negative values and
* the source is set to SourceType::Unknown.
*/
PositionWGS84::PositionWGS84()
{
	resetToDefault();
}


PositionWGS84::~PositionWGS84()
{
}


double PositionWGS84::distanceToPos(const PositionWGS84& pos) const
{
	double x, y, dist;
	transformToTangentialPlane(pos, &x, &y);
	dist = hypot(x, y);
	return dist;
}


bool PositionWGS84::operator==(const PositionWGS84& other) const
{
	return
		m_timeOfMeasurement == other.m_timeOfMeasurement
		&& m_sourceId == other.m_sourceId
		&& m_latitude == other.m_latitude
		&& m_latitudeSigma == other.m_latitudeSigma
		&& m_longitude == other.m_longitude
		&& m_longitudeSigma == other.m_longitudeSigma
		&& m_altitudeMSL == other.m_altitudeMSL
		&& m_altitudeMSLSigma == other.m_altitudeMSLSigma
		&& m_courseAngle == other.m_courseAngle
		&& m_courseAngleSigma == other.m_courseAngleSigma
		&& m_yawAngle == other.m_yawAngle
		&& m_yawAngleSigma == other.m_yawAngleSigma
		&& m_pitchAngle == other.m_pitchAngle
		&& m_pitchAngleSigma == other.m_pitchAngleSigma
		&& m_rollAngle == other.m_rollAngle
		&& m_rollAngleSigma == other.m_rollAngleSigma
		&& m_source == other.m_source
		;
}

/**
* \brief Resets all values of the position to their default values.
*
* \sa PositionWGS84()
*/
void PositionWGS84::resetToDefault()
{
	m_sourceId				= 0;
	m_latitude				= NaN_double;
	m_latitudeSigma			= NaN_double;
	m_longitude				= NaN_double;
	m_longitudeSigma		= NaN_double;
	m_altitudeMSL			= NaN_double;
	m_altitudeMSLSigma		= NaN_double;
	m_courseAngle			= NaN_double;
	m_courseAngleSigma		= NaN_double;
	m_yawAngle				= NaN_double;
	m_yawAngleSigma			= NaN_double;
	m_pitchAngle			= NaN_double;
	m_pitchAngleSigma		= NaN_double;
	m_rollAngle				= NaN_double;
	m_rollAngleSigma		= NaN_double;
	m_source 				= Unknown;
}

void PositionWGS84::setLatitudeInRad(double new_latitude)
{
	m_latitude = new_latitude;

	if ((new_latitude < -M_PI_2) || (new_latitude > M_PI_2)) // note: M_PI_2 = M_PI / 2
	{
		printWarning("PositionWGS84::setLatitude: The given latitude (" +
						 ::toString(new_latitude * rad2deg, 6) +
						 " deg.) is outside the definition range for this value which is [-90,90]. " +
						 "Ignoring this error condition here - hopefully this value isn't used anywhere.");
	}
}

void PositionWGS84::setLongitudeInRad(double new_longitude)
{
	m_longitude = new_longitude;

	if ((new_longitude < -M_PI) || (new_longitude > M_PI))
	{
		printWarning("PositionWGS84::setLongitude: The given longitude (" +
						 ::toString(new_longitude * rad2deg, 6) +
						 " deg.) is outside the definition range for this value which is [-180,180]. " +
						 "Ignoring this error condition here - hopefully this value isn't used anywhere.");
	}
}

/**
* \brief Calculates the WGS84 coordinate in rad from the mixed degree-minute value usually found on NMEA compatible GPS receivers.
*
* \param[in] Dm Mixed degree-minute value DDDmm.mmmm
* \return Angle in [rad].
*/
double PositionWGS84::NMEA2rad( double Dm )
{
	// Check this website for more information: http://home.online.no/~sigurdhu/Deg_formats.htm
	INT16 D = static_cast<INT16>(Dm / 100.0);
	double m = Dm - 100.0 * D;
	return (D + m / 60.0) * deg2rad;
}

/**
* \brief Sets the latitude value of the WGS-84 position.
*
* \param[in] Dm Latitude value [0...9000] where format is DDmm.mmmmm. D stands for value in degree and m for value in decimal minutes.
* \param[in] H Hemisphere. For Latitude it can be [N]orth or [S]outh.
*
* \sa setLatitudeInDeg()
* \sa setLatitudeInRad()
* \sa setLongitudeInRad()
* \sa setLongitudeInDeg()
* \sa setLongitudeInNMEA()
* \sa setAltitudeInMeter()
* \sa getLatitudeInRad()
* \sa getLatitudeInDeg()
* \sa getLongitudeInRad()
* \sa getLongitudeInDeg()
*/
void PositionWGS84::setLatitudeInNMEA( double Dm, char H )
{
	if ( H == 'S' || H == 's' )
	{
		setLatitudeInRad(-NMEA2rad( Dm ));
	}
	else
	{
		setLatitudeInRad( NMEA2rad( Dm ));
	}
}

/**
* \brief Sets the longitude value of the WGS-84 position.
*
* \param[in] Dm Longitude value [0...18000] where format is DDDmm.mmmmm. D stands for value in degree and m for value in decimal minutes.
* \param[in] H Hemisphere. For Longitude it can be [W]est or [E]ast.
*
* \sa setLongitudeInDeg()
* \sa setLongitudeInNMEA()
* \sa setLatitudeInRad()
* \sa setLatitudeInDeg()
* \sa setLatitudeInNMEA()
* \sa setAltitudeInMeter()
* \sa getLatitudeInRad()
* \sa getLatitudeInDeg()
* \sa getLongitudeInRad()
* \sa getLongitudeInDeg()
*/
void PositionWGS84::setLongitudeInNMEA( double Dm, char H )
{
	if ( H == 'W' || H == 'w' )
	{
		setLongitudeInRad(-NMEA2rad( Dm ));
	}
	else
	{
		setLongitudeInRad( NMEA2rad( Dm ));
	}
}


/**
* \brief Returns a std::string with a describtion of the WGS-84 position.
*
* \return String with describtion.
*/
std::string PositionWGS84::toString() const
{
	std::stringstream out;
	out << "[PositionWGS84(lat:"
		<< std::setprecision(12) << rad2deg * m_latitude
		<< " deg; lon:"
		<< std::setprecision(12) << rad2deg * m_longitude
		<< " deg; hdg:"
		<< rad2deg * m_courseAngle
		<< " deg; alt:"
		<< m_altitudeMSL
		<< " m; yaw:"
		<< rad2deg * m_yawAngle
		<< " deg; pitch:"
		<< rad2deg * m_pitchAngle
		<< " deg; roll:"
		<< rad2deg * m_rollAngle
		<< " deg; source:"
		<< ::toString(m_source)
		<< ")]";
	return out.str();
}

/**
* \brief Returns a std::string with a describtion of the WGS-84 position source.
*
* \return String with current source description.
*/
std::string PositionWGS84::getSourceString() const
{
	return ::toString(m_source);
}

double PositionWGS84::dist( const PositionWGS84& pos ) const
{
	return 6378388 * acos(sin(m_latitude) * sin(pos.getLatitudeInRad()) + cos(m_latitude) * cos(pos.getLatitudeInRad()) * cos(pos.getLongitudeInRad() - m_longitude));
}


/** transformToTangentialPlane()
 *
 * Transforms this point (given in WGS-coordinates) onto a tangential (x-y-) plane. The plane is
 * defined by a reference point (origin), relative to which the x-y-coordinates are calculated.
 * Note that this point should be somewhat close to the origin (several km is ok) as the tangential
 * plane is flat.
 *
 * Die Ausrichtung ist in beiden Koordinatensystemen gleich.
 */
//void PositionWGS84::transformToTangentialPlane(double dLat, double dLong, double dOriginLat, double dOriginLong, double* pdX, double* pdY)
void PositionWGS84::transformToTangentialPlane(const PositionWGS84& origin, double* easting, double* northing) const
{
	double height;
	transformToTangentialPlane(origin, easting, northing, &height);
}

void PositionWGS84::transformToTangentialPlane(const PositionWGS84& origin, double* easting, double* northing, double* height) const
{
	if (isNaN(getLatitudeInRad()) ||
		isNaN(getLongitudeInRad()) ||
		isNaN(getAltitudeInMeterMSL()))
	{
		printWarning("transformToTangentPlan(): At least one of the values latitude, longitude or altitude is NaN. Can't perform transformation! Returning NaN values!");
		*easting = *northing = *height = NaN_double;
		return;
	}

	if (isNaN(origin.getLatitudeInRad()) ||
		isNaN(origin.getLongitudeInRad()) ||
		isNaN(origin. getAltitudeInMeterMSL()))
	{
		printWarning("transformToTangentPlan(): At least one of the origin's values latitude, longitude or altitude is NaN. Can't perform transformation! Returning NaN values!");
		*easting = *northing = *height = NaN_double;
		return;
	}

	// Conversion from WGS84 to ENU (East, North, Up)

	// WGS84 Konstanten
	const double dWGS84_SemiMajorAxis	= 6378137.0;
	const double dWGS84_SemiMinorAxis	= 6356752.3142;
	const double dWGS84_Flatness		= (dWGS84_SemiMajorAxis - dWGS84_SemiMinorAxis) / dWGS84_SemiMajorAxis;
	const double dWGS84_Eccentricity2	= dWGS84_Flatness * (2.0 - dWGS84_Flatness); // Eccentricity squared

	// WGS84 --> ECEF
	const double dLat = getLatitudeInRad();
	const double dLong = getLongitudeInRad();
	const double dHeight = getAltitudeInMeterMSL();		// TODO: Wrong! This should be height above ellipsoid!

	const double dN = dWGS84_SemiMajorAxis / (sqrt(1.0 - (dWGS84_Eccentricity2 * sin(dLat) * sin(dLat))));
	const double dX_ECEF = (dN + dHeight) * cos(dLat) * cos(dLong);
	const double dY_ECEF = (dN + dHeight) * cos(dLat) * sin(dLong);
	const double dZ_ECEF = ((dN * (1.0 - dWGS84_Eccentricity2)) + dHeight) * sin(dLat);

	// ECEF --> TP

	// orgin of the Tangentplane in ECEF-coordinates
	const double dOriginLat = origin.getLatitudeInRad();
	const double dOriginLong = origin.getLongitudeInRad();
	const double dOriginHeight = origin.getAltitudeInMeterMSL();	// TODO: Wrong! This should be height above ellipsoid!

	const double dN_0 = dWGS84_SemiMajorAxis / (sqrt(1.0 - (dWGS84_Eccentricity2 * sin(dOriginLat) * sin(dOriginLat))));
	const double dX_0_ECEF = (dN_0 + dOriginHeight) * cos(dOriginLat) * cos(dOriginLong);
	const double dY_0_ECEF = (dN_0 + dOriginHeight) * cos(dOriginLat) * sin(dOriginLong);
	const double dZ_0_ECEF = ((dN_0 * (1.0 - (dWGS84_Eccentricity2))) + dOriginHeight) * sin(dOriginLat);

	// see "Geographic Coordiante Transformation and
	//      Landmark Navigation" (T.Weiss)
	*easting  = - sin(dOriginLong) * (dX_ECEF - dX_0_ECEF)
				+ cos(dOriginLong) * (dY_ECEF - dY_0_ECEF);

	*northing = - sin(dOriginLat) * cos(dOriginLong) * (dX_ECEF - dX_0_ECEF)
				- sin(dOriginLat) * sin(dOriginLong) * (dY_ECEF - dY_0_ECEF)
				+ cos(dOriginLat)                    * (dZ_ECEF - dZ_0_ECEF);

	*height   =   cos(dOriginLat) * cos(dOriginLong) * (dX_ECEF - dX_0_ECEF)
				  + cos(dOriginLat) * sin(dOriginLong) * (dY_ECEF - dY_0_ECEF)
				  + sin(dOriginLat)                    * (dZ_ECEF - dZ_0_ECEF);
}

/**
 * transformFromTangentialPlane()
 *
 * Inverse function of transformToTangentialPlane: Generates this point from x-y-coordinates
 * on a tangential plane defined by the origin.
 * Note that the x-y-coordinates should be somewhat close to the origin to get accurate results.
 */
//void PositionWGS84::GPS_TransformToWGS(double dX, double dY, double dOriginLat, double dOriginLong, double* pdLat, double* pdLong)
void PositionWGS84::transformFromTangentialPlane(double dX, double dY, const PositionWGS84& origin)
{
	INT32 nNoIterationSteps;
	double dX_ECEF;
	double dY_ECEF;
	double dZ_ECEF;
	double dLat, dLong;
	double dX_0_ECEF;
	double dY_0_ECEF;
	double dZ_0_ECEF;
	double dN;
	double dPhi, dLambda;
	double dP;
	INT32 nCount;


	// Konstanten setzen
	const double dGps_SemiMajorAxis	= 6378137.0;
	const double dGps_SemiMinorAxis	= 6356752.3142;
	const double dGps_Flatness		= (dGps_SemiMajorAxis - dGps_SemiMinorAxis) / dGps_SemiMajorAxis;
	const double dGps_Eccentricity2	= dGps_Flatness * (2.0 - dGps_Flatness);
//	const double dGps_Eccentricity	= sqrt(dGps_Eccentricity2);


	// Winkel werden in Grad gegeben, zum rechnen brauchen wir rad.
	const double dOriginLat = origin.getLatitudeInRad();	// (GPS_PI / 180.0);
	const double dOriginLong = origin.getLongitudeInRad();	// (GPS_PI / 180.0);

	nNoIterationSteps = 200;
	dX_ECEF = 0.0;
	dY_ECEF = 0.0;
	dZ_ECEF = 0.0;

	// Origin of the Tangentplane in ECEF-coordinates
	dN = dGps_SemiMajorAxis /
		 (sqrt(1 - (dGps_Eccentricity2 * sin(dOriginLat) * sin(dOriginLat))));

	dX_0_ECEF = (dN + 0.0) * cos(dOriginLat) * cos(dOriginLong);
	dY_0_ECEF = (dN + 0.0) * cos(dOriginLat) * sin(dOriginLong);
	dZ_0_ECEF = ((dN * (1.0 - (dGps_Eccentricity2))) + 0.0) * sin(dOriginLat);

	// see "Geographic Coordiante Transformation and
	//      Landmark Navigation" (T.Weiss)

	dLambda = dOriginLat;
	dPhi	= dOriginLong;

	dX_ECEF = -cos(dPhi) * sin(dLambda) * dX + sin(dPhi) * dY + cos(dPhi) * cos(dLambda) * 0.0 + dX_0_ECEF;
	dY_ECEF = -sin(dPhi) * sin(dLambda) * dX - cos(dPhi) * dY + sin(dPhi) * cos(dLambda) * 0.0 + dY_0_ECEF;
	dZ_ECEF =  cos(dLambda) * dX + sin(dLambda) * 0.0 + dZ_0_ECEF;


	dN = dGps_SemiMajorAxis;
	dP = sqrt(dX_ECEF * dX_ECEF + dY_ECEF * dY_ECEF);

	//////////////////////////////////////////////////////////////////////////
	// transforamtion from ECEF to geodic coordinates
	// by an iterative numeric algorithm:
	// perform following iteration until convergence
	dLambda = 0.0;
	for (nCount = 0 ; nCount < nNoIterationSteps ; nCount++)
	{
		dLambda = atan((dZ_ECEF + dGps_Eccentricity2 * dN * sin(dLambda)) / dP);
		dN = dGps_SemiMajorAxis / (sqrt(1.0 - (dGps_Eccentricity2 * sin(dLambda) * sin(dLambda))));
	}

	dLong = atan2(dY_ECEF, dX_ECEF);
	dLat = dLambda;

	resetToDefault();
	setLongitudeInRad(dLong);	// *pdLong = dLong * (180.0 / GPS_PI);
	setLatitudeInRad(dLat);		// *pdLat = dLat * (180.0 / GPS_PI);
}

/**
 * Get a cartesian coordinate from the latitude/longitude position relative to the gigen reference point (origin).
 * Note: this function is only valid, if the reference point is not to far (some km) from the given reference point.
 */
Point3D PositionWGS84::getCartesianRelPos(const PositionWGS84& orign) const
{
	double x, y;
	transformToTangentialPlane(orign, &x, &y);
	Point3D pos(x, y, getAltitudeInMeterMSL() - orign.getAltitudeInMeterMSL());
	return pos;
}

std::string toString(const PositionWGS84::PositionWGS84SourceType& type)
{
	switch (type)
	{
	case PositionWGS84::GPS_SPS:
		return "GPS (SPS)";
	case PositionWGS84::GPS_PPS:
		return "GPS (PPS)";
	case PositionWGS84::GPS_SBAS:
		return "GPS (SBAS)";
	case PositionWGS84::GPS_SBAS_Omnistar_VBS:
		return "GPS (Omnistar VBS)";
	case PositionWGS84::GPS_SBAS_Omnistar_HP:
		return "GPS (Omnistar HP)";
	case PositionWGS84::GPS_GBAS:
		return "GPS (GBAS)";
	case PositionWGS84::GPS_GBAS_RTK_Float:
		return "GPS (RTK float)";
	case PositionWGS84::GPS_GBAS_RTK_Integer:
		return "GPS (RTK integer)";
	case PositionWGS84::IMU:
		return "IMU";
	case PositionWGS84::LandmarkPositioning:
		return "Landmark positioning";
	case PositionWGS84::Manual:
		return "Manual";
	case PositionWGS84::CAN:
		return "CAN";
	default:
	{
		std::stringstream ret;
		ret << "Unknown (" << type << ")";
		return ret.str();
	}
	}
}

} 	// namespace datatypes
