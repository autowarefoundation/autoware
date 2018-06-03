//
// PositionWGS84.hpp
//
// Type of positions in the WGS-84 coordinate system.
//
// 2011-11-11, Willhvo
//

#ifndef POSITIONWGS84_HPP
#define POSITIONWGS84_HPP

#include "../BasicDatatypes.hpp"
#include "../tools/MathToolbox.hpp"
#include "Point3D.hpp"

namespace datatypes
{
	
//
// Position in the WGS-84 coordinate system.
//
// WGS-84 coordinates can be provided by several data sources like GPS
// receivers, IMUs or other positioning systems.
//
class PositionWGS84 : public BasicData
{
public:

	/**
	 * \Sources that provide WGS-84 position information.
	 *
	 */
	enum PositionWGS84SourceType
	{
		Unknown = 0,
		GPS_SPS,
		GPS_PPS,
		GPS_SBAS,
		GPS_SBAS_Omnistar_VBS,
		GPS_SBAS_Omnistar_HP,
		GPS_GBAS,
		GPS_GBAS_RTK_Float,
		GPS_GBAS_RTK_Integer,
		IMU,
		LandmarkPositioning,
		Manual,
		CAN
	};

	PositionWGS84();

	virtual ~PositionWGS84();

	// Estimate the memory usage of this object
	inline virtual const UINT32 getUsedMemory() const {return (sizeof(*this));};
	
	/** @name Setter methods
	 * Use these methods to change the object'S data fields.
	 */

	/**
	 * \brief Sets the time when the position measurement was taken.
	 *
	 * The time should be as close to the real measurement time as possible.
	 *
	 * \param[in] val Time specific to the measurement device.
	 */
	void setMeasurementTime(time_t val) { m_timeOfMeasurement = val; }

	/**
	 * \brief Sets the latitude value of the WGS-84 position.
	 *
	 * \param[in] val Latitude value in [rad]. Must be in the interval
	 * [-Pi/2, Pi/2] radians which corresponds to [-90,90] degree.
	 *
	 * \sa setLatitudeInDeg()
	 * \sa setLatitudeInNMEA()
	 * \sa setLongitudeInRad()
	 * \sa setLongitudeInDeg()
	 * \sa setLongitudeInNMEA()
	 * \sa setAltitudeInMeter()
	 * \sa getLatitudeInRad()
	 * \sa getLatitudeInDeg()
	 * \sa getLongitudeInRad()
	 * \sa getLongitudeInDeg()
	 */
	void setLatitudeInRad(double val);

	void transformToTangentialPlane(const PositionWGS84& origin, double* easting, double* northing, double* height) const;
	void transformToTangentialPlane(const PositionWGS84& origin, double* easting, double* northing) const;
	void transformFromTangentialPlane(double dX, double dY, const PositionWGS84& origin);

	/**
	 * Returns the distance, in [m], to the given position. Note that this distance
	 * is reliable only in the vicinity of our position, for some 100 m, before the
	 * curvature of the earth introduces a relatively high error.
	 */
	double distanceToPos(const PositionWGS84& pos) const;

	/**
	 * \brief Sets the latitude value of the WGS-84 position.
	 *
	 * \param[in] val Latitude value in [deg]. Must be in the interval
	 * [-90,90] degree.
	 *
	 * \sa setLatitudeInRad()
	 * \sa setLatitudeInNMEA()
	 * \sa setLongitudeInRad()
	 * \sa setLongitudeInDeg()
	 * \sa setLongitudeInNMEA()
	 * \sa setAltitudeInMeter()
	 * \sa getLatitudeInRad()
	 * \sa getLatitudeInDeg()
	 * \sa getLongitudeInRad()
	 * \sa getLongitudeInDeg()
	 */
	void setLatitudeInDeg(double val) { setLatitudeInRad(val * deg2rad); }

	void setLatitudeInNMEA(double Dm, char H);

	/**
	 * \brief Sets the latitude value of the WGS-84 position.
	 *
	 * \param[in] Dm Latitude value in DDmm.mmmmm, where D stands for value in degree and m for value in decimal minutes (signed).
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
	void setLatitudeInNMEA(double Dm) { setLatitudeInNMEA(Dm, 'N'); }

	/**
	 * \brief Sets the sigma value of the normal distribution describing
	 *        the confidence about the latitude measurement.
	 *
	 * Negative values mean that no accuracy knowledge is available.
	 *
	 * ATTENTION: Sigma value is stored in meter. If you use the value
	 *            in combination with the latitude value be aware that
	 *            the units differ!
	 *
	 * \param[in] val Sigma value in [m].
	 *
	 * \sa setLongitudeSigmaInMeter()
	 * \sa setAltitudeInMeter()
	 */
	void setLatitudeSigmaInMeter(double val) { m_latitudeSigma = val; }

	/**
	 * \brief Sets the longitude value of the WGS-84 position.
	 *
	 * \param[in] val Longitude value in [deg]. Must be in the range
	 * [-Pi, Pi] radians which corresponds to [-180, 180] degree.
	 *
	 */
	void setLongitudeInRad(double val);

	/**
	 * \brief Sets the longitude value of the WGS-84 position.
	 *
	 * \param[in] val Longitude value in [deg]. Must be in the range
	 * [-180, 180] degree.
	 *
	 * \sa setLongitudeInRad()
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
	void setLongitudeInDeg(double val) { setLongitudeInRad(val * deg2rad); }

	/**
	 * \brief Sets the longitude value of the WGS-84 position.
	 *
	 * \param[in] Dm Latitude value in DDmm.mmmmm, where D stands for value in degree and m for value in decimal minutes.
	 * \param[in] H Hemisphere. For longitude it can be [W]est or [E]ast.
	 *
	 * \sa setLongitudeInRad()
	 * \sa setLongitudeInDeg()
	 * \sa setLatitudeInRad()
	 * \sa setLatitudeInDeg()
	 * \sa setLatitudeInNMEA()
	 * \sa setAltitudeInMeter()
	 * \sa getLatitudeInRad()
	 * \sa getLatitudeInDeg()
	 * \sa getLongitudeInRad()
	 * \sa getLongitudeInDeg()
	 */
	void setLongitudeInNMEA(double Dm, char H);

	/**
	 * \brief Sets the longitude value of the WGS-84 position.
	 *
	 * \param[in] Dm Latitude value in DDmm.mmmmm, where D stands for value in degree and m for value in decimal minutes (signed).
	 *
	 * \sa setLongitudeInRad()
	 * \sa setLongitudeInDeg()
	 * \sa setLatitudeInRad()
	 * \sa setLatitudeInDeg()
	 * \sa setLatitudeInNMEA()
	 * \sa setAltitudeInMeter()
	 * \sa getLatitudeInRad()
	 * \sa getLatitudeInDeg()
	 * \sa getLongitudeInRad()
	 * \sa getLongitudeInDeg()
	 */
	void setLongitudeInNMEA(double Dm) { setLongitudeInNMEA(Dm, 'E'); }

	/**
	 * \brief Sets the sigma value of the normal distribution describing
	 *        the confidence about the longitude measurement.
	 *
	 * Negative values mean that no accuracy knowledge is available.
	 *
	 * ATTENTION: Sigma value is stored in meter. If you use the value
	 *            in combination with the longitude value be aware that
	 *            the units differ!
	 *
	 * \param[in] val Sigma value in [m].
	 *
	 * \sa setLatitudeSigmaInMeter()
	 * \sa setAltitudeSigmaInMeter()
	 */
	void setLongitudeSigmaInMeter(double val) { m_longitudeSigma = val; }

	/**
	 * \brief Sets altitude value.
	 *
	 * This function sets the altitude above the mean sea level (MSL) in [m].
	 *
	 * \param[in] val Altitude value in [m].
	 *
	 * \sa setLongitudeInRad()
	 * \sa setLongitudeInDeg()
	 * \sa setLongitudeInNMEA()
	 * \sa setLatitudeInRad()
	 * \sa setLatitudeInDeg()
	 * \sa setLatitudeInNMEA()
	 */
	void setAltitudeInMeterMSL(double val) { m_altitudeMSL = val; }

	/**
	 * \brief Sets the sigma value of the normal distribution describing
	 *        the confidence about the altitude measurement.
	 *
	 * Negative values mean that no accuracy knowledge is available.
	 *
	 * \param[in] val Sigma value in [m].
	 *
	 * \sa setLongitudeSigmaInMeter()
	 * \sa setLatitudeSigmaInMeter()
	 */
	void setAltitudeSigmaInMeterMSL(double val) { m_altitudeMSLSigma = val; }

	/**
	 * \brief Sets the course angle.
	 *
	 * \param[in] val Course angle value in [rad].
	 *
	 * \sa setCourseAngleInDeg()
	 * \sa setCourceAngleSigma()
	 */
	void setCourseAngleInRad(double val)
	{
		m_courseAngle = normalizeRadians(val);
		if (m_courseAngle <= 0.0)
		{
			m_courseAngle += 2.0 * PI;
		}
	}

	/**
	 * \brief Sets the course angle.
	 *
	 * \param[in] val Course angle value in [deg]. (0 = North, 90 = West)
	 *
	 * \sa setCourseAngleInRad()
	 * \sa setCourceAngleSigma()
	 */
	void setCourseAngleInDeg(double val) { setCourseAngleInRad(val * deg2rad); }

	/**
	 * \brief Sets the sigma of the normal distribution describing
	 *        the confidence about the course angle.
	 *
	 * Negative values mean that no accuracy knowledge is available.
	 *
	 * \param[in] val Sigma value in [rad].
	 *
	 * \sa setCourseAngleInRad()
	 * \sa setCourseAngleInDeg()
	 */
	void setCourseAngleSigmaInRad(double val) { m_courseAngleSigma = val; }

	/**
	 * \brief Sets the sigma of the normal distribution describing
	 *        the confidence about the course angle.
	 *
	 * Negative values mean that no accuracy knowledge is available.
	 *
	 * \param[in] val Sigma value in [dev].
	 *
	 * \sa setCourseAngleInRad()
	 * \sa setCourseAngleInDeg()
	 */
	void setCourseAngleSigmaInDeg(double val) { setCourseAngleSigmaInRad(val * deg2rad); }

	void setYawAngleInRad(double val) { m_yawAngle = val; }
	void setYawAngleInDeg(double val) { setYawAngleInRad(val * deg2rad); }

	void setYawAngleSigmaInRad(double val) { m_yawAngleSigma = val; }
	void setYawAngleSigmaInDeg(double val) { setYawAngleSigmaInRad(val * deg2rad); }

	void setPitchAngleInRad(double val) { m_pitchAngle = val; }
	void setPitchAngleInDeg(double val) { setPitchAngleInRad(val * deg2rad); }

	void setPitchAngleSigmaInRad(double val) { m_pitchAngleSigma = val; }
	void setPitchAngleSigmaInDeg(double val) { setPitchAngleSigmaInRad(val * deg2rad); }

	void setRollAngleInRad(double val) { m_rollAngle = val; }
	void setRollAngleInDeg(double val) { setRollAngleInRad(val * deg2rad); }

	void setRollAngleSigmaInRad(double val) { m_rollAngleSigma = val; }
	void setRollAngleSigmaInDeg(double val) { setRollAngleSigmaInRad(val * deg2rad); }

	/**
	 * \brief Sets the source of the position measurement.
	 *
	 * \param[in] val Source of measurement.
	 *
	 * \sa SourceType
	 */
	void setSource(const PositionWGS84SourceType val) { m_source = val; }

	void resetToDefault();

	//@}

	/** @name Getter methods
	 * Use these methods to retrieve position information.
	 */
	//@{

	/**
	 * \brief Returns the time when the position measurement was taken.
	 *
	 * The time should be as close to the real measurement time as possible.
	 *
	 * \return Time specific to the measurement device.
	 */
	const time_t& getMeasurementTime() const { return m_timeOfMeasurement; }

	/**
	 * \brief Returns the time when the position measurement was taken.
	 *
	 * The time should be as close to the real measurement time as possible.
	 *
	 * \return UTC time stamp of measurement recording time.
	 */
	const time_t& getTimestamp() const { return m_timeOfMeasurement; }

	/**
	 * \brief Returns the latitude value of the WGS-84 position.
	 *
	 * \return Latitude value in [rad].
	 *
	 * \sa getLatitudeInDeg()
	 * \sa getLongitudeInRad()
	 * \sa getLongitudeInDeg()
	 */
	double getLatitudeInRad() const { return m_latitude; }

	/**
	 * \brief Returns the latitude value of the WGS-84 position.
	 *
	 * \return Latitude value in [deg].
	 *
	 * \sa getLatitudeInRad()
	 * \sa getLongitudeInRad()
	 * \sa getLongitudeInDeg()
	 */
	double getLatitudeInDeg() const { return (m_latitude * rad2deg); }

	/**
	 * \brief Returns the longitude value of the WGS-84 position.
	 *
	 * \return Longitude value in [rad].
	 *
	 * \sa getLongitudeInRad()
	 * \sa getLatitudeInRad()
	 * \sa getLatitudeInDeg()
	 */
	double getLongitudeInRad() const { return m_longitude; }

	/**
	 * \brief Returns the longitude value of the WGS-84 position.
	 *
	 * \return Longitude value in [deg].
	 *
	 * \sa getLongitudeInRad()
	 * \sa getLatitudeInRad()
	 * \sa getLatitudeInDeg()
	 */
	double getLongitudeInDeg() const { return (m_longitude * rad2deg); }

	/**
	 * \brief Returns the course angle in [rad].
	 *
	 * The course angle is the angle the vehicle is travelling to.
	 * If you drift, it's different to the yaw angle, which is the direction
	 * of the vehicle is heading/looking at.
	 *
	 * The angle in between is called slip angle.
	 *
	 * The angle is always counted positive in counter-clockwise direction, since
	 * in our system the z-axis is pointing upwards.
	 *
	 * Example: If the car is heading to the north, but driving backwards,
	 * the yaw angle is 0 degrees and the course angle 180 degrees.
	 *
	 * \return Course angle in [rad]. (0 = North, pi/2 = West)
	 */
	double getCourseAngleInRad() const { return m_courseAngle; }

	/**
	 * \brief Returns the course angle in [deg].
	 *
	 * The course angle is the angle the vehicle is travelling to.
	 * If you drift, it's different to the yaw angle, which is the direction
	 * of the vehicle is heading/looking at.
	 *
	 * The angle in between is called slip angle.
	 *
	 * The angle is always counted positive in counter-clockwise direction, since
	 * in our system the z-axis is pointing upwards.
	 *
	 * Example: If the car is heading to the north, but driving backwards,
	 * the yaw angle is 0 degrees and the course angle 180 degrees.
	 *
	 * \return Course angle in [deg]. (0 = North, 90 = West)
	 */
	double getCourseAngleInDeg() const { return (m_courseAngle * rad2deg); }

	/**
	 * \brief Returns the yaw angle in [rad].
	 *
	 * The yaw angle is the angle the vehicle is heading/looking at.
	 * If you drift, it's different to the course angle, which is the direction
	 * of travelling or the track angle.
	 *
	 * The angle in between is called slip angle.
	 *
	 * The angle is always counted positive in counter-clockwise direction, since
	 * in our system the z-axis is pointing upwards.
	 *
	 * Example: If the car is heading to the north, but driving backwards,
	 * the yaw angle is 0 degrees and the course angle 180 degrees.
	 *
	 * \return Yaw angle in [rad]. (0 = North, pi/2 = West)
	 */
	double getYawAngleInRad() const { return m_yawAngle; }

	/**
	 * \brief Returns the yaw angle.
	 *
	 * The yaw angle is the angle the vehicle is heading/looking at.
	 * If you drift, it's different to the course angle, which is the direction
	 * of travelling or the track angle.
	 *
	 * The angle in between is called slip angle.
	 *
	 * The angle is always counted positive in counter-clockwise direction, since
	 * in our system the z-axis is pointing upwards.
	 *
	 * Example: If the car is heading to the north, but driving backwards,
	 * the yaw angle is 0 degrees and the course angle 180 degrees.
	 *
	 * \return Yaw angle in [deg]. (0 = North, 90 = West)
	 */
	double getYawAngleInDeg() const { return (m_yawAngle * rad2deg); }

	double getYawAngleSigmaInRad() const { return m_yawAngleSigma; }
	double getYawAngleSigmaInDeg() const { return (m_yawAngleSigma * rad2deg); }

	double getPitchAngleInRad() const { return m_pitchAngle; }
	double getPitchAngleInDeg() const { return (m_pitchAngle * rad2deg); }

	double getPitchAngleSigmaInRad() const { return m_pitchAngleSigma; }
	double getPitchAngleSigmaInDeg() const { return (m_pitchAngleSigma * rad2deg); }

	double getRollAngleInRad() const { return m_rollAngle; }
	double getRollAngleInDeg() const { return (m_rollAngle * rad2deg); }

	double getRollAngleSigmaInRad() const { return m_rollAngleSigma; }
	double getRollAngleSigmaInDeg() const { return (m_rollAngleSigma * rad2deg); }

	/// Equality predicate
	bool operator==(const PositionWGS84& other) const;
	bool operator!=(const PositionWGS84& other) const { return !(*this == other); }

	//@}

	/**
	 * \brief Returns the altitude in meter above mean sea level.
	 *
	 * \return Altitude in [m] above mean sea level (MSL).
	 */
	double getAltitudeInMeterMSL() const { return m_altitudeMSL; }

	std::string toString() const;

	/**
	 * \brief Returns the type of source that identifies the type of the device
	 *        that created this object.
	 *
	 * \return Type of source that created this object.
	 */
	PositionWGS84SourceType getSource() const { return m_source; }

	std::string getSourceString() const;

	
	/**
	* \brief Calculates the distance in [m] from this to position in argument
	*
	* \param[in] pos Position of other point on planet.
	* \return Distance in [m] between the two positions.
	*/
	double dist( const PositionWGS84& pos ) const;

	Point3D getCartesianRelPos(const PositionWGS84& orign) const;

private:
	double NMEA2rad( double Dm );

	/** Version of this file. Used for trace and debug output. */
	// NOTE 2008/01/18 (dw) : Change VERSION in cpp file when the interface is changed or bugs are fixed.
	//						  Document changes in the HISTORY above.
	static const std::string VERSION;

	time_t  m_timeOfMeasurement;

	//								   Unit								Serialized size:
//	UINT8 m_deviceID;				// ID of device						+ 1 Bytes
	double m_latitude;				// [rad]							+ 8 Bytes
	double m_latitudeSigma;			// [m]								+ 8 Bytes
	double m_longitude;				// [rad]							+ 8 Bytes
	double m_longitudeSigma;		// [m]								+ 8 Bytes
	double m_altitudeMSL;			// [m]								+ 8 Bytes
	double m_altitudeMSLSigma;		// [m]								+ 8 Bytes
	double m_courseAngle;			// [rad]							+ 8 Bytes
	double m_courseAngleSigma;		// [rad]							+ 8 Bytes
	double m_yawAngle;				// [rad]							+ 8 Bytes
	double m_yawAngleSigma;			// [rad]							+ 8 Bytes
	double m_pitchAngle;			// [rad]							+ 8 Bytes
	double m_pitchAngleSigma;		// [rad]							+ 8 Bytes
	double m_rollAngle;				// [rad]							+ 8 Bytes
	double m_rollAngleSigma;		// [rad]							+ 8 Bytes
	PositionWGS84SourceType m_source;			// Source of position information	+ 2 Bytes (encoded as UINT16)
	//																	= 115 Bytes
};

std::string toString(const PositionWGS84::PositionWGS84SourceType& type);


}	// namespace datatypes



#endif // POSITIONWGS84_HPP
