//
// ScanPoint.hpp
//
// Stores one ScanPoint
//


#ifndef SCANPOINT_HPP
#define SCANPOINT_HPP


#include <cassert>
#include "../BasicDatatypes.hpp"

namespace datatypes
{

class Point2D;
class Point3D;

// Class for a single scan point
//
// This class stores data of a single scan point:
// - Cartesian coordinates
// - polar coordinates (redundant information, for run-time efficiency only)
// - echo pulse width / RSSI Werte
// - flags (properties)
// - ID, channel, and subchannel of the laserscanner
// * - time offset relative to the scan start
//
// The public access methods ensure consistency
// - of the Cartesian with the polar coordinates and
// - of the original data with the decompressed data
//   (i.e. of the live data with the replay data)
//
class ScanPoint
{
public:

	// Property flags
	//
	// (The "marker" flag is a flag that can be used locally by
	// modules to mark scanpoints. The initial state of this flag
	// should be considered unknown and it should *not* be used to
	// transport information between modules as other configurations
	// may then corrupt the flags.)
	//
	enum Flags
	{
		FlagGround			    = 0x0001,                            ///< Invalid scan point, echo from ground
		FlagDirt			    = 0x0002,                            ///< Invalid scan point, echo from dirt
		FlagRain				= 0x0004,                            ///< Invalid scan point, echo from rain drop
		FlagThresholdSwitching	= 0x0010,							 ///< Scan point was measured using the higher threshold (since FGPA version 8.0.08)
		///< Old: Scan point was measured in a shot with threshold switching (earlier FPGA versions)
		FlagReflector			= 0x0020,							 ///< EPW of scan point is high enough to be a reflector
		FlagLeftCovered			= 0x0100,                            ///< Left neighbour point may be covered
		FlagRightCovered		= 0x0200,                            ///< Right neighbour point may be covered
		FlagBackground			= 0x0400,                            ///< Point has been recognized as background and should not be used in the tracking anymore
		FlagMarker				= 0x0800,							 ///< Point is "marked" (see above)
		FlagTransparent			= 0x1000,							 ///< There is at least one more echo behind this scan point (B or C echo)
		MaskInvalid				= FlagGround | FlagDirt | FlagRain | FlagBackground,  ///< All flags of invalid scan points
		MaskCovered				= FlagLeftCovered | FlagRightCovered ///< All coverage flags
	};

protected:
	// Cartesian coordinates
	double m_x;
	double m_y;
	double m_z;

	// Polar coordinates
	double m_dist;
	double m_hAngle; 	// Polar azimuth (horizontal) angle
	double m_vAngle;	// Polar elevation (vertical) angle

	// Echo pulse width
	double m_echoWidth;

	
	UINT16 m_flags;		// Property flags
	UINT8 m_sourceId;	// ID of the source device
	UINT8 m_channel;	// Measurement channel (= "Layer")
	UINT8 m_subchannel;	// Measurement subchannel (=Echo-Nummer im Schuss)

public:
	ScanPoint();		// Default constructor
	void clear();		// Clears this scan point. Sets all data to zero.

	double getX() const { return m_x; }	// Returns the Cartesian x-coordinate, in [m]
	double getY() const { return m_y; }
	double getZ() const { return m_z; }

	double getDist() const { return m_dist; }	// Returns the polar distance coordinate, in [m]
	double getHAngle() const { return m_hAngle; } // Returns the polar azimuth (horizontal) angle, in [rad]
	double getVAngle() const { return m_vAngle; }	// Returns the polar elevation (vertical) angle, in [rad]

	double getEchoWidth() const { return m_echoWidth; }	// Returns the echo pulse width
	UINT16 getFlags() const { return m_flags; }	// Returns the point properties
	UINT8 getSourceId() const { return m_sourceId; }	// Returns the source device ID
	UINT8 getLayer() const { return m_channel; }	// Returns the channel ("Layer", "Scan-Ebene")
	UINT8 getEchoNum() const { return m_subchannel; }	// Returns the recording subchannel ("Echo-Nummer")

	/// Checks if the scan point is valid (no ground, dirt, or rain)
	bool isValid() const { return (m_flags & MaskInvalid) == 0; }
	/// Checks if the marker is set
	bool isMarked() const { return (m_flags & FlagMarker); }
	/// Checks if the scan point is labeled as ground
	bool isGround() const { return m_flags & FlagGround; }
	/// Checks if the scan point is labeled as background
	bool isBackground() const { return m_flags & FlagBackground; }
	/// Checks if the scan point is labeled as dirt
	bool isDirt() const { return m_flags & FlagDirt; }
	/// Checks if the scan point is labeled as rain
	bool isRain() const { return m_flags & FlagRain; }
	/// Checks if the "left covered" flag is set
	bool isLeftCovered() const { return m_flags & FlagLeftCovered; }
	/// Checks if the "right covered" flag is set
	bool isRightCovered() const { return m_flags & FlagRightCovered; }
	/// Checks if the reflector flag is set
	bool isReflector() const { return m_flags & FlagReflector; }
	/// Checks if the threshold switching flag is set
	bool isThresholdSwitching() const { return m_flags & FlagThresholdSwitching; }
	/// Checks if there is at least one more echo behind this scan point (B or C echo)
	bool isTransparent() const { return m_flags & FlagTransparent; }

	/** \brief Returns the distance between this and the given other scanpoint, in [m].
	 *
	 * This method calculates the actual three-dimensional distance in
	 * x,y,z. dist2D() uses only x and y.
	 */
	double getDist(const ScanPoint& other) const;

	/** \brief Returns the two-dimensional distance between this and the
	 * given other scanpoint, in [m].
	 *
	 * This method calculates the distance only in x and y
	 * coordinates; the z dimension is ignored. dist() uses all three
	 * dimensions.
	 */
	double getDist2D(const ScanPoint& other) const;

	/// Returns the x,y,z coordinates as a Point3D structure
	Point3D toPoint3D() const;

	/// Returns the x and y coordinates as a Point2D structure
	Point2D toPoint2D() const;


	// Returns the distance between the two scanpoint coordinates, in [m]
	static double getDistanceBetweenScanpoints(const ScanPoint& pt1, const ScanPoint& pt2);


	void setPoint3D (const Point3D& pt);	// Set the Cartesian point coordinates
	void setCartesian (double x, double y, double z);
	void setPolar (double dist, double hAngle, double vAngle);	// Set the polar point coordinates

	// Adds offsets to the Cartesian coordinates
	void addCartesianOffset (double xOffset, double yOffset, double zOffset);

	// Adds offsets to the polar coordinates
	void addPolarOffset (double distOffset, double hAngleOffset, double vAngleOffset);

	/// Set the echo pulse width, typically in [m]
	void setEchoWidth (double echoWidth);

	void setSourceId (UINT8 id) { m_sourceId = id; }	// Set the device ID of the source scanner

	void setLayer (UINT8 ch) { m_channel = ch; }	// Set the recording layer / channel
	void setEchoNum (UINT8 sub) { m_subchannel = sub; }	// Nummer des Echos

	/// Sets the scan point flags directly
	void setFlags (UINT16 flags)
	{
		m_flags = flags;
	}
	/// Set or clear the "Marker" flag
	void setMarker (bool isMarked = true)
	{
		if (isMarked)
			m_flags |=  FlagMarker;
		else
			m_flags &= ~FlagMarker;
	}
	/// Labels the scan point as invalid because it is in the background area
	void setBackground (bool isBackground = true)
	{
		if (isBackground)
			m_flags |=  FlagBackground;
		else
			m_flags &= ~FlagBackground;
	}
	/// Labels the scan point as invalid because it is an echo from the ground
	void setGround (bool isGround = true)
	{
		if (isGround)
			m_flags |=  FlagGround;
		else
			m_flags &= ~FlagGround;
	}
	/// Labels the scan point as invalid because its an echo from dirt
	void setDirt (bool isDirt = true)
	{
		if (isDirt)
			m_flags |=  FlagDirt;
		else
			m_flags &= ~FlagDirt;
	}
	/// Labels the scan point as invalid because its an echo from rain
	void setRain (bool isRain = true)
	{
		if (isRain)
			m_flags |=  FlagRain;
		else
			m_flags &= ~FlagRain;
	}
	/// Labels the scan point: Left neighbour point may be covered
	void setLeftCovered (bool isLeftCovered = true)
	{
		if (isLeftCovered)
			m_flags |=  FlagLeftCovered;
		else
			m_flags &= ~FlagLeftCovered;
	}
	/// Labels the scan point: Right neighbour point may be covered
	void setRightCovered (bool isRightCovered = true)
	{
		if (isRightCovered)
			m_flags |=  FlagRightCovered;
		else
			m_flags &= ~FlagRightCovered;
	}
	/// Remove cover status.
	void setNotCovered () { m_flags &= ~MaskCovered; }

	/// Labels the scan point: EPW is high enough to be a reflector
	void setReflector (bool isReflector = true)
	{
		if (isReflector)
			m_flags |= FlagReflector;
		else
			m_flags &= ~FlagReflector;
	}

	/// Removes all scan status information.
	void setValid()	{ m_flags &= ~MaskInvalid; }

	friend bool operator==(const ScanPoint &, const ScanPoint &);
	
	std::string toString() const;	// Text output

private:
	/// Compute polar coordinates from the current Cartesian coordinates
	void updatePolar();

	/// Compute Cartesian coordinates from the current polar coordinates
	void updateCartesian();

};

/// Text output for debugging
std::ostream& operator<<(std::ostream& os, const ScanPoint& point);

/// Equality predicate
bool operator==(const ScanPoint &p1, const ScanPoint &p2);

/// Inequality predicate
bool operator!=(const ScanPoint &p1, const ScanPoint &p2);

}	// namespace datatypes

#endif //  SCANPOINT_HPP
