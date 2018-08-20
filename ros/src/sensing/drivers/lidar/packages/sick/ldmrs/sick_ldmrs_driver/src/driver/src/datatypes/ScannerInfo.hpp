//
// ScannerInfo.hpp
//

#ifndef SCANNERINFO_HPP
#define SCANNERINFO_HPP

#include "../BasicDatatypes.hpp"
#include "Position3D.hpp"
#include "../tools/Time.hpp"
#include <vector>

namespace datatypes
{

enum ScannerType
{
	Scannertype_UNKNOWN	= 0,
	Scannertype_LMS1xx	= 1
};

// Information about one Laserscanner which has sent a Scan
class ScannerInfo : public BasicData
{
public:

	/// Empty constructor
	ScannerInfo();
	~ScannerInfo();
	
	// Estimate the memory usage of this object
	virtual const UINT32 getUsedMemory() const { return sizeof(*this); };
	
	static std::string scannerTypeToString(UINT8 st);

	/// Equality predicate
	bool operator==(const ScannerInfo& other) const;

	typedef std::vector<std::pair<double, double> > ResolutionMap;

	/// Returns the ID of the device that has recorded this scan
	UINT8 getDeviceID() const { return m_deviceID; }
	void setDeviceID(UINT8 v) { m_deviceID = v; }

	// Returns the type of scanner that has recorded this scan
	UINT8 getScannerType() const { return m_scannerType; }

	// Set the type of the scanner that has recorded this scan
	void setScannerType(UINT8 newScannerType);

	/// Returns the scan counter from scanner device
	UINT16 getScanNumber() const { return m_scanNumber; }
	void setScanNumber(UINT16 v) { m_scanNumber = v; }

	// Returns the start angle of this scan in radians.
	//
	// Field of view of one every single scanner given in its
	// local coordinate system, in radians, and normalized to
	// [-pi, pi[
	//
	double getStartAngle() const { return m_startAngle; }
	void setStartAngle(double v);

	// Returns the end angle of this scan in radians.
	//
	// Field of view of one every single scanner given in its
	// local coordinate system, in radians, and normalized to
	// [-pi, pi[
	double getEndAngle() const { return m_endAngle; }
	void setEndAngle(double v);

	// Returns the start timestamp of the scan received by this scanner (in terms of the host computer clock)
	const Time& getStartTimestamp() const { return m_scanStartTime; }

	/// Returns the end timestamp of the scan (in terms of the host computer clock)
	const Time& getEndTimestamp() const { return m_scanEndTime; }

	/// Set the start and end timestamp of the scan received by this scanner (in terms of the host computer clock)
	void setTimestamps (const Time& start, const Time& end);


	/// Returns the start timestamp of this scan as given by the measuring device (which most probably has a clock offset)
//	const boost::posix_time::ptime& getStartDeviceTimestamp() const { return m_scanStartDeviceTime; }

	/// Returns the end timestamp of this scan as given by the measuring device (which most probably has a clock offset)
//	const boost::posix_time::ptime& getEndDeviceTimestamp() const { return m_scanEndDeviceTime; }

	/// Set the start and end timestamp of the scan as given by the measuring device (which most probably has a clock offset)
//	void setDeviceTimestamps (const boost::posix_time::ptime& start, const boost::posix_time::ptime& end);

	/// Get the frequency this scanner is running at in [Hz].
	double getScanFrequency() const { return m_scanFrequency; }
	
	/// Set the scanner's scan frequency in [Hz]. Must be non-negative.
	void setScanFrequency(double freq);
	
	// Sets the processing flags of the MRS:
	//
	// Bit 0:  ground detection performed: 0 = false, 1 = true
	// Bit 1:  dirt detection performed: 0 = false, 1 = true
	// Bit 2:  rain detection performed: 0 = false, 1 = true
	// Bit 5:  transparency detection performed: 0 = false, 1 = true
	// Bit 6:  horizontal angle offset added: 0 = false, 1 = true
	// Bit 10: mirror side: 0=front (for 8-Layer, tilted downward), 1=rear (for 8-layer, tilted upward)
	// All other flags are reserved-internal and should not be evaluated.
	//
	void setProcessingFlags(const UINT16 processingFlags);
	
	// Returns true if the scan was scanned with the rear mirror side. In 8-layer scanners, this
	// side is usually tilted upward.
	// This information is part of the processing flags.
	bool isRearMirrorSide();

	// Returns true if the scan was scanned with the front mirror side. In 8-layer scanners, this
	// side is usually tilted downward.
	// This information is part of the processing flags.
	bool isFrontMirrorSide();

	// Returns the angle by which the laser beam is tilted (pitched) in [rad].
	//
	// Returns the actual angle by which the laser beam is pitched up
	// (negative angles) or down (positive angles) if this was an 8L
	// scanner. The angle is measured relatively to the sensor's
	// x-y-plane at a horizontal view angle of zero degree.
	//
	// Watch out: If this angle is nonzero, the actual beam tilt
	// changes with the horizontal angle. The formula to obtain the
	// actual pitch angle \f$\theta\f$ from the beam tilt \f$\beta\f$
	// at horizontal angle \f$\psi\f$ is
	// \f[
	// \theta = \beta\sqrt{2}\sin\left(\frac{\pi}{4}-\frac{\psi}{2}\right)
	// \f]
	//
	// or, written in C++ code,
	//   currentBeamTilt = getBeamTilt() * std::sqrt(2) * std::sin(0.25*::PI - 0.5*hAngle)
	double getBeamTilt() const { return m_beamTilt; }

	// Set the beam tilt of this scanner in radians.
	void setBeamTilt(double tilt);

	// Get the flags of the single scan belonging to this scanner info.
	UINT32 getScanFlags() const { return m_scanFlags; }
	
	// Set the flags of the single scan belonging to this scanner info.
	void setScanFlags(UINT32 flags) { m_scanFlags = flags; }

	// Mounting Position relative to vehicle reference point.
	//
	// All angle values in radians, and normalized to  [-pi, pi[
	//
	// All distance values in [m]
	//
	const Position3D& getMountingPosition() const { return m_mountingPosition; }
	void setMountingPosition(const Position3D& v) { m_mountingPosition = v; }

	// Get the scanner's horizontal resolution.
	//
	// For each element in the returned vector, the \c first value is
	// the start angle in radians. The \c second value is the
	// horizontal angle resolution from the start angle onwards in
	// radians. The returned vector of resolutions has at most 8
	// elements, but it might have less elements.
	//
//	const ResolutionMap& getResolutionMap() const { return m_resolutionMap; }

	// Set the scanner's horizontal resolution.
	//
	// For each element, \c first is the start angle [rad] and \c
	// second is the horizontal angle resolution [rad]. This vector
	// must not have more than 8 elements, but it might have less
	// elements.
	//
	// Elements at the end which are all-zero will be removed in the
	// internally stored ResolutionMap so that only the non-zero
	// elements will be returned on getResolutionMap().
	//
//	void setResolutionMap(const ResolutionMap& m);

	// Returns the ScannerProperties which belong to the laserscanner
	// type of this object.
	//
	// If an unknown or unimplemented scanner type was set during
	// setScannerType(), the returned ScannerProperties object has its
	// values set to its defaults (NaN and zero).
	//
	//
//	const ScannerProperties& getScannerProperties() const;

	/// Accessor functions for scan flags.
// 	bool isGroundLabeled()		const;
// 	bool isDirtLabeled()		const;
// 	bool isRainLabeled()		const;
// 	bool isCoverageLabeled()	const;
// 	bool isBackgroundLabeled()	const;
// 	bool isReflectorLabeled()	const;
// 	bool isUpsideDown()			const;
// 	bool isRearMirrorSide()		const;


private:
	/// Create and set scanner properties dependent on stored device ID
//	void updateScannerProperties();

	/// Remove the all-zero elements at the end of the resolution map
	/// so that only the non-zero elements will remain
//	void truncateResolutionMap();

	/// Check for legal beam tilt of an 8L scanner.
//	void checkLegalBeamTilt();

private:
	UINT8 m_deviceID;
	UINT8 m_scannerType;
	UINT16 m_scanNumber;
//	UINT32 m_scannerStatus;
	double m_startAngle;
	double m_endAngle;
//	float m_syncAngle;
	UINT16 m_processingFlags;

	/// Start timestamp of the scan received by this scanner.
	Time m_scanStartTime;
	/// End  timestamp of the scan received by this scanner.
	Time m_scanEndTime;

	/// Raw start timestamp of the scan received by this scanner as
	/// given by the measuring device, i.e. including clock offset.
//	boost::posix_time::ptime m_scanStartDeviceTime;

	/// Raw end timestamp of the scan received by this scanner as
	/// given by the measuring device, i.e. including clock offset.
//	boost::posix_time::ptime m_scanEndDeviceTime;

	/// Scan frequency of this scanner
	double m_scanFrequency;
	/// Beam tilt of this scanner in radians
	double m_beamTilt;
	/// Flags for the single scan belonging to this scanner info.
	UINT32 m_scanFlags;

	Position3D m_mountingPosition;

	/// Scanner's horizontal resolution
//	ResolutionMap m_resolutionMap;

	/// Corresponding scanner properties related to scanner type.
//	boost::shared_ptr<ScannerProperties> m_scannerProperties;
};

}	// namespace datatypes

#endif // SCANNERINFO_HPP
