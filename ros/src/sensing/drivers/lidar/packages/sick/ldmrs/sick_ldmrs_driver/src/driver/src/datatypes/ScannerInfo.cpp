//
// ScannerInfo.cpp
//
//

#include "ScannerInfo.hpp"

#include "Position3D.hpp"
#include "Scan.hpp"
#include <cassert>
#include "../tools/Time.hpp"
#include "../tools/errorhandler.hpp"
#include "../tools/toolbox.hpp"

namespace datatypes
{
	
ScannerInfo::ScannerInfo()
	: m_deviceID(0)
	, m_scannerType(0)
	, m_scanNumber(0)
	, m_startAngle(NaN_double)
	, m_endAngle(NaN_double)
	, m_processingFlags(0)
	, m_scanStartTime(Time())
	, m_scanEndTime(Time())
	, m_scanFrequency(NaN_double)
	, m_beamTilt(0)
	, m_scanFlags(0)
	, m_mountingPosition()
{
	m_datatype = Datatype_Scannerinfo;
}

ScannerInfo::~ScannerInfo()
{
}

std::string ScannerInfo::scannerTypeToString(UINT8 st)
{
	switch (st)
	{
		case Scannertype_UNKNOWN:
			return "UNKNOWN";
			break;
		case Scannertype_LMS1xx:
			return "LMS1xx";
			break;
		default:
			printError("scannerTypeToString: Unknown scanner type, code=" + ::toString((UINT16)st) + ".");
			return "(error - unknown type)";
	}
}


bool ScannerInfo::operator==(const ScannerInfo& other) const
{
	return
		m_deviceID == other.m_deviceID
		&& m_scannerType == other.m_scannerType
		&& m_scanNumber == other.m_scanNumber
		&& (m_startAngle == other.m_startAngle || (isNaN(m_startAngle) && isNaN(other.m_startAngle)))
		&& (m_endAngle == other.m_endAngle || (isNaN(m_endAngle) && isNaN(other.m_endAngle)))
		&& m_scanStartTime == other.m_scanStartTime
		&& m_scanEndTime == other.m_scanEndTime
		&& (m_scanFrequency == other.m_scanFrequency || (isNaN(m_scanFrequency) && isNaN(other.m_scanFrequency)))
		&& m_beamTilt == other.m_beamTilt
		&& m_scanFlags == other.m_scanFlags
		&& m_mountingPosition == other.m_mountingPosition
		;
}

void ScannerInfo::setProcessingFlags(const UINT16 processingFlags)
{
	m_processingFlags = processingFlags;
}

bool ScannerInfo::isRearMirrorSide()
{
	if ((m_processingFlags & 0x0400) != 0)
	{
		return true;
 	}

 	return false;
}

bool ScannerInfo::isFrontMirrorSide()
{
 	return !isRearMirrorSide();
}


void ScannerInfo::setStartAngle(double v)
{
	assert(v >= -PI);
	assert(v < PI);
	m_startAngle = v;
}

void ScannerInfo::setEndAngle(double v)
{
	assert(v >= -PI);
	assert(v < PI);
	m_endAngle = v;
}

void ScannerInfo::setScannerType(UINT8 v)
{
	m_scannerType = v;
}

void ScannerInfo::setBeamTilt(double v)
{
	assert(v >= -PI);
	assert(v < PI);
	m_beamTilt = v;
}

/**
 * The given start and end must not be not_a_date_time. The given end
 * time must be greater than the start time.
 */
void ScannerInfo::setTimestamps (const Time& start,
								 const Time& end)
{
	assert(start < end);
	m_scanStartTime = start;
	m_scanEndTime = end;
}

/**
 * The given start and end must not be not_a_date_time. The given end
 * time must be greater than the start time.
 */
/*
void ScannerInfo::setDeviceTimestamps (const boost::posix_time::ptime& start,
									   const boost::posix_time::ptime& end)
{
	assert(!start.is_not_a_date_time());
	assert(!end.is_not_a_date_time());
	assert(start < end);
	m_scanStartDeviceTime = start;
	m_scanEndDeviceTime = end;
}
*/

void ScannerInfo::setScanFrequency(double freq)
{
	assert(freq >= 0);
	m_scanFrequency = freq;
}


// bool ScannerInfo::isGroundLabeled()		const { return (m_scanFlags & Scan::FlagGroundLabeled)		!= 0; }
// bool ScannerInfo::isDirtLabeled()		const { return (m_scanFlags & Scan::FlagDirtLabeled )		!= 0; }
// bool ScannerInfo::isRainLabeled()		const { return (m_scanFlags & Scan::FlagRainLabeled)		!= 0; }
// bool ScannerInfo::isCoverageLabeled()	const { return (m_scanFlags & Scan::FlagCoverageLabeled)	!= 0; }
// bool ScannerInfo::isBackgroundLabeled()	const { return (m_scanFlags & Scan::FlagBackgroundLabeled)	!= 0; }
// bool ScannerInfo::isReflectorLabeled()	const { return (m_scanFlags & Scan::FlagReflectorLabeled)	!= 0; }
// bool ScannerInfo::isUpsideDown()		const { return (m_scanFlags & Scan::FlagUpsideDown)			!= 0; }
// bool ScannerInfo::isRearMirrorSide()	const { return (m_scanFlags & Scan::FlagRearMirrorSide)		!= 0; }

}	// namespace datatypes
