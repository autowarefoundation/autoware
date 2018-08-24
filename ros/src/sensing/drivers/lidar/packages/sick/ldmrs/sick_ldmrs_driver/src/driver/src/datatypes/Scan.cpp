//
// Scan.cpp
//
// Scan data container.
//
// Version history
// 17.11.2011, willhvo: First version
//

#include "Scan.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include "ScannerInfo.hpp"
#include "../tools/errorhandler.hpp"
#include <set>
#include "../tools/toolbox.hpp"

namespace datatypes
{
	
//
// maxPoints The maximum number of scan points that are allocated by the
//				 constructor. This can be increased later by calling reserve().
//
Scan::Scan (size_type maxPoints)
	: m_flags(0)
	, m_scanNumber(0x0000)
	, m_points()
{
	m_datatype = Datatype_Scan;
	m_points.reserve(maxPoints); // allocate memory for the point array

	m_beVerbose = false;
}

Scan::Scan (const Scan& other)
	: m_points()
{
	copy(other);
}

// Estimate the memory usage of this object
const UINT32 Scan::getUsedMemory() const
{
	return (sizeof(*this) +
			(getNumPoints() * sizeof(ScanPoint)) +
			(m_scannerInfos.size() * sizeof(ScannerInfo)));
}


Scan& Scan::copy (const Scan& other)
{
	// These are the trivial by-value assignments
//	m_scanStartTime  = other.m_scanStartTime;
//	m_scanEndTime    = other.m_scanEndTime;
//	m_scanMidTime    = other.m_scanMidTime;
//	m_syncTimestamp  = other.m_syncTimestamp;
//	m_startAngle     = other.m_startAngle;
//	m_endAngle       = other.m_endAngle;
//	m_scannerStatus  = other.m_scannerStatus;
	m_flags          = other.m_flags;
	m_scanNumber     = other.m_scanNumber;
//	m_deviceID       = other.m_deviceID;
//	m_scannerType    = other.m_scannerType;
	m_sourceId			= other.m_sourceId; 
	m_datatype			= other.m_datatype;

	// The PointList vector itself is copied element-by-element.
	m_points         = other.m_points;
	m_points.reserve (other.capacity());

	// This vector is copied element-by-element, by value.
	m_scannerInfos   = other.m_scannerInfos;

	// Verbose-Flag
	m_beVerbose = other.m_beVerbose;
	
	return *this;
}

//
// Tries its best to estimate the current memory usage of this object.
// This function is required to estimate the required storage space in buffers.
//
UINT32 Scan::getTotalObjectSize()
{
	UINT32 size;

	size = sizeof(*this);										// Dieses Objekt selber
	size += getNumPoints() * sizeof(ScanPoint);			// Seine Scanpunkte
	size += m_scannerInfos.size() * sizeof(ScannerInfo);	// Die Scanner-Infos

	return size;
}

Scan& Scan::operator= (const Scan& other)
{
	return copy(other);
}


/*
bool Scan::operator==(const Scan& other) const
{
	return
		m_scanNumber     == other.m_scanNumber
		&& m_flags          == other.m_flags
		&& m_scanStartTime  == other.m_scanStartTime
		&& m_scanEndTime    == other.m_scanEndTime
		&& m_scanMidTime    == other.m_scanMidTime
		&& m_syncTimestamp  == other.m_syncTimestamp
		&& m_points         == other.m_points
		&& m_scannerInfos   == other.m_scannerInfos
		// Either scannerInfos is not empty (in which case the
		// deprecated variables are ignored anyway), or the old
		// variables must be equal
		&& (!m_scannerInfos.empty()
			|| ((m_startAngle        == other.m_startAngle
				 || (isNaN(m_startAngle) && isNaN(other.m_startAngle)))
				&& (m_endAngle       == other.m_endAngle
					|| (isNaN(m_endAngle) && isNaN(other.m_endAngle)))
				&& m_scannerStatus  == other.m_scannerStatus
				&& m_deviceID       == other.m_deviceID
				&& m_scannerType    == other.m_scannerType
			   ));
}
*/

//
// After clear(), the scan will not contain any scanpoints at all.
//
void Scan::clear ()
{
	// Clear member variables
	m_flags			= 0;
	m_scanNumber	= 0x0000;
	m_sourceId		= 0;

	m_points.clear();
	m_scannerInfos.clear();
}

// Default destructor
Scan::~Scan()
{
}

void Scan::resize(size_type new_size, const ScanPoint& default_point)
{
	if (size_type((UINT16)new_size) != new_size)
	{
		throw std::out_of_range ("Scan::resize was called with new size larger than what fits into UINT16: " + toString(new_size) + " but only 65536 is allowed!");
	}

	m_points.resize(new_size, default_point);
}

void Scan::reserve(size_type new_capacity)
{
	if (size_type((UINT16)new_capacity) != new_capacity)
	{
		throw std::out_of_range ("Scan::reserve was called with new capacity larger than what fits into UINT16: " + toString(new_capacity) + " but only 65536 is allowed!");
	}

	m_points.reserve(new_capacity);
}

/**
 * The new point is added to the list of scan points
 * and the number of scan points is increased.
 */
ScanPoint& Scan::addNewPoint()
{
	m_points.push_back(ScanPoint());
	return m_points.back();
}

void Scan::setVehicleCoordinates(bool inVehicleCoordinates)
{
	if (inVehicleCoordinates)
		m_flags |= FlagVehicleCoordinates;
	else
		m_flags &= ~FlagVehicleCoordinates;
}


// Sort criterion. This method is passed to std::sort() to sort scan
// points by descending azimuth angles.
static bool isDescendingAngle (const ScanPoint& P1, const ScanPoint& P2)
{
	return (P1.getHAngle() > P2.getHAngle());
}

void Scan::sort()
{
	std::sort(getPointListBegin(), getPointListEnd(), isDescendingAngle);
}

void Scan::addCartesianOffset(double offsetX, double offsetY, double offsetZ)
{
	for (PointList::iterator point = getPointListBegin();  point != getPointListEnd();  point++)
		point->addCartesianOffset(offsetX, offsetY, offsetZ);
}

void Scan::addPolarOffset(double distOffset, double hAngleOffset, double vAngleOffset)
{
	for (PointList::iterator point = getPointListBegin();  point != getPointListEnd();  point++)
		point->addPolarOffset(distOffset, hAngleOffset, vAngleOffset);
}

void Scan::setScannerInfos(const ScannerInfoVector& v)
{
	// Sanity check: Only one ScannerInfo per deviceID allowed, except
	// for 8-Layer scanners.
	std::set<UINT8> availableDevicesOnce;
	std::set<UINT8> availableDevicesTwice;
	for (ScannerInfoVector::const_iterator it = v.begin();
		 it != v.end(); it++)
	{
		const ScannerInfo& sinfo = *it;
		const UINT8 deviceID = sinfo.getDeviceID();

		// Did we already see this deviceID previously?
		if (availableDevicesOnce.count(deviceID) == 0)
		{
			// No, so record this deviceID for later.
			availableDevicesOnce.insert(deviceID);
		}
		else
		{
			// Error: 4L-deviceID is here a second time!
			throw std::logic_error("Scan::setScannerInfos called with a list (size=" +
									::toString(v.size()) + ") that contains two ScannerInfos with device ID " +
									::toString(int(deviceID)) + ", type " + ScannerInfo::scannerTypeToString(sinfo.getScannerType()) +
									". This is not allowed - for each scanner at most one ScannerInfo may be stored in the ScannerInfoVector.");
		}
	}

	m_scannerInfos = v;
}



// Flags
/*
void Scan::setGroundLabeled(bool isGroundLabeled)
{
	if (isGroundLabeled)
		m_flags |=  FlagGroundLabeled;
	else
		m_flags &= ~FlagGroundLabeled;
}

void Scan::setRainLabeled(bool isRainLabeled)
{
	if (isRainLabeled)
		m_flags |=  FlagRainLabeled;
	else
		m_flags &= ~FlagRainLabeled;
}

void Scan::setDirtLabeled(bool isDirtLabeled)
{
	if (isDirtLabeled)
		m_flags |=  FlagDirtLabeled;
	else
		m_flags &= ~FlagDirtLabeled;
}

void Scan::setCoverageLabeled(bool isCoverageLabeled)
{
	if (isCoverageLabeled)
		m_flags |=  FlagCoverageLabeled;
	else
		m_flags &= ~FlagCoverageLabeled;
}

void Scan::setBackgroundLabeled(bool isBackgroundLabeled)
{
	if (isBackgroundLabeled)
		m_flags |=  FlagBackgroundLabeled;
	else
		m_flags &= ~FlagBackgroundLabeled;
}

void Scan::setReflectorLabeled(bool isReflectorLabeled)
{
	if (isReflectorLabeled)
		m_flags |=  FlagReflectorLabeled;
	else
		m_flags &= ~FlagReflectorLabeled;
}
*/

//
// Transform the scan to vehicle coordinates, but do not sort the resulting scanpoints by angles.
//
bool Scan::transformToVehicleCoordinatesUnsorted()
{
	// Check if Scan might already be converted
	if ((getFlags() & FlagVehicleCoordinates) != 0)
	{
		// Ist schon in Fahrzeugkoordinaten.
		// traceDebug(SCAN_VERSION) << "convertToVehicleCoordinates: Scan (#" << getScanNumber() << ") already in vehicle coordinates. Nothing to do." << std::endl;
		return true; // it's ok, job is already done
	}

	// If there are no scanner informations available -> no transformation
	if (getScannerInfos().empty())
	{
		printWarning("Scan::convertToVehicleCoordinates: Scan (#" + ::toString(getScanNumber()) +
						") has empty ScannerInfos - no conversion possible, aborting!");
		return false;
	}

	// Hole die Scanner-Position
	Position3D mountPos = m_scannerInfos.at(0).getMountingPosition();
//	printInfoMessage("Scan::transformToVehicleCoordinatesUnsorted: Mounting position is " + mountPos.toString() + ".", true);
	
	// Transformiere alle Punkte
	for (Scan::iterator sptIt = begin(); sptIt != end(); sptIt++)
	{
		Point3D pt = sptIt->toPoint3D();
		mountPos.transformToVehicle(&pt);
		sptIt->setPoint3D(pt);
	}


	// Set transformation flag (if everything went well)
	setFlags(getFlags() | FlagVehicleCoordinates );

	// Also set the transformation flag in all the ScannerInfo entries
	for (ScannerInfoVector::iterator siIt = getScannerInfos().begin(); siIt != getScannerInfos().end(); siIt++)
	{
		ScannerInfo& sinfo = *siIt;
		sinfo.setScanFlags(sinfo.getScanFlags() | FlagVehicleCoordinates);
	}

	return true;
}

bool Scan::transformToVehicleCoordinates()
{
	const bool r = transformToVehicleCoordinatesUnsorted();

	// We also sort this because this got
	// forgotten all too often.
	sort();

	return r;
}


Scan::ScannerInfoVector& Scan::getScannerInfos()
{
	return m_scannerInfos;
}

const ScannerInfo* Scan::getScannerInfoByDeviceId(UINT8 id) const
{
	for (ScannerInfoVector::const_iterator it = m_scannerInfos.begin();
		 it != m_scannerInfos.end();
		 it++)
	{
		if ((*it).getDeviceID() == id)
			return &(*it);
	}

	return NULL;
}

void Scan::clearLabelFlag(Scan::ScanFlags scanFlag)
{
	UINT32 test = ~(scanFlag);
	m_flags &= test;
}

}	// namespace datatypes
