//
// LdmrsSectorChangeApp.cpp
//
// Demonstrates setting and checking the FlexibleResolution feature of the
// LDMRS. This feature is not available in all firmware versions.
//
// The App starts a thread that, after some time, sets the ResolutionMap of the scanner,
// activates it and later deactivates it.
//
// All changes to the sensor configuration are non-permanent.
//

#include "LdmrsSectorChangeApp.hpp"
#include "../tools/errorhandler.hpp"	// for printInfoMessage()
#include "../tools/toolbox.hpp"			// for toString()
#include "../tools/MathToolbox.hpp"
#include "../datatypes/Scan.hpp"
#include "../datatypes/Object.hpp"
#include "../datatypes/Msg.hpp"
#include "../datatypes/Measurement.hpp"
#include "../datatypes/Fields.hpp"
#include "../datatypes/EvalCases.hpp"
#include "../datatypes/EvalCaseResults.hpp"
#include "../devices/LD_MRS.hpp"

namespace application
{

//
// Constructor
//
LdmrsSectorChangeApp::LdmrsSectorChangeApp(Manager* manager) :
		m_manager(manager)
{
	m_beVerbose = true;

	printInfoMessage("LdmrsSectorChangeApp constructor done.", m_beVerbose);

	// Start the thread that changes the configuration.
	if (m_changeThread.isRunning() == false)
	{
		m_changeThread.run(this);
	}
}


// Destructor
// Clean up all dynamic data structures
LdmrsSectorChangeApp::~LdmrsSectorChangeApp()
{
	printInfoMessage("LdmrsSectorChangeApp says Goodbye!", m_beVerbose);
}

//
// Convert the current Sector or ResolutionMap (which is a vector of sectors) to a readable string.
//
std::string sectorToString(const ScannerInfo::ResolutionMap& sector)
{
	UINT32 i = 0;
	std::ostringstream oss;
	oss << std::endl;
	for(ScannerInfo::ResolutionMap::const_iterator s = sector.begin(); s != sector.end(); ++s)
	{

		oss  << "Sector (" << i << "): start=" << doubleToString(s->first*rad2deg, 3) << " res=" << doubleToString(s->second*rad2deg,3) << std::endl;
		i++;
	}

	return oss.str();
}

Scan::const_iterator getNextPointInSameLayer(Scan::const_iterator iter, const Scan& scan)
{
	Scan::const_iterator ret = iter;
	for (++iter /*start with the next point*/ ; iter != scan.end(); ++iter)
	{
		if (iter->getLayer() == ret->getLayer())
		{
			ret = iter;
			break;
		}
	}
	return ret;

}

Scan::const_iterator getNextPoint(Scan::const_iterator iter, const Scan& scan)
{
	Scan::const_iterator ret = iter;
	++iter;
	if (iter != scan.end())
	{
		ret = iter;
	}
	return ret;

}

void LdmrsSectorChangeApp::checkResolution(Scan& scan)
{
	if (scan.size() < 10)
	{
		// do not process on scans with too few scan points
		return;
	}

	// We need the scan in scanner coordinate system (sorted)
	if ((scan.getFlags() & Scan::FlagVehicleCoordinates) != 0)
	{
		// in vehicle coordinate
		printWarning("LdmrsSectorChangeApp::checkResolution: Scan is in vehicle coordinates and thus cannot be processed!");
		return;
	}

	// iterate through all scan points and make a diff of the angles
	Scan::const_iterator p;
	float angleDiff; // compute differences

	ScannerInfo::ResolutionMap sector; // first = start angle, second = resolution

	for (p = scan.begin(); p != scan.end(); ++p)
	{
		Scan::const_iterator p2 = getNextPoint(p, scan);
		Scan::const_iterator p3 = getNextPointInSameLayer(p, scan);
		float interlaced = 1.;
		if (fuzzyCompare(p2->getHAngle(), p3->getHAngle()))
		{
			// we are close to the border -> only 2 scan layers left
			interlaced = 0.5;

		}
		angleDiff = std::fabs(p2->getHAngle() - p->getHAngle()) * interlaced;

		if (angleDiff > 0)
		{
			if (sector.size() == 0)
			{
				sector.push_back(std::make_pair(p->getHAngle(), angleDiff));
			}
			else
			{
				// we detected a new resolution
				if (fuzzyCompare(float(sector.back().second), angleDiff) == false)
				{
					sector.push_back(std::make_pair(p->getHAngle(),angleDiff));
				}
			}
		}
	}

	if (sector != m_lastMeasuredSector)
	{
		std::ostringstream oss;
		oss << "Sector:" << sectorToString(sector);
		printInfoMessage(oss.str(), m_beVerbose);
		m_lastMeasuredSector = sector;
	}
}

//
// Receiver for new data from the manager.
//
void LdmrsSectorChangeApp::setData(BasicData& data)
{
	// we are just interested in scan data
	switch (data.getDatatype())
	{
		case Datatype_Scan:
			{
				Scan* scan = dynamic_cast<Scan*>(&data);

				if (scan)
				{
					checkResolution(*scan);
				}
			}
			break;
		default:
			break;
	}
}

/**
 * Convert an angle from radians to code-wheel ticks.
 *
 * \param radAngle angle in radians
 *
 * \return The angle \a radAngle as raw data.
 *
 */
INT16 radian2Int (float radAngle)
{
	float rad2ticks = float(devices::LuxBase::ANGULAR_TICKS_PER_ROTATION) / (2.f * PI);	// conversion factor from [rad] to [ticks]
	radAngle = normalizeRadians(radAngle);									// shift [pi, 2*pi) to [-pi, 0)
	INT16 angleInt = round_to_int<INT16> (radAngle * rad2ticks);
	return angleInt;
}

bool LdmrsSectorChangeApp::readDetailedErrorCode(UINT32* errCode)
{
	// read error variable
	UINT32 code;

	devices::LDMRS* scanner = dynamic_cast<devices::LDMRS*>(m_manager->getFirstDeviceByType(Sourcetype_LDMRS));
	if (scanner)
	{
		if(!scanner->getParameter(devices::ParaDetailedError, &code))
		{
			printWarning("NOT ABLE to to read detailed error code");
			return false;
		}
		switch(code)
		{
		case devices::ErrFlexResFreqInvalid:
			printInfoMessage("Detailed Error code: FlexResFreqInvalid", m_beVerbose);
			break;
		case devices::ErrFlexResNumSectorsInvalid:
			printInfoMessage("Detailed Error code: FlexResNumSectorsInvalid", m_beVerbose);
			break;
		case devices::ErrFlexResNumShotsInvalid:
			printInfoMessage("Detailed Error code: FlexResNumShotsInvalid", m_beVerbose);
			break;
		case devices::ErrFlexResResolutionInvalid:
			printInfoMessage("Detailed Error code: FlexResResolutionInvalid", m_beVerbose);
			break;
		case devices::ErrFlexResScannerNotIdle:
			printInfoMessage("Detailed Error code: FlexResScannerNotIdle", m_beVerbose);
			break;
		case devices::ErrFlexResSectorsOverlapping:
			printInfoMessage("Detailed Error code: FlexResSectorsOverlapping", m_beVerbose);
			break;
		case devices::ErrFlexResSizeOneEighthSectorInvalid:
			printInfoMessage("Detailed Error code: FlexResSizeOneEighthSectorInvalid", m_beVerbose);
			break;
		default:
			break;
		}

		if (errCode != NULL)
		{
			*errCode = code;
		}
	}
	else
	{
		return false;
	}

	return true;
}

bool LdmrsSectorChangeApp::changeFlexResConfiguration(const ScannerInfo::ResolutionMap& configuredRM)
{
	if(configuredRM.size() == 0 || configuredRM.size() > 8)
	{
		printWarning("Invalid resolution map size.");
		return false;
	}

	devices::LDMRS* scanner = dynamic_cast<devices::LDMRS*>(m_manager->getFirstDeviceByType(Sourcetype_LDMRS));
	if (scanner)
	{
		if (!scanner->setParameter(devices::ParaNumSectors, UINT32(configuredRM.size() ) ) )
		{
			printInfoMessage("LdmrsSectorChangeApp::changeFlexResConfiguration(): set NumSectors not successful", m_beVerbose);
			readDetailedErrorCode();
			return false;
		}

		for (UINT32 i = 0; i < configuredRM.size(); ++i)
		{
			// set start angle
			if (!scanner->setParameter((devices::MrsParameterId)(0x4001 + i), radian2Int(configuredRM[i].first) ) )
			{
				printInfoMessage("LdmrsSectorChangeApp::changeFlexResConfiguration(): set start angle not successful: " + doubleToString(configuredRM[i].first, 3) , m_beVerbose);
				readDetailedErrorCode();
				return false;
			}
			// set resolution
			if (!scanner->setParameter((devices::MrsParameterId)(0x4009 + i), radian2Int(configuredRM[i].second) ) )
			{
				printInfoMessage("LdmrsSectorChangeApp::changeFlexResConfiguration(): set resolution not successful: " + doubleToString(configuredRM[i].first, 3) , m_beVerbose);
				readDetailedErrorCode();
				return false;
			}
		}
	}

	return true;
}

//
// Set the AngularResolutionType to either Focused, Constant or FlexRes.
//
bool LdmrsSectorChangeApp::changeAngularResolutionType(devices::AngularResolutionType type)
{
	devices::LDMRS* scanner = dynamic_cast<devices::LDMRS*>(m_manager->getFirstDeviceByType(Sourcetype_LDMRS));
	if (scanner)
	{
		if (scanner->setParameter(devices::ParaAngularResolutionType, UINT32(type) ))
		{
//			printInfoMessage("LdmrsSectorChangeApp::changeThreadFunction(): set NumSectors to 9 not successful", m_beVerbose);
		}
		else
		{
//			traceNote(VERSION) << "NOT ABLE to change ResType to: type=" << type  << std::endl;
			readDetailedErrorCode();
			return false;
		}
	}

	return true;
}

//
// Thread that does the actual changing of parameters.
//
void LdmrsSectorChangeApp::changeThreadFunction(bool& endThread, UINT16& waitTimeMs)
{
	printInfoMessage("LdmrsSectorChangeApp::changeThreadFunction(): started", m_beVerbose);

	devices::LDMRS* ldmrs = dynamic_cast<devices::LDMRS*>(m_manager->getFirstDeviceByType(Sourcetype_LDMRS));

	if (ldmrs)
	{
		ldmrs->setScanAngles(50.0 * deg2rad, -60.0 * deg2rad);

		UINT32 value = 0;
		if (!ldmrs->getParameter(devices::ParaNumSectors, &value))
		{
			// could not read parameter
		}
		else
		{
			printInfoMessage("LdmrsSectorChangeApp::changeThreadFunction(): read parameter NumSectors=" + toString(value), m_beVerbose);
		}

		if (!ldmrs->setParameter(devices::ParaNumSectors, 9))
		{
			// could not set parameter
			printInfoMessage("LdmrsSectorChangeApp::changeThreadFunction(): set NumSectors to 9 not successful", m_beVerbose);

			// read detailed error
			UINT32 code;
			readDetailedErrorCode(&code);

			if (code != devices::ErrFlexResNumSectorsInvalid)
			{
				printWarning("Received wrong detailed error code.");
			}
		}

		// sleep some time to receive some scans first
		UINT32 sleepTimeMs = 10000;
		usleep(sleepTimeMs * 1000);

		ScannerInfo::ResolutionMap configuredRM;
		configuredRM.push_back(std::make_pair(50.f * deg2rad, 1.f * deg2rad));
		configuredRM.push_back(std::make_pair(35.f * deg2rad, .5f * deg2rad));
		configuredRM.push_back(std::make_pair(30.f * deg2rad, .25f * deg2rad));
		configuredRM.push_back(std::make_pair(20.f * deg2rad, .125f * deg2rad));
		configuredRM.push_back(std::make_pair(0.f * deg2rad, .25f * deg2rad));
		configuredRM.push_back(std::make_pair(-20.f * deg2rad, .5f * deg2rad));
		configuredRM.push_back(std::make_pair(-31.f * deg2rad, 1.f * deg2rad));
		configuredRM.push_back(std::make_pair(-40.f * deg2rad, .5f * deg2rad));

		changeFlexResConfiguration(configuredRM);

		changeAngularResolutionType(devices::ResolutionTypeFlexRes);

		usleep(sleepTimeMs * 1000);

		// change back
		changeAngularResolutionType(devices::ResolutionTypeConstant);
	}

	endThread = true;
	waitTimeMs = 0;
	printInfoMessage("LdmrsSectorChangeApp::changeThreadFunction():  All done, leaving.", m_beVerbose);
}

}	// namespace application
