//
// LdmrsNtpTimeApp.cpp
//
// Demonstrates setting and receiving the NTP timestamp of the
// LDMRS. Sets the timestamp to Jan 1st, 2000.
// Note that during startup, the LD-MRS device class already sets the current
// local time.
// This feature is available in all firmware versions.
//
// The App starts a thread that, after some time, sets the new time in the scanner.
//
// All changes to the sensor configuration are non-permanent.
//

#include "LdmrsNtpTimeApp.hpp"
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
#include <time.h>

namespace application
{

//
// Constructor
//
LdmrsNtpTimeApp::LdmrsNtpTimeApp(Manager* manager) :
		m_manager(manager)
{
	m_beVerbose = true;
	
	// Start the thread that sets the time.
	if (m_changeThread.isRunning() == false)
	{
		m_changeThread.run(this);
	}
	
	printInfoMessage("LdmrsNtpTimeApp constructor done.", m_beVerbose);
}


// Destructor
// Clean up all dynamic data structures
LdmrsNtpTimeApp::~LdmrsNtpTimeApp()
{
	printInfoMessage("LdmrsNtpTimeApp says Goodbye!", m_beVerbose);
}


//
// Receiver for new data from the manager.
//
void LdmrsNtpTimeApp::setData(BasicData& data)
{
	// we are just interested in scan data
	switch (data.getDatatype())
	{
		case Datatype_Scan:
		{
			// Print the scan start timestamp (UTC time)
			Scan* scan = dynamic_cast<Scan*>(&data);
			const ScannerInfo* info = scan->getScannerInfoByDeviceId(1);

			if (info != NULL)
			{
				const Time& time = info->getStartTimestamp();
				printInfoMessage("LdmrsApp::setData(): Scan start time: " + time.toLongString(), m_beVerbose);
			}
			break;
		}
		default:
			break;
	}
}


//
// Convert UNIX system time (seconds since 1.1.1970) to NTP time (secons since 1.1.1900).
//
ntp_time_t LdmrsNtpTimeApp::convertUnixTimeToNtpTime(struct timeval& unixTime)
{
	ntp_time_t ntpTime;
	ntpTime.second = unixTime.tv_sec + 0x83AA7E80;
	ntpTime.fraction = (uint32_t)( (double)(unixTime.tv_usec+1) * (double)(1LL<<32) * 1.0e-6 );
	return ntpTime;
}

//
// Thread that does the actual changing of parameters.
//
void LdmrsNtpTimeApp::changeThreadFunction(bool& endThread, UINT16& waitTimeMs)
{
	printInfoMessage("LdmrsNtpTimeApp::changeThreadFunction(): Started.", m_beVerbose);
	bool result;

	devices::LDMRS* ldmrs = dynamic_cast<devices::LDMRS*>(m_manager->getFirstDeviceByType(Sourcetype_LDMRS));

	// Ensure that we have a valid pointer to the device
	if (ldmrs != NULL)
	{
		// Sleep some time to receive some scans first.
		UINT32 sleepTimeMs = 2000;	// 2000 ms = 2 s
		usleep(sleepTimeMs * 1000);

		// Set the date and time to Jan 1st, 2000.
		struct timeval now;
		now.tv_sec = 946681200;			// 1.1.2000
		now.tv_usec = 0;				// 0 microseconds
		// Convert to NTP
		ntp_time_t ntpTime = convertUnixTimeToNtpTime(now);
		ntpTime.fraction = (uint32_t)0;
	
		// Set the time
		result = ldmrs->setNtpTime(ntpTime.second, ntpTime.fraction);
		
		if (result == true)
		{
			// Success
			printInfoMessage("LdmrsNtpTimeApp::changeThreadFunction(): Successfully set the NTP time.", true);
		}
		else
		{
			// Failure
			printError("LdmrsNtpTimeApp::changeThreadFunction(): Failed to set NTP time!");
		}
	}
	else
	{
		// We do not have a pointer to the sensor!
		printError("LdmrsNtpTimeApp::changeThreadFunction():  Failed to get a valid pointer to the MRS, aborting!");
	}
		

	endThread = true;
	waitTimeMs = 0;
	printInfoMessage("LdmrsNtpTimeApp::changeThreadFunction():  All done, leaving.", m_beVerbose);
}

}	// namespace application
