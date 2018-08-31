//
// LD_MRS.cpp
//
// Device class of a simple LD-MRS laserscanner interface.
//
//
// To configure the device, add the parameters in the constructor below.
//
// VERSION 1.1.0
//
// Version history:
// 1.0.0  , willhvo
//   - Initial version
//
// 1.1.0  , willhvo
//   - Added reading from file
//


//
#include "LD_MRS.hpp"
#include "../tools/errorhandler.hpp"

namespace devices
{

using namespace datatypes;

LDMRS::LDMRS(Manager* manager)
	: m_manager(manager)
	, m_sopas(NULL)
	, m_lux(NULL)
{
	setDevicetype(Sourcetype_LDMRS);
	m_beVerbose = false;
	
	printInfoMessage("LDMRS::LDMRS: Starting constructor.", m_beVerbose);
	
	m_lux = NULL;
	
	m_weWantScanData = true;	// Enable or disable scan data reception via the MRS (=non-SOPAS) interface.
	m_weWantObjectData = false;	// Enable or disable object data reception via the MRS (=non-SOPAS) interface.
	m_weWantFieldData = false;	// Enable or disable protection field and eval case data reception via the SOPAS interface.
	m_weWantScanDataFromSopas = false;	// Enable or disable scan data reception via the SOPAS interface.

//	m_inputFileName = "/home/willhvo/Temp/raw_ldmrs_log.bin";		// File name of input file. If this name is given, connects to file instead of TCP.
	m_inputFileName = "";
	m_ipAddress = "192.168.0.1";	// , "Network IP address where the device is located.", "0.0.0.0");
	m_luxPortNumber = 12002;	// Network port on which the legacy (non-sopas) device is contacted (for scans and objects), typically 12002
	m_SopasPortNumber = 2111;	// Network port on which the device is contacted, typically 2111 or 2112.

	// parameters for LuxBase/Scans
	m_scanFrequency = 12.5;				// Scan frequency (12.5, 25 or 50 Hz)
	m_scanStartAngle = 45.0 * deg2rad;	// Start angle of the scan.
	m_scanEndAngle = -45.0 * deg2rad;	// End angle of the scan.
	m_readOnlyMode = false;		// Forbids parameter writes to the scanner if true.

//	m_scanAngleRes = 0.25;
//	m_skipNumScans = 0;		// Use this value if the sensor scan frequency is too high and if scans should be skipped.

	m_offsetX = 0.0;	// x-coordinate of sensor mounting position in the vehicle coordinate system.
	m_offsetY = 0.0;	// y-coordinate of sensor mounting position in the vehicle coordinate system
	m_offsetZ = 0.0;	// z-coordinate of sensor mounting position in the vehicle coordinate system
	m_yawAngle = 0.0;	// Yaw angle of sensor mounting position in the vehicle coordinate system ("left and right")
	m_pitchAngle = 0.0;	// Pitch angle of sensor mounting position in the vehicle coordinate system ("up and down")
	m_rollAngle = 0.0;	// Roll angle of sensor mounting position in the vehicle coordinate system ("to the sides")

	printInfoMessage("LDMRS::LDMRS: Constructor done.", m_beVerbose);
}



LDMRS::~LDMRS()
{
	printInfoMessage("LDMRS::~LDMRS: Starting destructor.", m_beVerbose);
	
	if (isRunning())
	{
		stop();
	}
	
	printInfoMessage("LDMRS::~LDMRS: Destructor done.", m_beVerbose);
}


//
// Initialising.
//
// It would be nicer if this function would be called with all the device parameters,
// instead of setting them as module variables...
//
bool LDMRS::init()
{
	printInfoMessage("LDMRS::init: Called.", m_beVerbose);

	std::string longName = "LD-MRS";
	bool result = true;

//	m_beVerbose = false;
	m_isRunning = false;
	
	// Altes Objekt loeschen
	if (m_lux != NULL)
	{
		m_lux->stop();
		delete m_lux;
		m_lux = NULL;
	}

	// initializing LUX part
	printInfoMessage("LDMRS::init: Creating new LuxBase object.", m_beVerbose);
	m_lux = new LuxBase(m_manager,
							getSourceId(),
							longName,
							m_ipAddress,
							m_luxPortNumber,
							m_scanFrequency,
							m_scanStartAngle,
							m_scanEndAngle,
							m_offsetX,
							m_offsetY,
							m_offsetZ,
							m_yawAngle,
							m_pitchAngle,
							m_rollAngle,
							m_beVerbose,
							m_inputFileName
   					);
	
	printInfoMessage("LDMRS::init: Initializing the LuxBase object.", m_beVerbose);
	if (m_inputFileName == "")
	{
		// TCP
		result = m_lux->initTcp(LDMRS::disconnectFunctionS, (void*)this);
	}
	else
	{
		// File
		result = m_lux->initFile(LDMRS::disconnectFunctionS, (void*)this);
	}

	if (result == true)
	{
		// Success
		printInfoMessage("LDMRS::init: LuxBase was successfully initialized.", m_beVerbose);
	}
	else
	{
		// Error
		printError("LDMRS::init: Failed to initialize LuxBase - Device init failed, aborting.");
		return false;
	}
	
	// Set the current time
	Time t = Time::now();
	if (m_inputFileName == "")
	{
		// TCP
		result = m_lux->cmd_setNtpTimestamp((UINT32)(t.seconds() + Time::secondsFrom1900to1970), 0);
		if (result == true)
		{
			// Success
			printInfoMessage("LDMRS::init: NTP timestamp was successfully set.", m_beVerbose);
		}
		else
		{
			// Error
			printError("LDMRS::init: Failed to set NTP timestamp - Device init failed, aborting.");
			return false;
		}
	}


	return true;
}


//
// Static-Einstiegspunkt fuer die DisconnectFunction.
//
void LDMRS::disconnectFunctionS(void* obj)
{
	printInfoMessage("LDMRS::disconnectFunctionS: Called.", true);

	if (obj != NULL)
	{
		((LDMRS*)obj)->disconnectFunction();
	}
	
//	SystemStateWatchdog::setNotConnected();

	printInfoMessage("LDMRS::disconnectFunctionS: Done.", true);
}

//
//
//
void LDMRS::disconnectFunction()
{
	printInfoMessage("LDMRS::disconnectFunction: LDMRS " + getDeviceName() + " was disconnected.", m_beVerbose);

//	SystemStateWatchdog::setNotConnected();
}


//
// Schliesse alle Schnittstellen.
//
void LDMRS::shutdown()
{
	printInfoMessage("LDMRS::shutdown: called. Shutting down the LDMRS" + getDeviceName() + ".", m_beVerbose);

	if (m_lux->isRunning())
	{
		stop();
	}
}

//
// Write a field to the sensor
//
bool LDMRS::writeField(UINT16 fieldNum, const FieldParameter& para)
{
	bool result;
	bool beVerboseHere = m_beVerbose;
	beVerboseHere = true;
	
	printInfoMessage("LDMRS::writeField: Called. Logging in now.", beVerboseHere);
	result = m_sopas->action_login();
	
	if (result == true)
	{
		printInfoMessage("LDMRS::writeField: Login was successful, writing field now.", beVerboseHere);
		result = m_sopas->action_writeField(fieldNum, para);
	}
	
	if (result == true)
	{
		printInfoMessage("LDMRS::writeField: Field was written, logging out now.", beVerboseHere);
		result = m_sopas->action_logout();
	}
	
	printInfoMessage("LDMRS::writeField: All done, leaving.", beVerboseHere);
	return result;
}

//
// Write an EvalCase configuration to the sensor.
// This is a structure with all (up to 16) eval cases.
//
bool LDMRS::writeEvalCases(const EvalCases& evalCases)
{
	bool result;
	bool beVerboseHere = m_beVerbose;
	beVerboseHere = true;
	
	printInfoMessage("LDMRS::writeEvalCases: Called. Logging in now.", beVerboseHere);
	result = m_sopas->action_login();
	
	if (result == true)
	{
		printInfoMessage("LDMRS::writeEvalCases: Login was successful, writing eval cases now.", beVerboseHere);
		result = m_sopas->action_writeEvalCases(evalCases);
	}
	
	if (result == true)
	{
		printInfoMessage("LDMRS::writeEvalCases: Eval cases were written, logging out now.", beVerboseHere);
		result = m_sopas->action_logout();
	}
	
	printInfoMessage("LDMRS::writeEvalCases: All done, leaving.", beVerboseHere);
	return result;
}

//
// Stores the SOPAS config data (fields and eval cases) permanently.
//
bool LDMRS::flashSopasConfig()
{
	bool result;
	bool beVerboseHere = m_beVerbose;
	beVerboseHere = true;
	
	printInfoMessage("LDMRS::flashSopasConfig: Called.", beVerboseHere);

	result = m_sopas->action_flashFieldParameters();
	if (result == true)
	{
		printInfoMessage("LDMRS::flashSopasConfig: Configuration was saved.", beVerboseHere);
	}
	else
	{
		// Failure
		printError("LDMRS::flashSopasConfig: Failed to save configuration!");
	}
	
	printInfoMessage("LDMRS::writeField: All done, leaving.", beVerboseHere);
	return result;
}


//
// Sets the MRS-internal clock to the given time.
//
bool LDMRS::setNtpTime(UINT32 seconds, UINT32 fractionalSec)
{
	bool result;
	bool beVerboseHere = m_beVerbose;
	beVerboseHere = true;
	
	printInfoMessage("LDMRS::setNtpTime: Called.", beVerboseHere);
	
	if (m_lux == NULL)
	{
		printError("LDMRS::setNtpTime: No LUX-Base object available, aborting!");
		return false;
	}

	result = m_lux->cmd_setNtpTimestamp(seconds, fractionalSec);
	if (result == true)
	{
		printInfoMessage("LDMRS::setNtpTime: Timestamp was set.", beVerboseHere);
	}
	else
	{
		// Failure
		printError("LDMRS::setNtpTime: Failed to set timestamp!");
	}
	
	printInfoMessage("LDMRS::setNtpTime: All done, leaving.", beVerboseHere);
	return result;
}

bool LDMRS::setScanAngles(double startAngle, double endAngle)
{
	bool result;
	bool beVerboseHere = m_beVerbose;

	printInfoMessage("LDMRS::setScanAngles: Called.", beVerboseHere);

	if (m_lux == NULL)
	{
		printError("LDMRS::setScanAngles: No LUX-Base object available, aborting!");
		return false;
	}

	result = m_lux->cmd_setScanAngles(startAngle, endAngle);
	if (result == true)
	{
		printInfoMessage("LDMRS::setScanAngles: scan angles were set.", beVerboseHere);
	}
	else
	{
		// Failure
		printError("LDMRS::setScanAngles: Failed to set scan angles!");
	}

	printInfoMessage("LDMRS::setScanAngles: All done, leaving.", beVerboseHere);
	return result;
}

bool LDMRS::setSyncAngleOffset(double syncAngle)
{
	bool result;
	bool beVerboseHere = m_beVerbose;

	printInfoMessage("LDMRS::setSyncAngleOffset: Called.", beVerboseHere);

	if (m_lux == NULL)
	{
		printError("LDMRS::setSyncAngleOffset: No LUX-Base object available, aborting!");
		return false;
	}

	result = m_lux->cmd_setSyncAngleOffset(syncAngle);
	if (result == true)
	{
		printInfoMessage("LDMRS::setSyncAngleOffset: sync angle offset was set.", beVerboseHere);
	}
	else
	{
		// Failure
		printError("LDMRS::setSyncAngleOffset: Failed to set sync angle offset!");
	}

	printInfoMessage("LDMRS::setSyncAngleOffset: All done, leaving.", beVerboseHere);
	return result;
}

bool LDMRS::setScanFrequency(double scanFreq)
{
	bool result;
	bool beVerboseHere = m_beVerbose;

	printInfoMessage("LDMRS::setScanFrequency: Called.", beVerboseHere);

	if (m_lux == NULL)
	{
		printError("LDMRS::setScanFrequency: No LUX-Base object available, aborting!");
		return false;
	}

	result = m_lux->cmd_setScanFrequency(scanFreq);
	if (result == true)
	{
		printInfoMessage("LDMRS::setScanFrequency: scan frequency was set.", beVerboseHere);
	}
	else
	{
		// Failure
		printError("LDMRS::setScanFrequency: Failed to set scan frequency!");
	}

	printInfoMessage("LDMRS::setScanFrequency: All done, leaving.", beVerboseHere);
	return result;
}

std::string LDMRS::getIpAddress()
{
	return m_ipAddress;
}

void LDMRS::setIpAddress(std::string ipAdress)
{
	m_ipAddress = ipAdress;
}

void LDMRS::setWeWantObjectData(bool weWantObjectData) {
	m_weWantObjectData = weWantObjectData;
}

std::string LDMRS::getSerialNumber()
{
	if (m_lux == NULL)
	{
		return "(none)";
	} else {
		return m_lux->getSerialNumber();
	}
}

std::string LDMRS::getFirmwareVersion()
{
	if (m_lux == NULL)
	{
		return "(none)";
	} else {
		return m_lux->getFirmwareVersion();
	}
}


//
// Starte das Einlesen von Daten (Scans, Objekte, Schutzfelder, ...)
//
bool LDMRS::run()
{
	if (m_lux == NULL)
	{
		printError("LDMRS::run: called, but pointer to internal data handler is NULL. Was init() called? Aborting!");
		return false;
	}
	
	//
	// LUX-Part
	//
	printInfoMessage("LDMRS::run: Called. Run the LDMRS" + getDeviceName() + ".", m_beVerbose);

	// This will cause that we get scan and/or obect data via lux protocol
	bool luxSuccess = m_lux->run(m_weWantScanData, m_weWantObjectData);
	if (luxSuccess == true)
	{
		// Success
		printInfoMessage("LDMRS::run: Device " + getDeviceName() + " is running.", m_beVerbose);
		m_lux->setOnScanReceiveCallbackFunction(LDMRS::onScanReceivedS, (void*)this);
	}
	else
	{
		// Fail
		printError("LDMRS::run: Failed to run device " + getDeviceName() + ", aborting!");
		return false;
	}

	//
	// SOPAS part
	//
	if ((m_inputFileName == "") &&
		((m_weWantFieldData == true) ||
		 (m_weWantScanDataFromSopas == true)))
	{
		// TCP only
		bool sopasSuccess = true;
		printInfoMessage("LDMRS::run: Now starting SOPAS part (eval cases) of device " + getDeviceName() + ".", m_beVerbose);

		// Initialize Sopas part
		m_sopas = new LdmrsSopasLayer(m_manager,
									getSourceId(),
									m_ipAddress,
									m_SopasPortNumber,
									m_weWantFieldData,
									m_weWantScanDataFromSopas,
									m_readOnlyMode);
		sopasSuccess = m_sopas->init(LDMRS::disconnectFunctionS, (void*)this);

		if (sopasSuccess == false)
		{
			printError("LDMRS::run: Failed to initialize SOPAS part of device " + getDeviceName() + ", aborting!");
			stop();
			return false;
		}

		// Success, now run the device
		printInfoMessage("LDMRS::run: SOPAS part is initialized, now running it.", m_beVerbose);
		sopasSuccess = m_sopas->run();
		if (sopasSuccess == false)
		{
			printError("LDMRS::run: Failed to run SOPAS part of device " + getDeviceName() + ", aborting!");
			stop();
			return false;
		}
			
		printInfoMessage("LDMRS::run: SOPAS part is running, device " + getDeviceName() + " is running, all done.", m_beVerbose);
	}
	
	return true;
}


//
// Stop data reception, but do not shut down the connection.
//
bool LDMRS::stop()
{
	printInfoMessage("LDMRS::stop: Called. Stopping the LDMRS" + getDeviceName() + ".", m_beVerbose);
	
	bool luxSuccess = true;
	bool sopasSuccess = true;
	
	if (m_sopas != NULL)
	{
		if (m_sopas->isFieldDataSubscribed())
		{
			sopasSuccess = m_sopas->action_unSubscribeEvalCaseResults();
		}
	}

	if (m_lux != NULL)
	{
		luxSuccess = m_lux->stop();
	}
	
	return (luxSuccess && sopasSuccess);
}


bool LDMRS::isRunning()
{
	if (m_lux == NULL) // no m_lux object yet? Is not running.
	{
		return false;
	}
	bool luxSuccess = m_lux->isRunning();
	//bool sopasSuccess = m_sopas.isRunning();
	return (luxSuccess);	//  && sopasSuccess
}

//
// Statischer Einstiegspunkt
//
void LDMRS::onScanReceivedS(void* obj)
{
	if (obj != NULL)
	{
		((LDMRS*)obj)->onScanReceived();
	}
}

// after / before every scan we have to post the current sensor state as SensorInfo Structure
void LDMRS::onScanReceived()
{
//	printInfoMessage("LDMRS::onScanReceived: Called.", m_beVerbose);
	
//	SensorStateInfo sensorInfo = m_sopas.getSensorStateInfo();
//	getSerializer().notifyMessageHandlers(sensorInfo, getDeviceID());
}

bool LDMRS::getParameter(MrsParameterId id, UINT32* value)
{
	return m_lux->cmd_getParameter(id, value);
}

bool LDMRS::setParameter(MrsParameterId id, UINT32 value)
{
	return m_lux->cmd_setParameter(id, value);
}


} // namespace devices

