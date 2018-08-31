//
// LuxBase.cpp
// Generic functions of the LD-MRS laserscanner interface.
//

//
// HISTORY
//
// This device is indended as a customer reference for the LD-MRS interface and coding / decoding of
// messages. Its interaction with other parts of the system and its usage of functions is intentionally
// kept simple, although this results in a somewhat lower performance than the original LUX/MRS
// device classes.
//
// Usage notes
//
// This device opens a TCP connection to an LD-MRS scanner. It then sets the desired parameters and
// receives incoming data. Currently, this is:
// - scans
// - objects
// - warning and error messages
//
// Internally, it has 2 buffers. The first level, called inputBuffer, is filled with whatever data is
// received on the interface. Scans and Objects are decoded right out of this buffer. Other messages,
// such as command replies and error messages, are forwarded to the cmdReplyBuffer where they are
// decoded. Note the different thread contexts: Scans and Objects are decoded in the receive-thread
// context, while data in the cmdReplyBuffer is decoded in the main device context asynchronously.
//
//  1.0.0, 2011-06-20, VWi
//          Initial version.
//  1.1.0, 2013-02-12, VWi
//          Added reading binary data from file.

//
#define LUXBASE_VERSION "1.1.0"

#include "LuxBase.hpp"
#include "../tools/errorhandler.hpp"
#include "../tools/MathToolbox.hpp"	// for NAN_double
#include <ostream>	// fuer ostringstream
#include <iomanip>	// fuer std::setfill
#include "../datatypes/Msg.hpp"
#include "../datatypes/Object.hpp"		// for ObjectList

// Note that LuxBase is not a device, but a helper class.
namespace devices
{
	
using namespace datatypes;

LuxBase::LuxBase (Manager* manager, const UINT8 deviceId, const std::string longName,
				  std::string ipAddress, UINT16 tcpPortNumber,
				  double scanFrequency, double scanStartAngle, double scanEndAngle, double offsetX,
				  double offsetY, double offsetZ, double yawAngle, double pitchAngle, double rollAngle,
				  bool beVerbose, std::string inputFileName)
	: m_manager(manager),
		m_longName(longName),
		m_deviceId(deviceId),
		m_ipAddress(ipAddress),
		m_tcpPortNumber(tcpPortNumber),
		m_inputFileName(inputFileName),
		m_scanFrequency(scanFrequency),
		m_scanStartAngle(scanStartAngle),
		m_scanEndAngle(scanEndAngle),
		m_offsetX(offsetX),
		m_offsetY(offsetY),
		m_offsetZ(offsetZ),
		m_yawAngle(yawAngle),
		m_pitchAngle(pitchAngle),
		m_rollAngle(rollAngle),
		m_inBufferLevel(0),
		m_beVerbose(beVerbose)
{
	printInfoMessage("LuxBase::LuxBase: Constructor running.", m_beVerbose);
	
	m_weWantScanData = false;		// Flag if received data is to be decoded.
	m_weWantObjectData = false;		// Flag if received data is to be decoded.
	
	m_onScanReceiveCallback = NULL;
	m_onScanReceiveCallbackObjPtr = NULL;
	m_disconnectFunctionObjPtr = NULL;
	m_tcpDisconnectFunction = NULL;
	m_fileDisconnectFunction = NULL;
	
	m_beamTiltAngle = 0.0;
	m_upsideDownActive = false;
	
	printInfoMessage("LuxBase::LuxBase: Constructor done.", m_beVerbose);
}


//
// Destructor
//
LuxBase::~LuxBase()
{
	if (isRunning() == true)
	{
		stop();
	}
}

void LuxBase::readCallbackFunctionS(void* obj, BYTE* buffer, UINT32& numOfBytes)
{
	((LuxBase*)(obj))->readCallbackFunction(buffer, numOfBytes);
}


void LuxBase::setOnScanReceiveCallbackFunction(OnScanReceiveCallbackFunction function, void* obj)
{
	m_onScanReceiveCallback = function;
	m_onScanReceiveCallbackObjPtr = obj;
}

//
// Initialisation:
// Set-up variables and call base-class to connect
//
bool LuxBase::initTcp(Tcp::DisconnectFunction function, void* obj)	// , bool beVerbose)
{
	// Initialise base variables
//	m_beVerbose = beVerbose; // true = Show extended status info (DEBUG)
	m_firmwareVersion = 0;
	m_isRunning = false;
	m_inBufferLevel = 0;
	
	
	// Select the interface (file or TCP)
	bool result = false;
	if (m_inputFileName != "")
	{
		// File
		printError("LuxBase::initTcp: Called in file mode - this is an error, aborting.");
		return false;
	}
	
	printInfoMessage("LuxBase::initTcp: Opening the tcp interface (Addr=" + m_ipAddress + ":" + toString(m_tcpPortNumber) + ").", m_beVerbose);

	// Open the interface. Here, we are using our TCP wrapper.
	result = m_tcp.open(m_ipAddress, m_tcpPortNumber, false);	// m_beVerbose);
	if (result == true)
	{
		m_tcpDisconnectFunction = function;
		m_disconnectFunctionObjPtr = obj;
		m_tcp.setDisconnectCallbackFunction(function, obj);

		printInfoMessage("LuxBase::initTcp(): TCP connection established.", m_beVerbose);
	}
	else
	{
		printError("LuxBase::initTcp(): ERROR: Failed to establish TCP connection, aborting.");
		return false;
	}
	
	// Set the data input callback for our TCP connection
	m_tcp.setReadCallbackFunction(LuxBase::readCallbackFunctionS, (void*)this);	// , this, _1, _2));


	// Get the status of the scanner, e.g. its firmware version. 
	printInfoMessage("LuxBase::initTcp(): Calling cmd_getStatus().", m_beVerbose);
	result = cmd_getStatus();

	if (result == true)
	{
		// Success, we have a valid firmware version
		if (m_beVerbose == true)
		{
			UINT16 first = m_firmwareVersion >> 12;						// Example: 0x3021 ==> 3
			UINT16 second = ((m_firmwareVersion & 0x0F00) >> 8) * 10;	// Example: 0x3021 ==> 0
			second += (m_firmwareVersion & 0x00F0) >> 4;				// Example: 0x3021 ==> 02
			UINT16 third = (m_firmwareVersion & 0x000F);				// Example: 0x3021 ==> 1
			infoMessage(m_longName + " Firmware version is " + toString(first)+ "." + toString(second) + "." + toString(third) + ".");
		}
	}
	else
	{
		printError("LuxBase::initTcp(): ERROR: Failed to read scanner status, aborting.");
		return false;
	}
	
	//
	// Read the beam tilt.
	// We need this parameter later for the calculation of cartesian scan point coordinates.
	//
	printInfoMessage("LuxBase::initTcp(): Calling readBeamTilt().", m_beVerbose);
	result = readBeamTilt();

	if (result == true)
	{
		// Success, we have a valid beam tilt
		infoMessage(m_longName + " Beam tilt angle is " + toString(m_beamTiltAngle * rad2deg, 1)+ " degrees.", m_beVerbose);
	}
	else
	{
		printError("LuxBase::initTcp(): ERROR: Failed to read scanner beam tilt angle, aborting.");
		return false;
	}
	

	//
	// Read the UpsideDown mode.
	// We need this parameter later for the calculation of cartesian scan point coordinates.
	//
	printInfoMessage("LuxBase::initTcp(): Calling readUpsideDown().", m_beVerbose);
	result = readUpsideDown();

	if (result == true)
	{
		// Success, we have a valid upsideDown flag
		if (m_upsideDownActive == true)
		{
			infoMessage(m_longName + " UpsideDown is active.", m_beVerbose);
		}
		else
		{
			infoMessage(m_longName + " UpsideDown is not active.", m_beVerbose);
		}
	}
	else
	{
		// Some devices dont have the UpsideDown flag so just ignore this error
		infoMessage(m_longName + " UpsideDown not supported by firmware.", m_beVerbose);
	}

	// Start thread for reading temperature once a minute
//	m_updateThread.run(this);
	
	return true;
}

//
// Initialisation:
// Set-up variables and call base-class to connect.
//
// Call this function when file read is required.
//
bool LuxBase::initFile(File::DisconnectFunction function, void* obj)	// , bool beVerbose)
{
	// Initialise base variables
//	m_beVerbose = beVerbose; // true = Show extended status info (DEBUG)
	m_firmwareVersion = 0;
	m_isRunning = false;
	m_inBufferLevel = 0;
	
	// Set these values here as we cannot request them from the scanner!
	m_weWantScanData = true;
	m_weWantObjectData = true;
		
	// Select the interface (file or TCP)
	bool result = false;
	if (m_inputFileName == "")
	{
		// Error - no file!
		printError("LuxBase::initFile: called without file - aborting!");
		return false;
	}

	// File
	printInfoMessage("LuxBase::init: Opening the input file (Name=" + m_inputFileName + ").", m_beVerbose);
	result = m_file.open(m_inputFileName, false);	// m_beVerbose);
	if (result == true)
	{
		printInfoMessage("LuxBase::initFile(): File opened successfully.", m_beVerbose);
	}
	else
	{
		printError("LuxBase::initFile(): ERROR: Failed to open file, aborting.");
		return false;
	}
	
	m_fileDisconnectFunction = function;
	m_disconnectFunctionObjPtr = obj;
	m_file.setDisconnectCallbackFunction(function, obj);
	m_file.setReadCallbackFunction(LuxBase::readCallbackFunctionS, (void*)this);	// , this, _1, _2));

	return true;
}

//
//
//
void LuxBase::updateThreadFunction(bool& endThread, UINT16& sleepTimeMs)
{
	// Status-Update anfordern
	bool result = cmd_getStatus();	// bool result = ...

	if (result == true)
	{
		printInfoMessage("updateThreadFunction(): Got new status. Temp= " + toString(getTemperature(), 1) + " deg.C.", m_beVerbose);
	}
	else
	{
		printWarning("updateThreadFunction(): Failed to read the status.");
	}

	// Call next time in 60s.
	sleepTimeMs = 1000 * 60;
	endThread = false; // !( /*m_updateThreadShouldStop &&*/ result);
}


//
//
//
bool LuxBase::isRunning()
{
	return m_isRunning;
}



//
// Writes the mounting position (angles and offsets) to the scanner.
//
bool LuxBase::cmd_setMountingPos(Position3D mp)
{
	UINT32 uValue;
	bool result;

	// Mounting pos x
	INT16 value = (INT16)(mp.getX() * 100.0);	// in [cm]
	uValue = (UINT32)value;	// Note the cast, including the sign transformation to unsigned!
	uValue = uValue & 0x0000FFFF;
	result = cmd_setParameter(ParaMountingX, uValue);
	if (result == false)
	{
		// We failed to set the parameter
		printError("Failed to set MountingX parameter!");
		return false;
	}

	// Mounting pos y
	value = (INT16)(mp.getY() * 100.0);	// in [cm]
	uValue = (UINT32)value;	// Note the cast, including the sign transformation to unsigned!
	uValue = uValue & 0x0000FFFF;
	result = cmd_setParameter(ParaMountingY, uValue);
	if (result == false)
	{
		// We failed to set the parameter
		printError("Failed to set MountingY parameter!");
		return false;
	}

	// Mounting pos z
	value = (INT16)(mp.getZ() * 100.0);	// in [cm]
	uValue = (UINT32)value;	// Note the cast, including the sign transformation to unsigned!
	uValue = uValue & 0x0000FFFF;
	result = cmd_setParameter(ParaMountingZ, uValue);
	if (result == false)
	{
		// We failed to set the parameter
		printError("Failed to set MountingZ parameter!");
		return false;
	}

	// Mounting pos yaw angle
	value = (INT16)(mp.getYawAngle() * rad2deg * 32.0);
	makeIntValueEven(value);
	uValue = (UINT32)value;	// Note the cast, including the sign transformation to unsigned!
	uValue = uValue & 0x0000FFFF;
	result = cmd_setParameter(ParaMountingYaw, uValue);
	if (result == false)
	{
		// We failed to set the parameter
		printError("Failed to set MountingYaw parameter!");
		return false;
	}

	// Mounting pos pitch angle
	value = (INT16)(mp.getPitchAngle() * rad2deg * 32.0);
	makeIntValueEven(value);
	uValue = (UINT32)value;	// Note the cast, including the sign transformation to unsigned!
	uValue = uValue & 0x0000FFFF;
	result = cmd_setParameter(ParaMountingPitch, uValue);
	if (result == false)
	{
		// We failed to set the parameter
		printError("Failed to set MountingPitch parameter!");
		return false;
	}

	// Mounting pos roll angle
	value = (INT16)(mp.getRollAngle() * rad2deg * 32.0);
	makeIntValueEven(value);
	uValue = (UINT32)value;	// Note the cast, including the sign transformation to unsigned!
	uValue = uValue & 0x0000FFFF;
	result = cmd_setParameter(ParaMountingRoll, uValue);
	if (result == false)
	{
		// We failed to set the parameter
		printError("Failed to set MountingRoll parameter!");
		return false;
	}

	return true;	// Success
}



//
// Helper function to create even values, used for angles.
//
void LuxBase::makeIntValueEven(INT16& value)
{
	if ((2 * (value / 2)) != value)
	{
		// Value is not even, add 1
		value += 1;
	}
}



//
// Sets scan frequency.
// valid values are 12.5, 25.0 and 50.0 [Hz].
//
bool LuxBase::cmd_setScanFrequency(double scanFreq)
{
	bool result;
	//
	// Set the scan frequency
	//
	UINT32 uValue = (UINT32)(scanFreq * 256.0);
	if ((uValue > 3100) && (uValue < 3300))
	{
		uValue = 3200;	// 12.5 Hz
	}
	else if ((uValue > 6300) && (uValue < 6500))
	{
		uValue = 6400;	// 25 Hz
	}
	else if ((uValue > 12700) && (uValue < 12900))
	{
		uValue = 12800;	// 50 Hz
	}
	else
	{
		uValue = 3200;	// DEFAULT
		printWarning("Para 'ScanFrequency' out of range, setting a default scan frequency of 12.5 Hz!");
	}
	result = cmd_setParameter(ParaScanFrequency, uValue);
	if (result == false)
	{
		// We failed to set the parameter
		printError("The SetParameter command failed!");
		return false;
	}

	return true;
}



//
// Sets scan start and end angles.
//
// Note that the two angles are set one after the other. Still, in the sensor, there
// is a condition that the StartAngle must always be greater than the EndAngle. Obviously,
// there are cases in which the mechanism implemented here will fail as the correct order
// is violated. However, for most cases, this will work.
//
bool LuxBase::cmd_setScanAngles(double startAngle, double endAngle)
{
	//
	// Start angle
	//
	// ** NOTE **
	// Some combinations of start and end angles just do not work. The result is that no scans
	// are being sent by the scanner; this is a known issue of the scanner firmware.
	// Our workaround is to use even TIC values; this will work fine.
	//
	// ** NOTE 2 **
	// The scan start angle must always be greater than the scan end angle. Therefore, the method
	// used below will not work for all configurations. To set the angles correctly, you should
	// read the current angles, select the order in which to set the parameters, and then write
	// them.
	//

	if (endAngle > startAngle)
	{
		printError("Start angle must be greater than end angle!");
		return false;
	}
	bool result;
	UINT32 uValue;
	INT16 value = (INT16)(startAngle * rad2deg * 32.0);	// Note that startAngle is in radians
	// Value should be even
	makeIntValueEven(value);
	if (value > 1600)
	{
		value = 1600;	//
		printWarning("Para 'ScanStartAngle' out of range, limiting to 1600!");
	}
	if (value < -1918)
	{
		value = -1918;	//
		printWarning("Para 'ScanStartAngle' out of range, limiting to -1919!");
	}
	uValue = (UINT32)value;	// Note the cast, including the sign transformation to unsigned!
	uValue = uValue & 0x0000FFFF;
	result = cmd_setParameter(ParaStartAngle, uValue);
	if (result == false)
	{
		// We failed to set the parameter
		printWarning("The SetParameter command failed!");
		return false;
	}

	//
	// End angle
	//
	value = (INT16)(endAngle * rad2deg * 32.0);	// Note that endAngle is in radians
	// Value should be even
	makeIntValueEven(value);
	if (value > 1598)
	{
		value = 1598;	//
		printWarning("Para 'ScanEndAngle' out of range, limiting to 1599!");
	}
	if (value < -1920)
	{
		value = -1920;	//
		printWarning("Para 'ScanEndAngle' out of range, limiting to -1920!");
	}
	uValue = (UINT32)value;	// Note the cast, including the sign transformation to unsigned!
	uValue = uValue & 0x0000FFFF;
	result = cmd_setParameter(ParaEndAngle, uValue);
	if (result == false)
	{
		// We failed to set the parameter
		printError("The SetParameter command failed!");
		return false;
	}

	return true;
}

bool LuxBase::cmd_setSyncAngleOffset(double syncAngle)
{
	bool result;
	UINT32 uValue;
	INT16 value = (INT16)(syncAngle * rad2deg * 32.0);	// Note that syncAngle is in radians
	if (value > 5759)
	{
		value = 5759;
		printWarning("Para 'SyncAngleOffset' out of range, limiting to 5759!");
	}
	if (value < -5760)
	{
		value = -5760;
		printWarning("Para 'SyncAngleOffset' out of range, limiting to -5760!");
	}
	uValue = (UINT32)value;	// Note the cast, including the sign transformation to unsigned!
	uValue = uValue & (1 << 14) - 1; // = 0x00003FFF  (SyncAngleOffset is an INT14)
	result = cmd_setParameter(ParaSyncAngleOffset, uValue);
	if (result == false)
	{
		// We failed to set the parameter
		printWarning("The SetParameter command failed!");
		return false;
	}

	return true;
}


/**
 * Request the staus message from the scanner.
 */
bool LuxBase::cmd_getStatus()
{
	if (m_tcp.isOpen() == false)
	{
		// There is no connection
		infoMessage("cmd_getStatus() called, but there is no connection - aborting!");
		return false;
	}

	// Send command
	MrsCommandId cmdId = CmdMrsGetStatus;
	printInfoMessage("Sending MRS command 'GetStatus'.", m_beVerbose);
	bool result = sendMrsCommand(cmdId);

	if (result == true)
	{
		printInfoMessage("MRS command 'GetStatus' was sent successfully, waiting for reply.", m_beVerbose);

		// The command was sent
		result = receiveMrsReply(cmdId, 2000);
		if (result == true)
		{
			printInfoMessage("LuxBase::cmd_getStatus: " + m_longName +
								" MRS Status was read successfully. Firmware version is 0x" +
								toHexString(m_firmwareVersion) + ".", m_beVerbose);
		}
		else
		{
			printError("LuxBase::cmd_getStatus: " + m_longName + " ERROR: Failed to receive status reply, aborting.");
		}
	}
	else
	{
		// Failed to send command
		printError("LuxBase::cmd_getStatus: " + m_longName + " ERROR: Failed to send command, aborting.");
	}

	return result;
}



/**
 * Read the current beam tilt angle from the scanner.
 * The result is stored in m_beamTilt.
 */
bool LuxBase::readBeamTilt()
{
	if (m_tcp.isOpen() == false)
	{
		// There is no connection
		infoMessage("readBeamTilt() called, but there is no connection - aborting!");
		return false;
	}

	// Read beam tilt
	UINT32 value;
	bool success = cmd_getParameter(ParaBeamTilt, &value);
	if (success == true)
	{
		INT16 tilt = static_cast<INT16>(value & 0xFFFF);
		
		// decompress to angle, in [rad]
		m_beamTiltAngle = static_cast<double>(tilt) / 10000.0;

		infoMessage("LuxBase::readBeamTilt: " + m_longName + " Beam tilt read. Angle is " + toString(rad2deg * m_beamTiltAngle, 1) + " degrees.");
	}
	else
	{
		// Failed to read parameters
		printError("LuxBase::readBeamTilt: " + m_longName + " ERROR: Failed to read beam tilt angle, aborting.");
	}

	return success;
}

/**
 * Read the current UpsideDown flag from the scanner.
 * The result is stored in m_upsideDownActive.
 */
bool LuxBase::readUpsideDown()
{
	if (m_tcp.isOpen() == false)
	{
		// There is no connection
		infoMessage("readUpsideDown() called, but there is no connection - aborting!");
		return false;
	}

	// Read beam tilt
	UINT32 value;
	bool success = cmd_getParameter(ParaUpsideDownMode, &value);
	if (success == true)
	{
		if (value != 0)
		{
			m_upsideDownActive = true;
			infoMessage("LuxBase::readUpsideDown: " + m_longName + " UpsideDown is active.");
		}
		else
		{
			m_upsideDownActive = false;
			infoMessage("LuxBase::readUpsideDown: " + m_longName + " UpsideDown is not active.");
		}
	}
	else
	{
		// Failed to read parameter
		infoMessage("LuxBase::readUpsideDown: " + m_longName + " UpsideDown not supported by firmware.");
		// cannot read parameter so there is no upsideDown support
		m_upsideDownActive = false;
	}

	return success;
}


//
// Stop measuring.
//
bool LuxBase::cmd_stopMeasure()
{
	if (m_tcp.isOpen() == false)
	{
		// There is no connection
		printError("LuxBase::cmd_stopMeasure: " + m_longName + " ERROR: stopMeasure() called, but there is no connection!");
		return false;
	}

	// StopMeasure command
	MrsCommandId cmdId = CmdMrsStopMeasure;
	printInfoMessage("Sending MRS command 'StopMeasure'.", m_beVerbose);
	bool result = sendMrsCommand(cmdId);

	if (result == true)
	{
		printInfoMessage("MRS command 'StopMeasure' was sent successfully, waiting for reply.", m_beVerbose);

		// The command was sent
		result = receiveMrsReply(cmdId, 2000);
		if (result == true)
		{
			printInfoMessage("LuxBase::cmd_stopMeasure: " + m_longName + " StopMeasure was acknowledged by the scanner.", m_beVerbose);
		}
		else
		{
			printError("LuxBase::cmd_stopMeasure: " + m_longName + " ERROR: Failed to receive StopMeasure reply.");
		}
	}
	else
	{
		// Failed to send command
		printError("LuxBase::cmd_stopMeasure: " + m_longName + " ERROR: Failed to send command, aborting.");
		return result;
	}

	return result;
}

//
// Set NTP time.
//
bool LuxBase::cmd_setNtpTimestamp(UINT32 seconds, UINT32 fractionalSec)
{
	if (m_tcp.isOpen() == false)
	{
		// There is no connection
		printError("LuxBase::cmd_setNtpTimestamp: " + m_longName + " ERROR: stopMeasure() called, but there is no connection!");
		return false;
	}

	// Part 1:
	// setNTPTimestamp command
	MrsCommandId cmdId = CmdMrsSetNTPTimestampSec;
	printInfoMessage("Sending MRS command 'SetNTPTimestampSec'.", m_beVerbose);
	bool result = sendMrsCommand(cmdId, 0, seconds);

	std::stringstream secString;
	secString << "seconds: " << seconds << "; fractional: " << fractionalSec;
	printInfoMessage("LuxBase::cmd_setNtpTimestamp: " +  secString.str(), true);

	if (result == true)
	{
		printInfoMessage("LuxBase::cmd_setNtpTimestamp: MRS command 'SetNTPTimestampSec' was sent successfully, waiting for reply.", m_beVerbose);

		// The command was sent
		result = receiveMrsReply(cmdId, 2000);
		if (result == true)
		{
			printInfoMessage("LuxBase::cmd_setNtpTimestamp: " + m_longName + " Timestamp setting was acknowledged by the scanner.", m_beVerbose);
		}
		else
		{
			printError("LuxBase::cmd_setNtpTimestamp: " + m_longName + " ERROR: Failed to receive 'SetNTPTimestampSec' reply, aborting.");
			return false;
		}
	}
	else
	{
		// Failed to send command
		printError("LuxBase::cmd_setNtpTimestampSec: " + m_longName + " ERROR: Failed to send command, aborting.");
		return false;
	}
	
	// Now send the fractional seconds
	cmdId = CmdMrsSetNTPTimestampFracSec;
	printInfoMessage("Sending MRS command 'SetNTPTimestampFrac'.", m_beVerbose);
	result = sendMrsCommand(cmdId, 0, fractionalSec);
	
	if (result == true)
	{
		printInfoMessage("LuxBase::cmd_setNtpTimestamp: MRS command 'SetNTPTimestampFrac' was sent successfully, waiting for reply.", m_beVerbose);

		// The command was sent
		result = receiveMrsReply(cmdId, 2000);
		if (result == true)
		{
			printInfoMessage("LuxBase::cmd_setNtpTimestampFrac: " + m_longName + " Timestamp setting was acknowledged by the scanner.", m_beVerbose);
		}
		else
		{
			printError("LuxBase::cmd_setNtpTimestampFrac: " + m_longName + " ERROR: Failed to receive 'SetNTPTimestampFrac' reply, aborting.");
			return false;
		}
	}
	else
	{
		// Failed to send command
		printError("LuxBase::cmd_setNtpTimestampFrac: " + m_longName + " ERROR: Failed to send command, aborting.");
		return false;
	}

	return result;
}



//
// Set a parameter.
//
bool LuxBase::cmd_setParameter(MrsParameterId parameter, UINT32 value)
{
	if (m_tcp.isOpen() == false)
	{
		// There is no connection
		printError("LuxBase::cmd_setParameter: " + m_longName + " ERROR: setParameter() called, but there is no connection, aborting!");
		return false;
	}

	// SetParameter command
	MrsCommandId cmdId = CmdMrsSetParameter;
	
	printInfoMessage("LuxBase::cmd_setParameter: Sending MRS command 'SetParameter' with para=0x" + toHexString((UINT16)parameter) +
						" and value=0x" + toHexString(value) + ".", m_beVerbose);
	
	bool result = sendMrsCommand(cmdId, parameter, value);

	if (result == true)
	{
		printInfoMessage("LuxBase::cmd_setParameter: MRS command 'SetParameter' was sent successfully, waiting for reply.", m_beVerbose);

		// The command was sent
		result = receiveMrsReply(cmdId, 2000);
		if (result == true)
		{
			printInfoMessage("LuxBase::cmd_setParameter: " + m_longName + " SetParameter was acknowledged by the scanner.", m_beVerbose);
		}
		else
		{
			printError("LuxBase::cmd_setParameter: " + m_longName + " ERROR: Failed to receive SetParameter reply.");
		}
	}
	else
	{
		// Failed to send command
		printError("LuxBase::cmd_setParameter: " + m_longName + " ERROR: Failed to send command, aborting.");
	}

	return result;
}

bool LuxBase::cmd_getParameter(MrsParameterId parameter, UINT32* value)
{
	if (m_tcp.isOpen() == false)
	{
		// There is no connection
		printError("LuxBase::cmd_getParameter: " + m_longName + " ERROR: setParameter() called, but there is no connection, aborting!");
		return false;
	}

	// SetParameter command
	MrsCommandId cmdId = CmdMrsGetParameter;

	printInfoMessage("LuxBase::cmd_getParameter: Sending MRS command 'SetParameter' with para=0x" + toHexString((UINT16)parameter) +
	                 ".", m_beVerbose);

	bool result = sendMrsCommand(cmdId, parameter);

	if (result == true)
	{
		printInfoMessage("LuxBase::cmd_getParameter: MRS command 'SetParameter' was sent successfully, waiting for reply.", m_beVerbose);

		// The command was sent
		result = receiveMrsReply(cmdId, 2000, value);
		if (result == true)
		{
			printInfoMessage("LuxBase::cmd_getParameter: " + m_longName + " SetParameter was acknowledged by the scanner.", m_beVerbose);
		}
		else
		{
			printError("LuxBase::cmd_getParameter: " + m_longName + " ERROR: Failed to receive GetParameter reply.");
		}
	}
	else
	{
		// Failed to send command
		printError("LuxBase::cmd_getParameter: " + m_longName + " ERROR: Failed to send command, aborting.");
	}

	return result;
}

//
// Enable or disable the data output according to our needs.
//
// Note that the CAN output is disabled here!
// Note that false = enabled, true = disabled!!!!
//
// Bit 0 = Scan data (Ethernet)
// Bit 1 = reserved
// Bit 2 = Object data (Ethernet)
// Bit 3 = Vehicle data (Ethernet)
// Bit 4 = Errors/warnings (Ethernet)
// Bit 5 = Errors/warnings (CAN)
// Bit 6 = Object data (CAN)
//
bool LuxBase::cmd_setDataOutputFlags()
{
	UINT16 flags = 0xFFFF;
	if (m_weWantScanData == true)
	{
		flags &= 0xFFFE;		// Eth scan data
	}
	if (m_weWantObjectData == true)
	{
		flags &= 0xFFFB;		// Eth object data
	}
	flags &= 0xFFEF;			// Eth errors and warnings
	
	// Set the parameter
	bool result = cmd_setParameter(ParaDataOutputFlag, flags);
	
	if (result == true)
	{
		printInfoMessage("LuxBase::cmd_setDataOutputFlags: Output flags were set successfully to " +
						 toHexString(flags) + ".", m_beVerbose);
	}
	else
	{
		// Failed 
		printError("LuxBase::cmd_setDataOutputFlags:" + m_longName + " ERROR: Failed to set output flags, aborting.");
		return false;
	}

	return result;
}

bool LuxBase::cmd_saveConfiguration()
{
	printInfoMessage("LuxBase::cmd_saveConfiguration called", m_beVerbose);
	return sendMrsCommand(CmdMrsSaveConfig);
}

//
// Start measuring.
//
// Set both bool flags according to the device needs.
//
// Note 1: Not every MRS/LUX sends object data.
// Note 2: These flags may not prevent the data to be transferred, but it will not be decoded and
//         posted into the system.
//
bool LuxBase::cmd_startMeasure(bool weWantScanData, bool weWantObjectData)
{
	printInfoMessage("LuxBase::cmd_startMeasure: Called.", m_beVerbose);
	
	if (m_tcp.isOpen() == false)
	{
		// There is no connection
		printError("LuxBase::cmd_startMeasure:" + m_longName + " ERROR: startMeasure() called, but there is no connection, aborting!");
		return false;
	}

	m_weWantScanData = weWantScanData;
	m_weWantObjectData = weWantObjectData;
	if ((m_weWantScanData == false) && (m_weWantObjectData == false))
	{
		// We want no data?
		printWarning("LuxBase::cmd_startMeasure:" + m_longName + " Warning: StartMeasure called, but neither scans nor objects are requested!");
	}

	// Enable scan and/or object output
	printInfoMessage("LuxBase::cmd_startMeasure: Enabling data output.", m_beVerbose);
	bool result = cmd_setDataOutputFlags();
	if (result == true)
	{
		printInfoMessage("LuxBase::cmd_startMeasure: Data output was successfully enabled.", m_beVerbose);
	}
	else
	{
		// Failed
		printError("LuxBase::cmd_startMeasure:" + m_longName + " ERROR: Failed to set data output, aborting!");
		return false;
	}

	// Command
	MrsCommandId cmdId = CmdMrsStartMeasure;
	printInfoMessage("LuxBase::cmd_startMeasure: Sending MRS command 'StartMeasure'.", m_beVerbose);
	result = sendMrsCommand(cmdId);

	if (result == true)
	{
		printInfoMessage("LuxBase::cmd_startMeasure: MRS command 'StartMeasure' was sent successfully, waiting for reply.", m_beVerbose);

		// The command was sent
		result = receiveMrsReply(cmdId, 2000);
		if (result == true)
		{
			printInfoMessage("LuxBase::cmd_startMeasure: " + m_longName + " StartMeasure was acknowledged by the scanner.", m_beVerbose);
		}
		else
		{
			printError("LuxBase::cmd_startMeasure:" + m_longName + " ERROR: Failed to receive StartMeasure reply.");
		}
	}
	else
	{
		// Failed to send command
		printError("LuxBase::cmd_startMeasure:" + m_longName + " ERROR: Failed to send command, aborting.");
		return result;
	}

	return result;
}

/*
inline void memreadLE (const BYTE*& buf, T& value)
{
	BOOST_STATIC_ASSERT(boost::is_fundamental<T>::value);
	BOOST_STATIC_ASSERT(sizeof(T) == 1 || sizeof(T) == 2 || sizeof(T) == 4 || sizeof(T) == 8);
	value = detail::memreadcLE<T>(buf);
	buf += sizeof(T);
}
*/

//
// Lese einen 16-Bit-Wert, Little Endian.
//
void LuxBase::memreadLE(BYTE*& buffer, UINT16& value)
{
	value = ((UINT16)(buffer[0])) + ((UINT16)(buffer[1]) << 8);
	buffer += 2;
}

//
// Lese einen 32-Bit-Wert, Little Endian.
//
void LuxBase::memreadLE(BYTE*& buffer, UINT32& value)
{
	value = ((UINT32)(buffer[0])) + ((UINT32)(buffer[1]) << 8)
			+ ((UINT32)(buffer[2]) << 16) + ((UINT32)(buffer[3]) << 24);
	buffer += 4;
}


/**
 * Decodes a "GetStatus" message in the input buffer.
 */
bool LuxBase::decodeGetStatus()
{
	UINT32 pos = 24;	// 24 is the length of the data header
	UINT16 cmd = (UINT16)readUValueLE(&(m_inputBuffer[pos]), 2);
	pos += 2;
//	UINT16 firmwareVersion = (UINT16)readUValueLE(&(m_inputBuffer[pos]), 2);

	BYTE* bufferPos = &(m_inputBuffer[pos]);

	ScopedLock lock(&m_updateMutex);
	UINT16 dummy;

	memreadLE(bufferPos, m_firmwareVersion);
	memreadLE(bufferPos, m_FPGAVersion);
	memreadLE(bufferPos, m_scannerStatus);
	memreadLE(bufferPos, dummy);				// Reserved. This is the SVN repository index.
	memreadLE(bufferPos, dummy);				// Reserved. This is the scanner type.
	memreadLE(bufferPos, m_temperature);
	memreadLE(bufferPos, m_serialNumber[0]);
	memreadLE(bufferPos, m_serialNumber[1]);
	memreadLE(bufferPos, m_serialNumber[2]);
	memreadLE(bufferPos, m_FPGATimestamp[0]);
	memreadLE(bufferPos, m_FPGATimestamp[1]);
	memreadLE(bufferPos, m_FPGATimestamp[2]);
	memreadLE(bufferPos, m_firmwareTimestamp[0]);
	memreadLE(bufferPos, m_firmwareTimestamp[1]);
	memreadLE(bufferPos, m_firmwareTimestamp[2]);

	// Debug
	printInfoMessage("LuxBase::decodeGetStatus: " + m_longName + " Received a GetStatus with CMD " +
					 toHexString(cmd) + " and FirmwareVersion " + toHexString(m_firmwareVersion) + ".", m_beVerbose);

	// Here we could decode the rest of the message.

	return true;
}

bool LuxBase::decodeGetParameter(UINT32* value)
{
	UINT32 pos = 24;	// 24 is the length of the data header
	UINT16 cmd = (UINT16)readUValueLE(&(m_inputBuffer[pos]), 2);
	pos += 2;
	BYTE* bufferPos = &(m_inputBuffer[pos]);
    UINT16 index = 0;

    memreadLE(bufferPos, index);
	memreadLE(bufferPos, *value);
	// Debug
	printInfoMessage("LuxBase::decodeGetParameter: " + m_longName + " Received a GetParameter with CMD " +
	                 toHexString(cmd) + " and parameter index 0x" + toHexString(index) + " and value " + toString(*value) + ".", m_beVerbose);

	return true;
}

/// Converts degree Celsius to raw data
/**
 * Conversion from degree Celsius to raw temperature data. The raw data is
 * a 10-bit value as is returned from the internal temperature sensor of
 * the LUX.
 *
 * \param celsius The temperature value in degree Celsius
 * \return The temperature converted to raw data.
 * \sa int2Celsius()
 */
UINT16 celsius2Int (double celsius)
{
	// Step 1, temperature to voltage
	// (Taken from the data sheet of the temperature sensor LM 20.)
	double voltage = 1.8663f - 0.01169f * celsius;

	// Step 2, voltage to integer
	// The MRS has a 10 bit ADC (Analog-Digital Converter).
	// The ADC yields 0 at 0V and 1023 at 3.3V with a linear characteristic.
	UINT16 value = (UINT16)(((1023 * voltage) / 3.3f) + 0.5);
	return value;
}

/// Converts raw data to degree Celsius
/**
 * This is the inverse function to celsius2Int(), see there.
 *
 * \param intValue The raw temperature value. Although raw data is integer,
 *				 here a double argument is passed to support the exact
 *				 conversion of \e averaged temperature raw data.
 * \return The temperature converted to degree Celsius.
 * \sa celsius2Int()
 */
double int2Celsius (double intValue)
{
	// Step 1, integer value to voltage
	// The MRS has a 10 bit ADC (Analog/Digital Converter).
	// The ADC yields 0 at 0V and 1023 at 3.3V with a linear characteristic.
	double voltage = ((double)intValue * 3.3f) / 1023.0;

	// Step 2, voltage to temperature
	// (Taken from the data sheet of the temperature sensor LM 20.)
	return (1.8663f - voltage) / 0.01169f;
}

double LuxBase::getTemperature()
{
	double temperature = 0;

	ScopedLock lock(&m_updateMutex);

	static UINT16 maxRawTemperature = celsius2Int (-273.16f);	// = 1568 = theoretical raw value of 0 Kelvin
	if (m_temperature <= maxRawTemperature)
		temperature = int2Celsius (m_temperature);
	else
	{
		// In this case, the raw temperature is most probably equal to 0xFFFF.
		// This happens when the LUX firmware is currently unable to read the
		// temperature from the FPGA, e.g. in MEASURE mode.
		temperature = ::NaN_double;	// Not-a-Number
	}

	return temperature;
}

//
//
//
std::string LuxBase::getSerialNumber()
{
	ScopedLock lock(&m_updateMutex);

	if (m_serialNumber[0] == 0xFFFF)
	{
		return "<not set>";
	}
	else
	{
		std::ostringstream oss;

		// Low byte of m_serialNumber[2] indicates the style of the serial number:
		// 0 = old style with MAC address
		// 1 = new style with consecutive counter
		bool isNewStyle = ((m_serialNumber[2] & 0x00FF) == 1);

		if (isNewStyle)
		{
			// Output format...
			// Example: "0829 0123" = LUX #123 in week 29 of year '08
			oss << std::setfill ('0')
				<< std::hex << std::setw(4) << m_serialNumber[0] << ' '
				<< std::dec << std::setw(4) << m_serialNumber[1];
		}
		else
		{
			// Output format...
			// Example: "0823-0A7E3F" = LUX with MAC address "**:**:**:0A:7E:3F"
			// produced in week 23 of year '08
			oss << std::setfill ('0') << std::hex
				<< std::setw(4) <<  m_serialNumber[0] << '-'
				<< std::setw(4) <<  m_serialNumber[1]
				<< std::setw(2) << (m_serialNumber[2] >> 8);
		}
		return oss.str();
	}
}

/**
 * \par Examples:
 * - 0x1230 --> "1.2.30"
 * - 0x123B --> "1.2.3B"
 * - 0xFFFF --> "n/a" (special value for "not available")
 */
std::string LuxBase::int2Version (UINT16 val)
{
	if (val == 0xFFFF)
	{
		return "n/a";	// not available
	}
	else
	{
		std::ostringstream oss;
		oss << std::hex << std::uppercase
			<< ((val & 0xF000) >> 12) << '.'
			<< std::setfill('0') << std::setw(2) << ((val & 0x0FF0) >>  4) << '.'
			<< (val & 0x000F);
		return oss.str();
	}
}

std::string LuxBase::version2string (UINT16 version, const UINT16 timestamp[3])
{
	if (isValidVersion (version))
	{
		UINT16 year   = timestamp[0];				// Debugging: Display the data as *hex*
		UINT16 month  = timestamp[1] >> 8;			// to see the "correct" values, e.g.
		UINT16 day    = timestamp[1] & 0x00FF;		// 8200 = 0x2008 = year 2008
		UINT16 hour   = timestamp[2] >> 8;
		UINT16 minute = timestamp[2] & 0x00FF;

		std::ostringstream oss;
		oss << int2Version (version) << ' ' << std::setfill ('0') << std::hex
			<< std::setw (4) << year   << '-'
			<< std::setw (2) << month  << '-'
			<< std::setw (2) << day    << ' '
			<< std::setw (2) << hour   << ':'
			<< std::setw (2) << minute;

		return oss.str();
	}
	else
	{
		return "<unknown>";
	}
}



/**
 * Little-Endian-Read (unsigned)
 *
 * Reads a value of 1, 2 or 4 bytes from the buffer. The value encoding in the buffer
 * is little endian, that means the lowest-value byte is first in the buffer.
 * Example: Buffer = 0x12 0x34 ==> Value = 0x3412
 */
UINT32 LuxBase::readUValueLE(UINT8* buffer, UINT8 bytes)
{
	UINT32 value;

	switch (bytes)
	{
	case 1:
		value = buffer[0];
		break;
	case 2:
		value = buffer[0];
		value += ((UINT32)buffer[1]) << 8;
		break;
	case 4:
		value = buffer[0];
		value += ((UINT32)buffer[1]) << 8;
		value += ((UINT32)buffer[2]) << 16;
		value += ((UINT32)buffer[3]) << 24;
		break;
	default:
		printError("LuxBase::readUValueLE: " + m_longName + " ERROR: Invalid number of bytes to read, can only handle 1,2 or 4.");
		value = 0xFFFFFFFF;
	}

	return value;
}

/**
 * Little-Endian-Read (unsigned)
 *
 * Reads a value of 8 bytes from the buffer. The value encoding in the buffer
 * is little endian, that means the lowest-value byte is first in the buffer.
 * Example: Buffer = 0x12 0x34 ==> Value = 0x3412
 */
UINT64 LuxBase::readUINT64ValueLE(UINT8* buffer)
{
	UINT64 value;

	value = buffer[0];
	value += ((UINT64)buffer[1]) << 8;
	value += ((UINT64)buffer[2]) << 16;
	value += ((UINT64)buffer[3]) << 24;
	value += ((UINT64)buffer[4]) << 32;
	value += ((UINT64)buffer[5]) << 40;
	value += ((UINT64)buffer[6]) << 48;
	value += ((UINT64)buffer[7]) << 56;

	return value;
}


//
// Little-Endian-Read (signed)
//
INT32 LuxBase::readValueLE(UINT8* buffer, UINT8 bytes)
{
//	UINT32 uValue;
	INT32 value;

	switch (bytes)
	{
	case 1:
		value = (INT32)(buffer[0]);
		if (value > 0x7F)
		{
//				value = value - 0x100;
			value |= 0xFFFFFF00;
		}
		break;
	case 2:
		value = (INT32)(buffer[0]);
		value += ((INT32)buffer[1]) << 8;
		if (value > 0x7FFF)
		{
			value = value - 0x10000;
			value |= 0xFFFF0000;
		}
		break;
	case 4:
		value = buffer[0];
		value += ((INT32)buffer[1]) << 8;
		value += ((INT32)buffer[2]) << 16;
		value += ((INT32)buffer[3]) << 24;
		break;
	default:
		printError("LuxBase::readValueLE: " + m_longName + " ERROR: Invalid number of bytes to read, can only handle 1,2 or 4.");
		value = 0xFFFFFFFF;
	}

	return value;
}



/**
 * Big-Endian-Read (unsigned)
 *
 * Reads a value of 1, 2 or 4 bytes from the buffer. The value encoding in the buffer
 * is big endian, that means the highest-value byte is first in the buffer.
 * Example: Buffer = 0x12 0x34 ==> Value = 0x1234
*/
UINT32 LuxBase::readUValueBE(UINT8* buffer, UINT8 bytes)
{
	UINT32 value;

	switch (bytes)
	{
	case 1:
		value = buffer[0];
		break;
	case 2:
		value = buffer[1];
		value += ((UINT32)buffer[0]) << 8;
		break;
	case 4:
		value = buffer[3];
		value += ((UINT32)buffer[2]) << 8;
		value += ((UINT32)buffer[1]) << 16;
		value += ((UINT32)buffer[0]) << 24;
		break;
	default:
		printError("LuxBase::readUValueBE: " + m_longName + " ERROR: Invalid number of bytes to read, can only handle 1,2 or 4.");
		value = 0xFFFFFFFF;
	}

	return value;
}



//
// Decodes the data in the input buffer:
// - Syncs to magic word.
// - Decodes data type and length
// - If dataset is complete,
//     - calls decoding of scan and object data directly
//     - transfers command replys to reply buffer
//
// Returns the datatype of the processed message. Note that when this
// function returns, the message is already decoded and removed from the buffer!
//
// Note: Access to input buffer (and reply buffer) must be mutex'ed.
//
UINT16 LuxBase::decodeAnswerInInputBuffer()
{
	bool beVerboseHere = false;	// = m_beVerbose;
	printInfoMessage("LuxBase::decodeAnswerInInputBuffer: Called.", beVerboseHere);

	const UINT32 headerLen = 24;	// Length of data header, in [bytes]
	UINT16 datatype = 0;

	// Enough data for a header?
	if (m_inBufferLevel <= headerLen)
	{
		// Not enough data
		printInfoMessage("LuxBase::decodeAnswerInInputBuffer(): Not enough data in input buffer, just " +
						 toString(m_inBufferLevel) + " bytes available, done.", beVerboseHere);
		return 0;
	}

	// Sync to magic word. This should not happen too often, as data should come in structured - unless
	// we start to miss complete IP packages...
	UINT32 magicWord = 0;
	UINT32 end = m_inBufferLevel - 4 + 1;
	UINT32 i;
	for (i = 0; i < end; i++)
	{
		magicWord = readUValueBE(&(m_inputBuffer[i]), 4);
		if (magicWord == 0xAFFEC0C2)
		{
			printInfoMessage("LuxBase::decodeAnswerInInputBuffer(): Magic word found at pos " + toString(i) + ".", beVerboseHere);
			break;
		}
	}

	if ((i > 0) && (magicWord != 0xAFFEC0C2))
	{
		printInfoMessage("LuxBase::decodeAnswerInInputBuffer(): Out of sync, and no magic word found - clearing buffer.", beVerboseHere);
		m_inBufferLevel = 0;

		// That was it
		return 0;
	}
	else if ((i > 0) && (magicWord == 0xAFFEC0C2))
	{
		// Adjust buffer by i bytes
		printInfoMessage("LuxBase::decodeAnswerInInputBuffer(): Out of sync, adjusting the buffer by " + toString(i) + " bytes.", beVerboseHere);

		removeDataFromInputBuffer(i);
	}

	// Magic word found?
	if (magicWord != 0xAFFEC0C2)
	{
		// Not found.
		printInfoMessage("LuxBase::decodeAnswerInInputBuffer(): Magic word not found, aborting!", beVerboseHere);
		return 0;
	}
	else
	{
		printInfoMessage("LuxBase::decodeAnswerInInputBuffer(): Magic word found, now decoding header.", beVerboseHere);
	}

	// Complete message header found?
	if (m_inBufferLevel >= headerLen)
	{
		// Yes, we have a data header. We now calculate the size of the complete message.
		UINT32 payloadLen = readUValueBE(&(m_inputBuffer[8]), 4);

		printInfoMessage("LuxBase::decodeAnswerInInputBuffer(): Message payload length is " + toString(payloadLen) + " bytes.", beVerboseHere);

		// Is the message complete?
		if (m_inBufferLevel >= (payloadLen + headerLen))
		{
			// The command is completely in the buffer, so now get its datatype
			datatype = readUValueBE(&(m_inputBuffer[14]), 2);

			// What is it?
			switch (datatype)
			{
			case 0x2202:		// Scan data
				if (m_weWantScanData == true)
				{
					decodeScan();
				}
				removeDataFromInputBuffer(headerLen + payloadLen);
				break;
			case 0x2221:		// Object data
				if (m_weWantObjectData == true)
				{
					decodeObjects();
				}
				removeDataFromInputBuffer(headerLen + payloadLen);
				break;
			case 0x2020:		// Command reply
				// It is a reply, transfer to cmd reply buffer
				moveDataFromInputToCmdBuffer(headerLen + payloadLen);
				break;
			case 0x2030:		// Error message
				decodeErrorMessage();
				removeDataFromInputBuffer(headerLen + payloadLen);
				break;
			case 0x2805:		// VehicleStateBasic
				removeDataFromInputBuffer(headerLen + payloadLen);
				break;
			case 0x7100:		// SensorInfo
				decodeSensorInfo();
				removeDataFromInputBuffer(headerLen + payloadLen);
				break;
			default:
				// Unknown!
				printWarning("LuxBase::decodeAnswerInInputBuffer(): Unknown datatype 0x" + toHexString(datatype) +
								"(Length=" + ::toString(headerLen + payloadLen) +  " bytes) in buffer, removing this dataset.");
				removeDataFromInputBuffer(headerLen + payloadLen);
			}
		}
	}
	
	if (beVerboseHere == true)
	{
		if (datatype > 0)
		{
			infoMessage("LuxBase::decodeAnswerInInputBuffer(): Header decoded successfully, datatype 0x" + toHexString(datatype) + ".");
		}
		else
		{
			infoMessage("LuxBase::decodeAnswerInInputBuffer(): Header decoded successfully, but message is incomplete.");
			dumpHeader();
		}
	}
	return datatype;
}



//
// Decodes an error message that is completely present as the first message in the input buffer.
//
void LuxBase::decodeErrorMessage()
{
//	printInfoMessage("decodeErrorMessage(): There is an error/warning message.", m_beVerbose);

	UINT8* errorBuffer = &(m_inputBuffer[24]);	// Skip the data header

	m_errorRegister1 = (UINT16)readUValueLE(&(errorBuffer[0]), 2);
	m_errorRegister2 = (UINT16)readUValueLE(&(errorBuffer[2]), 2);
	m_warnRegister1 = (UINT16)readUValueLE(&(errorBuffer[4]), 2);
	m_warnRegister2 = (UINT16)readUValueLE(&(errorBuffer[6]), 2);

	// Was there an error?
	if ((m_errorRegister1 != 0) || (m_errorRegister2 != 0))
	{
		// There is an error. Generate a readable error message.
		std::string text = "LuxBase::decodeErrorMessage: LD-MRS reports an error: errReg1=0x" + toHexString(m_errorRegister1) +
					  ", errReg2=0x" + toHexString(m_errorRegister2) + ". Meaning: ";
		
		// First, error register 1
		if ((m_errorRegister1 & 0x0004) != 0)
		{
			text += "<Scan buffer transmitted incompletely> ";
		}
		if ((m_errorRegister1 & 0x0008) != 0)
		{
			text += "<Scan buffer overflow> ";
		}
		if ((m_errorRegister1 & 0x0300) == 0x0300)
		{
			text += "<APD temperature sensor defective> ";
		}
		else
		{
			if ((m_errorRegister1 & 0x0100) != 0)
			{
				text += "<APD undertemperature> ";
			}
			if ((m_errorRegister1 & 0x0200) != 0)
			{
				text += "<APD overtemperature> ";
			}
		}
	
		// Then, error register 2
		if ((m_errorRegister2 & 0x0001) != 0)
		{
			text += "<No scan data received> ";
		}
		if ((m_errorRegister2 & 0x0010) != 0)
		{
			text += "<Incorrect configuration data> ";
		}
		if ((m_errorRegister2 & 0x0020) != 0)
		{
			text += "<Configuration contains incorrect parameters> ";
		}
		if ((m_errorRegister2 & 0x0040) != 0)
		{
			text += "<Data processing timeout> ";
		}
		if ((m_errorRegister2 & Err_FlexResParameter) != 0)
		{
			text += "<Incorrect flex. res. configurarion> ";
		}


		// Finally, all other internal errors
		if (((m_errorRegister1 & 0x3C13) != 0) ||
			((m_errorRegister2 & 0x008E) != 0))
		{
			text += "<Some other error> ";
		}
		
		
		printWarning(text);
	}
	else
	{
		// There must have been a warning.
		std::string text = "LuxBase::decodeErrorMessage: LD-MRS reports a warning: warnReg1=0x" + toHexString(m_warnRegister1) +
						  ", warnReg2=0x" + toHexString(m_warnRegister2) + ". Meaning: ";

		// First, warn register 1

		if ((m_warnRegister1 & 0x0001) != 0)
		{
			text += "<Internal communication error> ";
		}
		if ((m_warnRegister1 & 0x0008) != 0)
		{
			text += "<Warning of insufficient temperature> ";
		}
		if ((m_warnRegister1 & 0x0010) != 0)
		{
			text += "<Warning of exceeding temperature> ";
		}
		if ((m_warnRegister1 & 0x0080) != 0)
		{
			text += "<Check syncronisation- and scan frequency> ";
		}

		// Then, warn register 2
		if ((m_warnRegister2 & 0x0001) != 0)
		{
			text += "<CAN interface blocked> ";
		}
		if ((m_warnRegister2 & 0x0002) != 0)
		{
			text += "<Ethernet interface blocked> ";
		}
		if ((m_warnRegister2 & 0x0004) != 0)
		{
			text += "<Incorrect CAN message received> ";
		}
		if ((m_warnRegister2 & 0x0010) != 0)
		{
			text += "<Check ethernet data> ";
		}
		if ((m_warnRegister2 & 0x0020) != 0)
		{
			text += "<Incorrect or forbidden command received> ";
		}
		if ((m_warnRegister2 & 0x0040) != 0)
		{
			text += "<Memory access failure> ";
		}
		
		if ((m_warnRegister2 & 0x0008) != 0)
		{
			text += "<Some other warning> ";
		}
		
		printInfoMessage(text, true);
	}

	// Send the data also to the manager for use in the application(s)
 	std::ostringstream registers;
 	registers << "FPGA Error: 0x" << std::hex << m_errorRegister1 << "; ";
 	registers << "DSP Error: 0x" << std::hex << m_errorRegister2 << "; ";
 	registers << "FPGA Warn: 0x" << std::hex << m_warnRegister1 << "; ";
 	registers << "DSP Warn: 0x" << std::hex << m_warnRegister2;
	Msg* msg = new Msg(m_deviceId, registers.str());
	m_manager->setDeviceData(msg);
}



//
// Decodes a scan that is completely present as the first message in the input buffer.
///
void LuxBase::decodeScan()
{
//	printInfoMessage("LuxBase::decodeScan(): We have received a scan that is now being decoded.", m_beVerbose);

	// Sollen wir jemanden informieren?
	if (m_onScanReceiveCallback != NULL)
	{
		// Ja, machen.
		m_onScanReceiveCallback(m_onScanReceiveCallbackObjPtr);
	}

	// Scan decodieren
	Scan* scan = new Scan;
	UINT8* scanBuffer = &(m_inputBuffer[24]);	// Skip the data header

	// Decode the scan
	UINT16 scanNumber = (UINT16)readUValueLE(&(scanBuffer[0]), 2);
	scan->setScanNumber(scanNumber);

//	UINT16 scannerStatus = (UINT16)readUValueLE(&(scanBuffer[2]), 2);
//	UINT16 syncPhaseOffset = (UINT16)readUValueLE(&(scanBuffer[4]), 2);

	UINT64 timestampStart = (UINT64)readUINT64ValueLE(&(scanBuffer[6]));
//	NTPTime startTime = NTPTime (timestamp);	// read NTPTime using Little Endian
	UINT64 timestampEnd = (UINT64)readUINT64ValueLE(&(scanBuffer[14]));
//	NTPTime endTime   = NTPTime (timestamp);

	INT16 startAngleTicks =	(INT16)readValueLE(&(scanBuffer[24]), 2);
	double startAngle = convertTicktsToAngle(startAngleTicks);				// startAngle is in [rad]
	INT16 endAngleTicks =	(INT16)readValueLE(&(scanBuffer[26]), 2);
	double endAngle = convertTicktsToAngle(endAngleTicks);					// endAngle is in [rad]
	UINT16 scanPoints =	(UINT16)readUValueLE(&(scanBuffer[28]), 2);

	// Scanner mounting position
	INT16 mountingPosYawTicks 	= (INT16)readValueLE(&(scanBuffer[30]), 2);
	INT16 mountingPosPitchTicks = (INT16)readValueLE(&(scanBuffer[32]), 2);
	INT16 mountingPosRollTicks 	= (INT16)readValueLE(&(scanBuffer[34]), 2);
	INT16 mountingPosX		 	= (INT16)readValueLE(&(scanBuffer[36]), 2);
	INT16 mountingPosY 			= (INT16)readValueLE(&(scanBuffer[38]), 2);
	INT16 mountingPosZ		 	= (INT16)readValueLE(&(scanBuffer[40]), 2);

	// Processing flags
	// Meaning:
	// Bit 0:  ground detection performed: 0 = false, 1 = true
	// Bit 1:  dirt detection performed: 0 = false, 1 = true
	// Bit 2:  rain detection performed: 0 = false, 1 = true
	// Bit 5:  transparency detection performed: 0 = false, 1 = true
	// Bit 6:  horizontal angle offset added: 0 = false, 1 = true
	// Bit 10: mirror side: 0=front (for 8-Layer, tilted downward), 1=rear (for 8-layer, tilted upward)
	// All other flags are reserved-internal and should not be evaluated.
	volatile bool isRearMirrorSide = true;
	INT16 processingFlags	 	= (INT16)readValueLE(&(scanBuffer[42]), 2);
	if ((processingFlags & 0x0400) == 0)
	{
// 		printInfoMessage("LuxBase::decodeScan(): Mirror side 0.", true);
		isRearMirrorSide = false;
 	}
 	else
 	{
// 		printInfoMessage("LuxBase::decodeScan(): Mirror side 1.", true);
		isRearMirrorSide = true;
 	}

	// Time per angle (rad). Used later to calculate the time of the beam inside the scan.
// 	double angleTime;	// Time in the scan for 1 degree
// 	if (m_scanFrequency > 0)
// 	{
// 		angleTime = (1.0 / (double)m_scanFrequency) / (360.0 * deg2rad); // in [s/rad]
// 	}
// 	else
// 	{
// 		angleTime = 0.001; // Default to avoid errors: 1 ms in [s]; for emergencies only.
// 	}

	// Scan point list
	UINT8* scanPtBuffer;
	for (UINT16 s = 0; s < scanPoints; s++)
	{
		scanPtBuffer = &(scanBuffer[44 + s*10]);	// Start of the scanpoint
		UINT8 layerAndEcho = (UINT8)readUValueLE(&(scanPtBuffer[0]), 1);
		UINT8 layer = layerAndEcho & 0x0F;											// One of the 4 layers
		UINT8 echo = (layerAndEcho >> 4) & 0x0F;									// Number of the echo of this pulse
		UINT16 flags = (UINT16)readUValueLE(&(scanPtBuffer[1]), 1);					// 0x01:transparent; 0x02:clutter; 0x04:ground; 0x08: dirt
		INT16 horzAngleTicks = (INT16)readValueLE(&(scanPtBuffer[2]), 2);			// H-Angle of this shot
		UINT16 radialDistanceCm = (UINT16)readUValueLE(&(scanPtBuffer[4]), 2);		// Radial distance in scanner coords
		UINT16 echoPulseWidthCm = (UINT16)readUValueLE(&(scanPtBuffer[6]), 2);		// Echo pulse width; Indicator for energy of incoming beam
		// Bytes 8 and 9 are reserved.

		// Now add the data to the scan structure
		ScanPoint& newPoint = scan->addNewPoint();

		// Horizontal angle
		double hAngle = convertTicktsToAngle(horzAngleTicks);	// hAngle is in [rad]

		// Vertical angle
		double vAngle = 0.0;
		vAngle = getVAngleOfLayer(isRearMirrorSide, layer, hAngle);
		
		// Radial distance
		double dist = (double)radialDistanceCm / 100.0;	// cm to m

		// Set the coordinates of the new point. Also automatically generates x-y-z coordinates.
		newPoint.setPolar (dist, hAngle, vAngle);

		// Copy data to new scan point
		newPoint.setEchoWidth ((float)echoPulseWidthCm / 100.0);
		newPoint.setFlags (flags);
		newPoint.setSourceId (m_deviceId);
		newPoint.setLayer (layer);
		newPoint.setEchoNum (echo); // 0 or 1 or ...
//		newPoint.setTimeOffset (boost::posix_time::microseconds ((UINT32)((startAngle - hAngle) * angleTime * 1000000.0))); // Time offset of scan point
//		newPoint.setSegmentId(0); // Not available
	}

//	if (m_enableCoordTransformation == true)
//	{
//		scanFlags = Scan::FlagVehicleCoordinates;
//	}
	scan->setFlags(processingFlags);

	//
	// Set some information about the scanner
	//
	// Create Scanner Info
	ScannerInfo si;
	si.setStartAngle(startAngle);
	si.setEndAngle(endAngle);

	si.setScanFrequency(m_scanFrequency);
	si.setBeamTilt(m_beamTiltAngle);
//	si.setScannerStatus(scannerStatus);
	si.setScanFlags(processingFlags);
	si.setScanNumber(scanNumber);
	si.setDeviceID(m_deviceId);
	si.setScannerType(Sourcetype_LDMRS); // for compatibility, if no value is set in the scanner's config.
	Time start, end;
	start.set(timestampStart);
	end.set(timestampEnd);
	si.setTimestamps(start, end);
	si.setProcessingFlags(processingFlags);	// Mirror side etc.

	// Mounting position
	double yawAngle, pitchAngle, rollAngle, offsetX, offsetY, offsetZ;
	yawAngle = convertTicktsToAngle(mountingPosYawTicks);
	pitchAngle = convertTicktsToAngle(mountingPosPitchTicks);
	rollAngle = convertTicktsToAngle(mountingPosRollTicks);
	offsetX = (double)mountingPosX / 100.0;
	offsetY = (double)mountingPosY / 100.0;
	offsetZ = (double)mountingPosZ / 100.0;
	Position3D mp(yawAngle, pitchAngle, rollAngle, offsetX, offsetY, offsetZ);
	si.setMountingPosition(mp);
	scan->setScannerInfos(Scan::ScannerInfoVector(1, si));
//	scan.setStartTimestamp(startTime);
//	scan.setEndTimestamp(endTime);

	// Post this scan to the world above...
	scan->setSourceId(m_deviceId);
	m_manager->setDeviceData(scan);

//	printInfoMessage("decodeScan(): Decoded scan with " + toString(scanPoints) + " points.", m_beVerbose);
}

void LuxBase::decodeSensorInfo()
{
	UINT8* sensorInfoBuffer = &(m_inputBuffer[24]);   // Skip the data header

	// decode sensor info
//	UINT16 sensorInfoVersion = (UINT16)readUValueLE(&(scanBuffer[0]), 2);   // here: 1
//	UINT16 relatedScanNumber = (UINT16)readUValueLE(&(scanBuffer[2]), 2);
//	UINT16 fpgaErrorRegister1 = (UINT16)readUValueLE(&(scanBuffer[4]), 2);
//	UINT16 fpgaErrorRegister2 = (UINT16)readUValueLE(&(scanBuffer[6]), 2);
//	UINT16 fpgaWarningRegister = (UINT16)readUValueLE(&(scanBuffer[8]), 2);
//	UINT16 dspWarningRegister = (UINT16)readUValueLE(&(scanBuffer[10]), 2);

	m_temperature = (UINT16)readUValueLE(&(sensorInfoBuffer[12]), 2);

//	...
}


/*
 * Calculate the elevation angle of the scanpoint depending on scan layer, mirror side, horziontal angle and UpsideDown flag.
 */
double LuxBase::getVAngleOfLayer(bool isRearMirrorSide, UINT8 layerNumber, double hAngle)
{
	// Calculate the effective mirror tilt as a function of the beam tilt at 0 degree horizontally and the current horizontal angle
	const double effMirrorTilt = m_beamTiltAngle / 2.0 * M_SQRT2 * std::sin(0.5 * hAngle - M_PI_4);

	// Distinguish between the front and back side of the mirror.
	// Front mirror side: beam tilted/pitched down (positive pitch), rear mirror side: beam tilted/pitched up (negative pitch)
	double mirrorDirection = 1.0;
	if (isRearMirrorSide == false)
	{
		 mirrorDirection = -1.0;
	}

	// Take UpsideDown into account
	if (m_upsideDownActive == true)
	{
		mirrorDirection *= -1.0;
	}

	// Calculate the layer offset of each layer. This calculation defines 0 degree as a symetrical layer.
	const double numLayers = 4.0;
	const double verticalBeamDivergence = 0.8 * deg2rad;
	const double layerOffset = ((numLayers - 1) / 2 - layerNumber) * verticalBeamDivergence;
	const double vAngle = layerOffset + mirrorDirection * 2 * effMirrorTilt;

	return vAngle;
}



//
// Converts an angle from ticks (MRS-internal unit) to radians.
//
double LuxBase::convertTicktsToAngle(INT16 angleTicks)
{
	double angle = ((double)angleTicks / 32.0) * deg2rad;

	if ((angle > -1000.0) && (angle < 1000.0))
	{
		// Angle ist in "sinnvollem" Bereich
		while (angle > PI)
		{
			angle -= 2.0 * PI;
		}
		while (angle <= -PI)
		{
			angle += 2.0 * PI;
		}
	}
	else
	{
		// Winkel ist unsinning
		printError("convertTicktsToAngle(): The angle value " + toString(angle, 2) + " is widely out of the reasonable range, setting to 0.0.");
		angle = 0.0;
	}

	return angle;
}



//
// Decodes object data that is completely present as the first message in the input buffer.
//
// Note that not all LD-MRS versions can deliver object data.
//
void LuxBase::decodeObjects()
{
	printInfoMessage("decodeObjects(): We have received a dataset and are now decoding objects.", m_beVerbose);
//	printWarning("decodeObjects(): We have received a dataset, BUT THIS FUNCTION IS NOT IMPLEMENTED, ignoring the data!");
//	return;

	// Unlike the scan data decoder, we are using a bufferOffset pointer here. As the number of contour
	// points varies for each object, this is a simple way to keep track of our position in the buffer.
	UINT32 bufferOffset = 24;

	ObjectList* objectList = new ObjectList;	// The container for the output data

	// Decode the number of objects
	UINT16 numObjects = (UINT16)readUValueLE(&(m_inputBuffer[bufferOffset + 8]), 2);
	bufferOffset = 24 + 10;

	for (UINT16 i = 0; i < numObjects; i++)
	{
		Object newObject;

		// Offset 0: Object ID
		UINT16 objectId = (UINT16)readUValueLE(&(m_inputBuffer[bufferOffset]), 2);
		bufferOffset += 2;
		newObject.setObjectId (objectId);

		// Offset 2: Object age
		UINT16 objectAge = (UINT16)readUValueLE(&(m_inputBuffer[bufferOffset]), 2);
		bufferOffset += 2;
		newObject.setObjectAge (objectAge);

		// Offset 4: Object prediction age
		UINT16 objectPredictionAge = (UINT16)readUValueLE(&(m_inputBuffer[bufferOffset]), 2);
		bufferOffset += 2;
		newObject.setHiddenStatusAge (objectPredictionAge);

		// Offset 6: Relative timestamp
		UINT16 relativeTimestamp = (UINT16)readUValueLE(&(m_inputBuffer[bufferOffset]), 2);
		bufferOffset += 2;
		Time t;
		t.set((double)relativeTimestamp);
		newObject.setTimestamp (t);

		// Offset 8: Reference point
		Point2D referencePoint = readPoint2D(&(m_inputBuffer[bufferOffset]));
		bufferOffset += 4;
		newObject.setCenterPoint (referencePoint);

		// Offset 12: Reference point sigma
		Point2D referencePointSigma = readPoint2D(&(m_inputBuffer[bufferOffset]));
		bufferOffset += 4;
		newObject.setCenterPointSigma(referencePointSigma);

		Point2D closestPoint = readPoint2D(&(m_inputBuffer[bufferOffset]));
		bufferOffset += 4;
		newObject.setClosestPoint (closestPoint);

		Point2D boundingBoxCenter = readPoint2D(&(m_inputBuffer[bufferOffset]));
		bufferOffset += 4;
		newObject.setBoundingBoxCenter(boundingBoxCenter);

		Point2D boundingBoxSize = readSize2D(&(m_inputBuffer[bufferOffset]));
		double tmp = boundingBoxSize.getX();
		// x and y are flipped on the wire
		boundingBoxSize.setX(boundingBoxSize.getY());
		boundingBoxSize.setY(tmp);
		bufferOffset += 4;
		newObject.setBoundingBox(boundingBoxSize);

		//Point2D objectBoxCenter = readPoint2D(&(m_inputBuffer[bufferOffset]));
		bufferOffset += 4;

		Point2D objectBoxSize = readSize2D(&(m_inputBuffer[bufferOffset]));
		bufferOffset += 4;
		newObject.setObjectBox (objectBoxSize);

		INT16 objectBoxOrientationTicks = (INT16)readValueLE(&(m_inputBuffer[bufferOffset]), 2);
		bufferOffset += 2;
		newObject.setCourseAngle (convertTicktsToAngle(objectBoxOrientationTicks));

		Point2D absoluteVelocity = readPoint2D(&(m_inputBuffer[bufferOffset]));
		bufferOffset += 4;
		Point2D absoluteVelocitySigma = readSize2D(&(m_inputBuffer[bufferOffset]));
		bufferOffset += 4;
		Point2D relativeVelocity = readPoint2D(&(m_inputBuffer[bufferOffset]));
		bufferOffset += 4;

		if (absoluteVelocity.getX() < -320.0)
		{
			// Absolute velocity is invalid, use relative velocity instead
			newObject.setAbsoluteVelocity (relativeVelocity);
			newObject.setRelativeVelocitySigma (absoluteVelocitySigma);
			newObject.setAbsoluteVelocitySigma (absoluteVelocitySigma);
		}
		else
		{
			// Absolute velocity is valid. This will only be the case if vehicle movement data is
			// sent to the sensor via CAN.
			newObject.setAbsoluteVelocity (absoluteVelocity);
			newObject.setRelativeVelocitySigma (Point2D(NaN_double, NaN_double));
			newObject.setAbsoluteVelocitySigma (absoluteVelocitySigma);
		}
		newObject.setRelativeVelocity (relativeVelocity);

		UINT16 classification = (UINT16)readUValueLE(&(m_inputBuffer[bufferOffset]), 2);
		bufferOffset += 2;
		switch (classification)
		{
		case ClassUnclassified:
			newObject.setClassification (Object::Unclassified);
			break;
		case ClassUnknownSmall:
			newObject.setClassification (Object::UnknownSmall);
			break;
		case ClassUnknownBig:
			newObject.setClassification (Object::UnknownBig);
			break;
		case ClassPedestrian:
			newObject.setClassification (Object::Pedestrian);
			break;
		case ClassBike:
			newObject.setClassification (Object::Bike);
			break;
		case ClassCar:
			newObject.setClassification (Object::Car);
			break;
		case ClassTruck:
			newObject.setClassification (Object::Truck);
			break;
		default:
			newObject.setClassification(Object::Unknown);
		}

		UINT16 classificationAge = (UINT16)readUValueLE(&(m_inputBuffer[bufferOffset]), 2);
		bufferOffset += 2;
		newObject.setClassificationAge (classificationAge);

		UINT16 classificationCertainty = (UINT16)readUValueLE(&(m_inputBuffer[bufferOffset]), 2);
		bufferOffset += 2;
		if (classificationCertainty <= 100)
		{
			newObject.setClassificationQuality(classificationCertainty / 100.f);
		}
		else
		{
			// Invalid value, set to 0
			newObject.setClassificationQuality(0);
		}

		// Contour points of this object
		UINT16 numContourPoints = (UINT16)readUValueLE(&(m_inputBuffer[bufferOffset]), 2);
		bufferOffset += 2;
		// Bugfix: If the scanner reports 0xFFFF, he means "1"...
		if (numContourPoints == 0xFFFF)
		{
			numContourPoints = 1;
		}
		
		Point2D cp;
		for (UINT16 c = 0; c < numContourPoints; c++)
		{
			cp = readPoint2D(&(m_inputBuffer[bufferOffset]));
			bufferOffset += 4;
			newObject.addContourPoint(cp);
		}

		// Transfer the object to the object list
		objectList->push_back (newObject);
	}

	// Set the remaining fields of the object list and send it
	objectList->setSourceId(m_deviceId);
	m_manager->setDeviceData(objectList);

	printInfoMessage("LuxBase::decodeObjects(): We have received " + toString(numObjects) + " objects. Now leaving.", m_beVerbose);
}

//
// Reads a Point2D structure, represented in the buffer by two INT16 values.
// Output unit is [m].
//
Point2D LuxBase::readPoint2D(UINT8* buffer)
{
	INT16 value1 = (INT16)readValueLE(buffer, 2);
	INT16 value2 = (INT16)readValueLE(&(buffer[2]), 2);

	return Point2D(((double)value1 / 100.0), ((double)value2 / 100.0));
}

//
// Reads a Size2D structure, represented in the buffer by two INT16 values.
// For simplicity, a Point2D container is used for the data.
// Output unit is [m].
//
Point2D LuxBase::readSize2D(UINT8* buffer)
{
	UINT16 value1 = (UINT16)readUValueLE(buffer, 2);
	UINT16 value2 = (UINT16)readUValueLE(&(buffer[2]), 2);
	Point2D s((double)value1 / 100.0, (double)value2 / 100.0);
	return s;
}

//
// Moves data from the input to the command reply buffer, and removes this data from the
// input buffer. Note that any data that may have been present in the reply buffer is
// overwritten, so be sure to mutex this section against cross-thread access.
//
void LuxBase::moveDataFromInputToCmdBuffer(UINT32 bytesToBeMoved)
{
	if (m_beVerbose == true)
	{
		infoMessage("moveDataFromInputToCmdBuffer(): Moving " + toString(bytesToBeMoved) + " bytes from input to command buffer.");
		if (m_cmdBufferLevel > 0)
		{
			infoMessage("moveDataFromInputToCmdBuffer(): ...and " + toString(m_cmdBufferLevel) + " bytes in the command buffer were destroyed in the process!");
		}
	}

	memcpy(m_cmdReplyBuffer, m_inputBuffer, bytesToBeMoved);
	m_cmdBufferLevel = bytesToBeMoved;
	removeDataFromInputBuffer(bytesToBeMoved);
}


/**
 * Remove the first x bytes from the input buffer.
 *
 * Note that if the host system is fast enough, this does not happen, but the buffer is
 * just invalidated.
 */
void LuxBase::removeDataFromInputBuffer(UINT32 bytesToBeRemoved)
{
	if (bytesToBeRemoved == m_inBufferLevel)
	{
		// All data should be removed
		m_inBufferLevel = 0;
		return;
	}
	else if (bytesToBeRemoved > m_inBufferLevel)
	{
		// Error: We do not have so much data
		printError("removeDataFromInputBuffer(): The buffer holds " + toString(m_inBufferLevel) + " bytes, but " +
					   toString(bytesToBeRemoved) + " bytes should be removed - clearing buffer.");
		m_inBufferLevel = 0;
		return;
	}

	// Do the shift
	UINT32 bytesRemaining = m_inBufferLevel - bytesToBeRemoved;
	memmove(&(m_inputBuffer[0]), &(m_inputBuffer[bytesToBeRemoved]), bytesRemaining);
	m_inBufferLevel = bytesRemaining;
}


/**
 * Returns the ID ("command") of the message in the reply buffer. If a complete dataset is in the buffer,
 * the return value is non-zero (=valid). Note that the buffer may hold only one complete (!) dataset.
 *
 * Note: Access to command and input buffer must be mutex'ed, with same mutex to avoid cross-overwrite while
 * decoding!!
 */
UINT16 LuxBase::decodeAnswerInCmdReplyBuffer()
{
	bool beVerboseHere = false;	// = m_beVerbose;
	printInfoMessage("Entering decodeAnswerInCmdReplyBuffer().", beVerboseHere);

	const UINT32 headerLen = 24;	// Length of data header, in [bytes]
	UINT16 commandId = 0;

	// Check to be sure the data in the buffer is complete
	if (m_cmdBufferLevel == 0)
	{
		// No data. Check is there is news from the input buffer.
		decodeAnswerInInputBuffer();

		if (m_cmdBufferLevel == 0)
		{
			// Still no data
			printInfoMessage("decodeAnswerInCmdReplyBuffer(): No data in input buffer.", beVerboseHere);
			return 0;
		}
	}

	// There is data in the buffer. Now perform some consistency checks.
	if (m_cmdBufferLevel <= headerLen)
	{
		printError("decodeAnswerInCmdReplyBuffer(): ERROR: Not enough data in reply buffer for the data header, just " +
					toString(m_cmdBufferLevel) + " bytes available. Clearing buffer!");
		m_cmdBufferLevel = 0;
		return 0;
	}

	// Ensure that data starts with magic word
	UINT32 magicWord;
	magicWord = readUValueBE(&(m_cmdReplyBuffer[0]), 4);
	if (magicWord != 0xAFFEC0C2)
	{
		printError("decodeAnswerInCmdReplyBuffer(): Magic word not found, clearing cmd buffer.");
		m_cmdBufferLevel = 0;
		return 0;
	}
	else
	{
		// Magic word found
		printInfoMessage("decodeAnswerInCmdReplyBuffer(): Magic word found, now decoding header.", beVerboseHere);
	}

	// Yes, we have a data header. We now calculate the size of the complete message.
	UINT32 payloadLen = readUValueBE(&(m_cmdReplyBuffer[8]), 4);

	printInfoMessage("decodeAnswerInCmdReplyBuffer(): Message payload length is " + toString(payloadLen) + " bytes.", beVerboseHere);

	// Is the message complete and length consistent?
	if (m_cmdBufferLevel == (payloadLen + headerLen))
	{
		// The command is completely in the buffer, so now return its command ID
		commandId = readUValueLE(&(m_cmdReplyBuffer[24]), 2);
		printInfoMessage("decodeAnswerInCmdReplyBuffer(): Header decoded successfully, command = " +
						toHexString(commandId) + ".", beVerboseHere);
	}
	else
	{
		printError("decodeAnswerInCmdReplyBuffer(): Inconsistent length of data in reply buffer, expected length was " +
					  toString(payloadLen + headerLen) + ", but buffer length is " + toString(m_cmdBufferLevel) + " bytes - clearing buffer.");
		m_cmdBufferLevel = 0;
		return 0;
	}

	printInfoMessage("decodeAnswerInCmdReplyBuffer(): Leaving successfully, command = " + toHexString(commandId) + ".", beVerboseHere);
	return commandId;
}

//
// Debug helper.
//
void LuxBase::dumpHeader()
{
	std::ostringstream s;
	s << "Header:";
	for (UINT32 i = 0; i < 24; i++)
	{
		s << " " << toHexString(m_inputBuffer[i]);
	}

	infoMessage(s.str());
}



void LuxBase::dumpMessage()
{
	std::ostringstream s;
	s << "Message:";
	UINT32 payloadLen = readUValueBE(&(m_inputBuffer[8]), 4);

	for (UINT32 i = 0; i < payloadLen; i++)
	{
		s << " " << toHexString(m_inputBuffer[24+i]);
	}

	infoMessage(s.str());
}



/**
 * Removes the first complete command from the input buffer.
 * It is assumed that a valid command is in the buffer.
 *
 * Note: Access to input buffer must be mutex'ed.
 */
void LuxBase::removeAnswerFromInputBuffer()
{
	const UINT32 headerLen = 24;	// Length of data header, in [bytes]

	// Check if a command can be complete. Any command has at least the header + 2 bytes command ID.
	if (m_inBufferLevel <= (headerLen + 1))
	{
		printWarning("LuxBase::removeAnswerFromInputBuffer: Not enough data in input buffer to remove a dataset, aborting!");
		return;
	}

	// Check magic word
	UINT32 magicWord;
	magicWord = readUValueBE(&(m_inputBuffer[0]), 4);
	if (magicWord != 0xAFFEC0C2)
	{
		printError("LuxBase::removeAnswerFromInputBuffer: Magic word does not match, aborting!");
		return;
	}

	// Complete message?
	UINT32 msgLen = headerLen + readUValueBE(&(m_inputBuffer[8]), 4);
	if (m_inBufferLevel < msgLen)
	{
		printError("LuxBase::removeAnswerFromInputBuffer: The buffer does not hold enough data for the message!");
		return;
	}

	// Ok, remove the data
	if (m_inBufferLevel == msgLen)
	{
		// There is only this message in the buffer, so just invalidate it
		m_inBufferLevel = 0;
		printInfoMessage("removeAnswerFromInputBuffer(): Clearing the whole buffer.", m_beVerbose);
	}
	else
	{
		// Remove the first message. Can be replaced by memmove().
		for (UINT32 i = msgLen; i < m_inBufferLevel; i++)
		{
			m_inputBuffer[i-msgLen] = m_inputBuffer[i];
		}
		m_inBufferLevel -= msgLen;

		printInfoMessage("LuxBase::removeAnswerFromInputBuffer(): Removed " + toString(msgLen) + " bytes from buffer, new size is " +
							toString(m_inBufferLevel) + " bytes.", m_beVerbose);
	}
}



//
//
//
bool LuxBase::receiveMrsReply(MrsCommandId cmd, UINT32 timeoutMs, UINT32* value)
{
	bool beVerboseHere = m_beVerbose;
	
	printInfoMessage("LuxBase::receiveMrsReply: Entering.", beVerboseHere);

	// For the timeout
	UINT32 maxLoops = timeoutMs;
	bool result = false;

	for (UINT32 i = 0; i < maxLoops; i++)
	{
		// Read the data, if any
		{
			ScopedLock lock(&m_inputBufferMutex);		// Mutex for access to the input buffer
			UINT16 cmdInBuffer = decodeAnswerInCmdReplyBuffer();	// Get the command ID, if any
			if (cmdInBuffer == cmd)
			{
				// Ok, it is the wanted reply
				printInfoMessage("LuxBase::receiveMrsReply: " + m_longName + " There is a valid message (cmd=0x" +
									toHexString(cmdInBuffer) + ").", beVerboseHere);
				if (cmd == CmdMrsGetStatus)
				{
					printInfoMessage("LuxBase::receiveMrsReply: " + m_longName + " Decoding a status message.", beVerboseHere);
					decodeGetStatus();
				}
				if (cmd == CmdMrsStopMeasure)
				{
					printInfoMessage("LuxBase::receiveMrsReply: " + m_longName + " StopMeasure acknowledge received.", beVerboseHere);
				}
				if (cmd == CmdMrsStartMeasure)
				{
					printInfoMessage("LuxBase::receiveMrsReply: " + m_longName + " StartMeasure acknowledge received.", beVerboseHere);
				}
				if (cmd == CmdMrsGetParameter)
				{
					printInfoMessage("LuxBase::receiveMrsReply: " + m_longName + " GetParameter acknowledge received.", beVerboseHere);
					if (value != NULL)
					{
						decodeGetParameter(value);
					}
				}

				result = true;
			}

			// Remove the received answer from the reply buffer
			if (cmdInBuffer != 0)
			{
				printInfoMessage("LuxBase::receiveMrsReply: " + m_longName + " Removing the command from the reply buffer.", beVerboseHere);
				m_cmdBufferLevel = 0;
			}

			if (cmdInBuffer == 0x8000 + cmd)
			{
				printWarning("LuxBase::receiveMrsReply: " + m_longName + " Received that an error occurred.");
				result = false;
				break;
			}
		}

		if (result == true)
		{
			break;
		}

		// Schlafe eine ms
		usleep(1000);
	}

	printInfoMessage("LuxBase::receiveMrsReply: " + m_longName + " Leaving.", beVerboseHere);
	return result;
}



/**
 * Send a command to the scanner.
 */
bool LuxBase::sendMrsCommand(MrsCommandId cmd, UINT16 para, UINT32 value)	// , boost::posix_time::time_duration timeOut)
{
	UINT8 cmdBuffer[256];
	UINT32 sizeOfThisMsg = 0;

	// Determine the size of the message to be sent
	switch (cmd)
	{
	case CmdMrsGetStatus:
		sizeOfThisMsg = 2 + 2;	// 4 Bytes
		break;
	case CmdMrsStopMeasure:
		sizeOfThisMsg = 2 + 2;	// 4 Bytes
		break;
	case CmdMrsStartMeasure:
		sizeOfThisMsg = 2 + 2;	// 4 Bytes
		break;
	case CmdMrsGetParameter:
		sizeOfThisMsg = 2 + 2 + 2;	// 6 Bytes
		break;
	case CmdMrsSetParameter:
		sizeOfThisMsg = 2 + 2 + 2 + 4;	// 4+6 = 10 Bytes
		break;
	case CmdMrsSetNTPTimestampSec:
		sizeOfThisMsg = 2 + 2 + 2 + 4;	// 4+6 = 10 Bytes
		break;
	case CmdMrsSetNTPTimestampFracSec:
		sizeOfThisMsg = 2 + 2 + 2 + 4;	// 4+6 = 10 Bytes
		break;
	default:
		printError("LuxBase::sendMrsCommand: " + m_longName + " ERROR: Unknown command to be sent (ID=0x" +
					toHexString((UINT16)cmd) + "), aborting!");
		return false;
	}

	// Build the header in the buffer
	// DataTypeCommand	= 0x2010,	///< Command for a device
	// DataTypeReply	= 0x2020,	///< Reply of a previous command
	UINT32 bufferPos;
	bufferPos = buildDataHeader(cmdBuffer, sizeOfThisMsg, m_deviceId, 0x2010);	// DataTypeCommand);

	// Add the payload
	bufferPos += writeValueToBufferLE(&(cmdBuffer[bufferPos]), cmd, 2); // Note: Little Endian always!
	UINT16 version = 3;
	bufferPos += writeValueToBufferLE(&(cmdBuffer[bufferPos]), version, 2); // Note: Little Endian!

	// For a setParameter command, add the parameter index and the value to be set
	if (cmd == CmdMrsSetParameter
			|| cmd == CmdMrsSetNTPTimestampSec
			|| cmd == CmdMrsSetNTPTimestampFracSec)
	{
		bufferPos += writeValueToBufferLE(&(cmdBuffer[bufferPos]), para, 2);
		bufferPos += writeValueToBufferLE(&(cmdBuffer[bufferPos]), value, 4);
	}
	else if (cmd == CmdMrsGetParameter)
	{
		bufferPos += writeValueToBufferLE(&(cmdBuffer[bufferPos]), para, 2);
	}

	// Write the data to the interface
	UINT32 bytesToSend = bufferPos;
	bool result = m_tcp.write(cmdBuffer, bytesToSend);

	return result;
}



//
// Big-Endian-Write
//
UINT32 LuxBase::writeValueToBufferBE(UINT8* buffer, UINT32 value, UINT8 numBytes)
{
	switch (numBytes)
	{
	case 1:
		buffer[0] = value & 0xff;
		break;
	case 2:
		buffer[0] = (value & 0xff00) >> 8;
		buffer[1] = value & 0xff;
		break;
	case 4:
		buffer[0] = (value & 0xff000000) >> 24;
		buffer[1] = (value & 0xff0000) >> 16;
		buffer[2] = (value & 0xff00) >> 8;
		buffer[3] = value & 0xff;
		break;
	default:
		printError("LuxBase::writeValueToBufferBE: " + m_longName + " ERROR: Invalid number of bytes to write, can only handle 1,2 or 4.");
	}

	return (UINT32)numBytes;
}



//
// Little-Endian-Write
//
UINT32 LuxBase::writeValueToBufferLE(UINT8* buffer, UINT32 value, UINT8 numBytes)
{
	switch (numBytes)
	{
	case 1:
		buffer[0] = value & 0xff;
		break;
	case 2:
		buffer[1] = (value & 0xff00) >> 8;
		buffer[0] = value & 0xff;
		break;
	case 4:
		buffer[3] = (value & 0xff000000) >> 24;
		buffer[2] = (value & 0xff0000) >> 16;
		buffer[1] = (value & 0xff00) >> 8;
		buffer[0] = value & 0xff;
		break;
	default:
		printError("LuxBase::writeValueToBufferLE: " + m_longName + " ERROR: Invalid number of bytes to write, can only handle 1,2 or 4.");
	}

	return (UINT32)numBytes;
}



/**
 * Write the data header for the command into the buffer.The buffer must be at least 24 bytes long.
 *
 * Return value is the number of bytes that have been written.
 */
UINT32 LuxBase::buildDataHeader(UINT8* buffer, UINT32 sizeOfThisMsg, UINT8 deviceId, UINT16 dataType)
{
	UINT32 magicWord = 0xAFFEC0C2;
	UINT32 sizeOfPrevMsg = 0;
	UINT8 reserved = 0;
	
	// Status: AFFEC0C2 00000000 00000004 00 07 2010 00000000 00000000 0100 0000
	// Reset:  AFFEC0C2 00000000 00000004 00 07 2010 00000000 00000000 0000 0000
	//
	// Fill the buffer
	writeValueToBufferBE (&(buffer[0]), magicWord, 4);			// 4
	writeValueToBufferBE (&(buffer[4]), sizeOfPrevMsg, 4);		// 4 (8)
	writeValueToBufferBE (&(buffer[8]), sizeOfThisMsg, 4);		// 4 (12)
	writeValueToBufferBE (&(buffer[12]), reserved, 1);			// 1 (13)
	writeValueToBufferBE (&(buffer[13]), deviceId, 1);			// 1 (14)
	writeValueToBufferBE (&(buffer[14]), dataType, 2);			// 2 (16)

	// Timestamp
	UINT32 timeSec = 0;
	UINT32 timeSecFractionOffset = 0;
	writeValueToBufferBE (&(buffer[16]), timeSec, 4);				// 4 (20)
	writeValueToBufferBE (&(buffer[20]), timeSecFractionOffset, 4);	// 4 (24)

	return 24;	// Header is always 24 Bytes long
}



//
// RUN-Modus:
//
// Starte den Scanner, und abonniere Scans und/oder Objektdaten.
//
bool LuxBase::run(bool weWantScanData, bool weWantObjectData)
{
	bool result;

	if ((m_inputFileName == "") && (m_tcp.isOpen() == true))
	{
		// TCP connection was opened by the MRS::init() function 
//		printWarning("LuxBase::run(): TCP connection is alread open.");
	}
	else
	{
		// We are not connected, so connect now
		printInfoMessage("LuxBase::run(): We are called, but we are not connected. Calling init() now.", m_beVerbose);
		bool result = false;
		if (m_inputFileName == "")
		{
			// TCP
			result = initTcp(m_tcpDisconnectFunction, m_disconnectFunctionObjPtr);
		}
		else
		{
			// TCP
			result = initFile(m_fileDisconnectFunction, m_disconnectFunctionObjPtr);
		}

		if (result == false)
		{
			// We failed to connect
			printError("LuxBase::run(): Call to init() was not successful, aborting!");
			return false;
		}
		else
		{
			// We are connected
			printInfoMessage("LuxBase::run(): Call to init() was successful, we are connected.", m_beVerbose);
		}
	}

	if (isRunning() == true)
	{
		// Error: Already running
		printInfoMessage("LuxBase::run(): Duplicate RUN command, " + m_longName + " is running already. Ignoring command.", true);
		return true;
	}

	printInfoMessage("LuxBase::run(): Beginning scanner start sequence for " + m_longName + ".", m_beVerbose);

	// Set the scanner parameters
	if ((m_readOnlyMode == false) && (m_inputFileName == ""))
	{
		// TCP
		// We may write parameters
		result = cmd_setScanFrequency(m_scanFrequency);
		if (result == false)
		{
			printError("LuxBase::run(): Failed to set scan frequency, aborting.");
			return false;
		}


		result = cmd_setScanAngles(m_scanStartAngle, m_scanEndAngle);
		if (result == false)
		{
			printError("LuxBase::run(): Failed to set scan angles, aborting.");
			return false;
		}

		// "MountingPosition" is a container for the 3 angles and 3 offsets.
		Position3D mp(m_yawAngle, m_pitchAngle, m_rollAngle, m_offsetX, m_offsetY, m_offsetZ);
		result = cmd_setMountingPos(mp);
		if (result == false)
		{
			printError("LuxBase::run(): Failed to set mounting pos, aborting.");
			return false;
		}
	}
	else
	{
		// We may not write parameters.
		infoMessage("LuxBase::run(): Running in read-only mode, so we're just using current sensor parameters.");
	}

	//
	// Start measuring. Note that here, also the output flags are set.
	//
	if (m_inputFileName == "")
	{
		// TCP
		result = cmd_startMeasure(weWantScanData, weWantObjectData);
		if (result == true)
		{
			// The scanner is running (= is delivering scans)
			m_isRunning = true;
		}
		else
		{
			// We failed to set the scan mode
			printError("LuxBase::run(): The StartMeasure command failed!");
		}
	}
	else
	{
		// File
		m_isRunning = true;	// The scanner is "running"
	}

	return result;
}



//
// The TCP read callback.
//
void LuxBase::readCallbackFunction(UINT8* buffer, UINT32& numOfBytes)
{
	bool beVerboseHere = false;	// = m_beVerbose;
	printInfoMessage("LuxBase::readCallbackFunction(): Called with " + toString(numOfBytes) + " available bytes.", beVerboseHere);

	ScopedLock lock(&m_inputBufferMutex);		// Mutex for access to the input buffer
	UINT32 remainingSpace = MRS_INPUTBUFFERSIZE - m_inBufferLevel;
	UINT32 bytesToBeTransferred = numOfBytes;
	if (remainingSpace < numOfBytes)
	{
		bytesToBeTransferred = remainingSpace;
	}

	printInfoMessage("LuxBase::readCallbackFunction(): Transferring " + toString(bytesToBeTransferred) +
						" bytes from TCP to input buffer.", beVerboseHere);

	if (bytesToBeTransferred > 0)
	{
		// Data can be transferred into our input buffer
		memcpy(&(m_inputBuffer[m_inBufferLevel]), buffer, bytesToBeTransferred);
		m_inBufferLevel += bytesToBeTransferred;

		// Now work on the input buffer until all received datasets are processed
		UINT16 datatype;
		do
		{
			datatype = decodeAnswerInInputBuffer();
		}
		while (datatype != 0);
	}
	else
	{
		// There was input data from the TCP interface, but our input buffer was unable to hold a single byte.
		// Either we have not read data from our buffer for a long time, or something has gone wrong. To re-sync,
		// we clear the input buffer here.
		m_inBufferLevel = 0;
	}

	printInfoMessage("LuxBase::readCallbackFunction(): Added " + toString(bytesToBeTransferred) +
						" bytes to input buffer, new fill level is now " + toString(m_inBufferLevel) + " bytes.", beVerboseHere);
}



//
// Stop: Stop scanning, but keep the interface open.
//
bool LuxBase::stop()
{
	if (m_isRunning == false)
	{
		printWarning("stop(): Device is not running, nothing to do.");
		return true;
	}

	printInfoMessage("LuxBase::stop: Stopping LD-MRS device.", m_beVerbose);

	// Stop
	bool result = cmd_stopMeasure();
	if (result == true)
	{
		// Erfolg
		printInfoMessage("LuxBase::stop: LD-MRS device stopped scanning.", m_beVerbose);
		m_isRunning = false;
	}
	else
	{
		// Fehler
		printError("LuxBase::stop: StopMeasure command to LD-MRS was not successful!");
	}

	return true;
}

}	// namespace devices
