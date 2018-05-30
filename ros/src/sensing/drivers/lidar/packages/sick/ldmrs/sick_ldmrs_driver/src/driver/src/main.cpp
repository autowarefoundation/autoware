//
// main.cpp
//
// LD-MRS example driver and application.
// This code should compile under Linux.
//
// The main() function contains calls to several demo applications. 
// Most of the scanner parameters - including its IP address - are set up
// directly in the LDMRS class constructor.
//
// All source code provided here is intended for demonstration and reference purposes only.
// SICK provides no warranty whatsoever for the function of this code.
//
// Note that not all features are available with all firmware versions of the MRS. Please refer
// to the manual if the intended functionality is available with your device.
//
// Change log:
// 2013-09-18, willhvo:
//  - Started change log.
// 2014-03-04, willhvo:
//  - Added NtpTimeApp demo.
// 2014-12-18, willhvo:
//  - Added ScanpointCoordinateApp demo.
//

#include "tools/errorhandler.hpp"
#include "BasicDatatypes.hpp"
#include "manager.hpp"

//
// This example demonstrates setting and receiving the NTP timestamp of the MRS. The timestamp
// can be set with two commands, and is automatically included in every data packet received
// from the sensor. 
// This feature is available in all firmware versions.
//
// The MRS device should be configured like this:
// m_weWantScanData:          true
// m_weWantObjectData:        false
// m_weWantFieldData:         false
// m_weWantScanDataFromSopas: false
//
int mrsNtpTimeApp()
{
	// First, create the manager object. The manager handles devices, collects
	// device data and forwards it to the application(s).
	printInfoMessage("mrsNtpTimeApp: Creating the manager.", true);
	Manager manager;
	
	Sourcetype type;
	std::string name;
	UINT16 id;
	bool result = false;
	
	// Add the MRS device (sensor)
	printInfoMessage("mrsNtpTimeApp: Adding the LDMRS device.", true);
	type = Sourcetype_LDMRS;
	name = "LDMRS-1";
	id = 1;
	result = manager.addAndRunDevice(type, name, id);
	if (result == false)
	{
		printError("mrsNtpTimeApp: Failed to add device " + name + ", aborting!");
		return 0;
	}
	
	// Add the application. As the devices may send configuration data only once
	// at startup, the applications may want to be present before the devices are
	// started. But - this is not needed here.
	printInfoMessage("mrsNtpTimeApp: Adding the application MrsNtpTimeApp.", true);
	type = Sourcetype_MrsNtpTimeApp;
	name = "MRS Example NTP time App";
	id = 51;
	result = manager.addApplication(type, name, id);
	if (result == false)
	{
		printError("mrsNtpTimeApp: Failed to add application " + name + ", aborting!");
		return 0;
	}
	printInfoMessage("mrsNtpTimeApp: Application NTP-time is running.", true);

	
	// This loop never ends
	while (1)
	{
		// Sleep 100 ms
		usleep(100000);
	}
	
	return 0;
}


//
// This example demonstrates setting and checking the FlexibleResolution feature of the
// LDMRS. This feature is not available in all firmware versions.
//
// The MRS device should be configured like this:
// m_weWantScanData:          true
// m_weWantObjectData:        false
// m_weWantFieldData:         false
// m_weWantScanDataFromSopas: false
//
int sectorChangeApp()
{
	// First, create the manager object. The manager handles devices, collects
	// device data and forwards it to the application(s).
	printInfoMessage("sectorChangeApp: Creating the manager.", true);
	Manager manager;
	
	Sourcetype type;
	std::string name;
	UINT16 id;
	bool result = false;
	
	// Add the MRS device (sensor)
	printInfoMessage("sectorChangeApp: Adding the LDMRS device.", true);
	type = Sourcetype_LDMRS;
	name = "LDMRS-1";
	id = 1;
	result = manager.addAndRunDevice(type, name, id);
	if (result == false)
	{
		printError("sectorChangeApp: Failed to add device " + name + ", aborting!");
		return 0;
	}
	
	// Add the application. As the devices may send configuration data only once
	// at startup, the applications may want to be present before the devices are
	// started. But - this is not needed here.
	printInfoMessage("sectorChangeApp: Adding the application MrsSectorChangeApp.", true);
	type = Sourcetype_MrsChangeApp;
	name = "MRS Example Sector Change App";
	id = 51;
	result = manager.addApplication(type, name, id);
	if (result == false)
	{
		printError("sectorChangeApp: Failed to add application " + name + ", aborting!");
		return 0;
	}
	printInfoMessage("sectorChangeApp: Application SectorChange is running.", true);

	
	// This loop never ends
	while (1)
	{
		// Sleep 100 ms
		usleep(100000);
	}
	
	return 0;
}


//
// SOPAS scan data reception application.
// This example shows the reception of scan data via the SOPAS interface.
// Note 1: If only scan data is required, it is recommended to use the MRS interface instead.
// Note 2: Not all MRS devices have a SOPAS interface.
//
// The MRS device should be configured like this:
// m_weWantScanData:          false
// m_weWantObjectData:        false
// m_weWantFieldData:         false
// m_weWantScanDataFromSopas: true
//
int mrsSopasScandataApp()
{
	// First, create the manager object. The manager handles devices, collects
	// device data and forwards it to the application(s).
	printInfoMessage("mrsSopasScandataApp: Creating the manager.", true);
	Manager manager;
	
	// Add the application. As the devices may send configuration data only once
	// at startup, the applications must be present before the devices are
	// started. Here, we are using the general data display app because we only want
	// to show the received data.
	Sourcetype type;
	std::string name;
	UINT16 id;
	bool result = false;

	printInfoMessage("mrsSopasScandataApp: Adding the application MrsApp.", true);
	type = Sourcetype_MrsApp;
	name = "MRS ExampleApp";
	id = 50;
	result = manager.addApplication(type, name, id);
	if (result == false)
	{
		printError("mrsSopasScandataApp: Failed to add application " + name + ", aborting!");
		return 0;
	}
	printInfoMessage("mrsSopasScandataApp: Application is running.", true);
	
	//
	// Add and run the sensor
	//
	printInfoMessage("mrsSopasScandataApp: Adding the LDMRS device.", true);
	type = Sourcetype_LDMRS;
	name = "LDMRS-1";
	id = 1;
	result = manager.addAndRunDevice(type, name, id);
	if (result == false)
	{
		printError("mrsSopasScandataApp: Failed to add device " + name + ", aborting!");
		return 0;
	}

	// This loop never ends
	while (1)
	{
		// Sleep 100 ms
		usleep(100000);
	}
	
	return 0;
}


//
// Field setting application.
//
// The MRS device should be configured like this:
// m_weWantScanData:          false
// m_weWantObjectData:        false
// m_weWantFieldData:         true
// m_weWantScanDataFromSopas: false
//
int mrsFieldApp()
{
	// First, create the manager object. The manager handles devices, collects
	// device data and forwards it to the application(s).
	printInfoMessage("mrsFieldApp: Creating the manager.", true);
	Manager manager;
	
	// Add the application. As the devices may send configuration data only once
	// at startup, the applications must be present before the devices are
	// started.
	Sourcetype type;
	std::string name;
	UINT16 id;
	bool result = false;

	printInfoMessage("mrsFieldApp: Adding the application MrsFieldApp.", true);
	type = Sourcetype_MrsFieldApp;
	name = "MRS FieldApp";
	id = 50;
	result = manager.addApplication(type, name, id);
	if (result == false)
	{
		printError("mrsFieldApp: Failed to add application " + name + ", aborting!");
		return 0;
	}
	printInfoMessage("mrsFieldApp: Application is running.", true);
	
	//
	// Add and run the sensor
	//
	printInfoMessage("mrsFieldApp: Adding the LDMRS device.", true);
	type = Sourcetype_LDMRS;
	name = "LDMRS-1";
	id = 1;
	result = manager.addAndRunDevice(type, name, id);
	if (result == false)
	{
		printError("mrsFieldApp: Failed to add device " + name + ", aborting!");
		return 0;
	}

	// This loop never ends
	while (1)
	{
		// Sleep 100 ms
		usleep(100000);
	}
	
	return 0;
}


//
// Demo application for scanpoint coordinate calculation. Receives the scanpoints, and prints coordinates of the points at the
// configured scan angle.
//
// The MRS device could be configured like this:
// m_weWantScanData:          true
// m_weWantObjectData:        false
// m_weWantFieldData:         false
// m_weWantScanDataFromSopas: false
//
int mrsScanpointCoordinateApp()
{
	// First, create the manager object. The manager handles devices, collects
	// device data and forwards it to the application(s).
	printInfoMessage("mrsScanpointCoordinateApp: Creating the manager.", true);
	Manager manager;
	
	// Add the application. As the devices may send configuration data only once
	// at startup, the applications must be present before the devices are
	// started.
	Sourcetype type;
	std::string name;
	UINT16 id;
	bool result = false;

	printInfoMessage("mrsScanpointCoordinateApp: Adding the application MrsScanpointCoordinateApp.", true);
	type = Sourcetype_MrsScanpointCoordinateApp;
	name = "MRS CoordinateApp";
	id = 50;
	result = manager.addApplication(type, name, id);
	if (result == false)
	{
		printError("mrsScanpointCoordinateApp: Failed to add application " + name + ", aborting!");
		return 0;
	}
	printInfoMessage("mrsScanpointCoordinateApp: Application is running.", true);
	
	//
	// Add and run the sensor
	//
	printInfoMessage("mrsScanpointCoordinateApp: Adding the LDMRS device.", true);
	type = Sourcetype_LDMRS;
	name = "LDMRS-1";
	id = 1;
	result = manager.addAndRunDevice(type, name, id);
	if (result == false)
	{
		printError("mrsScanpointCoordinateApp: Failed to add device " + name + ", aborting!");
		return 0;
	}

	// This loop never ends
	while (1)
	{
		// Sleep 100 ms
		usleep(100000);
	}
	
	return 0;
}



//
// Minimal demo application. Receives all datatypes and just prints the data.
//
// The MRS device could be configured like this:
// m_weWantScanData:          true
// m_weWantObjectData:        true
// m_weWantFieldData:         false
// m_weWantScanDataFromSopas: false
//
int mrsApp()
{
	// First, create the manager object. The manager handles devices, collects
	// device data and forwards it to the application(s).
	printInfoMessage("mrsApp: Creating the manager.", true);
	Manager manager;
	
	// Add the application. As the devices may send configuration data only once
	// at startup, the applications must be present before the devices are
	// started.
	Sourcetype type;
	std::string name;
	UINT16 id;
	bool result = false;

	printInfoMessage("mrsApp: Adding the application MrsApp.", true);
	type = Sourcetype_MrsApp;
	name = "MRS ExampleApp";
	id = 50;
	result = manager.addApplication(type, name, id);
	if (result == false)
	{
		printError("mrsApp: Failed to add application " + name + ", aborting!");
		return 0;
	}
	printInfoMessage("mrsApp: Application is running.", true);
	
	//
	// Add and run the sensor
	//
	printInfoMessage("mrsApp: Adding the LDMRS device.", true);
	type = Sourcetype_LDMRS;
	name = "LDMRS-1";
	id = 1;
	result = manager.addAndRunDevice(type, name, id);
	if (result == false)
	{
		printError("mrsApp: Failed to add device " + name + ", aborting!");
		return 0;
	}

	// This loop never ends
	while (1)
	{
		// Sleep 100 ms
		usleep(100000);
	}
	
	return 0;
}


//
// The main program.
// Choose the desired application here.
//
int main(int argc, char **argv)
{
	//
	// LD-MRS Example
	//
	int result;
	
	// The MRS-App connects to an MRS, reads its configuration and receives all incoming data.
	result = mrsApp();
	
	// The MRS-App connects to an MRS, reads its configuration and displays scanpoint coordinates.
//	result = mrsScanpointCoordinateApp();
	
	// The MRS NtpTimeApp demonstrates setting and reading the NTP timestamp.
//	result = mrsNtpTimeApp();
	
	// The SectorChangeApp demonstrates the usage of the FlexRes feature.
//	result = sectorChangeApp();
	
	// The FieldApp demonstrates the usage of the SOPAS interface. It removes and creates fields and
	// eval cases.
//	result = mrsFieldApp();
	
	// The SopasScandataApp demonstrates reading scan data via the SOPAS interface.
//	result = mrsSopasScandataApp();

	
	return result;
}
