//
// manager.hpp
//

#ifndef MANAGER_HPP
#define MANAGER_HPP

#include "BasicDatatypes.hpp"
#include "tools/SickThread.hpp"
#include "tools/BasicDataBuffer.hpp"
#include "devices/BasicDevice.hpp"
#include "application/BasicApplication.hpp"
#include <vector>	// for std::vector

//
// The Manager.
//
class Manager
{
public:
	/// Default constructor.
	Manager();

	/// Destructor.
	~Manager();
	
	bool addApplication(Sourcetype appType, std::string appName, UINT16 wantedId = 0xFFFF);
	bool addApplication(application::BasicApplication* app, UINT16 wantedId = 0xFFFF);
	bool addAndRunDevice(Sourcetype deviceType, std::string deviceName, UINT16 wantedId = 0xFFFF);
	bool addAndRunDevice(devices::BasicDevice* device, std::string deviceName, UINT16 wantedId = 0xFFFF);
	bool importApplications();
	bool runAllDevices();
	void stopAllDevices();
	void setDeviceData(BasicData* data);	// Datenquelle: Hier laden die Devices ihre Daten ab
	devices::BasicDevice* getDeviceById(UINT32 id);
	devices::BasicDevice* getFirstDeviceByType(Sourcetype type);

private:
	bool m_beVerbose;
	UINT16 getNextSourceId();
	UINT16 m_nextSourceId;		// Die als naechstes verwendete ID

	// Die Devices
	typedef std::vector<devices::BasicDevice*> DeviceList;
	DeviceList m_deviceList;
	
	// The applications
	typedef std::vector<application::BasicApplication*> ApplicationList;
	ApplicationList m_appList;
	
	// The buffer of the source
	BasicDataBuffer m_sourceBuffer;
	void sourceThreadFunction(bool& endThread, UINT16& waitTimeMs);		// Die Verteiler-Funktion
	SickThread<Manager, &Manager::sourceThreadFunction> m_sourceThread;	// Der Verteiler-Thread
	Mutex m_sourceBufferMutex;												// Zugriffsschutz des Source-Buffers
};


#endif	// #ifndef MANAGER_HPP
