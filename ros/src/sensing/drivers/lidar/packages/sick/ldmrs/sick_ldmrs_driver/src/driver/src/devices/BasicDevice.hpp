//
// BasicDevice.hpp
//
// Basis-Datentyp fuer alle Devices (Geraetetreiber, Datenquellen)
//

#ifndef BASICDEVICE_HPP
#define BASICDEVICE_HPP

#include "../BasicDatatypes.hpp"

namespace devices
{
	
// Container fuer alle Devices
class BasicDevice
{
public:
	BasicDevice();
	virtual ~BasicDevice();
	
	UINT16 getDevicetype();
	UINT16 getSourceId();
	void setSourceId(UINT16 deviceId);
	void setDeviceName(std::string name);
	std::string getDeviceName();
	
	// Diese Funktionen koennen da sein.
	virtual bool init() {return true;};
	
	// Diese Funktionen muessen da sein.
	virtual bool run() = 0;
	virtual bool stop() = 0;

protected:
	void setDevicetype(UINT16 devicetype);	//  {m_devicetype = devicetype;}
	bool m_beVerbose;	// Debug-Ausgaben an oder aus.

private:
	UINT16 m_devicetype;		// Der Geratetyp (=Sourcetype)
	UINT16 m_sourceId;			// Die ID
	std::string m_deviceName;		// Ein sprechender Name
};


}	// namespace devices

#endif
