//
// BasicDevice.cpp
//

#include "BasicDevice.hpp"
#include "../tools/errorhandler.hpp"

namespace devices
{

// ****************************************************************************
//     Basisklasse fuer Devices
// ****************************************************************************
BasicDevice::BasicDevice()
	: m_beVerbose(false)
{
	infoMessage("BasicDevice: Starting constructor.", m_beVerbose);
	
	m_devicetype = Sourcetype_Unknown;
	m_deviceName = "(uninitialized)";
	m_sourceId = 0xFFFF;
}

BasicDevice::~BasicDevice()
{
	infoMessage("BasicDevice(): Destructor called.", m_beVerbose);
	
	infoMessage("BasicDevice(): Destructor is done - object is dead.", m_beVerbose);
}

/**
 * Der Typ des Devices, als Zahl.
 */
UINT16 BasicDevice::getDevicetype()
{
	return m_devicetype;
}

/**
 * Der Typ des Devices, als Zahl.
 */
void BasicDevice::setDevicetype(UINT16 devicetype)
{
	m_devicetype = devicetype;
}

/**
 * Die eindeutige ID des Devices, als Zahl.
 */
UINT16 BasicDevice::getSourceId()
{
	return m_sourceId;
}

/**
 * Die eindeutige ID des Devices, als Zahl.
 */
void BasicDevice::setSourceId(UINT16 sourceId)
{
	m_sourceId = sourceId;
}

//
// Setzt den sprechenden Namen des Geraets.
//
void BasicDevice::setDeviceName(std::string name)
{
	m_deviceName = name;
}

//
// Liefert den sprechenden Namen des Geraets.
//
std::string BasicDevice::getDeviceName()
{
	return m_deviceName;
}


} // namespace devices
