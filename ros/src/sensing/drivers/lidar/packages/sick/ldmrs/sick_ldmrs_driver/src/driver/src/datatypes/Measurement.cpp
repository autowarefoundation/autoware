//
// Measurement.cpp
//
// Defines a very basic measurement and its list
//
// Copyright (c) Sick AG
// created: 31.05.2010
//
// HISTORY
//
// 1.0.0	31.05.2010, VWi
//			Initial version.


#include <string>	// for std::string
#include <vector>	// for std::vector
#include "Measurement.hpp"
#include "../tools/toolbox.hpp"

namespace datatypes
{

//
// Returns the type as a readable string.
//
std::string Measurement::getName() const
{
	std::string name;
	
	switch (m_measType)
	{
		case Meastype_Unknown:
			name = "Unknown";
			break;
		case Meastype_Brightness:
			name = "Brightness";
			break;
		case Meastype_DeviceName:
			name = "DeviceName";
			break;
		case Meastype_DeviceVersion:
			name = "DeviceVersion";
			break;
		case Meastype_ScanFreq:
			name = "ScanFreq";
			break;
		case Meastype_ScanResolution:
			name = "ScanResolution";
			break;
		case Meastype_ScanStartAngle:
			name = "ScanStartAngle";
			break;
		case Meastype_ScanStopAngle:
			name = "ScanStopAngle";
			break;
		default:
			name = "(uninitialized)";
	}
		
	return name;
}

//
// Returns the value as a readable string.
//
std::string Measurement::valueToString() const
{
	std::string value;
	
	switch (m_measType)
	{
		case Meastype_Unknown:
			value = "Unknown";
			break;
		case Meastype_DeviceName:
		case Meastype_DeviceVersion:
			value = m_textValue;
			break;
		case Meastype_ScanFreq:
		case Meastype_ScanResolution:
		case Meastype_ScanStartAngle:
		case Meastype_ScanStopAngle:
			value = ::toString(m_doubleValue, 8);
			break;
		default:
			value = "(uninitialized)";
	}
		
	return value;
}


}	// namespace datatypes
