//
// Measurement.hpp
//
// Defines a very basic measurement structure
// Copyright (c) Sick AG
// created: 31.05.2010
//
// HISTORY
//
// 1.0.0	31.05.2010, VWi
//			Initial version.


#ifndef MEASUREMENT_HPP
#define MEASUREMENT_HPP

#include <string>	// for std::string
#include <vector>	// for std::vector
#include "../BasicDatatypes.hpp"

//
// Advanced types
//
enum MeasurementType
{
	Meastype_Unknown 				= 0x0000,
	
	Meastype_Voltage_Supply_Panel	= 0x0001,
	Meastype_Temperature_Panel		= 0x0002,
	Meastype_Brightness				= 0x0003,
	Meastype_Voltage_Supply_Video	= 0x0004,
	Meastype_Sniffpad_Panel			= 0x0005,
	
	Meastype_DeviceName				= 0x0006,
	Meastype_DeviceVersion			= 0x0007,
	Meastype_ScanFreq				= 0x0008,
	Meastype_ScanStartAngle			= 0x0009,
	Meastype_ScanStopAngle			= 0x000A,
	Meastype_ScanResolution			= 0x000B
};

namespace datatypes
{

class Measurement
{
public:
	Measurement() {m_measType = Meastype_Unknown;};
	virtual ~Measurement() {};

	// Estimate the memory usage of this object
	inline virtual const UINT32 getUsedMemory() const {return ((sizeof(*this)) + m_textValue.length());};

	std::string getName() const;		// Returns the type as a readable string
	std::string valueToString() const;	// Returns the value as a readable string
	
	MeasurementType m_measType;
	

	
	double m_doubleValue;
	INT32 m_intValue;
	std::string m_textValue;
};


// -----------------------------------------------


class MeasurementList : public BasicData
{
public:
//	MeasurementList():  m_datatype(Datatype_MeasurementList) {}	//  {m_datatype = Datatype_MeasurementList;};
	MeasurementList() {m_datatype = Datatype_MeasurementList;};
	virtual ~MeasurementList() {};

	// Estimate the memory usage of this object
	inline virtual const UINT32 getUsedMemory() const
	{
		UINT32 sum = sizeof(*this);
		std::vector<Measurement>::const_iterator iter;
		for (iter = m_list.begin(); iter != m_list.end(); iter++)
		{
			sum += iter->getUsedMemory();
		}
		return sum;
	}
	
	std::vector<Measurement> m_list;
};

}	// namespace datatypes

#endif // MEASUREMENT_HPP
