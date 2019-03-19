//
// Trigger.cpp
// Contains the class "Trigger"
//

//	HISTORY
//
// 1.0.0	2011-06-06, VWi
//			Initial version


#include "Trigger.hpp"
#include "../tools/errorhandler.hpp"
#include <sstream>	// for std::ostringstream

namespace datatypes
{
	
Trigger::Trigger()
	: m_number (0)
{
	m_datatype = Datatype_Trigger;
}

/**
 * \param number Number of this message (0..UINT32)
 */
Trigger::Trigger (UINT32 number, UINT8 sourceId)
	: m_number (number)
{
	m_datatype = Datatype_Trigger;
	m_sourceId = sourceId;
}


// MSVC requires that virtual destructors are located in the cpp-file ...
Trigger::~Trigger()
{}


void Trigger::setNumber (UINT32 number)
{
	m_number = number;
}


//
// For debug
//
std::string Trigger::toString()
{
	std::ostringstream stream;

	stream	<< "DeviceID="
			<< m_sourceId
			<< ", Number="
			<< m_number;

	return stream.str();
}

}	// namespace datatypes

