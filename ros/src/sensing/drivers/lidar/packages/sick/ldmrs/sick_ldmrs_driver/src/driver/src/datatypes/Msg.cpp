//
//
// Msg.cpp
//

#include "Msg.hpp"
#include <sstream>


// ////////////////////////////////////////////////////////////
namespace datatypes
{
	
Msg::Msg(const UINT16 sourceId)
{
	m_sourceId = sourceId;
	m_datatype = Datatype_Msg;
}

Msg::Msg(const UINT16 sourceId, const std::string message)
{
	m_sourceId = sourceId;
	m_datatype = Datatype_Msg;
	m_msgText = message;
}

/// Conversion to string for debugging
std::string Msg::toString() const
{
	return m_msgText;
}

std::string Msg::getMsg() const
{
	return toString();
}

// Set the text
void Msg::setMsg(std::string message)
{
	m_msgText = message;
}

/// Text output for debugging
std::ostream& operator<<(std::ostream& os, const Msg& msg)
{
	os << msg.getMsg();
	return os;
}


}	// namespace datatypes
