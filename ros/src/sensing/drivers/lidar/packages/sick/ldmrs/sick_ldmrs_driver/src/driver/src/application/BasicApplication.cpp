//
// BasicApplication.cpp
//

#include "BasicApplication.hpp"
#include "../tools/errorhandler.hpp"

namespace application
{

// ****************************************************************************
//     Basisklasse fuer Applikationen
// ****************************************************************************
BasicApplication::BasicApplication()
	: m_beVerbose(false)
	, m_applicationType(Sourcetype_Unknown)
	, m_sourceId(0)
{
	infoMessage("BasicApplication: Starting constructor.", m_beVerbose);
}

BasicApplication::~BasicApplication()
{
	infoMessage("BasicApplication(): Destructor called.", m_beVerbose);
	
	infoMessage("BasicApplication(): Destructor is done - object is dead.", m_beVerbose);
}

//
// Setze einen sprechenden Namen.
//
void BasicApplication::setApplicationName(std::string appName)
{
	m_applicationName = appName;
}

//
// Liefert den sprechenden Namen.
//
std::string BasicApplication::getApplicationName()
{
	return m_applicationName;
}

//
// Der Typ der Applikation, als Zahl.
//
// Basistyp ist "Sourcetype".
//
UINT16 BasicApplication::getApplicationType()
{
	return m_applicationType;
}

//
// Der Typ der Applikation, als Zahl.
//
// Basistyp ist "Sourcetype".
//
void BasicApplication::setApplicationType(UINT16 applicationtype)
{
	m_applicationType = applicationtype;
}

/**
 * Die System-Eindeutige ID der Applikation, als Zahl.
 */
UINT16 BasicApplication::getSourceId()
{
	return m_sourceId;
}

/**
 * Die System-Eindeutige ID der Applikation, als Zahl.
 */
void BasicApplication::setSourceId(UINT16 sourceId)
{
	m_sourceId = sourceId;
}

}	// namespace application
