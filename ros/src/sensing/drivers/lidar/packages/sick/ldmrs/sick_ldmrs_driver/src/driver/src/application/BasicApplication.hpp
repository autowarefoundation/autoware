/**
 * \file BasicApplication.hpp
 */

#ifndef BASICAPPLICATION_HPP
#define BASICAPPLICATION_HPP

#include "../BasicDatatypes.hpp"

namespace application
{

using namespace datatypes;
	
// Container fuer alle Applikationen ("Worker", "CROWNlets")
class BasicApplication
{
public:
	BasicApplication();
	virtual ~BasicApplication();
	
	UINT16 getApplicationType();
	UINT16 getSourceId();
	void setSourceId(UINT16 applicationId);					// Eindeutige ID im System, wird vom Manager vergeben.
	void setApplicationName(std::string appName);
	std::string getApplicationName();
	
	// Diese Funktionen muessen da sein.
	virtual void setData(BasicData& data) = 0;

protected:
	void setApplicationType(UINT16 applicationtype);

private:
	bool m_beVerbose;	// Debug-Ausgaben an oder aus.
	UINT16 m_applicationType;
	UINT16 m_sourceId;
	std::string m_applicationName;
};

}	// namespace application

#endif
