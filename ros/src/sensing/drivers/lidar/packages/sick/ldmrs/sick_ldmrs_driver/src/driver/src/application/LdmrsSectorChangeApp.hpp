//
// LdmrsSectorChangeApp.hpp
//

#ifndef LDMRSSECTORCHANGEAPP_HPP
#define LDMRSSECTORCHANGEAPP_HPP

#include "../manager.hpp"
#include "../tools/Mutex.hpp"
#include "../datatypes/Scan.hpp"
#include "../devices/LuxBase.hpp"

namespace application
{
	
//
// LdmrsApp
//
class LdmrsSectorChangeApp : public BasicApplication
{
public:
	LdmrsSectorChangeApp(Manager* manager);
	virtual ~LdmrsSectorChangeApp();

protected:
	void setData(BasicData& data);	// Receiver
	
private:
	void checkResolution(Scan& scan);
	void changeThreadFunction(bool& endThread, UINT16& waitTimeMs);
	bool changeFlexResConfiguration(const ScannerInfo::ResolutionMap& configuredRM);
	bool changeAngularResolutionType(devices::AngularResolutionType type);
	bool readDetailedErrorCode(UINT32* code = NULL);

private:
	bool m_beVerbose;
	Manager* m_manager;
	ScannerInfo::ResolutionMap m_lastMeasuredSector;
	SickThread<LdmrsSectorChangeApp, &LdmrsSectorChangeApp::changeThreadFunction> m_changeThread;
};

}	// namespace application

#endif
