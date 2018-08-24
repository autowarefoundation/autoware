//
// LdmrsFieldApp.hpp
//

#ifndef LDMRSFIELDAPP_HPP
#define LDMRSFIELDAPP_HPP

#include "../manager.hpp"
#include "../tools/Mutex.hpp"
#include "../devices/LD_MRS.hpp"

namespace application
{
	
//
// LdmrsApp
//
class LdmrsFieldApp : public BasicApplication
{
public:
	LdmrsFieldApp(Manager* manager);
	virtual ~LdmrsFieldApp();

protected:
	void setData(BasicData& data);	// Receiver
	
private:
	bool m_beVerbose;
	Manager* m_manager;
	void changeThreadFunction(bool& endThread, UINT16& waitTimeMs);
	SickThread<LdmrsFieldApp, &LdmrsFieldApp::changeThreadFunction> m_changeThread;
	
	void thread_removeAllFields(devices::LDMRS* ldmrs);
	void thread_createRectangularField(devices::LDMRS* ldmrs);
	void thread_removeAllEvalCases(devices::LDMRS* ldmrs);
	void thread_createEvalCase(devices::LDMRS* ldmrs);
};

}	// namespace application

#endif
