//
// LdmrsNtpTimeApp.hpp
//

#ifndef LDMRSNTPTIMEAPP_HPP
#define LDMRSNTPTIMEAPP_HPP

#include "../manager.hpp"
#include "../tools/Mutex.hpp"
#include "../datatypes/Scan.hpp"
#include "../devices/LuxBase.hpp"

namespace application
{

struct ntp_time_t
{
	uint32_t   second;		// Seconds since 1.1.1900
	uint32_t   fraction;
};

//
// LdmrsNtpTimeApp
//
class LdmrsNtpTimeApp : public BasicApplication
{
public:
	LdmrsNtpTimeApp(Manager* manager);
	virtual ~LdmrsNtpTimeApp();

protected:
	void setData(BasicData& data);	// Receiver
	
private:
	ntp_time_t convertUnixTimeToNtpTime(struct timeval& unixTime);
	
//	void checkResolution(Scan& scan);
	void changeThreadFunction(bool& endThread, UINT16& waitTimeMs);
//	bool changeFlexResConfiguration(const ScannerInfo::ResolutionMap& configuredRM);
//	bool changeAngularResolutionType(devices::AngularResolutionType type);
//	bool readDetailedErrorCode(UINT32* code = NULL);

private:
	bool m_beVerbose;
	Manager* m_manager;
	SickThread<LdmrsNtpTimeApp, &LdmrsNtpTimeApp::changeThreadFunction> m_changeThread;
};

}	// namespace application

#endif
