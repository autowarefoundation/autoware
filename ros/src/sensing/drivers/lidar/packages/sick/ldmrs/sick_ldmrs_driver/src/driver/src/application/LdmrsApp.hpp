//
// LdmrsApp.hpp
//

#ifndef LDMRSAPP_HPP
#define LDMRSAPP_HPP

#include "../manager.hpp"

namespace application
{
	
//
// LdmrsApp
//
class LdmrsApp : public BasicApplication
{
public:
	LdmrsApp(Manager* manager);
	virtual ~LdmrsApp();

protected:
	void setData(BasicData& data);	// Receiver
	
private:
	bool m_beVerbose;
};

}	// namespace application

#endif
