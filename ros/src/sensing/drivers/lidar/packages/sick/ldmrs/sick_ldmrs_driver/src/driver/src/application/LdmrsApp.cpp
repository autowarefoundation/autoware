//
// LdmrsApp.cpp
//
// Demo application. Receives all datatypes and just prints the data.
//

#include "LdmrsApp.hpp"
#include "../tools/errorhandler.hpp"	// for printInfoMessage()
#include "../tools/toolbox.hpp"			// for toString()
#include "../datatypes/Scan.hpp"
#include "../datatypes/Object.hpp"
#include "../datatypes/Msg.hpp"
#include "../datatypes/Measurement.hpp"
#include "../datatypes/Fields.hpp"
#include "../datatypes/EvalCases.hpp"
#include "../datatypes/EvalCaseResults.hpp"

namespace application
{

//
// Constructor
//
LdmrsApp::LdmrsApp(Manager* manager)
{
	m_beVerbose = true;

	printInfoMessage("LdmrsApp constructor done.", m_beVerbose);
}


// Destructor
// Clean up all dynamic data structures
LdmrsApp::~LdmrsApp()
{
	printInfoMessage("LdmrsApp says Goodbye!", m_beVerbose);
}


//
// Receiver for new data from the manager.
//
void LdmrsApp::setData(BasicData& data)
{
	//
	// Do something with it.
	//
	// Here, we just print what we've got.
	//
	std::string datatypeStr;
	std::string sourceIdStr;

	switch (data.getDatatype())
	{
		case Datatype_Scan:
			datatypeStr = "Scan (" + ::toString(((Scan&)data).getNumPoints()) + " points)";
			{
				// Print the scan start timestamp (NTP time) 
				Scan* scan = dynamic_cast<Scan*>(&data);
				const ScannerInfo* info = scan->getScannerInfoByDeviceId(1);
 
				if (info != NULL)
				{
					const Time& time = info->getStartTimestamp();
					printInfoMessage("LdmrsApp::setData(): Scan start time: " + time.toString(), m_beVerbose);
				}
			}
			break;
		case Datatype_Objects:
			datatypeStr = "Objects (" + ::toString(((ObjectList&)data).size()) + " objects)";
			break;
		case Datatype_Fields:
			datatypeStr = "Fields (" + ::toString(((Fields&)data).getFields().size()) + " fields, " +
							::toString(((Fields&)data).getNumberOfValidFields()) + " of which are valid)";
			break;
		case Datatype_EvalCases:
			datatypeStr = "EvalCases (" + ::toString(((EvalCases&)data).getEvalCases().size()) + " cases)";
			break;
		case Datatype_EvalCaseResults:
			datatypeStr = "EvalCaseResults (" + ::toString(((EvalCaseResults&)data).size()) + " case results)";
			break;
		case Datatype_Msg:
			datatypeStr = "Msg (" + ((Msg&)data).toString() + ")";
			break;
		case Datatype_MeasurementList:
			datatypeStr = "MeasurementList (" + ::toString(((MeasurementList&)data).m_list.size()) +" entries)";
			break;
		default:
			datatypeStr = "(unknown)";
	}
	
	sourceIdStr = ::toString(data.getSourceId());
	
	printInfoMessage("LdmrsApp::setData(): Called with data of type " + datatypeStr + " from ID " + sourceIdStr + ".", m_beVerbose);
}

}	// namespace application
